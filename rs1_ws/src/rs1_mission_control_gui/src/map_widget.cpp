#include "rs1_mission_control_gui/map_widget.hpp"

#include <QPainter>
#include <QPen>
#include <QBrush>
#include <QPainterPath>
#include <QPolygonF>

#include <algorithm>
#include <cmath>

namespace rs1_mission_control_gui
{

MapWidget::MapWidget(QWidget * parent)
: QWidget(parent)
{
  setMinimumSize(400, 400);
  setAutoFillBackground(true);
  QPalette pal = palette();
  pal.setColor(QPalette::Window, QColor(214, 226, 214));
  setPalette(pal);
}

void MapWidget::setAgbMap(
  int width, int height, double resolution,
  double origin_x, double origin_y,
  const std::vector<int8_t> & data)
{
  map_width_ = width;
  map_height_ = height;
  map_resolution_ = resolution;
  map_origin_x_ = origin_x;
  map_origin_y_ = origin_y;
  map_data_ = data;
  update();
}

void MapWidget::updateRobotPose(const QString & robot_name, double x, double y, double yaw)
{
  RobotPose & pose = robot_poses_[robot_name];
  pose.x = x;
  pose.y = y;
  pose.yaw = yaw;
  pose.valid = true;

  auto & trail = robot_trails_[robot_name];
  trail.emplace_back(x, y);
  while (static_cast<int>(trail.size()) > kMaxTrailPoints) {
    trail.pop_front();
  }

  update();
}

void MapWidget::setRobotColor(const QString & robot_name, const QColor & color)
{
  robot_colors_[robot_name] = color;
}

void MapWidget::setOtherTrees(const std::vector<QPointF> & tree_world_positions)
{
  other_trees_ = tree_world_positions;
  update();
}

void MapWidget::setSelectedTrees(const std::vector<QPointF> & tree_world_positions)
{
  selected_trees_ = tree_world_positions;
  update();
}

void MapWidget::setRoiBoundary(const std::vector<QPointF> & boundary_world_points)
{
  roi_boundary_ = boundary_world_points;
  update();
}

void MapWidget::zoomIn()
{
  zoom_factor_ = std::min(zoom_factor_ * 1.25, 8.0);
  update();
}

void MapWidget::zoomOut()
{
  zoom_factor_ = std::max(zoom_factor_ / 1.25, 0.2);
  update();
}

void MapWidget::resetView()
{
  zoom_factor_ = 1.0;
  pan_offset_px_ = QPointF(0.0, 0.0);
  update();
}

double MapWidget::effectiveScale() const
{
  double world_w = (map_width_ > 0) ? map_width_ * map_resolution_ : 40.0;
  double world_h = (map_height_ > 0) ? map_height_ * map_resolution_ : 40.0;
  double base_scale = std::min(width() / world_w, height() / world_h);
  return base_scale * zoom_factor_;
}

QPointF MapWidget::worldToWidget(double wx, double wy) const
{
  double scale = effectiveScale();
  double origin_x = (map_width_ > 0) ? map_origin_x_ : -20.0;
  double origin_y = (map_height_ > 0) ? map_origin_y_ : -20.0;

  double px = (wx - origin_x) * scale + pan_offset_px_.x();
  // Flip Y: world Y increases "up", widget Y increases "down".
  double py = height() - ((wy - origin_y) * scale) - pan_offset_px_.y();
  return QPointF(px, py);
}

void MapWidget::wheelEvent(QWheelEvent * event)
{
  if (event->angleDelta().y() > 0) {
    zoomIn();
  } else if (event->angleDelta().y() < 0) {
    zoomOut();
  }
  event->accept();
}

void MapWidget::mousePressEvent(QMouseEvent * event)
{
  if (event->button() == Qt::LeftButton) {
    dragging_ = true;
    last_mouse_pos_ = event->pos();
    setCursor(Qt::ClosedHandCursor);
  }
}

void MapWidget::mouseMoveEvent(QMouseEvent * event)
{
  if (dragging_) {
    QPoint delta = event->pos() - last_mouse_pos_;
    pan_offset_px_ += QPointF(delta.x(), -delta.y());
    last_mouse_pos_ = event->pos();
    update();
  }
}

void MapWidget::mouseReleaseEvent(QMouseEvent * /*event*/)
{
  dragging_ = false;
  unsetCursor();
}

void MapWidget::paintEvent(QPaintEvent * /*event*/)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.fillRect(rect(), QColor(214, 226, 214));

  // --- AGB map as a biomass heat-map ---
  if (map_width_ > 0 && map_height_ > 0 &&
    static_cast<int>(map_data_.size()) == map_width_ * map_height_)
  {
    double scale = effectiveScale();
    double cell_px = std::max(1.0, map_resolution_ * scale);

    for (int row = 0; row < map_height_; ++row) {
      for (int col = 0; col < map_width_; ++col) {
        int8_t value = map_data_[row * map_width_ + col];
        if (value < 0) {
          continue;  // unknown cell - leave background showing
        }
        double wx = map_origin_x_ + (col + 0.5) * map_resolution_;
        double wy = map_origin_y_ + (row + 0.5) * map_resolution_;
        QPointF center = worldToWidget(wx, wy);

        double t = std::clamp(value / 100.0, 0.0, 1.0);
        int r = static_cast<int>(70 + t * 60);
        int g = static_cast<int>(140 - t * 50);
        int b = static_cast<int>(60 - t * 30);
        painter.fillRect(
          QRectF(center.x() - cell_px / 2.0, center.y() - cell_px / 2.0, cell_px, cell_px),
          QColor(r, g, b));
      }
    }
  } else {
    painter.setPen(QColor(90, 100, 90));
    painter.drawText(rect(), Qt::AlignCenter, "Waiting for AGB map on /agb_map ...");
  }

  // --- ROI boundary (dashed white polygon) ---
  if (roi_boundary_.size() >= 2) {
    painter.setPen(QPen(Qt::white, 2, Qt::DashLine));
    painter.setBrush(Qt::NoBrush);
    QPolygonF polygon;
    for (const auto & pt : roi_boundary_) {
      polygon << worldToWidget(pt.x(), pt.y());
    }
    painter.drawPolygon(polygon);
  }

  // --- Robot travelled paths (dashed, colored per robot) ---
  for (const auto & entry : robot_trails_) {
    const QString & name = entry.first;
    const std::deque<QPointF> & trail = entry.second;
    if (trail.size() < 2) {
      continue;
    }
    QColor color = robot_colors_.count(name) ? robot_colors_.at(name) : QColor(Qt::yellow);
    painter.setPen(QPen(color, 2, Qt::DashLine));
    QPolygonF polyline;
    for (const auto & pt : trail) {
      polyline << worldToWidget(pt.x(), pt.y());
    }
    painter.drawPolyline(polyline);
  }

  // --- Ground station icon (fixed at world origin) ---
  {
    QPointF gs = worldToWidget(ground_station_world_.x(), ground_station_world_.y());
    QPainterPath house;
    house.moveTo(gs.x() - 8, gs.y() + 6);
    house.lineTo(gs.x() - 8, gs.y() - 2);
    house.lineTo(gs.x(), gs.y() - 10);
    house.lineTo(gs.x() + 8, gs.y() - 2);
    house.lineTo(gs.x() + 8, gs.y() + 6);
    house.closeSubpath();
    painter.setPen(QPen(Qt::black, 1));
    painter.setBrush(Qt::white);
    painter.drawPath(house);
  }

  // --- Other trees (gray) ---
  painter.setPen(Qt::NoPen);
  painter.setBrush(QColor(150, 150, 150));
  for (const auto & tree_world : other_trees_) {
    QPointF p = worldToWidget(tree_world.x(), tree_world.y());
    painter.drawEllipse(p, 4, 4);
  }

  // --- Selected trees (green) ---
  painter.setBrush(QColor(34, 197, 94));
  for (const auto & tree_world : selected_trees_) {
    QPointF p = worldToWidget(tree_world.x(), tree_world.y());
    painter.drawEllipse(p, 5, 5);
  }

  // --- Robot markers ---
  for (const auto & entry : robot_poses_) {
    const QString & name = entry.first;
    const RobotPose & pose = entry.second;
    if (!pose.valid) {
      continue;
    }
    QPointF p = worldToWidget(pose.x, pose.y);
    QColor color = robot_colors_.count(name) ? robot_colors_.at(name) : QColor(Qt::yellow);

    painter.setPen(QPen(Qt::black, 1));
    painter.setBrush(color);
    painter.drawEllipse(p, 7, 7);

    double hx = p.x() + 12.0 * std::cos(-pose.yaw);
    double hy = p.y() + 12.0 * std::sin(-pose.yaw);
    painter.setPen(QPen(color, 2));
    painter.drawLine(p, QPointF(hx, hy));

    painter.setPen(QPen(Qt::black, 1));
    painter.drawText(p + QPointF(10, -10), name);
  }

  // --- Compass ---
  painter.setPen(QPen(Qt::black, 2));
  painter.drawLine(QPointF(20, 40), QPointF(20, 15));
  painter.drawLine(QPointF(20, 15), QPointF(14, 25));
  painter.drawLine(QPointF(20, 15), QPointF(26, 25));
  painter.drawText(QPointF(14, 55), "N");

  // --- Scale bar (single segment, snapped to a "nice" round distance) ---
  {
    double scale = effectiveScale();
    if (scale > 0.0) {
      const double candidates[] = {1, 2, 5, 10, 20, 50, 100, 200, 500, 1000};
      double target_px = 100.0;
      double best_meters = candidates[0];
      double best_diff = std::abs(candidates[0] * scale - target_px);
      for (double m : candidates) {
        double diff = std::abs(m * scale - target_px);
        if (diff < best_diff) {
          best_diff = diff;
          best_meters = m;
        }
      }
      double bar_px = best_meters * scale;
      double base_y = height() - 20.0;
      double base_x = 20.0;
      painter.setPen(QPen(Qt::black, 2));
      painter.drawLine(QPointF(base_x, base_y), QPointF(base_x + bar_px, base_y));
      painter.drawLine(QPointF(base_x, base_y - 4), QPointF(base_x, base_y + 4));
      painter.drawLine(
        QPointF(base_x + bar_px, base_y - 4), QPointF(base_x + bar_px, base_y + 4));
      painter.drawText(
        QPointF(base_x, base_y - 8), QString("%1 m").arg(static_cast<int>(best_meters)));
    }
  }
}

}  // namespace rs1_mission_control_gui
