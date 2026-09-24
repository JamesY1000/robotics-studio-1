#ifndef RS1_MISSION_CONTROL_GUI__MAP_WIDGET_HPP_
#define RS1_MISSION_CONTROL_GUI__MAP_WIDGET_HPP_

#include <QWidget>
#include <QString>
#include <QColor>
#include <QPointF>
#include <QMouseEvent>
#include <QWheelEvent>

#include <cstdint>
#include <deque>
#include <map>
#include <vector>

namespace rs1_mission_control_gui
{

struct RobotPose
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  bool valid = false;
};

// Renders the AGB grid, ROI boundary, ground station, robot positions +
// travelled paths, and tree markers (selected vs other), all in the
// map's world frame. Supports zoom (buttons or scroll wheel) and
// click-drag pan.
class MapWidget : public QWidget
{
  Q_OBJECT

public:
  explicit MapWidget(QWidget * parent = nullptr);

  // Mirrors nav_msgs/OccupancyGrid layout: row-major cells, each 0-100
  // (biomass "density"), -1 for unknown.
  void setAgbMap(
    int width, int height, double resolution,
    double origin_x, double origin_y,
    const std::vector<int8_t> & data);

  void updateRobotPose(const QString & robot_name, double x, double y, double yaw);
  void setRobotColor(const QString & robot_name, const QColor & color);

  void setOtherTrees(const std::vector<QPointF> & tree_world_positions);
  void setSelectedTrees(const std::vector<QPointF> & tree_world_positions);
  void setRoiBoundary(const std::vector<QPointF> & boundary_world_points);

public Q_SLOTS:
  void zoomIn();
  void zoomOut();
  void resetView();

protected:
  void paintEvent(QPaintEvent * event) override;
  void wheelEvent(QWheelEvent * event) override;
  void mousePressEvent(QMouseEvent * event) override;
  void mouseMoveEvent(QMouseEvent * event) override;
  void mouseReleaseEvent(QMouseEvent * event) override;

private:
  QPointF worldToWidget(double wx, double wy) const;
  double effectiveScale() const;

  // AGB map
  int map_width_ = 0;
  int map_height_ = 0;
  double map_resolution_ = 1.0;
  double map_origin_x_ = 0.0;
  double map_origin_y_ = 0.0;
  std::vector<int8_t> map_data_;

  std::map<QString, RobotPose> robot_poses_;
  std::map<QString, QColor> robot_colors_;
  std::map<QString, std::deque<QPointF>> robot_trails_;
  static constexpr int kMaxTrailPoints = 300;

  std::vector<QPointF> other_trees_;
  std::vector<QPointF> selected_trees_;
  std::vector<QPointF> roi_boundary_;

  // Fixed convention point for the "Ground Station" icon. Could be wired
  // to a topic later if the home position becomes dynamic.
  const QPointF ground_station_world_{0.0, 0.0};

  double zoom_factor_ = 1.0;
  QPointF pan_offset_px_{0.0, 0.0};
  bool dragging_ = false;
  QPoint last_mouse_pos_;
};

}  // namespace rs1_mission_control_gui

#endif  // RS1_MISSION_CONTROL_GUI__MAP_WIDGET_HPP_
