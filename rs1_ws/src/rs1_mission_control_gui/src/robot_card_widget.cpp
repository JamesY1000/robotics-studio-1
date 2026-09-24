#include "rs1_mission_control_gui/robot_card_widget.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QFont>
#include <QPixmap>

namespace rs1_mission_control_gui
{

namespace
{
QLabel * makeImagePlaceholder(const QString & text)
{
  auto * label = new QLabel(text);
  label->setAlignment(Qt::AlignCenter);
  label->setMinimumHeight(100);
  label->setStyleSheet(
    "background-color: #111318; color: #8a8f98; border-radius: 6px; font-size: 10px;");
  label->setWordWrap(true);
  return label;
}
}  // namespace

RobotCardWidget::RobotCardWidget(
  const QString & display_name, bool is_uav, const QColor & accent_color, QWidget * parent)
: QWidget(parent), is_uav_(is_uav)
{
  setObjectName("Card");
  setStyleSheet(
    QString(
      "QLabel { color: #1f2937; } "
      "#Card { background: white; border: 1px solid #e2e5e9; "
      "border-left: 4px solid %1; border-radius: 10px; }")
    .arg(accent_color.name()));

  auto * outer = new QVBoxLayout(this);

  // --- Header row ---
  auto * header = new QHBoxLayout();
  auto * name_label = new QLabel(display_name);
  QFont name_font = name_label->font();
  name_font.setBold(true);
  name_font.setPointSize(name_font.pointSize() + 1);
  name_label->setFont(name_font);
  header->addWidget(name_label);

  header->addStretch();

  status_dot_ = new QLabel();
  status_dot_->setFixedSize(10, 10);
  header->addWidget(status_dot_);
  status_text_ = new QLabel("Offline");
  header->addWidget(status_text_);

  battery_label_ = new QLabel("--%");
  header->addWidget(battery_label_);

  outer->addLayout(header);

  // --- Camera / LiDAR row ---
  auto * feeds = new QHBoxLayout();
  camera_label_ = makeImagePlaceholder("RGB Camera\n(waiting for image)");
  lidar_label_ = makeImagePlaceholder("LiDAR View\n(point cloud rendering\nnot implemented\nin this build)");
  feeds->addWidget(camera_label_);
  feeds->addWidget(lidar_label_);
  outer->addLayout(feeds);

  // --- Telemetry row ---
  auto * telemetry = new QGridLayout();
  QString label1 = is_uav_ ? "Altitude" : "Position (x, y)";

  telemetry_value_1_ = new QLabel("--");
  telemetry_value_2_ = new QLabel("--");
  telemetry_value_3_ = new QLabel("--");

  auto addTelemetryColumn = [&](int col, const QString & label, QLabel * value_label) {
    auto * label_widget = new QLabel(label);
    label_widget->setStyleSheet("color: #6b7280; font-size: 10px;");
    telemetry->addWidget(label_widget, 0, col);
    telemetry->addWidget(value_label, 1, col);
  };
  addTelemetryColumn(0, label1, telemetry_value_1_);
  addTelemetryColumn(1, "Speed", telemetry_value_2_);
  addTelemetryColumn(2, "Mode", telemetry_value_3_);

  outer->addLayout(telemetry);

  setOnline(false);
}

void RobotCardWidget::setOnline(bool online)
{
  status_dot_->setStyleSheet(
    QString("border-radius: 5px; background-color: %1;")
    .arg(online ? "#22c55e" : "#9ca3af"));
  status_text_->setText(online ? "Online" : "Offline");
}

void RobotCardWidget::setBatteryPercent(float percent)
{
  battery_label_->setText(QString("%1%").arg(static_cast<int>(percent)));
}

void RobotCardWidget::setMode(const QString & mode)
{
  telemetry_value_3_->setText(mode);
}

void RobotCardWidget::setSpeed(float speed_m_s)
{
  telemetry_value_2_->setText(QString("%1 m/s").arg(speed_m_s, 0, 'f', 1));
}

void RobotCardWidget::setAltitude(float altitude_m)
{
  if (is_uav_) {
    telemetry_value_1_->setText(QString("%1 m").arg(altitude_m, 0, 'f', 0));
  }
}

void RobotCardWidget::setPosition(double x, double y)
{
  if (!is_uav_) {
    telemetry_value_1_->setText(QString("(%1, %2) m").arg(x, 0, 'f', 1).arg(y, 0, 'f', 1));
  }
}

void RobotCardWidget::setCameraImage(const QImage & image)
{
  if (image.isNull()) {
    return;
  }
  QPixmap pixmap = QPixmap::fromImage(image).scaled(
    camera_label_->width() > 0 ? camera_label_->width() : 220, 100,
    Qt::KeepAspectRatio, Qt::SmoothTransformation);
  camera_label_->setPixmap(pixmap);
  camera_label_->setText("");
}

}  // namespace rs1_mission_control_gui
