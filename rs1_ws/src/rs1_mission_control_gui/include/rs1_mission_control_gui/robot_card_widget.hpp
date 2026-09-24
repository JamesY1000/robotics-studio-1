#ifndef RS1_MISSION_CONTROL_GUI__ROBOT_CARD_WIDGET_HPP_
#define RS1_MISSION_CONTROL_GUI__ROBOT_CARD_WIDGET_HPP_

#include <QWidget>
#include <QLabel>
#include <QString>
#include <QColor>
#include <QImage>

namespace rs1_mission_control_gui
{

// A dashboard card for one robot: header (name + online/offline +
// battery), an RGB camera feed, a LiDAR-view placeholder, and telemetry
// fields that differ depending on whether the robot is a UAV (altitude)
// or a ground robot (position).
class RobotCardWidget : public QWidget
{
  Q_OBJECT

public:
  RobotCardWidget(
    const QString & display_name, bool is_uav, const QColor & accent_color,
    QWidget * parent = nullptr);

  void setOnline(bool online);
  void setBatteryPercent(float percent);
  void setMode(const QString & mode);
  void setSpeed(float speed_m_s);
  void setAltitude(float altitude_m);   // used when is_uav == true
  void setPosition(double x, double y);  // used when is_uav == false
  void setCameraImage(const QImage & image);

private:
  bool is_uav_;

  QLabel * status_dot_;
  QLabel * status_text_;
  QLabel * battery_label_;
  QLabel * camera_label_;
  QLabel * lidar_label_;
  QLabel * telemetry_value_1_;  // altitude (UAV) or position (ground)
  QLabel * telemetry_value_2_;  // speed
  QLabel * telemetry_value_3_;  // mode
};

}  // namespace rs1_mission_control_gui

#endif  // RS1_MISSION_CONTROL_GUI__ROBOT_CARD_WIDGET_HPP_
