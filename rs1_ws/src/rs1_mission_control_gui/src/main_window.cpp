#include "rs1_mission_control_gui/main_window.hpp"

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QFont>
#include <QFrame>
#include <QHeaderView>
#include <QAbstractItemView>
#include <QTableWidgetItem>
#include <QBrush>
#include <QColor>
#include <QDateTime>
#include <QMessageBox>

#include <algorithm>
#include <cmath>

namespace rs1_mission_control_gui
{

namespace
{
// Order matches the reference layout: UAV card first, then UGV.
const std::vector<QString> kRobotNamespaces = {"parrot1", "husky1"};

QImage imageMsgToQImage(const sensor_msgs::msg::Image & msg)
{
  if (msg.encoding == "rgb8") {
    QImage img(msg.data.data(), msg.width, msg.height, msg.step, QImage::Format_RGB888);
    return img.copy();  // deep copy - msg.data goes out of scope after the callback
  }
  if (msg.encoding == "bgr8") {
    QImage img(msg.data.data(), msg.width, msg.height, msg.step, QImage::Format_RGB888);
    return img.rgbSwapped();  // rgbSwapped() already returns a new, owned image
  }
  return QImage();  // unsupported encoding - caller ignores a null image
}

QLabel * makeCardTitle(const QString & text)
{
  auto * label = new QLabel(text);
  QFont font = label->font();
  font.setBold(true);
  font.setPointSize(font.pointSize() + 1);
  label->setFont(font);
  return label;
}

QFrame * makeCard()
{
  auto * frame = new QFrame();
  frame->setObjectName("Card");
  frame->setStyleSheet(
    "#Card { background: white; border: 1px solid #e2e5e9; border-radius: 10px; }");
  return frame;
}
}  // namespace

MainWindow::MainWindow(rclcpp::Node::SharedPtr node, QWidget * parent)
: QMainWindow(parent), node_(std::move(node))
{
  setWindowTitle("Mission Control - Forest Carbon Monitoring");
  resize(1500, 950);
  setStyleSheet("QMainWindow { background: #f4f6f8; } QLabel { color: #1f2937; }");

  auto * central = new QWidget(this);
  auto * root_layout = new QHBoxLayout(central);
  root_layout->setContentsMargins(0, 0, 0, 0);
  root_layout->setSpacing(0);

  root_layout->addWidget(buildSidebar());

  auto * right_side = new QWidget();
  auto * right_layout = new QVBoxLayout(right_side);
  right_layout->addWidget(buildTopBar());

  auto * content = new QHBoxLayout();
  auto * left_column = new QVBoxLayout();
  left_column->addWidget(buildMapCard(), /*stretch=*/3);

  auto * bottom_row = new QHBoxLayout();
  bottom_row->addWidget(buildMissionProgressCard());
  bottom_row->addWidget(buildSelectedTreesCard());
  bottom_row->addWidget(buildAgbEstimateCard());
  left_column->addLayout(bottom_row, /*stretch=*/1);

  content->addLayout(left_column, /*stretch=*/3);
  content->addWidget(buildRobotColumn(), /*stretch=*/1);

  right_layout->addLayout(content);
  root_layout->addWidget(right_side, /*stretch=*/1);

  setCentralWidget(central);

  setupRosInterfaces();

  ros_spin_timer_ = new QTimer(this);
  connect(ros_spin_timer_, &QTimer::timeout, this, &MainWindow::spinRos);
  ros_spin_timer_->start(20);

  clock_timer_ = new QTimer(this);
  connect(clock_timer_, &QTimer::timeout, this, &MainWindow::updateClock);
  clock_timer_->start(1000);
  updateClock();

  updateReadyBadge();
  updateMissionProgressUi();
}

MainWindow::~MainWindow() = default;

QWidget * MainWindow::buildSidebar()
{
  auto * sidebar = new QWidget();
  sidebar->setFixedWidth(220);
  sidebar->setStyleSheet("background: white; border-right: 1px solid #e2e5e9;");
  auto * layout = new QVBoxLayout(sidebar);

  auto * brand = new QLabel("ROBS1");
  QFont brand_font = brand->font();
  brand_font.setBold(true);
  brand_font.setPointSize(brand_font.pointSize() + 3);
  brand->setFont(brand_font);
  layout->addWidget(brand);

  auto * subtitle = new QLabel("Forest Carbon Monitoring");
  subtitle->setStyleSheet("color: #6b7280; font-size: 11px;");
  layout->addWidget(subtitle);
  layout->addSpacing(16);

  const QStringList nav_items = {
    "Mission", "Robots", "Map", "Tree Data", "Results", "Settings"};
  for (int i = 0; i < nav_items.size(); ++i) {
    auto * button = new QPushButton(nav_items[i]);
    button->setCheckable(true);
    if (i == 0) {
      button->setChecked(true);
      button->setStyleSheet(
        "QPushButton { background: #e8f5ec; color: #15803d; text-align: left; "
        "padding: 8px 12px; border-radius: 6px; border: none; }");
    } else {
      // Only the Mission view is implemented in this build - the rest
      // are shown for layout parity with the reference design but
      // disabled rather than faked.
      button->setEnabled(false);
      button->setToolTip("Not implemented in this build");
      button->setStyleSheet(
        "QPushButton { color: #9ca3af; text-align: left; padding: 8px 12px; "
        "border-radius: 6px; border: none; }");
    }
    layout->addWidget(button);
  }

  layout->addStretch();

  auto * divider = new QFrame();
  divider->setFrameShape(QFrame::HLine);
  layout->addWidget(divider);

  layout->addWidget(makeCardTitle("System Status"));

  auto addStatusRow = [&](const QString & label, QLabel *& dot_out) {
    auto * row = new QHBoxLayout();
    dot_out = new QLabel();
    dot_out->setFixedSize(8, 8);
    dot_out->setStyleSheet("border-radius: 4px; background-color: #9ca3af;");
    row->addWidget(dot_out);
    row->addWidget(new QLabel(label));
    row->addStretch();
    layout->addLayout(row);
  };
  addStatusRow("UAV", uav_status_dot_);
  addStatusRow("UGV", ugv_status_dot_);
  addStatusRow("Comms", comms_status_dot_);

  layout->addSpacing(16);
  auto * uts_label = new QLabel("UTS");
  QFont uts_font = uts_label->font();
  uts_font.setBold(true);
  uts_label->setFont(uts_font);
  layout->addWidget(uts_label);
  auto * studio_label = new QLabel("Robotics Studio 1");
  studio_label->setStyleSheet("color: #6b7280; font-size: 10px;");
  layout->addWidget(studio_label);

  return sidebar;
}

QWidget * MainWindow::buildTopBar()
{
  auto * bar = new QWidget();
  auto * layout = new QHBoxLayout(bar);

  auto * title_col = new QVBoxLayout();
  auto * title = new QLabel("Mission Control");
  QFont title_font = title->font();
  title_font.setBold(true);
  title_font.setPointSize(title_font.pointSize() + 6);
  title->setFont(title_font);
  title_col->addWidget(title);
  auto * subtitle = new QLabel("Autonomous forest surveys for reliable carbon estimation");
  subtitle->setStyleSheet("color: #6b7280;");
  title_col->addWidget(subtitle);
  layout->addLayout(title_col);

  layout->addStretch();

  clock_label_ = new QLabel();
  clock_label_->setStyleSheet("color: #6b7280;");
  layout->addWidget(clock_label_);

  ready_badge_ = new QLabel("NOT READY");
  layout->addWidget(ready_badge_);

  auto * start_button = new QPushButton("\u25B6  Start Mission");
  start_button->setStyleSheet(
    "QPushButton { background: #16a34a; color: white; padding: 8px 14px; border-radius: 6px; }"
    "QPushButton:pressed { background: #15803d; }");
  connect(start_button, &QPushButton::clicked, this, &MainWindow::onStartMissionClicked);
  layout->addWidget(start_button);

  auto * estop_button = new QPushButton("Emergency Stop");
  estop_button->setStyleSheet(
    "QPushButton { background: #ef4444; color: white; padding: 8px 14px; border-radius: 6px; }"
    "QPushButton:pressed { background: #b91c1c; }");
  connect(estop_button, &QPushButton::clicked, this, &MainWindow::onEstopAllClicked);
  layout->addWidget(estop_button);

  auto * home_button = new QPushButton("\u2302  Return Home");
  home_button->setStyleSheet(
    "QPushButton { background: #e5e7eb; color: #1f2937; padding: 8px 14px; border-radius: 6px; }"
    "QPushButton:pressed { background: #d1d5db; }");
  connect(home_button, &QPushButton::clicked, this, &MainWindow::onReturnHomeAllClicked);
  layout->addWidget(home_button);

  return bar;
}

QWidget * MainWindow::buildMapCard()
{
  auto * card = makeCard();
  auto * layout = new QVBoxLayout(card);

  auto * header = new QHBoxLayout();
  header->addWidget(makeCardTitle("Map / Mission View"));
  header->addStretch();
  layout->addLayout(header);

  auto * view_toggle_row = new QHBoxLayout();
  const QStringList views = {"AGB Map", "Traversability", "Selected Trees", "Robot Paths"};
  for (int i = 0; i < views.size(); ++i) {
    auto * button = new QPushButton(views[i]);
    button->setCheckable(true);
    if (i == 0) {
      button->setChecked(true);
      button->setStyleSheet(
        "QPushButton { background: #16a34a; color: white; padding: 6px 10px; "
        "border-radius: 6px; }");
    } else {
      // This build always renders every map layer together rather than
      // switching between them - these remain visual placeholders.
      button->setEnabled(false);
      button->setToolTip("This build always shows all map layers together");
      button->setStyleSheet(
        "QPushButton { background: #f3f4f6; color: #9ca3af; padding: 6px 10px; "
        "border-radius: 6px; }");
    }
    view_toggle_row->addWidget(button);
  }
  view_toggle_row->addStretch();
  layout->addLayout(view_toggle_row);

  auto * map_row = new QHBoxLayout();
  map_widget_ = new MapWidget();
  map_widget_->setMinimumHeight(420);
  map_row->addWidget(map_widget_, /*stretch=*/1);

  auto * zoom_col = new QVBoxLayout();
  auto * zoom_in = new QPushButton("+");
  auto * zoom_out = new QPushButton("\u2212");
  auto * recenter = new QPushButton("\u25CE");
  for (auto * b : {zoom_in, zoom_out, recenter}) {
    b->setFixedSize(32, 32);
    b->setStyleSheet(
      "QPushButton { background: white; border: 1px solid #d1d5db; border-radius: 4px; }");
  }
  connect(zoom_in, &QPushButton::clicked, map_widget_, &MapWidget::zoomIn);
  connect(zoom_out, &QPushButton::clicked, map_widget_, &MapWidget::zoomOut);
  connect(recenter, &QPushButton::clicked, map_widget_, &MapWidget::resetView);
  zoom_col->addWidget(zoom_in);
  zoom_col->addWidget(zoom_out);
  zoom_col->addSpacing(6);
  zoom_col->addWidget(recenter);
  zoom_col->addStretch();
  map_row->addLayout(zoom_col);

  layout->addLayout(map_row);
  return card;
}

QWidget * MainWindow::buildMissionProgressCard()
{
  auto * card = makeCard();
  auto * layout = new QVBoxLayout(card);
  layout->addWidget(makeCardTitle("Mission Progress"));

  auto * bar_row = new QHBoxLayout();
  mission_progress_bar_ = new QProgressBar();
  mission_progress_bar_->setRange(0, 100);
  mission_progress_bar_->setTextVisible(false);
  mission_progress_bar_->setStyleSheet(
    "QProgressBar { background: #e5e7eb; border-radius: 5px; }"
    "QProgressBar::chunk { background: #16a34a; border-radius: 5px; }");
  bar_row->addWidget(mission_progress_bar_);
  mission_progress_percent_ = new QLabel("0%");
  bar_row->addWidget(mission_progress_percent_);
  layout->addLayout(bar_row);

  step_uav_survey_ = new QLabel();
  step_traversability_ = new QLabel();
  step_sampling_ = new QLabel();
  step_returning_ = new QLabel();
  step_complete_ = new QLabel();
  for (auto * step_label :
    {step_uav_survey_, step_traversability_, step_sampling_, step_returning_, step_complete_})
  {
    layout->addWidget(step_label);
  }

  layout->addStretch();
  return card;
}

QWidget * MainWindow::buildSelectedTreesCard()
{
  auto * card = makeCard();
  auto * layout = new QVBoxLayout(card);
  layout->addWidget(makeCardTitle("Selected Trees"));

  selected_trees_table_ = new QTableWidget(0, 4);
  selected_trees_table_->setHorizontalHeaderLabels({"ID", "Species", "DBH (cm)", "Status"});
  selected_trees_table_->horizontalHeader()->setStretchLastSection(true);
  selected_trees_table_->verticalHeader()->setVisible(false);
  selected_trees_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  selected_trees_table_->setSelectionMode(QAbstractItemView::NoSelection);
  selected_trees_table_->setStyleSheet("QTableWidget { border: none; }");
  layout->addWidget(selected_trees_table_);

  return card;
}

QWidget * MainWindow::buildAgbEstimateCard()
{
  auto * card = makeCard();
  auto * layout = new QVBoxLayout(card);
  layout->addWidget(makeCardTitle("AGB Estimate"));

  layout->addWidget(new QLabel("Current ROI Estimate"));

  auto * value_row = new QHBoxLayout();
  agb_estimate_value_ = new QLabel("-- t/ha");
  QFont value_font = agb_estimate_value_->font();
  value_font.setBold(true);
  value_font.setPointSize(value_font.pointSize() + 8);
  agb_estimate_value_->setFont(value_font);
  value_row->addWidget(agb_estimate_value_);
  value_row->addStretch();
  layout->addLayout(value_row);

  agb_estimate_delta_ = new QLabel("");
  layout->addWidget(agb_estimate_delta_);

  agb_estimate_subtext_ = new QLabel("Updated from 0 / 0 sampled trees");
  agb_estimate_subtext_->setStyleSheet("color: #6b7280; font-size: 11px;");
  layout->addWidget(agb_estimate_subtext_);

  layout->addStretch();
  auto * results_button = new QPushButton("View Full Results");
  results_button->setEnabled(false);
  results_button->setToolTip("Not implemented in this build");
  results_button->setStyleSheet(
    "QPushButton { background: #e5e7eb; color: #6b7280; padding: 8px; border-radius: 6px; }");
  layout->addWidget(results_button);

  return card;
}

QWidget * MainWindow::buildRobotColumn()
{
  auto * column_widget = new QWidget();
  auto * layout = new QVBoxLayout(column_widget);

  const QColor husky_color(0, 150, 200);
  const QColor parrot_color(230, 150, 0);

  for (const QString & ns : kRobotNamespaces) {
    RobotEntry entry;
    entry.name = ns;
    entry.is_uav = (ns == "parrot1");
    entry.display_name = entry.is_uav ? "Parrot UAV" : "Husky UGV";
    entry.color = entry.is_uav ? parrot_color : husky_color;
    entry.card = new RobotCardWidget(entry.display_name, entry.is_uav, entry.color);
    layout->addWidget(entry.card);
    map_widget_->setRobotColor(ns, entry.color);
    robots_.push_back(entry);
  }

  layout->addStretch();
  return column_widget;
}

void MainWindow::setupRosInterfaces()
{
  for (auto & robot : robots_) {
    const std::string ns = robot.name.toStdString();

    robot.cmd_vel_pub = node_->create_publisher<geometry_msgs::msg::Twist>(
      "/" + ns + "/cmd_vel", 10);
    robot.estop_pub = node_->create_publisher<std_msgs::msg::Bool>(
      "/" + ns + "/e_stop", 10);
    robot.return_home_pub = node_->create_publisher<std_msgs::msg::Bool>(
      "/" + ns + "/return_home", 10);

    QString name_copy = robot.name;
    bool is_uav = robot.is_uav;

    robot.odom_sub = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/" + ns + "/odom", 10,
      [this, name_copy, is_uav](const nav_msgs::msg::Odometry::SharedPtr msg) {
        const auto & p = msg->pose.pose.position;
        const auto & q = msg->pose.pose.orientation;
        double yaw = std::atan2(
          2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        map_widget_->updateRobotPose(name_copy, p.x, p.y, yaw);

        for (auto & r : robots_) {
          if (r.name != name_copy) {
            continue;
          }
          if (!r.connected) {
            r.connected = true;
            r.card->setOnline(true);
            QLabel * dot = is_uav ? uav_status_dot_ : ugv_status_dot_;
            dot->setStyleSheet("border-radius: 4px; background-color: #22c55e;");
            updateReadyBadge();
          }
          if (is_uav) {
            r.card->setAltitude(static_cast<float>(p.z));
          } else {
            r.card->setPosition(p.x, p.y);
          }
          double vx = msg->twist.twist.linear.x;
          double vy = msg->twist.twist.linear.y;
          r.card->setSpeed(static_cast<float>(std::hypot(vx, vy)));
          break;
        }
      });

    robot.battery_sub = node_->create_subscription<std_msgs::msg::Float32>(
      "/" + ns + "/battery_percent", 10,
      [this, name_copy](const std_msgs::msg::Float32::SharedPtr msg) {
        for (auto & r : robots_) {
          if (r.name == name_copy) {
            r.card->setBatteryPercent(msg->data);
            break;
          }
        }
      });

    robot.mode_sub = node_->create_subscription<std_msgs::msg::String>(
      "/" + ns + "/mode", 10,
      [this, name_copy](const std_msgs::msg::String::SharedPtr msg) {
        for (auto & r : robots_) {
          if (r.name == name_copy) {
            r.card->setMode(QString::fromStdString(msg->data));
            break;
          }
        }
      });

    robot.camera_sub = node_->create_subscription<sensor_msgs::msg::Image>(
      "/" + ns + "/camera/image_raw", 5,
      [this, name_copy](const sensor_msgs::msg::Image::SharedPtr msg) {
        QImage image = imageMsgToQImage(*msg);
        if (image.isNull()) {
          return;
        }
        for (auto & r : robots_) {
          if (r.name == name_copy) {
            r.card->setCameraImage(image);
            break;
          }
        }
      });
  }

  agb_map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/agb_map", 1,
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      map_widget_->setAgbMap(
        msg->info.width, msg->info.height, msg->info.resolution,
        msg->info.origin.position.x, msg->info.origin.position.y, msg->data);
    });

  other_trees_sub_ = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
    "/tree_detections", 10,
    [this](const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
      std::vector<QPointF> positions;
      positions.reserve(msg->markers.size());
      for (const auto & marker : msg->markers) {
        positions.emplace_back(marker.pose.position.x, marker.pose.position.y);
      }
      map_widget_->setOtherTrees(positions);
    });

  selected_trees_sub_ = node_->create_subscription<rs1_mission_interfaces::msg::TreeSampleArray>(
    "/selected_trees", 10,
    [this](const rs1_mission_interfaces::msg::TreeSampleArray::SharedPtr msg) {
      selected_trees_table_->setRowCount(static_cast<int>(msg->trees.size()));
      std::vector<QPointF> positions;
      positions.reserve(msg->trees.size());
      for (size_t i = 0; i < msg->trees.size(); ++i) {
        const auto & tree = msg->trees[i];
        bool sampled = (tree.status == rs1_mission_interfaces::msg::TreeSample::STATUS_SAMPLED);
        int row = static_cast<int>(i);

        selected_trees_table_->setItem(
          row, 0, new QTableWidgetItem(QString::fromStdString(tree.id)));
        selected_trees_table_->setItem(
          row, 1, new QTableWidgetItem(QString::fromStdString(tree.species)));
        QString dbh_text = sampled ? QString::number(tree.dbh_cm, 'f', 1) : QString("\u2013");
        selected_trees_table_->setItem(row, 2, new QTableWidgetItem(dbh_text));
        auto * status_item = new QTableWidgetItem(sampled ? "Sampled" : "Pending");
        status_item->setForeground(
          sampled ? QBrush(QColor("#15803d")) : QBrush(QColor("#92400e")));
        selected_trees_table_->setItem(row, 3, status_item);

        positions.emplace_back(tree.position.x, tree.position.y);
      }
      map_widget_->setSelectedTrees(positions);
    });

  roi_boundary_sub_ = node_->create_subscription<geometry_msgs::msg::PolygonStamped>(
    "/roi_boundary", 1,
    [this](const geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
      std::vector<QPointF> points;
      points.reserve(msg->polygon.points.size());
      for (const auto & pt : msg->polygon.points) {
        points.emplace_back(pt.x, pt.y);
      }
      map_widget_->setRoiBoundary(points);
    });

  comms_ok_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/system/comms_ok", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      comms_ok_ = msg->data;
      comms_status_dot_->setStyleSheet(
        QString("border-radius: 4px; background-color: %1;")
        .arg(comms_ok_ ? "#22c55e" : "#ef4444"));
      updateReadyBadge();
    });

  mission_stage_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/mission/stage", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      current_stage_ = QString::fromStdString(msg->data);
      updateMissionProgressUi();
    });

  mission_progress_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
    "/mission/progress", 10,
    [this](const std_msgs::msg::Float32::SharedPtr msg) {
      mission_progress_percent_value_ = msg->data;
      updateMissionProgressUi();
    });

  trees_sampled_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
    "/mission/trees_sampled", 10,
    [this](const std_msgs::msg::Int32::SharedPtr msg) {
      trees_sampled_ = msg->data;
      updateMissionProgressUi();
    });

  trees_total_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
    "/mission/trees_total", 10,
    [this](const std_msgs::msg::Int32::SharedPtr msg) {
      trees_total_ = msg->data;
      updateMissionProgressUi();
    });

  agb_current_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
    "/agb_estimate/current_t_per_ha", 10,
    [this](const std_msgs::msg::Float32::SharedPtr msg) {
      agb_estimate_value_->setText(QString("%1 t/ha").arg(msg->data, 0, 'f', 1));
      agb_estimate_subtext_->setText(
        QString("Updated from %1 / %2 sampled trees").arg(trees_sampled_).arg(trees_total_));
    });

  agb_delta_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
    "/agb_estimate/delta_percent", 10,
    [this](const std_msgs::msg::Float32::SharedPtr msg) {
      QString sign = msg->data >= 0 ? "+" : "";
      agb_estimate_delta_->setText(
        QString("%1%2%% vs initial map").arg(sign).arg(msg->data, 0, 'f', 0));
      agb_estimate_delta_->setStyleSheet(
        QString("color: %1; font-weight: bold;").arg(msg->data >= 0 ? "#16a34a" : "#dc2626"));
    });
}

void MainWindow::updateReadyBadge()
{
  bool all_connected = std::all_of(
    robots_.begin(), robots_.end(), [](const RobotEntry & r) {return r.connected;});
  bool ready = comms_ok_ && all_connected;

  ready_badge_->setText(ready ? "READY" : "NOT READY");
  ready_badge_->setStyleSheet(
    ready
    ? "background: #dcfce7; color: #15803d; padding: 4px 10px; border-radius: 10px; "
      "font-weight: bold; font-size: 11px;"
    : "background: #fee2e2; color: #b91c1c; padding: 4px 10px; border-radius: 10px; "
      "font-weight: bold; font-size: 11px;");
}

void MainWindow::updateMissionProgressUi()
{
  int percent = static_cast<int>(std::clamp(mission_progress_percent_value_, 0.0f, 100.0f));
  mission_progress_bar_->setValue(percent);
  mission_progress_percent_->setText(QString("%1%").arg(percent));

  static const std::vector<QString> kStages = {
    "uav_survey", "traversability", "sampling", "returning", "complete"};
  int current_index = 0;
  for (size_t i = 0; i < kStages.size(); ++i) {
    if (kStages[i] == current_stage_) {
      current_index = static_cast<int>(i);
      break;
    }
  }

  auto formatStep = [](QLabel * label, int step_index, int current_index, const QString & text) {
    QString icon;
    QString color;
    if (step_index < current_index) {
      icon = "\u2705";
      color = "#111827";
    } else if (step_index == current_index) {
      icon = "\U0001F7E2";
      color = "#111827";
    } else {
      icon = "\u26AA";
      color = "#9ca3af";
    }
    label->setText(QString("%1  %2").arg(icon, text));
    label->setStyleSheet(QString("color: %1;").arg(color));
  };

  QString sampling_text = QString("Sampling in progress (%1 / %2 trees)")
    .arg(trees_sampled_).arg(trees_total_);

  formatStep(step_uav_survey_, 0, current_index, "UAV survey completed");
  formatStep(step_traversability_, 1, current_index, "Traversability map generated");
  formatStep(step_sampling_, 2, current_index, sampling_text);
  formatStep(step_returning_, 3, current_index, "Returning to base");
  formatStep(step_complete_, 4, current_index, "Mission complete");
}

void MainWindow::onEstopAllClicked()
{
  for (auto & robot : robots_) {
    std_msgs::msg::Bool estop_msg;
    estop_msg.data = true;
    robot.estop_pub->publish(estop_msg);

    // Belt-and-braces: also push a zero Twist directly, in case a
    // robot's driver doesn't act on /e_stop by itself.
    geometry_msgs::msg::Twist zero_twist;
    robot.cmd_vel_pub->publish(zero_twist);
  }
}

void MainWindow::onReturnHomeAllClicked()
{
  for (auto & robot : robots_) {
    std_msgs::msg::Bool msg;
    msg.data = true;
    robot.return_home_pub->publish(msg);
  }
}

void MainWindow::onStartMissionClicked()
{
  // Placeholder - wire this to a std_srvs/Trigger or action call once
  // a mission planner / state machine exists on the other end.
  QMessageBox::information(this, "Start Mission", "Mission start requested.");
}

void MainWindow::spinRos()
{
  rclcpp::spin_some(node_);
}

void MainWindow::updateClock()
{
  clock_label_->setText(QDateTime::currentDateTime().toString("d MMM yyyy   HH:mm:ss"));
}

}  // namespace rs1_mission_control_gui
