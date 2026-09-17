# command_panel

A custom RViz2 panel plugin (E-stop,) built with `rqt_gui_cpp::Plugin`, `pluginlib`, and Qt5 Widgets.

## What the buttons do

| Control                      | Topic                  | Message type             | Note                                                 |
|------------------------------|----------------------  |--------------------------|------------------------------------------------------|
| Forward/Reverse/Stop Buttons | `[robot_type]/cmd_vel` | `geometry_msgs/msg/Twist`| Controls the respective robot                        |
| Robot Selection dropdown     | -                      | -                        | Changes topic names for buttons based on robot chosen|

## Build

```bash
cd ros2_ws
colcon build --packages-select rs1_ui
source install/setup.zsh
rqt
```

## Add it in RQT

1. Menu: **Plugins → Robot Tools → Command Panel (C++)**.
2. The panel docks into the RQT window;

## TODO

- Have GUI launch together instead of having to manually do it

| Features to add | Errors to Fix                             | Other stuff                                                  |
|-----------------|-------------------------------------------|--------------------------------------------------------------|
| Return2Home     | E-stop not resetting when changing robots | Have GUI launch together instead of having to manually do it |
| AGB Map         | E-stop needs to clear goals               |                                                              |
| Tree Markers    |                                           |                                                              |
