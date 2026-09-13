# command_panel

A custom RViz2 panel plugin (E-stop,) built with `rviz_common`, `pluginlib`, and Qt5 Widgets.

## What it publishes

| Control                      | Topic                  | Message type             | Note                                                 |
|------------------------------|----------------------  |--------------------------|------------------------------------------------------|
| Forward/Reverse/Stop Buttons | `[robot_type]/cmd_vel` | `geometry_msgs/msg/Twist`| -                                                    |
| Robot Selection dropdown     | -                      | -                        | Changes topic names for buttons based on robot chosen|

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select command_panel
source install/setup.zsh
```

## Add it in Rviz

1. Menu: **Panels → Add New Panel**.
2. Select **command_panel → CommandPanel** and click OK.
3. The panel docks into the RViz window; drag it to any side or float it.
