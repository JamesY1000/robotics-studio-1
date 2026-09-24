#!/usr/bin/env python3
"""Self-contained ``BaseNode`` / ``BaseParams`` base classes.

The sample generator nodes were originally written against an external common
package (not installed in every environment, e.g. the UTS robotics-studio-1
workspace). This module provides the same API surface using only plain
``rclpy`` so the nodes run anywhere.

Usage::

    from sample_gen.base import BaseNode, BaseParams

    @dataclass
    class MyParams(BaseParams):
        some_param: str = "default"

    class MyNode(BaseNode):
        def __init__(self):
            self._params = MyParams()
            super().__init__("my_node")
            # self._params.some_param is now loaded from ROS parameters
"""

import dataclasses
from typing import Any

from rclpy.node import Node


class BaseParams:
    """Marker base class. Concrete params are dataclasses whose fields map 1:1
    to ROS parameters under the node's ``ros__parameters`` namespace."""


class BaseNode(Node):
    """rclpy ``Node`` that loads its dataclass parameters from the ROS
    parameter server."""

    def __init__(self, node_name: str, **kwargs: Any) -> None:
        super().__init__(node_name, **kwargs)

        # Make sure use_sim_time is declared even when the launch file does not
        # pass it explicitly (rcl uses this to switch the node clock to /clock).
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", False)

        params = getattr(self, "_params", None)
        if params is None:
            return

        for field in dataclasses.fields(params):
            if not self.has_parameter(field.name):
                self.declare_parameter(field.name, getattr(params, field.name))
            setattr(params, field.name, self.get_parameter(field.name).value)
