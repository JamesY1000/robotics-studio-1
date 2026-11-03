#!/usr/bin/env python3
"""Service node: reads tree CSV, computes AGB (Chave et al. 2014), publishes TreeDBH per tree.

Call:  ros2 service call /calculate_carbon std_srvs/srv/Trigger
Watch: ros2 topic echo /tree_dbh
"""

import csv

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from rs1_interfaces.msg import TreeDBH


class CarbonCalculator(Node):
    def __init__(self):
        super().__init__("carbon_calculator")
        self.declare_parameters(
            "",
            [
                ("csv_path", ""),
                ("default_dbh_m", 1.0),
                ("chave_a", 0.0673),
                ("chave_b", 0.976),
                ("oak.wood_density", 0.71),
                ("oak.height_m", 12.0),
                ("pine.wood_density", 0.545),
                ("pine.height_m", 10.0),
            ],
        )
        # Depth covers a full CSV burst (184 trees) so late subscribers don't drop messages.
        self.pub = self.create_publisher(TreeDBH, "tree_dbh", 200)
        self.create_service(Trigger, "calculate_carbon", self.calculate)
        self.get_logger().info("Ready: call /calculate_carbon")

    def p(self, name):
        return self.get_parameter(name).value

    def calculate(self, request, response):
        try:
            with open(self.p("csv_path"), newline="") as f:
                rows = list(csv.DictReader(f))
        except OSError as e:
            response.success = False
            response.message = f"Cannot read CSV: {e}"
            return response

        a, b = self.p("chave_a"), self.p("chave_b")
        total_agb, n_done, n_default, skipped = 0.0, 0, 0, []

        for row in rows:
            tree_id = row["name"].strip()
            tree_type = row["tree_type"].strip().lower()
            if not self.has_parameter(f"{tree_type}.wood_density"):
                skipped.append(tree_id)
                continue

            # DBH from CSV if present and valid, else the default (flagged in the message).
            try:
                dbh_m = float(row.get("DBH") or "")
                is_default = dbh_m <= 0.0
            except ValueError:
                is_default = True
            if is_default:
                dbh_m = self.p("default_dbh_m")
                n_default += 1

            rho = self.p(f"{tree_type}.wood_density")
            height = self.p(f"{tree_type}.height_m")
            dbh_cm = dbh_m * 100.0  # Chave (2014) is fitted with D in cm
            agb_kg = a * (rho * dbh_cm**2 * height) ** b

            msg = TreeDBH()
            msg.tree_id = tree_id
            msg.tree_type = tree_type
            msg.dbh_m = dbh_m
            msg.dbh_is_default = is_default
            msg.agb_kg = agb_kg
            self.pub.publish(msg)

            total_agb += agb_kg
            n_done += 1

        response.success = n_done > 0
        response.message = (
            f"{n_done} trees processed, total AGB {total_agb:.1f} kg. "
            f"DBH missing -> default {self.p('default_dbh_m')} m used for {n_default} trees."
        )
        if skipped:
            response.message += (
                f" Skipped {len(skipped)} unknown-species trees: {skipped}"
            )
        if n_default:
            self.get_logger().warn(f"Default DBH used for {n_default}/{n_done} trees")
        self.get_logger().info(response.message)
        return response


def main():
    rclpy.init()
    node = CarbonCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
