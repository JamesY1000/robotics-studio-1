import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan


class TreeObservation:
    def __init__(self, tree_type, x, y, dbh):
        self.tree_type = tree_type
        self.x = x
        self.y = y
        self.dbh = dbh


class DBHDetector(Node):
    def __init__(self):

        super().__init__("dbh_detector")

        self.scan_count = 0

        self.create_subscription(LaserScan, "/husky1/scan", self.scan_callback, 10)

        self.get_logger().info("DBH Detector started")

        self.known_trees = [
            (5.0, 0.5),
            (5.0, -0.5),
        ]  # Debug code to be replaced with TreeLocation data

    def scan_callback(self, msg):

        # Process every 5th scan
        self.scan_count += 1

        if self.scan_count % 5 != 0:
            return

        # Convert scan to XY points
        points = self.scan_to_points(msg)

        self.get_logger().info(f"Converted {len(points)} valid scan points")

        # Debug: distance distribution
        distances = [math.sqrt(x * x + y * y) for x, y in points]

        self.get_logger().info(f"Min distance = {min(distances):.2f} m")

        self.get_logger().info(f"Max distance = {max(distances):.2f} m")

        # Keep points between 1m and 5m
        nearby_points = [
            p for p in points if 1.0 < math.sqrt(p[0] ** 2 + p[1] ** 2) < 5.0
        ]

        self.get_logger().info(f"Nearby points = {len(nearby_points)}")

        # Generate clusters
        clusters = self.cluster_points(nearby_points)

        self.get_logger().info(f"Found {len(clusters)} total clusters")

        # Print cluster statistics
        for i, cluster in enumerate(clusters):
            if len(cluster) < 5:
                continue

            xs = [p[0] for p in cluster]
            ys = [p[1] for p in cluster]

            width_x = max(xs) - min(xs)
            width_y = max(ys) - min(ys)

            ratio = max(width_x, width_y) / max(min(width_x, width_y), 0.001)

            self.get_logger().info(
                f"Cluster {i}: "
                f"Points={len(cluster)} "
                f"Wx={width_x:.2f} "
                f"Wy={width_y:.2f} "
                f"Ratio={ratio:.2f}"
            )

        # Tree candidate filtering
        tree_clusters = [
            cluster
            for cluster in clusters
            if (8 < len(cluster) < 60 and self.is_tree_like(cluster))
        ]

        self.get_logger().info(f"Tree candidates = {len(tree_clusters)}")

        # Process candidate trees
        # TODO: Match detected cluster centroid against TreeLocation.position provided by Parrot
        for i, cluster in enumerate(tree_clusters):
            radius = self.fit_circle(cluster)
            dbh = 2 * radius

            _, width_x, width_y = self.estimate_diameter(cluster)

            ratio = max(width_x, width_y) / max(min(width_x, width_y), 0.001)
            distance = min(math.sqrt(x * x + y * y) for x, y in cluster)

            cx, cy = self.calculate_centroid(cluster)
            tree_id, error = self.validate_tree_location(cx, cy)

            observation = TreeObservation(
                tree_type="unknown",
                x=cx,
                y=cy,
                dbh=dbh,
            )

            self.get_logger().info(f"Observation = {observation}")

            self.get_logger().info(f"Closest known tree = {tree_id}")
            self.get_logger().info(f"Position error = {error:.2f} m")

            self.get_logger().info(f"Tree {i}: Estimated DBH = {dbh:.2f} m")

            self.get_logger().info(f"Tree {i}: Width X = {width_x:.2f} m")

            self.get_logger().info(f"Tree {i}: Width Y = {width_y:.2f} m")

            self.get_logger().info(f"Tree {i}: Shape Ratio = {ratio:.2f}")

            self.get_logger().info(f"Tree {i}: Distance = {distance:.2f} m")

            self.get_logger().info(f"Tree {i}: Centre = ({cx:.2f}, {cy:.2f})")

    def scan_to_points(self, scan_msg):

        points = []

        angle = scan_msg.angle_min

        for r in scan_msg.ranges:
            if math.isinf(r) or math.isnan(r):
                angle += scan_msg.angle_increment
                continue

            x = r * math.cos(angle)
            y = r * math.sin(angle)

            points.append((x, y))

            angle += scan_msg.angle_increment

        return points

    def calculate_centroid(self, cluster):
        cx = sum(x for x, y in cluster) / len(cluster)
        cy = sum(y for x, y in cluster) / len(cluster)
        return cx, cy

    def match_tree_location(self, cx, cy, tree_x, tree_y):
        error = math.sqrt((cx - tree_x) ** 2 + (cy - tree_y) ** 2)
        return error

    def fit_circle(self, cluster):
        xs = [p[0] for p in cluster]
        ys = [p[1] for p in cluster]

        width_x = max(xs) - min(xs)
        width_y = max(ys) - min(ys)

        radius = max(width_x, width_y) / 2.0
        return radius

    def estimate_agb(self, tree_type, dbh):
        if tree_type == "eucalyptus":
            return 0.0

        elif tree_type == "pine":
            return 0.0

        return None

    def cluster_points(self, points, distance_threshold=0.20):

        if not points:
            return []

        clusters = []

        current_cluster = [points[0]]

        for i in range(1, len(points)):
            x1, y1 = points[i - 1]
            x2, y2 = points[i]

            distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

            if i < 20:
                self.get_logger().info(f"Point spacing = {distance:.3f}")

            if distance < distance_threshold:
                current_cluster.append(points[i])

            else:
                clusters.append(current_cluster)

                current_cluster = [points[i]]

        clusters.append(current_cluster)

        self.get_logger().info(f"cluster_points(): generated {len(clusters)} clusters")

        return clusters

    def is_tree_like(self, cluster):

        xs = [p[0] for p in cluster]
        ys = [p[1] for p in cluster]

        width_x = max(xs) - min(xs)
        width_y = max(ys) - min(ys)

        if width_x < 0.05 or width_y < 0.05:
            return False

        ratio = max(width_x, width_y) / min(width_x, width_y)

        return ratio < 8.0

    def estimate_diameter(self, cluster):

        xs = [p[0] for p in cluster]
        ys = [p[1] for p in cluster]

        width_x = max(xs) - min(xs)
        width_y = max(ys) - min(ys)

        diameter = max(width_x, width_y)

        return diameter, width_x, width_y

    def validate_tree_location(
        self, detected_x, detected_y
    ):  # Debug code to be removed after getting TreeLocation data
        nearest_tree = None
        nearest_error = float("inf")

        for i, (tx, ty) in enumerate(self.known_trees):
            error = math.sqrt((detected_x - tx) ** 2 + (detected_y - ty) ** 2)

            if error < nearest_error:
                nearest_error = error
                nearest_tree = i

        return nearest_tree, nearest_error


def main():

    rclpy.init()

    node = DBHDetector()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
