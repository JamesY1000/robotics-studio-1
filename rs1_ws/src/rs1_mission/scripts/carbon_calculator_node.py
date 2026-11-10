#!/usr/bin/env python3
"""Estimate sampled AGB from confirmed Husky DBHs in the current tree survey"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from rs1_interfaces.msg import BiomassSummary, TreeDBH, TreeSurvey


def estimate_survey(survey, models, a, b):
    summary = BiomassSummary(header=survey.tree_map.header, mission_id=survey.mission_id,
                             tree_count=len(survey.tree_map.trees))
    readings = []
    seen = set()
    skipped = []
    try:
        if not summary.header.frame_id:
            raise ValueError('Missing tree map frame')
        if any(not math.isfinite(v) or v <= 0 for v in (a, b)):
            raise ValueError('Invalid AGB model coefficients')
        for measurement in survey.measurements:
            index, dbh = measurement.tree_index, measurement.dbh_m
            if index >= summary.tree_count or index in seen or not math.isfinite(dbh) or dbh <= 0:
                raise ValueError('Invalid or duplicate confirmed DBH')
            seen.add(index)
            species = survey.tree_map.trees[index].tree_type.strip().lower().removesuffix('_tree')
            if species not in models:
                skipped.append(str(index))
                continue
            density, height = models[species]
            if any(not math.isfinite(v) or v <= 0 for v in (density, height)):
                raise ValueError('Invalid height or wood density for '+species)
            # Our DBH is in metres, while the existing Chave model needs centimetres
            agb = a * (density * (dbh * 100.0)**2 * height)**b
            if not math.isfinite(agb) or agb <= 0:
                raise ValueError('AGB estimate is outside the numeric range')
            readings.append(TreeDBH(tree_id=str(index), tree_type=species, dbh_m=dbh,
                                    dbh_is_default=False, agb_kg=agb))
        summary.total_agb_kg = math.fsum(reading.agb_kg for reading in readings)
        summary.measured_count = len(seen)
        summary.estimated_count = len(readings)
        summary.valid = True
        summary.message = 'Uses measured DBH with configured species heights and wood densities'
        if skipped:
            summary.message += '; no species model for tree indices '+', '.join(skipped)
    except (ValueError, OverflowError) as error:
        # Don't retain a plausible-looking total after receiving an invalid snapshot
        summary.message = str(error)
        readings = []
    return summary, readings


class CarbonCalculator(Node):
    def __init__(self):
        super().__init__('carbon_calculator')
        self.declare_parameters('', [
            ('chave_a', 0.0673), ('chave_b', 0.976),
            ('oak.wood_density', 0.71), ('oak.height_m', 12.0),
            ('pine.wood_density', 0.545), ('pine.height_m', 10.0),
        ])
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.summary_pub = self.create_publisher(BiomassSummary, '/carbon/summary', latched)
        self.total_pub = self.create_publisher(Float64, '/carbon/total_agb_kg', latched)
        self.tree_pub = self.create_publisher(TreeDBH, '/tree_dbh', 200)
        self.survey = None
        self.create_subscription(TreeSurvey, '/mission/tree_survey', self.on_survey, latched)
        self.create_service(Trigger, '/calculate_carbon', self.calculate)

    def publish_estimate(self):
        models = {species: (self.get_parameter(species+'.wood_density').value,
                            self.get_parameter(species+'.height_m').value)
                  for species in ('oak', 'pine')}
        summary, readings = estimate_survey(self.survey, models,
            self.get_parameter('chave_a').value, self.get_parameter('chave_b').value)
        self.summary_pub.publish(summary)
        # Keep Noah's numeric topic too, but the dashboard uses the summary's map identity
        self.total_pub.publish(Float64(data=summary.total_agb_kg if summary.valid else math.nan))
        for reading in readings:
            self.tree_pub.publish(reading)
        return summary

    def on_survey(self, survey):
        # Recompute from the complete snapshot so pauses and resumes cannot double-count a tree
        # Pending large-tree views are deliberately excluded until their DBH is confirmed
        self.survey = survey
        self.publish_estimate()

    def calculate(self, request, response):
        if self.survey is None:
            response.message = 'Waiting for the current mission tree survey'
            return response
        summary = self.publish_estimate()
        response.success = summary.valid and summary.estimated_count > 0
        response.message = (f'{summary.estimated_count}/{summary.tree_count} trees estimated, '
                            f'sampled AGB {summary.total_agb_kg:.1f} kg; {summary.message}')
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


if __name__ == '__main__':
    main()
