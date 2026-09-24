#!/usr/bin/env python3
"""Service/topic names shared by the sample generator nodes.

The original code obtained these strings from the external ``brain3_common``
package (``RobotServices`` / ``RobotTopics``), which is not installed in
every environment. They are inlined here so the package is self-contained.

NOTE: If you later re-add ``brain3_common``, make sure these values match the
enum values defined there (clients and servers must agree on the names).
"""

# Services exposed by the collectors and used by model_importer / sample_generator
SERVICE_COLLECT_SAMPLE = "collect_sample"
SERVICE_UNREGISTER_GT_SUBS = "unregister_gt_subs"
SERVICE_UPDATE_GT_SUBS = "update_gt_subs"
SERVICE_CHECK_MODELS_LOADED = "check_models_loaded"
SERVICE_GET_CURRENT_SAMPLE_ID = "get_current_sample_id"
SERVICE_GET_GT_DATA = "get_gt_data"

# Visualization topics used by the (lidar) sample_collector / data_preprocessor
TOPIC_SAMPLE_COLLECTOR_ORIGINAL_GT = "/sample_collector/original_gt"
TOPIC_SAMPLE_COLLECTOR_CLEANED_GT = "/sample_collector/cleaned_gt"
TOPIC_SAMPLE_COLLECTOR_ORIGINAL_POINTS = "/sample_collector/original_points"
TOPIC_SAMPLE_COLLECTOR_CLEANED_POINTS = "/sample_collector/cleaned_points"

# Odometry topic (robot world broadcaster)
TOPIC_ODOMETRY = "/robot/odom"

# Camera YOLO collector visualization topics
TOPIC_CAMERA_ORIGINAL = "/camera_collector/original"
TOPIC_CAMERA_LABELLED = "/camera_collector/labelled"


class ServiceValue(str):
    """A ``str`` subclass that also exposes ``.value``.

    Some call sites use ``RobotServices.<NAME>.value`` while others pass
    ``RobotServices.<NAME>`` directly to ``create_client`` / ``create_service``
    (which expect a plain ``str``). Subclassing ``str`` makes both forms work
    without ``brain3_common``.
    """

    def __new__(cls, value: str) -> "ServiceValue":
        return super().__new__(cls, value)

    @property
    def value(self) -> str:
        return str(self)


def make_services_namespace():
    """Build a ``brain3_common.RobotServices``-like namespace from the local
    constants, so nodes that use ``RobotServices.<NAME>.value`` run without
    the external package."""
    import types

    return types.SimpleNamespace(
        COLLECT_SAMPLE=ServiceValue(SERVICE_COLLECT_SAMPLE),
        UNREGISTER_GT_SUBS=ServiceValue(SERVICE_UNREGISTER_GT_SUBS),
        UPDATE_GT_SUBS=ServiceValue(SERVICE_UPDATE_GT_SUBS),
        CHECK_MODELS_LOADED=ServiceValue(SERVICE_CHECK_MODELS_LOADED),
        GET_CURRENT_SAMPLE_ID=ServiceValue(SERVICE_GET_CURRENT_SAMPLE_ID),
        GET_GT_DATA=ServiceValue(SERVICE_GET_GT_DATA),
    )


def make_topics_namespace():
    """Build a ``brain3_common.RobotTopics``-like namespace from the local
    constants (used for the lidar pipeline's RViz visualization topics)."""
    import types

    return types.SimpleNamespace(
        SAMPLE_COLLECTOR_ORIGINAL_GT=ServiceValue(TOPIC_SAMPLE_COLLECTOR_ORIGINAL_GT),
        SAMPLE_COLLECTOR_CLEANED_GT=ServiceValue(TOPIC_SAMPLE_COLLECTOR_CLEANED_GT),
        SAMPLE_COLLECTOR_ORIGINAL_POINTS=ServiceValue(TOPIC_SAMPLE_COLLECTOR_ORIGINAL_POINTS),
        SAMPLE_COLLECTOR_CLEANED_POINTS=ServiceValue(TOPIC_SAMPLE_COLLECTOR_CLEANED_POINTS),
        ODOMETRY=ServiceValue(TOPIC_ODOMETRY),
    )
