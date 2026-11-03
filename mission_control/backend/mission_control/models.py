"""Validated, serializable public plan and image calibration."""
from typing import Literal
from uuid import UUID

from pydantic import BaseModel, ConfigDict, Field, model_validator


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)


class XY(StrictModel):
    x: float
    y: float


class SafetyStopRequest(StrictModel):
    robot: Literal["husky1", "parrot1", "all"]
    stopped: bool = Field(strict=True)
    expected_instance_id: str = ""
    expected_revision: int = Field(default=0, strict=True, ge=0, le=2**64 - 1)


class TeleopCommand(StrictModel):
    enabled: bool = Field(default=False, strict=True)
    x: float = Field(default=0, ge=-1, le=1)
    y: float = Field(default=0, ge=-1, le=1)
    z: float = Field(default=0, ge=-1, le=1)
    yaw: float = Field(default=0, ge=-1, le=1)


class Calibration(StrictModel):
    # Continuous image EDGES, not pixel sample centres. Axis aligned for v1.
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    @model_validator(mode="after")
    def positive_extent(self):
        if self.x_max <= self.x_min or self.y_max <= self.y_min:
            raise ValueError("Calibration maximum must exceed minimum on each axis")
        return self

    def to_world(self, u: float, v: float) -> XY:
        return XY(x=self.x_min + u * (self.x_max - self.x_min),
                  y=self.y_min + v * (self.y_max - self.y_min))

    def to_image(self, point: XY) -> tuple[float, float]:
        return ((point.x - self.x_min) / (self.x_max - self.x_min),
                (point.y - self.y_min) / (self.y_max - self.y_min))

    def contains(self, point: XY, margin: float = 0) -> bool:
        return (self.x_min + margin <= point.x <= self.x_max - margin and
                self.y_min + margin <= point.y <= self.y_max - margin)


class Plan(StrictModel):
    session_id: UUID
    world_id: Literal["v2_forest"] = "v2_forest"
    image_id: str = Field(default="forest-overhead", pattern=r"^[a-zA-Z0-9-]{1,64}$")
    calibration_id: str = Field(default="forest-edges-v1", pattern=r"^[a-zA-Z0-9-]{1,64}$")
    start: XY
    roi: XY
    # Second (opposite) corner of the axis-aligned ROI box. When omitted the
    # coordinator falls back to a square around roi.
    roi_second: XY | None = None
    station_offset: XY = Field(default_factory=lambda: XY(x=0, y=5))
    parrot_clearance: float = Field(default=4.0, ge=2, le=15)


class UploadCalibration(StrictModel):
    world_id: Literal["v2_forest"]
    calibration: Calibration


class ExternalStart(StrictModel):
    # Omit ROI to use the coordinator's existing launch/parameter configuration.
    roi: XY | None = None
    # Opposite corner of the ROI box; roi is the first-placed (entry) corner.
    roi_second: XY | None = None

    @model_validator(mode="after")
    def roi_pair(self):
        if self.roi is None and self.roi_second is not None:
            raise ValueError("roi_second requires roi")
        if self.roi is not None and self.roi_second is not None and (
            self.roi.x == self.roi_second.x or self.roi.y == self.roi_second.y
        ):
            raise ValueError("ROI box must have a positive width and height")
        return self


def roi_bounds(roi: XY, roi_second: XY | None, default_half: float = 5.0) -> tuple[XY, XY]:
    """Return the (min, max) corners of the axis-aligned ROI box."""
    if roi_second is None:
        return (XY(x=roi.x - default_half, y=roi.y - default_half),
                XY(x=roi.x + default_half, y=roi.y + default_half))
    return (XY(x=min(roi.x, roi_second.x), y=min(roi.y, roi_second.y)),
            XY(x=max(roi.x, roi_second.x), y=max(roi.y, roi_second.y)))


class SpawnPlan(StrictModel):
    session_id: UUID
    world_id: Literal["v2_forest"] = "v2_forest"
    image_id: str = Field(default="forest-overhead", pattern=r"^[a-zA-Z0-9-]{1,64}$")
    calibration_id: str = Field(default="forest-edges-v1", pattern=r"^[a-zA-Z0-9-]{1,64}$")
    start: XY
    parrot_clearance: float = Field(default=4.0, ge=2, le=15)


class RoiPreview(StrictModel):
    """Placement preview for an ROI entry point and its optional box corner."""
    x: float
    y: float
    second: XY | None = None
