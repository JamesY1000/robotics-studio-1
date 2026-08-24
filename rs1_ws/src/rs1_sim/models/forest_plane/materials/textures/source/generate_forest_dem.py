#!/usr/bin/env python3
"""
Generate a deterministic 200 m x 200 m forest DEM from an APPROVED water mask.

Inputs (same directory as this script)
--------------------------------------
forest_overhead.png   1024 x 1024 RGB albedo. Used ONLY for alignment overlay.
water_mask.png        1024 x 1024 binary mask. White = approved water, black = land.

Outputs
-------
forest_terrain_dem.tif
forest_heightmap_16.png
terrain_height_preview.png
terrain_albedo_overlay.png
terrain_slope_preview.png
terrain_metadata.json

Important
---------
The albedo's brightness, trees, bushes, shadows, rocks, etc. are NEVER used
to determine elevation. Terrain geometry depends only on:
  * the approved water mask / shoreline geometry
  * deterministic broad procedural hills
  * deterministic low-frequency variation
"""

from pathlib import Path
import json
import math

import cv2
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt, gaussian_filter
import rasterio
from rasterio.transform import Affine


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
BASE = Path(__file__).resolve().parent

ALBEDO_PATH = BASE / "forest_overhead.png"
WATER_MASK_PATH = BASE / "water_mask.png"

DEM_PATH = BASE / "forest_terrain_dem.tif"
HEIGHTMAP_PATH = BASE / "forest_heightmap_16.png"
HEIGHT_PREVIEW_PATH = BASE / "terrain_height_preview.png"
OVERLAY_PATH = BASE / "terrain_albedo_overlay.png"
SLOPE_PREVIEW_PATH = BASE / "terrain_slope_preview.png"
METADATA_PATH = BASE / "terrain_metadata.json"

SEED = 20260824

PHYSICAL_SIZE_M = 200.0
GRID_SIZE = 257
PIXEL_SPACING_M = PHYSICAL_SIZE_M / (GRID_SIZE - 1)  # 0.78125 m

TARGET_MAX_ELEV_M = 7.90
HEIGHTMAP_FULL_SCALE_M = 8.0

BANK_WIDTH_M = 15.0
INLAND_BASELINE_M = 2.55
SHORE_LAND_ELEV_M = 0.08

MAX_SLOPE_DEG = 20.0
SLOPE_LIMIT_TRIGGER_DEG = 19.8

# The source is synthetic / non-geographic. Forest3D discards absolute
# position, but a conventional projected metre-based CRS is still useful
# for GeoTIFF compatibility.
CRS = "EPSG:32631"  # WGS 84 / UTM zone 31N
UL_SAMPLE_EASTING_M = 500000.0
UL_SAMPLE_NORTHING_M = 1000200.0


# ---------------------------------------------------------------------------
# Utilities
# ---------------------------------------------------------------------------
def smoothstep01(t):
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def calculate_slope_deg(z, spacing_m):
    """Slope magnitude in degrees from first derivatives."""
    dz_dy, dz_dx = np.gradient(z.astype(np.float64), spacing_m, spacing_m)
    rise_run = np.hypot(dz_dx, dz_dy)
    return np.degrees(np.arctan(rise_run)).astype(np.float32)


def limit_slopes(z, water, spacing_m, trigger_deg=SLOPE_LIMIT_TRIGGER_DEG,
                 max_iterations=120):
    """
    Deterministic slope limiter.

    Only if a local slope exceeds trigger_deg, blend that neighborhood toward
    a Gaussian-smoothed surface. This avoids gradient clipping discontinuities.
    Water is kept at z=0. The current generated terrain normally already meets
    the limit, but this function guarantees a controlled fallback.
    """
    z = z.astype(np.float32, copy=True)

    for iteration in range(max_iterations):
        slope = calculate_slope_deg(z, spacing_m)
        bad = slope > trigger_deg
        if not np.any(bad):
            return z, iteration

        # Spread the correction smoothly into surrounding cells so first
        # derivatives remain continuous.
        blend = gaussian_filter(bad.astype(np.float32), sigma=2.0)
        blend = np.clip(blend * 0.35, 0.0, 0.35)

        smoothed = gaussian_filter(z, sigma=1.0)
        z = ((1.0 - blend) * z + blend * smoothed).astype(np.float32)
        z[water] = 0.0

    return z, max_iterations


def hillshade(z, spacing_m, azimuth_deg=315.0, altitude_deg=45.0):
    """Simple hillshade, returned as uint8."""
    dz_dy, dz_dx = np.gradient(z.astype(np.float64), spacing_m, spacing_m)
    slope = np.arctan(np.hypot(dz_dx, dz_dy))
    aspect = np.arctan2(-dz_dx, dz_dy)

    az = math.radians(azimuth_deg)
    alt = math.radians(altitude_deg)

    shaded = (
        np.sin(alt) * np.cos(slope)
        + np.cos(alt) * np.sin(slope) * np.cos(az - aspect)
    )
    shaded = np.clip((shaded + 1.0) * 0.5, 0.0, 1.0)
    return np.round(shaded * 255.0).astype(np.uint8)


def terrain_colorize(z, shade):
    """Natural elevation colour ramp multiplied by hillshade."""
    t = np.clip(z / HEIGHTMAP_FULL_SCALE_M, 0.0, 1.0)

    # Piecewise natural terrain ramp:
    # low = deep green/brown, mid = olive/tan, high = pale stone.
    stops = np.array([0.0, 0.30, 0.58, 0.80, 1.0], dtype=np.float32)
    colors = np.array([
        [52, 76, 46],
        [91, 105, 62],
        [137, 122, 76],
        [165, 145, 105],
        [210, 205, 184],
    ], dtype=np.float32)

    rgb = np.empty((*z.shape, 3), dtype=np.float32)
    for c in range(3):
        rgb[..., c] = np.interp(t, stops, colors[:, c])

    # Keep shading moderate so elevation colour remains readable.
    light = 0.62 + 0.50 * (shade.astype(np.float32) / 255.0)
    rgb *= light[..., None]
    return np.clip(rgb, 0, 255).astype(np.uint8)


def resize_float(arr, width, height, interpolation=cv2.INTER_CUBIC):
    return cv2.resize(arr.astype(np.float32), (width, height),
                      interpolation=interpolation)


# ---------------------------------------------------------------------------
# Load and validate inputs
# ---------------------------------------------------------------------------
if not ALBEDO_PATH.exists():
    raise FileNotFoundError(f"Missing {ALBEDO_PATH}")
if not WATER_MASK_PATH.exists():
    raise FileNotFoundError(f"Missing {WATER_MASK_PATH}")

albedo = np.asarray(Image.open(ALBEDO_PATH).convert("RGB"))
mask_hi = np.asarray(Image.open(WATER_MASK_PATH).convert("L"))

if albedo.shape != (1024, 1024, 3):
    raise ValueError(f"Albedo must be exactly 1024x1024 RGB; got {albedo.shape}")
if mask_hi.shape != (1024, 1024):
    raise ValueError(f"Water mask must be exactly 1024x1024; got {mask_hi.shape}")

water_hi = mask_hi >= 128

if not np.any(water_hi):
    raise ValueError("Approved water mask contains no water pixels.")
if np.all(water_hi):
    raise ValueError("Approved water mask contains no land pixels.")


# ---------------------------------------------------------------------------
# Signed distance to approved shoreline
#
# Use the original 1024x1024 mask so shoreline geometry is preserved at the
# highest available resolution. The 1024 image spans the same exact 200 m
# corner-to-corner footprint as the 257 elevation grid.
# ---------------------------------------------------------------------------
ALBEDO_SAMPLE_SPACING_M = PHYSICAL_SIZE_M / (1024 - 1)

land_distance_hi = (
    distance_transform_edt(~water_hi) * ALBEDO_SAMPLE_SPACING_M
)
water_distance_hi = (
    distance_transform_edt(water_hi) * ALBEDO_SAMPLE_SPACING_M
)

# Positive = land distance from water; negative = inside water.
signed_distance_hi = land_distance_hi - water_distance_hi

# Resample the continuous signed-distance field to the 257x257 DEM sample grid.
# No flip / mirror / crop / rotation is performed.
signed_distance = cv2.resize(
    signed_distance_hi.astype(np.float32),
    (GRID_SIZE, GRID_SIZE),
    interpolation=cv2.INTER_LINEAR,
)

water = signed_distance < 0.0
land_distance = np.maximum(signed_distance, 0.0).astype(np.float32)


# ---------------------------------------------------------------------------
# DEM coordinate grid
#
# Array convention:
#   row 0      = exact TOP of the albedo / DEM
#   row 256    = exact BOTTOM
#   column 0   = exact LEFT
#   column 256 = exact RIGHT
#
# No orientation changes are made.
# ---------------------------------------------------------------------------
coords = np.linspace(0.0, PHYSICAL_SIZE_M, GRID_SIZE, dtype=np.float32)
y_from_top, x_from_left = np.meshgrid(coords, coords, indexing="ij")


# ---------------------------------------------------------------------------
# Water + 15 m shoreline bank
# ---------------------------------------------------------------------------
bank_t = smoothstep01(land_distance / BANK_WIDTH_M)

# Water is exactly flat at zero. First land samples begin near 0.08 m and
# rise smoothly to the 2.55 m inland baseline over ~15 m.
bank_surface = np.where(
    water,
    0.0,
    SHORE_LAND_ELEV_M
    + (INLAND_BASELINE_M - SHORE_LAND_ELEV_M) * bank_t,
).astype(np.float32)

z = bank_surface.copy()


# ---------------------------------------------------------------------------
# Gentle deterministic low-frequency terrain variation
#
# This is procedural only and has NO dependence on albedo brightness.
# It is strongly suppressed near the shoreline.
# ---------------------------------------------------------------------------
rng = np.random.default_rng(SEED)

coarse = rng.normal(0.0, 1.0, size=(9, 9)).astype(np.float32)
coarse = cv2.resize(
    coarse, (GRID_SIZE, GRID_SIZE), interpolation=cv2.INTER_CUBIC
)
coarse = gaussian_filter(coarse, sigma=5.0).astype(np.float32)
coarse = (coarse - coarse.mean()) / (coarse.std() + 1e-6)

# Hills and variation only become fully active well inland so the 15 m bank
# remains a smooth, controlled shoreline transition.
inland_factor = smoothstep01((land_distance - 15.0) / 25.0)

z += 0.22 * coarse * inland_factor


# ---------------------------------------------------------------------------
# Broad natural hills
#
# Characteristic elliptical radii: 18-40 m.
# Centers are deterministically sampled on land >15 m from water.
# ---------------------------------------------------------------------------
hill_parameters = []
hill_total = np.zeros_like(z, dtype=np.float32)

for _ in range(9):
    for _attempt in range(1000):
        cx = float(rng.uniform(10.0, 190.0))
        cy = float(rng.uniform(10.0, 190.0))

        col = int(round(cx / PIXEL_SPACING_M))
        row = int(round(cy / PIXEL_SPACING_M))
        col = int(np.clip(col, 0, GRID_SIZE - 1))
        row = int(np.clip(row, 0, GRID_SIZE - 1))

        if land_distance[row, col] > 15.0:
            break
    else:
        raise RuntimeError("Could not place deterministic hill on inland land.")

    radius_x = float(rng.uniform(18.0, 38.0))
    radius_y = float(rng.uniform(18.0, 40.0))
    amplitude = float(rng.uniform(1.2, 3.4))

    hill = amplitude * np.exp(
        -0.5 * (
            ((x_from_left - cx) / radius_x) ** 2
            + ((y_from_top - cy) / radius_y) ** 2
        )
    )

    hill_total += hill.astype(np.float32)
    hill_parameters.append({
        "center_x_m_from_left": cx,
        "center_y_m_from_top": cy,
        "radius_x_m": radius_x,
        "radius_y_m": radius_y,
        "amplitude_m": amplitude,
    })

z += hill_total * inland_factor

# Gentle smoothing for continuous first derivatives.
z = gaussian_filter(z, sigma=1.0).astype(np.float32)

# Keep approved water flat and preserve a smooth minimum bank profile.
z[water] = 0.0
bank_floor = np.where(
    water,
    0.0,
    SHORE_LAND_ELEV_M
    + (2.35 - SHORE_LAND_ELEV_M) * bank_t,
).astype(np.float32)
z = np.where(water, 0.0, np.maximum(z, bank_floor)).astype(np.float32)


# ---------------------------------------------------------------------------
# Normalize broad hill relief to approximately 8 m max
# ---------------------------------------------------------------------------
extra_relief = np.maximum(z - bank_floor, 0.0)

if float(z.max()) > TARGET_MAX_ELEV_M:
    # Reduce only extra hill/variation relief, preserving shoreline profile.
    scale = (
        (TARGET_MAX_ELEV_M - float(bank_floor.max()))
        / max(float(extra_relief.max()), 1e-6)
    )
    scale = min(scale, 1.0)
    z = bank_floor + extra_relief * scale
    z[water] = 0.0

elif float(z.max()) < 7.60:
    # Increase only broad inland relief if the deterministic arrangement is
    # unexpectedly too low.
    scale = (
        (7.80 - float(bank_floor.max()))
        / max(float(extra_relief.max()), 1e-6)
    )
    z = bank_floor + extra_relief * scale
    z[water] = 0.0

z = z.astype(np.float32)


# ---------------------------------------------------------------------------
# Slope limiting
# ---------------------------------------------------------------------------
z, slope_limit_iterations = limit_slopes(
    z, water, PIXEL_SPACING_M,
    trigger_deg=SLOPE_LIMIT_TRIGGER_DEG,
    max_iterations=120,
)

# Reassert exact water minimum and numerical range.
z[water] = 0.0
z[np.abs(z) < 1e-8] = 0.0
z = np.clip(z, 0.0, HEIGHTMAP_FULL_SCALE_M).astype(np.float32)

if float(z.min()) != 0.0:
    z -= np.float32(z.min())
    z[water] = 0.0

slope_deg = calculate_slope_deg(z, PIXEL_SPACING_M)


# ---------------------------------------------------------------------------
# Final numerical validation before export
# ---------------------------------------------------------------------------
if z.shape != (GRID_SIZE, GRID_SIZE):
    raise RuntimeError(f"DEM has wrong shape: {z.shape}")
if not np.isfinite(z).all():
    raise RuntimeError("DEM contains NaN or infinity.")
if float(z.min()) != 0.0:
    raise RuntimeError(f"DEM minimum is not exactly 0.0: {z.min()}")
if float(z.max()) > 8.01:
    raise RuntimeError(f"DEM exceeds requested ~8 m range: {z.max()}")

pct_over_20 = float(np.mean(slope_deg > MAX_SLOPE_DEG) * 100.0)


# ---------------------------------------------------------------------------
# GeoTIFF transform
#
# GeoTIFF transforms conventionally reference pixel OUTER corners. The DEM is
# a 257-point sample grid whose first and last sample centers must be exactly
# 200 m apart. Therefore shift the outer origin by half a pixel:
#
# center(row=0,col=0)       -> (500000.0, 1000200.0)
# center(row=256,col=256)   -> (500200.0, 1000000.0)
#
# This preserves the user's required:
#   (257 - 1) * 0.78125 = exactly 200 m
# ---------------------------------------------------------------------------
half = PIXEL_SPACING_M / 2.0
transform = Affine(
    PIXEL_SPACING_M, 0.0, UL_SAMPLE_EASTING_M - half,
    0.0, -PIXEL_SPACING_M, UL_SAMPLE_NORTHING_M + half,
)

with rasterio.open(
    DEM_PATH,
    "w",
    driver="GTiff",
    width=GRID_SIZE,
    height=GRID_SIZE,
    count=1,
    dtype="float32",
    crs=CRS,
    transform=transform,
    nodata=None,
    compress="deflate",
    predictor=3,
) as dst:
    dst.write(z, 1)
    dst.set_band_description(1, "Elevation metres")
    dst.update_tags(
        AREA_OR_POINT="Point",
        units="metres",
        physical_width_m=str(PHYSICAL_SIZE_M),
        physical_height_m=str(PHYSICAL_SIZE_M),
        sample_spacing_m=str(PIXEL_SPACING_M),
        terrain_seed=str(SEED),
        orientation="row0=albedo_top,col0=albedo_left,no_flip_no_rotation",
    )


# ---------------------------------------------------------------------------
# 16-bit single-channel heightmap, derived ONLY from final DEM
# 0 m -> 0
# 8 m -> 65535
# ---------------------------------------------------------------------------
height_u16 = np.rint(
    np.clip(z / HEIGHTMAP_FULL_SCALE_M, 0.0, 1.0) * 65535.0
).astype(np.uint16)

Image.fromarray(height_u16, mode="I;16").save(HEIGHTMAP_PATH)


# ---------------------------------------------------------------------------
# Height preview
# ---------------------------------------------------------------------------
shade = hillshade(z, PIXEL_SPACING_M)
height_rgb = terrain_colorize(z, shade)

height_rgb_1024 = cv2.resize(
    height_rgb, (1024, 1024), interpolation=cv2.INTER_CUBIC
)
Image.fromarray(height_rgb_1024, mode="RGB").save(HEIGHT_PREVIEW_PATH)


# ---------------------------------------------------------------------------
# Slope preview
# 0-20 degrees is colourized; >20 degrees is bright red.
# ---------------------------------------------------------------------------
slope_norm = np.clip(slope_deg / MAX_SLOPE_DEG, 0.0, 1.0)
slope_u8 = np.round(slope_norm * 255.0).astype(np.uint8)

# OpenCV colormap expects a single-channel uint8 image and returns BGR.
slope_bgr = cv2.applyColorMap(slope_u8, cv2.COLORMAP_VIRIDIS)
slope_rgb = cv2.cvtColor(slope_bgr, cv2.COLOR_BGR2RGB)

over_limit = slope_deg > MAX_SLOPE_DEG
slope_rgb[over_limit] = np.array([255, 0, 0], dtype=np.uint8)

slope_rgb_1024 = cv2.resize(
    slope_rgb, (1024, 1024), interpolation=cv2.INTER_NEAREST
)
Image.fromarray(slope_rgb_1024, mode="RGB").save(SLOPE_PREVIEW_PATH)


# ---------------------------------------------------------------------------
# Albedo overlay:
#   cyan = 1 m terrain contours
#   red  = approved shoreline boundary
#
# DEM and albedo are both mapped corner-to-corner with unchanged orientation.
# ---------------------------------------------------------------------------
overlay = albedo.copy()

dem_hi = resize_float(z, 1024, 1024, interpolation=cv2.INTER_CUBIC)

overlay_bgr = cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR)

# Elevation contours every 1 m.
for level in np.arange(1.0, 8.0, 1.0):
    binary = (dem_hi >= level).astype(np.uint8) * 255
    contours, _ = cv2.findContours(
        binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
    )
    cv2.drawContours(overlay_bgr, contours, -1, (255, 255, 0), 1)  # cyan in BGR

# Exact approved high-resolution shoreline.
water_u8 = (water_hi.astype(np.uint8) * 255)
shore_contours, _ = cv2.findContours(
    water_u8, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE
)
cv2.drawContours(overlay_bgr, shore_contours, -1, (0, 0, 255), 3)

overlay_rgb = cv2.cvtColor(overlay_bgr, cv2.COLOR_BGR2RGB)
Image.fromarray(overlay_rgb, mode="RGB").save(OVERLAY_PATH)


# ---------------------------------------------------------------------------
# Reopen and validate GeoTIFF
# ---------------------------------------------------------------------------
with rasterio.open(DEM_PATH) as src:
    exported = src.read(1)

    ul_center = src.xy(0, 0, offset="center")
    ur_center = src.xy(0, GRID_SIZE - 1, offset="center")
    ll_center = src.xy(GRID_SIZE - 1, 0, offset="center")
    lr_center = src.xy(GRID_SIZE - 1, GRID_SIZE - 1, offset="center")

    validation = {
        "band_count": int(src.count),
        "data_type": src.dtypes[0],
        "dimensions": [int(src.width), int(src.height)],
        "crs": str(src.crs),
        "geotransform_gdal": list(src.transform.to_gdal()),
        "pixel_spacing_m": [
            float(src.transform.a),
            float(abs(src.transform.e)),
        ],
        "nodata": src.nodata,
        "elevation_range_m": [
            float(np.min(exported)),
            float(np.max(exported)),
        ],
        "all_finite": bool(np.isfinite(exported).all()),
    }

    if src.count != 1:
        raise RuntimeError("Exported GeoTIFF does not have exactly one band.")
    if src.dtypes[0] != "float32":
        raise RuntimeError("Exported GeoTIFF is not Float32.")
    if src.width != GRID_SIZE or src.height != GRID_SIZE:
        raise RuntimeError("Exported GeoTIFF dimensions are wrong.")
    if src.nodata is not None:
        raise RuntimeError("Exported GeoTIFF unexpectedly has a NODATA value.")
    if not np.isfinite(exported).all():
        raise RuntimeError("Exported GeoTIFF contains non-finite values.")
    if float(np.min(exported)) != 0.0:
        raise RuntimeError("Exported GeoTIFF minimum is not exactly 0.0.")


# ---------------------------------------------------------------------------
# Metadata
# ---------------------------------------------------------------------------
metadata = {
    "random_seed": SEED,
    "raster_dimensions": {
        "rows": GRID_SIZE,
        "columns": GRID_SIZE,
    },
    "physical_dimensions_m": {
        "width": PHYSICAL_SIZE_M,
        "height": PHYSICAL_SIZE_M,
    },
    "pixel_spacing_m": {
        "x": PIXEL_SPACING_M,
        "y": PIXEL_SPACING_M,
    },
    "elevation_statistics_m": {
        "minimum": float(z.min()),
        "maximum": float(z.max()),
        "mean": float(z.mean()),
        "standard_deviation": float(z.std()),
    },
    "slope_statistics_degrees": {
        "maximum": float(slope_deg.max()),
        "percentage_exceeding_20_deg": pct_over_20,
        "slope_limit_iterations_used": int(slope_limit_iterations),
    },
    "water": {
        "approved_mask_source": WATER_MASK_PATH.name,
        "water_fraction_of_1024_mask_percent":
            float(np.mean(water_hi) * 100.0),
        "water_elevation_min_m": float(z[water].min()),
        "water_elevation_max_m": float(z[water].max()),
        "bank_width_m": BANK_WIDTH_M,
    },
    "validity": {
        "contains_nan": bool(np.isnan(z).any()),
        "contains_infinity": bool(np.isinf(z).any()),
        "nodata_pixels": False,
    },
    "orientation_and_corner_mapping": {
        "orientation_preserved": True,
        "crop": False,
        "rotation": False,
        "flip": False,
        "mirror": False,
        "stretch": False,
        "mapping": {
            "albedo_upper_left": {
                "albedo_pixel": [0, 0],
                "dem_sample": [0, 0],
                "local_xy_m_before_centering": [0.0, 200.0],
                "gazebo_xy_m_after_Forest3D_centering": [-100.0, 100.0],
                "geotiff_sample_center_xy": list(map(float, ul_center)),
            },
            "albedo_upper_right": {
                "albedo_pixel": [1023, 0],
                "dem_sample": [256, 0],
                "local_xy_m_before_centering": [200.0, 200.0],
                "gazebo_xy_m_after_Forest3D_centering": [100.0, 100.0],
                "geotiff_sample_center_xy": list(map(float, ur_center)),
            },
            "albedo_lower_left": {
                "albedo_pixel": [0, 1023],
                "dem_sample": [0, 256],
                "local_xy_m_before_centering": [0.0, 0.0],
                "gazebo_xy_m_after_Forest3D_centering": [-100.0, -100.0],
                "geotiff_sample_center_xy": list(map(float, ll_center)),
            },
            "albedo_lower_right": {
                "albedo_pixel": [1023, 1023],
                "dem_sample": [256, 256],
                "local_xy_m_before_centering": [200.0, 0.0],
                "gazebo_xy_m_after_Forest3D_centering": [100.0, -100.0],
                "geotiff_sample_center_xy": list(map(float, lr_center)),
            },
        },
        "note":
            "DEM array row 0 corresponds exactly to albedo top; column 0 "
            "corresponds exactly to albedo left. GeoTIFF north-up Y decreases "
            "as raster row increases."
    },
    "hill_parameters": hill_parameters,
    "geotiff_validation": validation,
}

with open(METADATA_PATH, "w", encoding="utf-8") as f:
    json.dump(metadata, f, indent=2)


# ---------------------------------------------------------------------------
# Console report
# ---------------------------------------------------------------------------
print("Forest DEM generation complete.")
print()
print(f"DEM size:              {GRID_SIZE} x {GRID_SIZE}")
print(f"Physical coverage:     {PHYSICAL_SIZE_M:.3f} x {PHYSICAL_SIZE_M:.3f} m")
print(f"Sample spacing:        {PIXEL_SPACING_M:.5f} m")
print(f"Elevation min/max:     {z.min():.6f} / {z.max():.6f} m")
print(f"Elevation mean/std:    {z.mean():.6f} / {z.std():.6f} m")
print(f"Maximum slope:         {slope_deg.max():.6f} deg")
print(f"Area >20 deg:          {pct_over_20:.6f}%")
print(f"Slope limiter passes:  {slope_limit_iterations}")
print()
print("Reopened GeoTIFF validation:")
print(f"  bands:               {validation['band_count']}")
print(f"  dtype:               {validation['data_type']}")
print(f"  dimensions:          {validation['dimensions']}")
print(f"  CRS:                 {validation['crs']}")
print(f"  geotransform:        {validation['geotransform_gdal']}")
print(f"  pixel spacing:       {validation['pixel_spacing_m']}")
print(f"  NODATA:              {validation['nodata']}")
print(f"  elevation range:     {validation['elevation_range_m']}")
print(f"  all finite:          {validation['all_finite']}")
print()
print("Outputs:")
for p in [
    DEM_PATH,
    HEIGHTMAP_PATH,
    HEIGHT_PREVIEW_PATH,
    OVERLAY_PATH,
    SLOPE_PREVIEW_PATH,
    METADATA_PATH,
]:
    print(f"  {p.name}")
