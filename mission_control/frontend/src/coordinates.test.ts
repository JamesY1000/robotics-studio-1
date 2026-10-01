import { describe, expect, it } from "vitest";
import { imageToWorld, worldToImage, zoomView, fitView } from "./coordinates";
const calibration = { x_min: -100, x_max: 100, y_min: -100, y_max: 100 };
describe("continuous image edge coordinates", () => {
  for (const [u, v, x, y] of [
    [0, 0, -100, -100],
    [1, 0, 100, -100],
    [0, 1, -100, 100],
    [1, 1, 100, 100],
    [0.5, 0.5, 0, 0],
    [0.73, 0.16, 46, -68],
  ]) {
    it(`${u},${v} maps to ${x},${y}`, () => {
      const point = imageToWorld(u, v, calibration);
      expect(point.x).toBeCloseTo(x);
      expect(point.y).toBeCloseTo(y);
      expect(worldToImage(point, calibration).x).toBeCloseTo(u);
      expect(worldToImage(point, calibration).y).toBeCloseTo(v);
    });
  }
  it("zooms about cursor and restores the calibrated extent", () => {
    const view = { x: -45, y: 22, width: 80, height: 50 };
    const cursor = { x: -21, y: 32 };
    const zoomed = zoomView(view, cursor, 0.5);
    expect((cursor.x - zoomed.x) / zoomed.width).toBeCloseTo(
      (cursor.x - view.x) / view.width,
    );
    expect((cursor.y - zoomed.y) / zoomed.height).toBeCloseTo(
      (cursor.y - view.y) / view.height,
    );
    expect(fitView(calibration)).toEqual({
      x: -100,
      y: -100,
      width: 200,
      height: 200,
    });
  });
});
