import type { Calibration, XY } from "./types";
export function imageToWorld(u: number, v: number, c: Calibration): XY {
  return {
    x: c.x_min + u * (c.x_max - c.x_min),
    y: c.y_min + v * (c.y_max - c.y_min),
  };
}
export function worldToImage(p: XY, c: Calibration): XY {
  return {
    x: (p.x - c.x_min) / (c.x_max - c.x_min),
    y: (p.y - c.y_min) / (c.y_max - c.y_min),
  };
}
export type View = { x: number; y: number; width: number; height: number };
export function zoomView(view: View, point: XY, factor: number): View {
  return {
    x: point.x + (view.x - point.x) * factor,
    y: point.y + (view.y - point.y) * factor,
    width: view.width * factor,
    height: view.height * factor,
  };
}
export function fitView(c: Calibration): View {
  return {
    x: c.x_min,
    y: c.y_min,
    width: c.x_max - c.x_min,
    height: c.y_max - c.y_min,
  };
}
