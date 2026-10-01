import { useEffect, useRef, useState } from "react";
import type { PointerEvent, WheelEvent } from "react";
import { fitView, zoomView } from "./coordinates";
import type { Preset, State, XY } from "./types";

export type Mode = "pan" | "start" | "roi" | "station";
type Props = {
  preset: Preset;
  start: XY | null;
  roi: XY | null;
  station: XY | null;
  mode: Mode;
  locked: boolean;
  state: State | null;
  onPlace: (mode: Mode, point: XY) => void;
};

export function TerrainMap({
  preset,
  start,
  roi,
  station,
  mode,
  locked,
  state,
  onPlace,
}: Props) {
  const ref = useRef<SVGSVGElement>(null);
  const [view, setView] = useState(() => fitView(preset.calibration));
  const [scale, setScale] = useState(3);
  const [cursor, setCursor] = useState<XY | null>(null);
  const drag = useRef<{
    mode: Mode;
    point: XY;
    origin: XY | null;
    moved: boolean;
    clientX: number;
    clientY: number;
  } | null>(null);
  useEffect(() => setView(fitView(preset.calibration)), [preset]);
  useEffect(() => {
    const svg = ref.current!;
    const stopPageScroll = (event: Event) => event.preventDefault();
    svg.addEventListener("wheel", stopPageScroll, { passive: false });
    return () => svg.removeEventListener("wheel", stopPageScroll);
  }, []);
  useEffect(() => {
    const update = () => setScale(ref.current?.getScreenCTM()?.a || 3);
    update();
    const observer = new ResizeObserver(update);
    if (ref.current) observer.observe(ref.current);
    return () => observer.disconnect();
  }, [view]);
  const point = (event: { clientX: number; clientY: number }): XY => {
    const matrix = ref.current!.getScreenCTM()!;
    const p = new DOMPoint(event.clientX, event.clientY).matrixTransform(
      matrix.inverse(),
    );
    return { x: p.x, y: p.y };
  };
  const inBounds = (p: XY) =>
    p.x >= preset.calibration.x_min &&
    p.x <= preset.calibration.x_max &&
    p.y >= preset.calibration.y_min &&
    p.y <= preset.calibration.y_max;
  const begin = (event: PointerEvent, marker?: Mode) => {
    if (event.button !== 0) return;
    event.stopPropagation();
    ref.current!.setPointerCapture(event.pointerId);
    drag.current = {
      mode: marker && !locked ? marker : "pan",
      point: point(event),
      origin:
        marker === "start"
          ? start
          : marker === "roi"
            ? roi
            : marker === "station"
              ? station
              : null,
      moved: false,
      clientX: event.clientX,
      clientY: event.clientY,
    };
  };
  const move = (event: PointerEvent) => {
    const p = point(event);
    setCursor(p);
    const d = drag.current;
    if (!d) return;
    if (Math.hypot(event.clientX - d.clientX, event.clientY - d.clientY) > 3)
      d.moved = true;
    if (d.mode === "pan")
      setView((v) => ({
        ...v,
        x: v.x + d.point.x - p.x,
        y: v.y + d.point.y - p.y,
      }));
    else if (d.origin) {
      const position = {
        x: d.origin.x + p.x - d.point.x,
        y: d.origin.y + p.y - d.point.y,
      };
      if (inBounds(position)) onPlace(d.mode, position);
    }
  };
  const end = (event: PointerEvent) => {
    const d = drag.current;
    drag.current = null;
    if (d && !d.moved && !d.origin && !locked && mode !== "pan") {
      const p = point(event);
      if (inBounds(p)) onPlace(mode, p);
    }
  };
  const zoom = (
    factor: number,
    p = { x: view.x + view.width / 2, y: view.y + view.height / 2 },
  ) => {
    if (view.width * factor > 600 || view.width * factor < 5) return;
    setView((v) => zoomView(v, p, factor));
  };
  const wheel = (event: WheelEvent) => {
    zoom(event.deltaY > 0 ? 1.15 : 1 / 1.15, point(event));
  };
  const r = 1 / scale;
  const drawRobot = (
    id: string,
    p: XY,
    color: string,
    label: string,
    preview: boolean,
    heading = 0,
  ) => (
    <g
      key={id}
      transform={`translate(${p.x} ${p.y})`}
      data-testid={id}
      onPointerDown={preview ? (e) => begin(e, "start") : undefined}
      className={preview && !locked ? "draggable" : ""}
    >
      <circle
        r={id === "parrot1" ? 17 * r : 10 * r}
        fill={id === "parrot1" ? "none" : color}
        stroke={id === "parrot1" ? color : "white"}
        strokeWidth={2.5 * r}
        strokeDasharray={preview ? `${3 * r} ${2 * r}` : undefined}
      />
      <path
        d={`M ${7 * r} ${-4 * r} L ${15 * r} 0 L ${7 * r} ${4 * r} Z`}
        fill={color}
        stroke="white"
        strokeWidth={r}
        transform={`rotate(${(heading * 180) / Math.PI})`}
      />
      <g
        transform={`translate(${id === "parrot1" ? -22 * r : 22 * r} ${id === "parrot1" ? -28 * r : 27 * r})`}
      >
        <rect
          x={-20 * r}
          y={-10 * r}
          width={40 * r}
          height={19 * r}
          rx={5 * r}
          fill="#fff"
        />
        <text
          fill={color}
          textAnchor="middle"
          dominantBaseline="middle"
          fontSize={10 * r}
          fontWeight="700"
        >
          {label}
        </text>
      </g>
    </g>
  );
  const live =
    state?.mode === "external" ||
    !!state?.plan ||
    (state?.mode === "deploy" && !!state.deployment?.plan);
  const calibration = preset.calibration;
  const scaleMetres =
    [1, 2, 5, 10, 20, 50, 100].find((m) => m * scale >= 65) || 100;
  return (
    <div className="map-wrap">
      <svg
        ref={ref}
        className={`terrain-map ${mode !== "pan" && !locked ? "placing" : ""}`}
        role="img"
        aria-label="Interactive terrain map. Use placement tools or numeric coordinate fields."
        viewBox={`${view.x} ${view.y} ${view.width} ${view.height}`}
        onPointerDown={(e) => begin(e)}
        onPointerMove={move}
        onPointerUp={end}
        onPointerCancel={() => {
          drag.current = null;
        }}
        onWheel={wheel}
      >
        <image
          href={preset.url}
          x={calibration.x_min}
          y={calibration.y_min}
          width={calibration.x_max - calibration.x_min}
          height={calibration.y_max - calibration.y_min}
          preserveAspectRatio="none"
        />
        {Object.entries(state?.trails || {}).map(([id, trail]) => (
          <polyline
            key={id}
            points={trail.map((p) => `${p[0]},${p[1]}`).join(" ")}
            fill="none"
            stroke={id === "parrot1" ? "#7edcf2" : "#ffe6a0"}
            strokeWidth={2.5 * r}
          />
        ))}
        {start && !live && (
          <>
            {drawRobot("husky1", start, "#bc7623", "UGV", true)}
            {drawRobot("parrot1", start, "#147eae", "UAV", true)}
          </>
        )}
        {live &&
          Object.entries(state?.ros.robots || {}).map(([id, p]) =>
            p.x !== undefined && p.y !== undefined ? (
              <g key={id} opacity={p.fresh ? 1 : 0.45}>
                {drawRobot(
                  id,
                  { x: p.x, y: p.y },
                  id === "parrot1" ? "#147eae" : "#bc7623",
                  id === "parrot1" ? "UAV" : "UGV",
                  false,
                  p.yaw,
                )}
              </g>
            ) : null,
          )}
        {roi && (
          <g
            transform={`translate(${roi.x} ${roi.y})`}
            onPointerDown={(e) => begin(e, "roi")}
            className={!locked ? "draggable" : ""}
            data-testid="roi-marker"
          >
            <circle
              r={14 * r}
              fill="#2e6957aa"
              stroke="white"
              strokeWidth={2 * r}
            />
            <circle r={4 * r} fill="white" />
            <text
              y={-22 * r}
              textAnchor="middle"
              fill="white"
              stroke="#224738"
              strokeWidth={3 * r}
              paintOrder="stroke"
              fontSize={12 * r}
              fontWeight="700"
            >
              ROI
            </text>
          </g>
        )}
        {station && (
          <g
            transform={`translate(${station.x} ${station.y})`}
            onPointerDown={(e) => begin(e, "station")}
            className={!locked ? "draggable" : ""}
            data-testid="station-marker"
          >
            <rect
              x={-10 * r}
              y={-9 * r}
              width={20 * r}
              height={18 * r}
              rx={4 * r}
              fill="white"
            />
            <path
              d={`M ${-7 * r} 0 L 0 ${-6 * r} L ${7 * r} 0 M ${-4 * r} ${-2 * r} V ${6 * r} H ${4 * r} V ${-2 * r}`}
              fill="none"
              stroke="#29443b"
              strokeWidth={2 * r}
            />
          </g>
        )}
      </svg>
      <div className="map-axis">
        +X →<br />
        +Y ↓ <span>world</span>
      </div>
      <div className="map-controls">
        <button aria-label="Zoom in" onClick={() => zoom(1 / 1.5)}>
          +
        </button>
        <button aria-label="Zoom out" onClick={() => zoom(1.5)}>
          −
        </button>
        <button
          aria-label="Fit map"
          onClick={() => setView(fitView(calibration))}
        >
          ⛶
        </button>
      </div>
      <div className="map-scale">
        <div style={{ width: scaleMetres * scale }}>{scaleMetres} m</div>
      </div>
      <div className="map-coordinates">
        {cursor
          ? `X ${cursor.x.toFixed(1)} · Y ${cursor.y.toFixed(1)} m`
          : "World coordinates · metres"}
      </div>
      {!locked && (
        <div className="map-hint">
          {mode === "pan"
            ? "Drag to pan · Scroll to zoom"
            : `Click to place ${mode === "start" ? "both robots" : mode === "roi" ? "the point ROI" : "the ground station"}`}
        </div>
      )}
    </div>
  );
}
