import { useEffect, useRef, useState } from "react";
import type { PointerEvent, WheelEvent } from "react";
import { fitView, zoomView } from "./coordinates";
import type {
  GroundTruthObstacle,
  GroundTruthTree,
  Preset,
  State,
  TreeMarker,
  XY,
} from "./types";

export type Mode = "pan" | "start" | "roi" | "station";
type Props = {
  preset: Preset;
  start: XY | null;
  roi: XY | null;
  roiSecond: XY | null;
  station: XY | null;
  mode: Mode;
  locked: boolean;
  state: State | null;
  groundTruth: GroundTruthTree[];
  obstacles: GroundTruthObstacle[];
  onPlace: (mode: Mode, point: XY) => void;
  onRoi: (entry: XY, corner: XY) => void;
};

const TREE_COLORS: Record<string, string> = {
  OAK_TREE: "#2e7d32",
  PINE_TREE: "#147eae",
};

// Ground clutter, deliberately distinct from the tree and detected colours.
const OBSTACLE_COLORS: Record<string, string> = {
  bush: "#ff9800",
  rock: "#6a1b9a",
};

export function TerrainMap({
  preset,
  start,
  roi,
  roiSecond,
  station,
  mode,
  locked,
  state,
  groundTruth,
  obstacles,
  onPlace,
  onRoi,
}: Props) {
  const ref = useRef<SVGSVGElement>(null);
  const [view, setView] = useState(() => fitView(preset.calibration));
  const [scale, setScale] = useState(3);
  const [cursor, setCursor] = useState<XY | null>(null);
  const DEFAULT_ROI_SIDE = 10;
  const MIN_ROI_SIDE = 2;
  type DragKind = "pan" | "start" | "station" | "roi-enter" | "roi-corner";
  const drag = useRef<{
    kind: DragKind;
    point: XY;
    origin: XY | null;
    opposite: XY | null;
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
  const begin = (event: PointerEvent, kind: DragKind = "pan") => {
    if (event.button !== 0) return;
    event.stopPropagation();
    ref.current!.setPointerCapture(event.pointerId);
    const effective = kind !== "pan" && locked ? "pan" : kind;
    drag.current = {
      kind: effective,
      point: point(event),
      origin:
        effective === "start"
          ? start
          : effective === "station"
            ? station
            : effective === "roi-enter" || effective === "roi-corner"
              ? roi
              : null,
      opposite:
        effective === "roi-enter" || effective === "roi-corner"
          ? roiSecond
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
    if (d.kind === "pan")
      setView((v) => ({
        ...v,
        x: v.x + d.point.x - p.x,
        y: v.y + d.point.y - p.y,
      }));
    else if (d.kind === "start" || d.kind === "station") {
      if (!d.origin) return;
      const position = {
        x: d.origin.x + p.x - d.point.x,
        y: d.origin.y + p.y - d.point.y,
      };
      if (inBounds(position)) onPlace(d.kind, position);
    } else if (d.kind === "roi-enter") {
      if (!d.origin) return;
      const entry = {
        x: d.origin.x + p.x - d.point.x,
        y: d.origin.y + p.y - d.point.y,
      };
      if (d.opposite) {
        const corner = {
          x: d.opposite.x + p.x - d.point.x,
          y: d.opposite.y + p.y - d.point.y,
        };
        if (inBounds(entry) && inBounds(corner)) onRoi(entry, corner);
      } else if (inBounds(entry)) {
        onRoi(entry, {
          x: entry.x + DEFAULT_ROI_SIDE,
          y: entry.y - DEFAULT_ROI_SIDE,
        });
      }
    } else if (d.kind === "roi-corner") {
      if (!d.origin) return;
      // Anchor the bottom-left entry corner and keep the box square.
      const side = Math.max(
        MIN_ROI_SIDE,
        Math.max(p.x - d.origin.x, d.origin.y - p.y),
      );
      const corner = { x: d.origin.x + side, y: d.origin.y - side };
      if (inBounds(d.origin) && inBounds(corner)) onRoi(d.origin, corner);
    }
  };
  const end = (event: PointerEvent) => {
    const d = drag.current;
    drag.current = null;
    if (d && !d.moved && !d.origin && !locked && mode !== "pan") {
      const p = point(event);
      if (inBounds(p)) {
        if (mode === "roi")
          onRoi(p, {
            x: p.x + DEFAULT_ROI_SIDE,
            y: p.y - DEFAULT_ROI_SIDE,
          });
        else onPlace(mode, p);
      }
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
  const recordedRoi = state?.ros.mission?.roi;
  // Don't draw an old survey's result over a newly edited ROI
  const sameRoi =
    roi &&
    roiSecond &&
    recordedRoi?.frame_id === "world" &&
    recordedRoi.min &&
    recordedRoi.max &&
    Math.abs(Math.min(roi.x, roiSecond.x) - recordedRoi.min.x) < 1e-6 &&
    Math.abs(Math.max(roi.x, roiSecond.x) - recordedRoi.max.x) < 1e-6 &&
    Math.abs(Math.min(roi.y, roiSecond.y) - recordedRoi.min.y) < 1e-6 &&
    Math.abs(Math.max(roi.y, roiSecond.y) - recordedRoi.max.y) < 1e-6;
  const biomass = sameRoi ? state?.ros.biomass : null;
  const totalAgbKg = biomass?.valid ? biomass.total_agb_kg : null;
  const hasAgb =
    typeof totalAgbKg === "number" &&
    Number.isFinite(totalAgbKg) &&
    totalAgbKg >= 0;
  const allEstimated =
    hasAgb &&
    biomass &&
    biomass.tree_count > 0 &&
    biomass.estimated_count === biomass.tree_count;
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
  const drawTree = (tree: TreeMarker, index: number) => {
    const color = TREE_COLORS[tree.type] || "#6b7280";
    return (
      <g
        key={`tree-${index}`}
        transform={`translate(${tree.x} ${tree.y})`}
        data-testid="tree-marker"
      >
        <circle
          r={7 * r}
          fill={`${color}cc`}
          stroke="white"
          strokeWidth={1.5 * r}
        />
        <path
          d={`M ${-4 * r} ${-2 * r} H ${4 * r} M 0 ${-6 * r} V ${5 * r}`}
          fill="none"
          stroke="white"
          strokeWidth={1.5 * r}
        />
        <text
          y={16 * r}
          textAnchor="middle"
          fill="white"
          stroke="#1f2d25"
          strokeWidth={3 * r}
          paintOrder="stroke"
          fontSize={9 * r}
          fontWeight="700"
        >
          {`#${index} ${tree.type.replace(/_TREE$/, "").toLowerCase()}`}
        </text>
      </g>
    );
  };
  const drawGroundTruthTree = (tree: GroundTruthTree, index: number) => (
    <g
      key={`gt-${index}`}
      transform={`translate(${tree.x} ${tree.y})`}
      data-testid="ground-truth-marker"
    >
      <rect
        x={-6 * r}
        y={-6 * r}
        width={12 * r}
        height={12 * r}
        rx={2 * r}
        fill="#e91e63"
        stroke="white"
        strokeWidth={1.5 * r}
      />
    </g>
  );
  const drawObstacle = (obstacle: GroundTruthObstacle, index: number) => {
    const color = OBSTACLE_COLORS[obstacle.type] || "#455a64";
    return (
      <g
        key={`obs-${index}`}
        transform={`translate(${obstacle.x} ${obstacle.y})`}
        data-testid="obstacle-marker"
      >
        {obstacle.type === "rock" ? (
          <path
            d={`M ${-6 * r} ${3 * r} L ${-3 * r} ${-4 * r} L ${2 * r} ${-5 * r} L ${6 * r} ${1 * r} L ${4 * r} ${5 * r} L ${-3 * r} ${5 * r} Z`}
            fill={color}
            stroke="white"
            strokeWidth={r}
          />
        ) : (
          <circle r={5 * r} fill={color} stroke="white" strokeWidth={r} />
        )}
      </g>
    );
  };
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
          <g data-testid="roi-marker">
            {roiSecond && (
              <rect
                x={Math.min(roi.x, roiSecond.x)}
                y={Math.min(roi.y, roiSecond.y)}
                width={Math.abs(roiSecond.x - roi.x)}
                height={Math.abs(roiSecond.y - roi.y)}
                fill={allEstimated ? "#22a44733" : "#2e695722"}
                stroke="#2e6957"
                strokeWidth={2 * r}
                strokeDasharray={locked ? undefined : `${6 * r} ${4 * r}`}
                onPointerDown={(e) => begin(e, "roi-enter")}
                className={!locked ? "draggable" : ""}
              />
            )}
            <g
              transform={`translate(${roi.x} ${roi.y})`}
              onPointerDown={(e) => begin(e, "roi-enter")}
              className={!locked ? "draggable" : ""}
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
            {roiSecond && biomass && biomass.tree_count > 0 && (
              <g
                transform={`translate(${(roi.x + roiSecond.x) / 2} ${(roi.y + roiSecond.y) / 2})`}
                data-testid="roi-agb-label"
              >
                <title>{`${biomass.measured_count} confirmed DBHs. ${biomass.message}`}</title>
                <text
                  textAnchor="middle"
                  dominantBaseline="central"
                  fill="white"
                  stroke="#123b25"
                  strokeWidth={3 * r}
                  paintOrder="stroke"
                  fontSize={12 * r}
                  fontWeight="700"
                >
                  {hasAgb
                    ? `${allEstimated ? "Total" : "Sampled"} AGB estimate: ${totalAgbKg.toLocaleString(undefined, { maximumFractionDigits: 1 })} kg`
                    : biomass.valid && biomass.measured_count === 0
                      ? "Awaiting DBH samples"
                      : "AGB unavailable"}
                  <tspan
                    x={0}
                    dy={16 * r}
                  >{`${biomass.estimated_count}/${biomass.tree_count} trees included`}</tspan>
                </text>
              </g>
            )}
            {roiSecond && (
              <g
                transform={`translate(${roiSecond.x} ${roiSecond.y})`}
                onPointerDown={(e) => begin(e, "roi-corner")}
                className={!locked ? "draggable" : ""}
                data-testid="roi-resize"
              >
                <rect
                  x={-8 * r}
                  y={-8 * r}
                  width={16 * r}
                  height={16 * r}
                  rx={3 * r}
                  fill="white"
                  stroke="#2e6957"
                  strokeWidth={2 * r}
                />
              </g>
            )}
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
        {(state?.ros.trees || []).map((tree, i) => drawTree(tree, i))}
        {state?.ros.sampling_target &&
          (() => {
            const target = state.ros.sampling_target;
            const husky = state.ros.robots.husky1;
            const distance =
              husky?.fresh && husky.x != null && husky.y != null
                ? Math.hypot(husky.x - target.x, husky.y - target.y)
                : null;
            return (
              <g
                data-testid="sampling-target"
                pointerEvents="none"
                transform={`translate(${target.x} ${target.y})`}
              >
                <circle
                  r={target.max_range_m}
                  fill="#22d3ee15"
                  stroke="#22d3ee"
                  strokeWidth={1.5 * r}
                  strokeDasharray={`${5 * r} ${3 * r}`}
                />
                <circle
                  r={target.min_range_m}
                  fill="#fb718520"
                  stroke="#fb7185"
                  strokeWidth={r}
                />
                <circle
                  r={10 * r}
                  fill="none"
                  stroke="#22d3ee"
                  strokeWidth={2 * r}
                />
                <text
                  y={-target.max_range_m - 5 * r}
                  textAnchor="middle"
                  fill="white"
                  stroke="#1f2d25"
                  strokeWidth={3 * r}
                  paintOrder="stroke"
                  fontSize={11 * r}
                >
                  {`Tree ${target.tree_index} · ${target.refined ? "located" : "mapped estimate"}${distance == null ? "" : ` · ${distance.toFixed(2)} m`}`}
                </text>
              </g>
            );
          })()}
        {obstacles.map((obstacle, i) => drawObstacle(obstacle, i))}
        {groundTruth.map((tree, i) => drawGroundTruthTree(tree, i))}
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
            : `Click to place ${mode === "start" ? "both robots" : mode === "roi" ? "the ROI box" : "the ground station"}`}
        </div>
      )}
    </div>
  );
}
