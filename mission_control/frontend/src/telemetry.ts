export type TelemetryConfig = {
  viewer_url: string | null;
  viewer_port: number;
  bridge_url: string | null;
  bridge_port: number;
  layout_url: string;
  // This is ROS graph discovery, not confirmation of a working data stream.
  bridge_available: boolean;
};

export type TelemetryUrls = {
  viewer: string;
  starterViewer: string;
  bridge: string;
  layout: string;
};

export function telemetryUrls(
  config: TelemetryConfig,
  pageUrl: string,
): TelemetryUrls {
  const page = new URL(pageUrl);
  const secure = page.protocol === "https:";
  const viewer = new URL(
    config.viewer_url ||
      `${secure ? "https" : "http"}://${page.hostname}:${config.viewer_port}/`,
    page,
  );
  const bridge = new URL(
    config.bridge_url ||
      `${secure ? "wss" : "ws"}://${page.hostname}:${config.bridge_port}/`,
  );
  if (!["http:", "https:"].includes(viewer.protocol)) {
    throw new Error("The telemetry viewer URL must use HTTP or HTTPS.");
  }
  if (!["ws:", "wss:"].includes(bridge.protocol)) {
    throw new Error("The telemetry bridge URL must use WS or WSS.");
  }
  if (
    viewer.username ||
    viewer.password ||
    bridge.username ||
    bridge.password
  ) {
    throw new Error("Telemetry URLs must not contain login credentials.");
  }
  if (secure && (viewer.protocol !== "https:" || bridge.protocol !== "wss:")) {
    throw new Error(
      "An HTTPS dashboard needs an HTTPS viewer and a WSS bridge. Configure secure telemetry URLs or open the local dashboard over HTTP.",
    );
  }
  viewer.searchParams.set("ds", "foxglove-websocket");
  viewer.searchParams.set("ds.url", bridge.href);
  const layout = new URL(config.layout_url, page.origin).href;
  // Lichtblick replaces same-name saved layouts when layoutUrl is present.
  // Keep that import/reset explicit; normal reloads preserve saved layouts.
  const starterViewer = new URL(viewer);
  starterViewer.searchParams.set("layoutUrl", layout);
  return {
    viewer: viewer.href,
    starterViewer: starterViewer.href,
    bridge: bridge.href,
    layout,
  };
}
