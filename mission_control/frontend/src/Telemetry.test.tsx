import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { TelemetryViewer } from "./Telemetry";
import { telemetryUrls, type TelemetryConfig } from "./telemetry";

const config: TelemetryConfig = {
  viewer_url: null,
  viewer_port: 8080,
  bridge_url: null,
  bridge_port: 8765,
  layout_url: "/api/telemetry/layout",
  bridge_available: false,
};

describe("telemetry endpoint selection", () => {
  it("connects on the browser host, including remotely accessed dashboards", () => {
    const urls = telemetryUrls(config, "http://robot-workstation:8000/");
    const viewer = new URL(urls.viewer);
    expect(viewer.origin).toBe("http://robot-workstation:8080");
    expect(viewer.searchParams.get("ds")).toBe("foxglove-websocket");
    expect(viewer.searchParams.get("ds.url")).toBe(
      "ws://robot-workstation:8765/",
    );
    expect(urls.layout).toBe(
      "http://robot-workstation:8000/api/telemetry/layout",
    );
    expect(viewer.searchParams.has("layoutUrl")).toBe(false);
    expect(new URL(urls.starterViewer).searchParams.get("layoutUrl")).toBe(
      urls.layout,
    );
  });

  it("supports IPv6 browser hosts and custom forwarded ports", () => {
    const urls = telemetryUrls(
      { ...config, viewer_port: 18080, bridge_port: 18765 },
      "http://[::1]:8000/",
    );
    expect(new URL(urls.viewer).origin).toBe("http://[::1]:18080");
    expect(urls.bridge).toBe("ws://[::1]:18765/");
  });

  it("uses secure protocols when opened over HTTPS", () => {
    const urls = telemetryUrls(config, "https://robot.example.org/dashboard/");
    expect(new URL(urls.viewer).origin).toBe("https://robot.example.org:8080");
    expect(urls.bridge).toBe("wss://robot.example.org:8765/");
  });

  it("preserves configured viewer paths, queries and fragments", () => {
    const urls = telemetryUrls(
      {
        ...config,
        viewer_url:
          "https://viewer.example.org/telemetry/?theme=dark#workspace",
        bridge_url: "wss://robot.example.org/live?session=a&mode=read",
      },
      "https://dashboard.example.org/",
    );
    const viewer = new URL(urls.viewer);
    expect(viewer.pathname).toBe("/telemetry/");
    expect(viewer.hash).toBe("#workspace");
    expect(viewer.searchParams.get("theme")).toBe("dark");
    expect(viewer.searchParams.get("ds.url")).toBe(
      "wss://robot.example.org/live?session=a&mode=read",
    );
  });

  it("keeps an explicitly configured custom layout", () => {
    const viewer = new URL("http://localhost:8080/");
    viewer.searchParams.set("layoutUrl", "http://localhost:9000/custom.json");
    const urls = telemetryUrls(
      { ...config, viewer_url: viewer.href },
      "http://localhost:8000/",
    );
    expect(new URL(urls.viewer).searchParams.get("layoutUrl")).toBe(
      "http://localhost:9000/custom.json",
    );
  });

  it("rejects invalid protocols and credentials before embedding", () => {
    expect(() =>
      telemetryUrls(
        { ...config, viewer_url: "javascript:alert(1)" },
        "http://localhost:8000/",
      ),
    ).toThrow("HTTP or HTTPS");
    expect(() =>
      telemetryUrls(
        { ...config, bridge_url: "https://localhost:8765" },
        "http://localhost:8000/",
      ),
    ).toThrow("WS or WSS");
    expect(() =>
      telemetryUrls(
        { ...config, bridge_url: "ws://user:password@localhost:8765" },
        "http://localhost:8000/",
      ),
    ).toThrow("login credentials");
  });

  it("explains mixed-content errors instead of embedding a blocked viewer", () => {
    expect(() =>
      telemetryUrls(
        { ...config, viewer_url: "http://localhost:8080" },
        "https://dashboard.example.org/",
      ),
    ).toThrow("HTTPS viewer and a WSS bridge");
    expect(() =>
      telemetryUrls(
        { ...config, bridge_url: "ws://localhost:8765" },
        "https://dashboard.example.org/",
      ),
    ).toThrow("HTTPS viewer and a WSS bridge");
  });
});

describe("telemetry workspace", () => {
  function render(bridgeAvailable: boolean, discoveryCurrent = true) {
    return renderToStaticMarkup(
      <TelemetryViewer
        config={{ ...config, bridge_available: bridgeAvailable }}
        urls={telemetryUrls(config, "http://localhost:8000/")}
        revision={0}
        onReload={() => {}}
        onLoadStarter={() => {}}
        discoveryCurrent={discoveryCurrent}
      />,
    );
  }

  it("offers a local embedded viewer, new-tab fallback and starter layout", () => {
    const markup = render(false);
    expect(markup).toContain('title="Lichtblick robot telemetry"');
    expect(markup).toContain("ds=foxglove-websocket");
    expect(markup).toContain('target="_blank"');
    expect(markup).toContain('rel="noopener noreferrer"');
    expect(markup).toContain('download="rs1-telemetry.json"');
    expect(markup).toContain("Reload viewer");
    expect(markup).toContain("Reset to starter layout");
    expect(markup).toContain("ros2 launch rs1_dashboard telemetry.launch.py");
    expect(markup).toContain("Bridge not detected");
  });

  it("reports ROS graph discovery without asserting a live data connection", () => {
    const markup = render(true);
    expect(markup).toContain("Bridge detected");
    expect(markup).toContain("ROS graph discovery");
    expect(markup).toContain("Check live data inside the viewer");
    expect(markup).not.toContain("Live connection");
    expect(markup).not.toContain("Connected to ROS");
  });

  it("does not present stale bridge discovery as current after a poll fails", () => {
    const markup = render(true, false);
    expect(markup).toContain("Bridge status unavailable");
    expect(markup).not.toContain("Bridge detected");
    expect(markup).toContain('title="Lichtblick robot telemetry"');
  });
});
