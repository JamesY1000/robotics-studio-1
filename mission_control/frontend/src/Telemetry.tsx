import { useEffect, useState } from "react";
import {
  telemetryUrls,
  type TelemetryConfig,
  type TelemetryUrls,
} from "./telemetry";

export function TelemetryViewer({
  config,
  urls,
  revision,
  onReload,
  starterLayout = false,
  onLoadStarter,
  discoveryCurrent = true,
}: {
  config: TelemetryConfig;
  urls: TelemetryUrls;
  revision: number;
  onReload: () => void;
  starterLayout?: boolean;
  onLoadStarter: () => void;
  discoveryCurrent?: boolean;
}) {
  return (
    <section className="card telemetry-card" aria-label="Telemetry viewer">
      <div className="telemetry-toolbar">
        <div>
          <h2>Lichtblick workspace</h2>
          <p>
            <span className="badge">
              {!discoveryCurrent
                ? "Bridge status unavailable"
                : config.bridge_available
                  ? "Bridge detected"
                  : "Bridge not detected"}
            </span>{" "}
            ROS graph discovery · Check live data inside the viewer.
          </p>
        </div>
        <div className="telemetry-actions">
          <button
            onClick={onLoadStarter}
            title="Replaces the previous starter layout. Export any changes first."
          >
            Reset to starter layout
          </button>
          <a
            className="telemetry-link"
            href={urls.layout}
            download="rs1-telemetry.json"
          >
            Download layout ↓
          </a>
          <button onClick={onReload}>Reload viewer</button>
          <a
            className="telemetry-link"
            href={urls.viewer}
            target="_blank"
            rel="noopener noreferrer"
          >
            Open in new tab ↗
          </a>
        </div>
      </div>
      <iframe
        key={revision}
        className="telemetry-frame"
        title="Lichtblick robot telemetry"
        src={starterLayout ? urls.starterViewer : urls.viewer}
        allowFullScreen
        referrerPolicy="no-referrer"
      />
      <details className="telemetry-help">
        <summary>Connection & layout help · viewer not loading?</summary>
        <p>
          The dashboard launch starts the viewer and ROS bridge automatically.
          Check its terminal for startup errors if the viewer is unavailable.
          To run telemetry independently, launch the dashboard with{" "}
          <code>telemetry:=false</code> and run this in another sourced workspace
          terminal:
        </p>
        <pre>
          <code>ros2 launch rs1_dashboard telemetry.launch.py</code>
        </pre>
        <p>
          If the viewer assets are missing, run the repository’s{" "}
          <code>.devcontainer/scripts/setup_dependencies.sh</code> first. The
          telemetry launch can stay running independently of this dashboard.
        </p>
        <p>
          Bridge address: <code>{urls.bridge}</code>. If needed, select a
          Foxglove WebSocket connection in Lichtblick and use this address.
          Bridge detection only confirms that its ROS node is present; the
          viewer reports connection errors and topic data.
        </p>
        <p>
          Select “Reset to starter layout” for the RS1 maps, robot models, paths,
          cameras, navigation scans, selected commands and ROS logs. This replaces
          any saved copy of that starter layout;
          duplicate it under a new name or export your changes first. If it
          cannot be fetched, download it and import it using the viewer’s Layout
          menu. Regular viewer reloads preserve saved layouts. Mission controls
          stay in this dashboard.
        </p>
        <p>
          For a remote devcontainer, make both the viewer and bridge ports
          reachable from your browser. HTTPS pages require an HTTPS viewer and a
          WSS bridge. If embedding is blocked, use “Open in new tab”.
        </p>
      </details>
    </section>
  );
}

export function Telemetry() {
  const [config, setConfig] = useState<TelemetryConfig | null>(null);
  const [error, setError] = useState("");
  const [revision, setRevision] = useState(0);
  const [starterLayout, setStarterLayout] = useState(false);
  const [retry, setRetry] = useState(0);
  useEffect(() => {
    let alive = true;
    let timer: ReturnType<typeof setTimeout>;
    const controller = new AbortController();
    const poll = async () => {
      try {
        const response = await fetch("/api/telemetry", {
          signal: controller.signal,
        });
        if (!response.ok)
          throw new Error("Telemetry configuration unavailable.");
        const next: TelemetryConfig = await response.json();
        // Validate before replacing an already running viewer configuration.
        telemetryUrls(next, window.location.href);
        if (alive) {
          setConfig(next);
          setError("");
        }
      } catch (e) {
        if (alive) setError((e as Error).message);
      } finally {
        if (alive) timer = setTimeout(poll, 3000);
      }
    };
    void poll();
    return () => {
      alive = false;
      controller.abort();
      clearTimeout(timer);
    };
  }, [retry]);

  return (
    <>
      {error && (
        <div className="alert" role="alert">
          <span>{error} Existing telemetry sessions are unaffected.</span>
          <button onClick={() => setRetry((n) => n + 1)}>Retry</button>
        </div>
      )}
      {config ? (
        <TelemetryViewer
          config={config}
          urls={telemetryUrls(config, window.location.href)}
          revision={revision}
          starterLayout={starterLayout}
          discoveryCurrent={!error}
          onLoadStarter={() => {
            setStarterLayout(true);
            setRevision((n) => n + 1);
          }}
          onReload={() => {
            setStarterLayout(false);
            setRevision((n) => n + 1);
          }}
        />
      ) : (
        <section className="card telemetry-empty" aria-label="Telemetry setup">
          <h2>Robot telemetry</h2>
          <p>
            {error
              ? "Restore the dashboard connection, then retry loading telemetry."
              : "Loading telemetry configuration…"}
          </p>
          <p>
            Lichtblick connects to ROS 2 through its own bridge. Viewing
            telemetry does not spawn robots or start a mission.
          </p>
        </section>
      )}
    </>
  );
}
