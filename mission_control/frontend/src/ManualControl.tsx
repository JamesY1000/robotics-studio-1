import { useEffect, useRef, useState } from "react";
import { KeyboardInput, type ManualRobot } from "./keyboard";

type ManualInput = "keyboard" | "controller";

export function ManualControl({
  robot,
  input,
  controllerConnected,
  onSelectInput,
}: {
  robot: ManualRobot;
  input?: ManualInput;
  controllerConnected: boolean;
  onSelectInput: (input: ManualInput) => void;
}) {
  return (
    <section
      className="manual-control"
      aria-label={`Manual control for ${robot}`}
    >
      <strong>Manual control: {robot === "husky1" ? "Husky" : "Parrot"}</strong>
      <div
        className="manual-control-actions"
        role="group"
        aria-label="Manual input device"
      >
        <button
          aria-pressed={input === "keyboard"}
          disabled={input === "keyboard"}
          onClick={() => onSelectInput("keyboard")}
        >
          Keyboard
        </button>
        <button
          aria-pressed={input === "controller"}
          disabled={input === "controller"}
          onClick={() => onSelectInput("controller")}
        >
          PS4 controller
        </button>
      </div>
      {input === "keyboard" && <KeyboardControl robot={robot} />}
      {input === "controller" && (
        <div role="status">
          <p>
            {controllerConnected
              ? "Controller detected"
              : "Waiting for controller input"}
          </p>
          <p>
            Connect the PS4 controller to the computer running ROS, using USB or
            Bluetooth.
          </p>
          <p>Release L1, then hold L1 to drive. Release L1 to stop.</p>
          <p>Left stick up/down: forward/back · Right stick left/right: turn</p>
          {robot === "parrot1" && (
            <p>
              Left stick left/right: sideways · Right stick up/down:
              ascend/descend
            </p>
          )}
          <p>
            Keyboard driving is disabled. Controller driving stays available
            when you leave this page.
          </p>
        </div>
      )}
      {!input && <p>Waiting for the coordinator's input selection</p>}
    </section>
  );
}

function KeyboardControl({ robot }: { robot: ManualRobot }) {
  const panel = useRef<HTMLDivElement>(null);
  const [connection, setConnection] = useState(0);
  const [ready, setReady] = useState(false);
  const [message, setMessage] = useState(
    "Connect keyboard control, then hold Space to drive.",
  );

  useEffect(() => {
    setReady(false);
    if (connection === 0) return;

    const element = panel.current!;
    const protocol = location.protocol === "https:" ? "wss:" : "ws:";
    const socket = new WebSocket(
      `${protocol}//${location.host}/api/teleop/${robot}`,
    );
    const input = new KeyboardInput();
    let alive = true;
    let waitingSince: number | null = null;

    function send() {
      if (socket.readyState !== WebSocket.OPEN) return;
      // Wait for acknowledgement so slow connections cannot queue old movement commands
      if (waitingSince !== null) {
        if (performance.now() - waitingSince > 200) {
          input.stop();
          socket.close();
          if (alive)
            setMessage(
              "Keyboard connection is too slow. Reconnect to continue.",
            );
        }
        return;
      }
      const focused = document.activeElement === element && !document.hidden;
      socket.send(JSON.stringify(input.command(robot, focused)));
      waitingSince = performance.now();
    }

    function stop() {
      input.stop();
      send();
    }

    function keyDown(event: KeyboardEvent) {
      if (
        document.activeElement !== element ||
        socket.readyState !== WebSocket.OPEN
      )
        return;
      if (event.ctrlKey || event.metaKey || event.altKey) {
        stop();
        return;
      }
      if (input.press(event.code, event.repeat)) {
        event.preventDefault();
        send();
      }
    }

    function keyUp(event: KeyboardEvent) {
      input.release(event.code);
      send();
    }

    function visibilityChanged() {
      if (document.hidden) stop();
    }

    socket.onopen = () => {
      if (!alive) return;
      setReady(true);
      setMessage("Hold Space to drive. Release Space or press Escape to stop.");
      element.focus();
    };
    socket.onmessage = () => {
      waitingSince = null;
    };
    socket.onclose = (event) => {
      input.stop();
      if (!alive) return;
      setReady(false);
      setMessage(
        event.reason || "Keyboard disconnected. Connect again to continue.",
      );
    };
    socket.onerror = () => socket.close();

    element.addEventListener("keydown", keyDown);
    element.addEventListener("blur", stop);
    window.addEventListener("keyup", keyUp);
    window.addEventListener("blur", stop);
    document.addEventListener("visibilitychange", visibilityChanged);
    const timer = window.setInterval(send, 50);

    return () => {
      // Mode changes and mission commands unmount this panel and release keyboard ownership
      alive = false;
      stop();
      window.clearInterval(timer);
      element.removeEventListener("keydown", keyDown);
      element.removeEventListener("blur", stop);
      window.removeEventListener("keyup", keyUp);
      window.removeEventListener("blur", stop);
      document.removeEventListener("visibilitychange", visibilityChanged);
      socket.close();
    };
  }, [robot, connection]);

  return (
    <div
      ref={panel}
      tabIndex={0}
      className="keyboard-control"
      aria-label={`Keyboard control for ${robot}`}
    >
      <p role="status">{message}</p>
      <p>
        <kbd>Space</kbd> Hold to drive · <kbd>W/S</kbd> Forward/back ·{" "}
        <kbd>A/D</kbd> Turn
      </p>
      {robot === "parrot1" && (
        <p>
          <kbd>Q/E</kbd> Sideways · <kbd>R/F</kbd> Up/down
        </p>
      )}
      <div className="manual-control-actions">
        <button
          disabled={ready}
          onClick={() => setConnection((value) => value + 1)}
        >
          Connect keyboard
        </button>
        <button
          onClick={() => {
            setConnection(0);
            setMessage("Keyboard control stopped. Reconnect to continue.");
          }}
        >
          Stop keyboard control
        </button>
        {ready && (
          <button onClick={() => panel.current?.focus()}>
            Focus driving controls
          </button>
        )}
      </div>
    </div>
  );
}
