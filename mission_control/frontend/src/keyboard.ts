export type ManualRobot = "husky1" | "parrot1";

const supported = new Set([
  "Space",
  "KeyW",
  "KeyS",
  "KeyA",
  "KeyD",
  "KeyQ",
  "KeyE",
  "KeyR",
  "KeyF",
  "Escape",
]);

export class KeyboardInput {
  private keys = new Set<string>();

  press(code: string, repeat: boolean) {
    if (!supported.has(code)) return false;
    if (code === "Escape") this.stop();
    // A held Space key cannot re-enable movement after focus loss or reconnect
    else if (!repeat) this.keys.add(code);
    return true;
  }

  release(code: string) {
    this.keys.delete(code);
    if (code === "Space") this.stop();
  }

  stop() {
    this.keys.clear();
  }

  command(robot: ManualRobot, focused: boolean) {
    if (!focused) this.stop();
    const enabled = focused && this.keys.has("Space");
    const direction = (positive: string, negative: string) =>
      enabled
        ? Number(this.keys.has(positive)) - Number(this.keys.has(negative))
        : 0;
    return {
      enabled,
      x: direction("KeyW", "KeyS"),
      y: robot === "parrot1" ? direction("KeyQ", "KeyE") : 0,
      z: robot === "parrot1" ? direction("KeyR", "KeyF") : 0,
      yaw: direction("KeyA", "KeyD"),
    };
  }
}
