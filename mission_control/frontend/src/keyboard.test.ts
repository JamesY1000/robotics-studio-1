import { describe, expect, it } from "vitest";
import { KeyboardInput } from "./keyboard";

describe("manual keyboard input", () => {
  it("requires Space and clears movement when Space is released", () => {
    const input = new KeyboardInput();
    input.press("KeyW", false);
    expect(input.command("husky1", true).x).toBe(0);
    input.press("Space", false);
    expect(input.command("husky1", true).x).toBe(1);
    input.release("Space");
    expect(input.command("husky1", true).enabled).toBe(false);
    input.press("Space", false);
    expect(input.command("husky1", true).x).toBe(0);
  });

  it("does not re-arm from key repeat after focus loss or Escape", () => {
    const input = new KeyboardInput();
    input.press("Space", false);
    input.press("KeyW", false);
    expect(input.command("husky1", false).x).toBe(0);
    input.press("Space", true);
    input.press("KeyW", true);
    expect(input.command("husky1", true).enabled).toBe(false);
    input.press("Space", false);
    input.press("KeyW", false);
    expect(input.command("husky1", true).x).toBe(1);
    input.press("Escape", false);
    input.press("Space", true);
    expect(input.command("husky1", true).enabled).toBe(false);
  });

  it("supports Parrot axes and cancels opposite directions", () => {
    const input = new KeyboardInput();
    for (const key of ["Space", "KeyW", "KeyS", "KeyQ", "KeyR", "KeyD"])
      input.press(key, false);
    expect(input.command("parrot1", true)).toEqual({
      enabled: true,
      x: 0,
      y: 1,
      z: 1,
      yaw: -1,
    });
    expect(input.command("husky1", true)).toEqual({
      enabled: true,
      x: 0,
      y: 0,
      z: 0,
      yaw: -1,
    });
    expect(new KeyboardInput().command("parrot1", true).enabled).toBe(false);
  });
});
