import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { SafetyControls, safetyClear } from "./SafetyControls";
import type { SafetyState } from "./types";

const stopped: SafetyState = {
  fresh: true,
  service_ready: true,
  reset_ready: true,
  instance_id: "test",
  revision: 3,
  husky_stopped: true,
  parrot_stopped: true,
};

describe("software stop panel", () => {
  it("does not claim a latched stop just because the service is unavailable", () => {
    const html = renderToStaticMarkup(
      <SafetyControls
        connected={true}
        safety={{
          ...stopped,
          service_ready: false,
          husky_stopped: false,
          parrot_stopped: false,
        }}
      />,
    );
    expect(html).toContain("Stop state unavailable");
    expect(html).not.toContain("mission held");
    expect(html).toMatch(/<button class="safety-stop">E-STOP BOTH ROBOTS/);
  });
  it("allows Stop even when state is missing, but cannot Reset", () => {
    const html = renderToStaticMarkup(<SafetyControls connected={true} />);
    expect(html).toMatch(/<button class="safety-stop">E-STOP BOTH ROBOTS/);
    expect(html).toMatch(/<button disabled="">Reset both stops/);
    expect(html).toContain("state unavailable");
  });
  it("separates reset from resume and shows per-robot latches", () => {
    const html = renderToStaticMarkup(
      <SafetyControls connected={true} safety={stopped} />,
    );
    expect(html).toContain("Husky: stopped");
    expect(html).toContain("Parrot: stopped");
    expect(html).toContain("<button>Reset both stops");
    expect(html).toContain("Then select Resume or manual assistance");
  });
  it("waits for coordinator cleanup before offering reset", () => {
    const html = renderToStaticMarkup(
      <SafetyControls
        connected={true}
        safety={{ ...stopped, reset_ready: false }}
      />,
    );
    expect(html).toMatch(/<button disabled="">Reset both stops/);
    expect(html).toMatch(/<button class="safety-stop">E-STOP BOTH ROBOTS/);
  });
  it("does not display cached released state as current while disconnected", () => {
    const html = renderToStaticMarkup(
      <SafetyControls
        connected={false}
        safety={{ ...stopped, husky_stopped: false, parrot_stopped: false }}
      />,
    );
    expect(html).toContain("Husky: unknown");
    expect(html).not.toContain("Stop released");
  });
  it("requires both latches clear and a fresh authority before enabling motion", () => {
    expect(safetyClear()).toBe(false);
    expect(safetyClear(stopped)).toBe(false);
    const released = {
      ...stopped,
      husky_stopped: false,
      parrot_stopped: false,
    };
    expect(safetyClear(released)).toBe(true);
    expect(safetyClear({ ...released, fresh: false })).toBe(false);
    expect(safetyClear({ ...released, service_ready: false })).toBe(false);
    expect(safetyClear({ ...released, parrot_stopped: true })).toBe(false);
  });
});
