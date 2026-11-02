import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { ManualControl } from "./ManualControl";

describe("manual input selection", () => {
  it("only offers keyboard driving when Keyboard is selected", () => {
    const html = renderToStaticMarkup(
      <ManualControl
        robot="husky1"
        input="keyboard"
        controllerConnected={true}
        onSelectInput={() => {}}
      />,
    );
    expect(html).toContain("Connect keyboard");
    expect(html).not.toContain("hold L1 to drive");
    expect(html).toContain("PS4 controller");
  });

  it("removes keyboard driving and shows Parrot axes for controller input", () => {
    const html = renderToStaticMarkup(
      <ManualControl
        robot="parrot1"
        input="controller"
        controllerConnected={true}
        onSelectInput={() => {}}
      />,
    );
    expect(html).not.toContain("Connect keyboard");
    expect(html).toContain("Controller detected");
    expect(html).toContain("ascend/descend");
    expect(html).toContain("Release L1");
  });

  it("reports missing controller input without offering keyboard movement", () => {
    const html = renderToStaticMarkup(
      <ManualControl
        robot="husky1"
        input="controller"
        controllerConnected={false}
        onSelectInput={() => {}}
      />,
    );
    expect(html).toContain("Waiting for controller input");
    expect(html).not.toContain("Connect keyboard");
    expect(html).not.toContain("ascend/descend");
  });
});
