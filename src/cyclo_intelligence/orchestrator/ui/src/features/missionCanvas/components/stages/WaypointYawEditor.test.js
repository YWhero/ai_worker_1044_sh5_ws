import {
  fireEvent,
  render,
  screen,
  waitFor,
} from "@testing-library/react";

import WaypointYawEditor, {
  normalizeYawDegrees,
  yawDegreesToRadians,
  yawRadiansToDegrees,
} from "./WaypointYawEditor";

test("converts and normalizes waypoint yaw values", () => {
  expect(normalizeYawDegrees(450)).toBe(90);
  expect(normalizeYawDegrees(-270)).toBe(90);
  expect(normalizeYawDegrees(" ")).toBeNull();
  expect(yawDegreesToRadians(90)).toBeCloseTo(Math.PI / 2);
  expect(yawRadiansToDegrees(-Math.PI / 2)).toBeCloseTo(-90);
});

test("applies an exact degree value as radians", async () => {
  const onApply = jest.fn().mockResolvedValue(undefined);
  render(
    <WaypointYawEditor
      label="Waypoint 1"
      yaw={Math.PI / 2}
      onApply={onApply}
    />,
  );

  const input = screen.getByRole("spinbutton", { name: "Yaw for Waypoint 1" });
  expect(input).toHaveValue(90);
  fireEvent.change(input, { target: { value: "-45.25" } });
  const applyButton = screen.getByRole("button", { name: "Apply" });
  fireEvent.click(applyButton);

  expect(onApply).toHaveBeenCalledTimes(1);
  expect(onApply.mock.calls[0][0]).toBeCloseTo(-45.25 * Math.PI / 180);
  await waitFor(() => expect(applyButton).toHaveTextContent("Apply"));
});

test("tracks yaw changes made by dragging the map arrow", () => {
  const { rerender } = render(
    <WaypointYawEditor label="Waypoint 1" yaw={0} onApply={jest.fn()} />,
  );
  rerender(
    <WaypointYawEditor label="Waypoint 1" yaw={Math.PI} onApply={jest.fn()} />,
  );
  expect(screen.getByRole("spinbutton", { name: "Yaw for Waypoint 1" })).toHaveValue(-180);
});
