import { fireEvent, render, screen, waitFor } from "@testing-library/react";

import MappingTeleopPanel, { teleopTwist } from "./MappingTeleopPanel";

test("publishes lateral velocity in the ROS Twist message", () => {
  expect(teleopTwist({ linearY: 0.2 })).toEqual({
    linear: { x: 0, y: 0.2, z: 0 },
    angular: { x: 0, y: 0, z: 0 },
  });
});

test("supports separate Q and E strafe commands", () => {
  const onPublish = jest.fn().mockResolvedValue(undefined);
  render(
    <MappingTeleopPanel
      disabled={false}
      onPublish={onPublish}
      onMessage={jest.fn()}
    />,
  );

  fireEvent.click(screen.getByRole("button", { name: "Activate" }));
  fireEvent.pointerDown(screen.getByRole("button", { name: "Q" }));
  expect(onPublish).toHaveBeenLastCalledWith({ linearX: 0, linearY: 0.2, angularZ: 0 });
  fireEvent.pointerUp(screen.getByRole("button", { name: "Q" }));
  fireEvent.pointerDown(screen.getByRole("button", { name: "E" }));
  expect(onPublish).toHaveBeenLastCalledWith({ linearX: 0, linearY: -0.2, angularZ: 0 });
});

test("pauses mobile motion without losing activation while upper body moves", async () => {
  const onMotionActiveChange = jest.fn();
  const onPublish = jest.fn().mockResolvedValue(undefined);
  const props = {
    disabled: false,
    motionBlocked: false,
    onMotionActiveChange,
    onPublish,
    onMessage: jest.fn(),
  };
  const { rerender } = render(<MappingTeleopPanel {...props} />);

  fireEvent.click(screen.getByRole("button", { name: "Activate" }));
  fireEvent.pointerDown(screen.getByRole("button", { name: "W" }));
  expect(onMotionActiveChange).toHaveBeenCalledWith(true);

  rerender(<MappingTeleopPanel {...props} motionBlocked />);

  await waitFor(() => expect(screen.getByText("Paused while upper body moves")).toBeInTheDocument());
  expect(screen.getByRole("button", { name: "Deactivate" })).toBeInTheDocument();
  expect(screen.getByRole("button", { name: "W" })).toBeDisabled();
  expect(onMotionActiveChange).toHaveBeenLastCalledWith(false);
  expect(onPublish).toHaveBeenLastCalledWith({ linearX: 0, linearY: 0, angularZ: 0 });
});

test.each([
  [new Error("ROS connection failed: connection refused"), "ROS connection failed: connection refused"],
  ["socket closed", "Teleop publish failed"],
])("shows publish failures locally and clears them when teleop is reactivated", async (failure, message) => {
  const onMessage = jest.fn();
  const onPublish = jest.fn().mockRejectedValueOnce(failure).mockResolvedValue(undefined);
  render(<MappingTeleopPanel disabled={false} onPublish={onPublish} onMessage={onMessage} />);
  fireEvent.click(screen.getByRole("button", { name: "Activate" }));
  fireEvent.pointerDown(screen.getByRole("button", { name: "W" }));
  await waitFor(() => expect(screen.getByRole("alert")).toHaveTextContent(message));
  expect(onMessage).toHaveBeenCalledWith(message);
  expect(screen.getByRole("button", { name: "W" })).toBeDisabled();
  fireEvent.click(screen.getByRole("button", { name: "Activate" }));
  expect(screen.queryByRole("alert")).not.toBeInTheDocument();
  fireEvent.pointerDown(screen.getByRole("button", { name: "W" }));
  expect(onPublish).toHaveBeenCalledTimes(2);
});

test("shows stop command failure even after teleop is deactivated", async () => {
  const onPublish = jest.fn().mockRejectedValue(new Error("Stop publish failed: socket closed"));
  render(<MappingTeleopPanel disabled={false} onPublish={onPublish} onMessage={jest.fn()} />);
  fireEvent.click(screen.getByRole("button", { name: "Activate" }));
  fireEvent.click(screen.getByRole("button", { name: "Deactivate" }));
  await waitFor(() => expect(screen.getByRole("alert")).toHaveTextContent("Stop publish failed: socket closed"));
  expect(onPublish).toHaveBeenCalledWith({ linearX: 0, linearY: 0, angularZ: 0 });
});

test("a blocked panel retains the failure and sends no motion command", async () => {
  const onPublish = jest.fn().mockRejectedValueOnce(new Error("ROS disconnected")).mockResolvedValue(undefined);
  const props = { disabled: false, onPublish, onMessage: jest.fn() };
  const view = render(<MappingTeleopPanel {...props} />);
  fireEvent.click(screen.getByRole("button", { name: "Activate" }));
  fireEvent.pointerDown(screen.getByRole("button", { name: "W" }));
  await waitFor(() => expect(screen.getByRole("alert")).toHaveTextContent("ROS disconnected"));
  view.rerender(<MappingTeleopPanel {...props} disabled />);
  fireEvent.click(screen.getByRole("button", { name: "Activate" }));
  fireEvent.pointerDown(screen.getByRole("button", { name: "W" }));
  expect(screen.getByRole("alert")).toHaveTextContent("ROS disconnected");
  expect(onPublish).toHaveBeenCalledTimes(1);
});
