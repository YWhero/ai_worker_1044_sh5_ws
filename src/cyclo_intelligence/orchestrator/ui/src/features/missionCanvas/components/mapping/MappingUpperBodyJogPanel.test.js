import { act, fireEvent, render, screen, waitFor } from "@testing-library/react";

import { useNavigationRosTopic } from "../../../../hooks/useNavigationRosTopic";
import MappingUpperBodyJogPanel, {
  HEAD_COMMAND_TOPIC,
  UPPER_BODY_MESSAGE_TYPE,
  resolveAiWorkerJointLimits,
} from "./MappingUpperBodyJogPanel";

jest.mock("../../../../hooks/useNavigationRosTopic", () => ({
  useNavigationRosTopic: jest.fn(),
}));

const jointState = {
  name: ["head_joint1", "head_joint2", "lift_joint"],
  position: [0.2, 0.1, -0.3],
};
const odometry = (speed = 0) => ({
  data: { twist: { twist: { linear: { x: speed, y: 0 }, angular: { z: 0 } } } },
});
let jointFeedback;
let baseFeedback;

beforeEach(() => {
  jest.clearAllMocks();
  jointFeedback = { data: jointState };
  baseFeedback = odometry();
  useNavigationRosTopic.mockImplementation((topic) => ({
    topicData: topic === "/joint_states" ? jointFeedback : topic === "/odom" ? baseFeedback : null,
  }));
});

afterEach(() => jest.useRealTimers());

function panel(publish = jest.fn().mockResolvedValue(undefined)) {
  const props = { disabled: false, onPublishRosTopic: publish, onMessage: jest.fn() };
  const view = render(<MappingUpperBodyJogPanel {...props} />);
  return {
    ...view, publish, props,
    refresh: (changes = {}) => view.rerender(<MappingUpperBodyJogPanel {...props} {...changes} />),
  };
}

async function request(view, button = "Look Up") {
  fireEvent.click(screen.getByRole("button", { name: "Activate" }));
  await act(async () => fireEvent.click(screen.getByRole("button", { name: button })));
  await waitFor(() => expect(screen.getByText("Confirming base stop")).toBeInTheDocument());
}

function feedback(view, speed = 0) {
  baseFeedback = odometry(speed);
  view.refresh();
}

test("waits for two new stopped odometry samples after publishing stop", async () => {
  const view = panel();
  await request(view);
  expect(view.publish).toHaveBeenCalledTimes(1);
  view.refresh();
  view.refresh();
  expect(view.publish).toHaveBeenCalledTimes(1);
  feedback(view);
  expect(view.publish).toHaveBeenCalledTimes(1);
  feedback(view);
  await waitFor(() => expect(view.publish).toHaveBeenCalledTimes(2));
  expect(view.publish.mock.calls[0][0]).toBe("/cmd_vel");
  expect(view.publish.mock.calls[0][2]).toEqual({
    linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 },
  });
  expect(view.publish.mock.calls[1][0]).toBe(HEAD_COMMAND_TOPIC);
  expect(view.publish.mock.calls[1][1]).toBe(UPPER_BODY_MESSAGE_TYPE);
  const target = view.publish.mock.calls[1][2];
  expect(target.joint_names).toEqual(["head_joint1", "head_joint2"]);
  expect(target.points[0].positions[0]).toBeCloseTo(0.15);
  expect(target.points[0].positions[1]).toBeCloseTo(0.1);
  expect(screen.queryByRole("alert")).not.toBeInTheDocument();
});

test.each([null, { data: {} }, odometry(NaN), odometry(Infinity)])(
  "missing or invalid base feedback blocks jog", (value) => {
    baseFeedback = value;
    const view = panel();
    fireEvent.click(screen.getByRole("button", { name: "Activate" }));
    expect(screen.getByRole("button", { name: "Look Up" })).toBeDisabled();
    expect(screen.getByRole("button", { name: "Up" })).toBeDisabled();
    expect(screen.getByText("Waiting for valid /odom")).toBeInTheDocument();
    expect(view.publish).not.toHaveBeenCalled();
  },
);

test("blocks commands while base motion is reported", () => {
  baseFeedback = odometry(0.2);
  panel();
  fireEvent.click(screen.getByRole("button", { name: "Activate" }));
  expect(screen.getByRole("button", { name: "Look Up" })).toBeDisabled();
  expect(screen.getByText("Stop the base first")).toBeInTheDocument();
});

test("movement between samples resets stop confirmation", async () => {
  const view = panel();
  await request(view);
  feedback(view);
  feedback(view, 0.2);
  feedback(view);
  expect(view.publish).toHaveBeenCalledTimes(1);
  feedback(view);
  await waitFor(() => expect(view.publish).toHaveBeenCalledTimes(2));
});

test("lost odometry cancels the queued joint command", async () => {
  const view = panel();
  await request(view);
  baseFeedback = null;
  view.refresh();
  expect(screen.getByRole("alert")).toHaveTextContent("Jog interrupted: check connection, joint feedback and base control before retrying.");
  feedback(view);
  feedback(view);
  expect(view.publish).toHaveBeenCalledTimes(1);
});

test("panel disable cancels the queued joint command", async () => {
  const view = panel();
  await request(view);
  view.refresh({ disabled: true });
  feedback(view);
  feedback(view);
  expect(view.publish).toHaveBeenCalledTimes(1);
});

test("changing joint pose while confirming stop requires retry", async () => {
  const view = panel();
  await request(view);
  jointFeedback = { data: { ...jointState, position: [0.4, 0.1, -0.3] } };
  feedback(view);
  feedback(view);
  expect(view.publish).toHaveBeenCalledTimes(1);
  expect(view.props.onMessage).toHaveBeenCalledWith(expect.stringContaining("Joint position changed"));
  expect(screen.getByRole("alert")).toHaveTextContent("Joint position changed while stopping");
});

test("timeout sends no joint command and permits retry", async () => {
  jest.useFakeTimers();
  const view = panel();
  await request(view);
  act(() => jest.advanceTimersByTime(3100));
  expect(view.publish).toHaveBeenCalledTimes(1);
  expect(view.props.onMessage).toHaveBeenCalledWith(expect.stringContaining("No joint command was sent"));
  expect(screen.getByRole("alert")).toHaveTextContent("Base stop was not confirmed. No joint command was sent");
  expect(screen.getByRole("button", { name: "Look Up" })).toBeEnabled();
  await act(async () => fireEvent.click(screen.getByRole("button", { name: "Look Up" })));
  expect(screen.queryByRole("alert")).not.toBeInTheDocument();
});

test("stop publish failure does not send a joint command", async () => {
  const view = panel(jest.fn().mockRejectedValue(new Error("ROS disconnected")));
  fireEvent.click(screen.getByRole("button", { name: "Activate" }));
  fireEvent.click(screen.getByRole("button", { name: "Look Up" }));
  await waitFor(() => expect(view.props.onMessage).toHaveBeenCalledWith("ROS disconnected"));
  expect(screen.getByRole("alert")).toHaveTextContent("ROS disconnected");
  feedback(view);
  feedback(view);
  expect(view.publish).toHaveBeenCalledTimes(1);
});

test("shows joint command failure and clears it when the panel is reactivated", async () => {
  const publish = jest.fn().mockResolvedValueOnce(undefined).mockRejectedValueOnce(new Error("Head publish failed"));
  const view = panel(publish);
  await request(view);
  feedback(view);
  feedback(view);
  await waitFor(() => expect(screen.getByRole("alert")).toHaveTextContent("Head publish failed"));
  expect(view.props.onMessage).toHaveBeenCalledWith("Head publish failed");
  fireEvent.click(screen.getByRole("button", { name: "Deactivate" }));
  expect(screen.getByRole("alert")).toHaveTextContent("Head publish failed");
  fireEvent.click(screen.getByRole("button", { name: "Activate" }));
  expect(screen.queryByRole("alert")).not.toBeInTheDocument();
});

test("shows target feedback timeout instead of a success message", async () => {
  jest.useFakeTimers();
  const view = panel();
  await request(view);
  feedback(view);
  feedback(view);
  await act(async () => {});
  expect(view.publish).toHaveBeenCalledTimes(2);
  act(() => jest.advanceTimersByTime(3100));
  expect(screen.getByRole("alert")).toHaveTextContent("Head did not reach the target. Check controller or leader input.");
  expect(view.props.onMessage).not.toHaveBeenCalledWith("Head adjustment complete");
});

test("unmount invalidates an in-flight stop request", async () => {
  let finishStop;
  const view = panel(jest.fn(() => new Promise((resolve) => { finishStop = resolve; })));
  fireEvent.click(screen.getByRole("button", { name: "Activate" }));
  fireEvent.click(screen.getByRole("button", { name: "Look Up" }));
  view.unmount();
  await act(async () => finishStop());
  expect(view.publish).toHaveBeenCalledTimes(1);
});

test("samples received before stop finishes cannot confirm it", async () => {
  let finishStop;
  const publish = jest.fn()
    .mockImplementationOnce(() => new Promise((resolve) => { finishStop = resolve; }))
    .mockResolvedValue(undefined);
  const view = panel(publish);
  fireEvent.click(screen.getByRole("button", { name: "Activate" }));
  fireEvent.click(screen.getByRole("button", { name: "Look Up" }));
  feedback(view);
  feedback(view);
  expect(publish).toHaveBeenCalledTimes(1);
  await act(async () => finishStop());
  feedback(view);
  expect(publish).toHaveBeenCalledTimes(1);
  feedback(view);
  await waitFor(() => expect(publish).toHaveBeenCalledTimes(2));
});

test.each([
  [["head_joint1", "head_joint2"], [0.2, 0.1], "Look Up", "Up"],
  [["lift_joint"], [-0.3], "Up", "Look Up"],
])("available group works without unrelated joint feedback", async (names, positions, available, missing) => {
  jointFeedback = { data: { name: names, position: positions } };
  const view = panel();
  await request(view, available);
  expect(screen.getByRole("button", { name: missing })).toBeDisabled();
  feedback(view);
  feedback(view);
  await waitFor(() => expect(view.publish).toHaveBeenCalledTimes(2));
});

test("uses robot-description limits without widening controller limits", () => {
  const result = resolveAiWorkerJointLimits(`
    <robot name="sh5">
      <joint name="head_joint1"><limit lower="-0.1" upper="0.4"/></joint>
      <joint name="head_joint2"><limit lower="-0.35" upper="0.35"/></joint>
      <joint name="lift_joint"><limit lower="-0.45" upper="-0.05"/></joint>
    </robot>
  `);
  expect(result.limits.head_joint1).toEqual({ lower: -0.1, upper: 0.4 });
  expect(result.limits.head_joint2).toEqual({ lower: -0.3, upper: 0.3 });
  expect(result.limits.lift_joint).toEqual({ lower: -0.45, upper: -0.05 });
});
