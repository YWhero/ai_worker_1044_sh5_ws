import { configureStore } from '@reduxjs/toolkit';
import { act, fireEvent, render, screen, waitFor } from '@testing-library/react';
import { Provider } from 'react-redux';
import RecordCameraPanel from './RecordCameraPanel';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';

jest.mock('../hooks/useRosServiceCaller', () => ({ useRosServiceCaller: jest.fn() }));
jest.mock('./RecordCameraPreview', () => (props) => <div data-testid="camera-preview">{props.topic}</div>);

const names = ['cam_left_head', 'cam_left_wrist', 'cam_right_wrist', 'cam_right_head'];
const response = (enabled = names) => ({
  success: true, locked: false, camera_names: names, enabled_cameras: enabled,
  image_topics: names.map((name) => `/${name}/image/compressed`),
  rotation_degrees: [0, 270, 270, 0],
});
const setup = (phase = 0, callService = jest.fn()) => {
  callService.mockImplementation(async (_, __, request) => response(request.apply ? request.enabled_cameras : names));
  useRosServiceCaller.mockReturnValue({ callService });
  const store = configureStore({ reducer: () => ({ tasks: {
    robotType: 'ffw_sh5_rev1', recordStatus: { recordPhase: phase }, recordingMonitor: { cameraTopics: [] },
  } }) });
  const view = render(<Provider store={store}><RecordCameraPanel /></Provider>);
  return { callService, ...view };
};

test('shows all four cameras and updates actual recording selection', async () => {
  const { callService } = setup();
  expect(await screen.findByRole('switch', { name: 'Head R' })).toBeChecked();
  expect(screen.getAllByTestId('camera-preview')).toHaveLength(4);
  fireEvent.click(screen.getByRole('switch', { name: 'Wrist L' }));
  await waitFor(() => expect(screen.getByRole('switch', { name: 'Wrist L' })).not.toBeChecked());
  expect(callService).toHaveBeenLastCalledWith('/data/recording/cameras', 'interfaces/srv/RecordingCameras', {
    robot_type: 'ffw_sh5_rev1', apply: true,
    enabled_cameras: ['cam_left_head', 'cam_right_wrist', 'cam_right_head'],
  });
  expect(screen.getAllByTestId('camera-preview')).toHaveLength(3);
});

test('all OFF sends an explicit empty selection and all ON restores four', async () => {
  const { callService } = setup();
  await screen.findByRole('switch', { name: 'Head R' });
  fireEvent.click(screen.getByText('All OFF'));
  await screen.findByText(/Only non-camera data will be recorded/);
  expect(callService.mock.calls.at(-1)[2]).toMatchObject({ apply: true, enabled_cameras: [] });
  expect(screen.queryAllByTestId('camera-preview')).toHaveLength(0);
  fireEvent.click(screen.getByText('All ON'));
  await waitFor(() => expect(screen.getAllByTestId('camera-preview')).toHaveLength(4));
});

test.each([1, 2, 3])('locks camera switches during phase %s', async (phase) => {
  setup(phase);
  await screen.findByRole('switch', { name: 'Head R' });
  screen.getAllByRole('switch').forEach((button) => expect(button).toBeDisabled());
  expect(screen.getByText('All OFF')).toBeDisabled();
});

test('failed apply is visible and requires a fresh read', async () => {
  const { callService } = setup();
  await screen.findByRole('switch', { name: 'Head R' });
  callService.mockResolvedValueOnce({ success: false, message: 'Saving in progress' });
  fireEvent.click(screen.getByText('All OFF'));
  expect(await screen.findByRole('alert')).toHaveTextContent('Saving in progress');
  expect(screen.queryAllByRole('switch')).toHaveLength(0);
  fireEvent.click(screen.getByText('Refresh'));
  expect(await screen.findByRole('switch', { name: 'Head R' })).toBeChecked();
});

test('ignores responses after unmount', async () => {
  const { callService, unmount } = setup();
  await screen.findByRole('switch', { name: 'Head R' });
  let resolve;
  callService.mockImplementationOnce(() => new Promise((done) => { resolve = done; }));
  fireEvent.click(screen.getByText('All OFF'));
  unmount();
  await act(async () => resolve(response([])));
});
