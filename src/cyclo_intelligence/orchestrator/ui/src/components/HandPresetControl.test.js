import { fireEvent, render, screen, waitFor } from '@testing-library/react';
import toast from 'react-hot-toast';

import HandPresetControl from './HandPresetControl';

jest.mock('react-hot-toast', () => ({
  success: jest.fn(),
  error: jest.fn(),
}));

const mockCallService = jest.fn();
jest.mock('../hooks/useRosServiceCaller', () => ({
  useRosServiceCaller: () => ({ callService: mockCallService }),
}));

jest.mock('./HandPreset3DViewer', () => (props) => (
  <div
    data-testid="hand-preset-3d-viewer"
    data-animate={String(props.animate)}
    data-target={JSON.stringify(props.targetPositions)}
  />
));

jest.mock('../hooks/useHandPresetStatus', () => () => ({
  presetIds: [0, 1, 2, 3, 4, 5],
  presets: [
    {
      id: 0,
      name: 'Open Hand',
      description: 'Fully opens the hand to release an object.',
      visual: 'open',
      editorCurls: [0, 0, 0, 0, 0],
      editorCurlsExact: false,
      previewOpenPositions: Array(20).fill(0),
      previewPositions: Array(20).fill(0),
    },
    {
      id: 1,
      name: 'Pinch Grasp',
      description: 'Thumb and index finger meet to pick up small objects.',
      visual: 'pinch',
      editorCurls: [0.8, 0.9, 0.2, 0.2, 0.2],
      editorCurlsExact: false,
      previewOpenPositions: Array(20).fill(0),
      previewPositions: Array(20).fill(0.5),
    },
    {
      id: 2,
      name: 'Three-Finger Grasp',
      description: 'Uses thumb, index, and middle fingertips while ring and little fingers stay folded.',
      visual: 'fingertip',
      editorCurls: [1, 1, 1, 1, 1],
      editorCurlsExact: false,
      previewOpenPositions: Array(20).fill(0),
      previewPositions: Array(20).fill(0.65),
    },
    {
      id: 3,
      name: 'Writing Tripod Grasp',
      description: 'Firmly holds a pencil or brush with thumb, index, and middle fingers while ring and little fingers stay folded.',
      visual: 'fingertip',
      editorCurls: [0.8, 0.9, 0.75, 1, 1],
      editorCurlsExact: false,
      previewOpenPositions: Array(20).fill(0),
      previewPositions: Array(20).fill(0.7),
    },
    {
      id: 4,
      name: 'Fist Grasp',
      description: 'Closes all fingers into the palm for a firm grip.',
      visual: 'fist',
      editorCurls: [1, 1, 1, 1, 1],
      editorCurlsExact: false,
      previewOpenPositions: Array(20).fill(0),
      previewPositions: Array(20).fill(1),
    },
    {
      id: 5,
      name: 'Bottle Base Grasp',
      description: 'Hooks four fingers around a bottle base while the thumb presses between the middle and ring fingers for a firm tilting grip.',
      visual: 'fingertip',
      note: 'Use for bottle bottoms.',
      editable: true,
      deletable: true,
      deleteReason: '',
      editorCurls: [0.85, 0.95, 0.95, 0.95, 0.9],
      editorCurlsExact: false,
      previewOpenPositions: Array(20).fill(0),
      previewPositions: Array(20).fill(0.75),
    },
  ],
  previewJointNames: Array.from({ length: 20 }, (_, index) => `finger_r_joint${index + 1}`),
  customEditor: {
    controlLabels: ['Thumb', 'Index', 'Middle', 'Ring', 'Little'],
    templatePresetId: 4,
    previewOpenPositions: Array(20).fill(0),
    previewClosedPositions: Array(20).fill(1),
  },
  enabledSides: ['left', 'right'],
  leftPresetId: 0,
  rightPresetId: 1,
  receivedAt: Date.now(),
}));

describe('HandPresetControl', () => {
  beforeEach(() => {
    mockCallService.mockReset();
    mockCallService.mockResolvedValue({
      result: { successful: true, reason: '' },
    });
    toast.success.mockReset();
    toast.error.mockReset();
  });

  test('renders only the preset IDs reported by ai_worker', () => {
    render(<HandPresetControl />);

    expect(screen.getByText('6 registered')).toBeInTheDocument();
    expect(screen.queryByRole('button', { name: 'Apply hand preset 0' }))
      .not.toBeInTheDocument();

    fireEvent.click(screen.getByRole('button', { name: 'Open hand preset selector' }));

    expect(screen.getByRole('button', { name: 'Apply hand preset 0' }))
      .toBeInTheDocument();
    expect(screen.getByRole('button', { name: 'Apply hand preset 1' }))
      .toBeInTheDocument();
    expect(screen.getByRole('button', { name: 'Apply hand preset 2' }))
      .toBeInTheDocument();
    expect(screen.getByRole('button', { name: 'Apply hand preset 3' }))
      .toBeInTheDocument();
    expect(screen.getByRole('button', { name: 'Apply hand preset 4' }))
      .toBeInTheDocument();
    expect(screen.getByRole('button', { name: 'Apply hand preset 5' }))
      .toBeInTheDocument();
    expect(screen.getByText('Three-Finger Grasp')).toBeInTheDocument();
    expect(screen.getByText('Writing Tripod Grasp')).toBeInTheDocument();
    expect(screen.getByText('Bottle Base Grasp')).toBeInTheDocument();
    expect(screen.getAllByText('Pinch Grasp')).toHaveLength(3);
    expect(screen.getByText('Closes all fingers into the palm for a firm grip.'))
      .toBeInTheDocument();
    expect(screen.getByTestId('hand-preset-3d-viewer')).toBeInTheDocument();

    fireEvent.mouseEnter(screen.getByRole('button', { name: 'Apply hand preset 4' }));
    expect(screen.getByTestId('hand-preset-3d-viewer'))
      .toHaveAttribute('data-animate', 'true');
  });

  test('forwards side and ID to the ai_worker service', async () => {
    render(<HandPresetControl />);

    fireEvent.click(screen.getByRole('button', { name: 'Open hand preset selector' }));
    fireEvent.click(screen.getByRole('button', { name: 'Apply hand preset 4' }));

    await waitFor(() => {
      expect(mockCallService).toHaveBeenCalledWith(
        '/leader/hand_preset/set',
        'rcl_interfaces/srv/SetParametersAtomically',
        {
          parameters: [
            { name: 'side', value: { type: 4, string_value: 'both' } },
            { name: 'preset_id', value: { type: 2, integer_value: 4 } },
          ],
        },
        3000
      );
    });
    expect(toast.success).toHaveBeenCalledWith('BOTH hand preset 4');
  });

  test('shows the preset node rejection reason', async () => {
    mockCallService.mockResolvedValue({
      result: { successful: false, reason: 'Release the left trigger' },
    });
    render(<HandPresetControl />);

    fireEvent.click(screen.getByRole('button', { name: 'Open hand preset selector' }));
    fireEvent.click(screen.getByRole('button', { name: 'left hand preset target' }));
    fireEvent.click(screen.getByRole('button', { name: 'Apply hand preset 4' }));

    await waitFor(() => {
      expect(toast.error).toHaveBeenCalledWith(
        'Preset change failed: Release the left trigger'
      );
    });
  });

  test('opens as a dialog and closes with Escape', () => {
    render(<HandPresetControl />);

    fireEvent.click(screen.getByRole('button', { name: 'Open hand preset selector' }));
    expect(screen.getByRole('dialog', { name: 'Hand preset studio' })).toBeInTheDocument();

    fireEvent.keyDown(window, { key: 'Escape' });
    expect(screen.queryByRole('dialog', { name: 'Hand preset studio' }))
      .not.toBeInTheDocument();
  });

  test('saves a custom five-finger pose through ai_worker', async () => {
    mockCallService.mockResolvedValue({
      result: { successful: true, reason: 'Saved custom preset 100' },
    });
    render(<HandPresetControl />);

    fireEvent.click(screen.getByRole('button', { name: 'Open hand preset selector' }));
    fireEvent.click(screen.getByRole('button', { name: /Create custom/i }));
    expect(screen.getByTestId('hand-preset-3d-viewer'))
      .toHaveAttribute('data-animate', 'true');
    fireEvent.change(screen.getByLabelText('Thumb curl'), { target: { value: '50' } });
    fireEvent.change(screen.getByLabelText('Preset name'), {
      target: { value: 'My custom grasp' },
    });
    fireEvent.click(screen.getByRole('button', { name: 'Save custom hand preset' }));

    await waitFor(() => {
      expect(mockCallService).toHaveBeenCalledWith(
        '/leader/hand_preset/custom/save',
        'rcl_interfaces/srv/SetParametersAtomically',
        expect.objectContaining({
          parameters: expect.arrayContaining([
            {
              name: 'curls',
              value: {
                type: 8,
                double_array_value: [0.5, 0.75, 0.35, 0.35, 0.35],
              },
            },
          ]),
        }),
        3000
      );
    });
  });

  test('loads registered five-finger values into the custom editor', () => {
    render(<HandPresetControl />);

    fireEvent.click(screen.getByRole('button', { name: 'Open hand preset selector' }));
    fireEvent.click(screen.getByRole('button', {
      name: 'Load hand preset 1 into custom editor',
    }));

    expect(screen.getByLabelText('Thumb curl')).toHaveValue('80');
    expect(screen.getByLabelText('Index curl')).toHaveValue('90');
    expect(screen.getByLabelText('Preset name')).toHaveValue('Pinch Grasp copy');
    expect(screen.getByTestId('hand-preset-3d-viewer'))
      .toHaveAttribute('data-animate', 'true');
  });

  test('updates a preset name, description, and operator note', async () => {
    mockCallService.mockResolvedValue({
      result: { successful: true, reason: 'Updated preset 1' },
    });
    render(<HandPresetControl />);

    fireEvent.click(screen.getByRole('button', { name: 'Open hand preset selector' }));
    fireEvent.click(screen.getByRole('button', { name: 'Edit hand preset 1' }));
    fireEvent.change(screen.getByLabelText('Name'), {
      target: { value: 'Small pinch' },
    });
    fireEvent.change(screen.getByLabelText('Operator note'), {
      target: { value: 'Use for the blue block.' },
    });
    fireEvent.click(screen.getByRole('button', { name: 'Save preset details' }));

    await waitFor(() => {
      expect(mockCallService).toHaveBeenCalledWith(
        '/leader/hand_preset/manage/update',
        'rcl_interfaces/srv/SetParametersAtomically',
        expect.objectContaining({
          parameters: expect.arrayContaining([
            { name: 'preset_id', value: { type: 2, integer_value: 1 } },
            { name: 'name', value: { type: 4, string_value: 'Small pinch' } },
            { name: 'note', value: { type: 4, string_value: 'Use for the blue block.' } },
          ]),
        }),
        3000
      );
    });
  });

  test('requires confirmation before deleting a deletable preset', async () => {
    mockCallService.mockResolvedValue({
      result: { successful: true, reason: 'Deleted preset 5' },
    });
    render(<HandPresetControl />);

    fireEvent.click(screen.getByRole('button', { name: 'Open hand preset selector' }));
    fireEvent.click(screen.getByRole('button', { name: 'Edit hand preset 5' }));
    fireEvent.click(screen.getByRole('button', { name: 'Delete hand preset 5' }));
    expect(screen.getByText('Delete preset 5?')).toBeInTheDocument();
    fireEvent.click(screen.getByRole('button', { name: 'Confirm delete hand preset' }));

    await waitFor(() => {
      expect(mockCallService).toHaveBeenCalledWith(
        '/leader/hand_preset/manage/delete',
        'rcl_interfaces/srv/SetParametersAtomically',
        {
          parameters: [
            { name: 'preset_id', value: { type: 2, integer_value: 5 } },
          ],
        },
        3000
      );
    });
    expect(screen.getByRole('button', { name: 'Undo preset deletion' }))
      .toBeInTheDocument();

    mockCallService.mockResolvedValue({
      result: { successful: true, reason: 'Restored preset 5' },
    });
    fireEvent.click(screen.getByRole('button', { name: 'Undo preset deletion' }));
    await waitFor(() => {
      expect(mockCallService).toHaveBeenLastCalledWith(
        '/leader/hand_preset/manage/restore',
        'rcl_interfaces/srv/SetParametersAtomically',
        {
          parameters: [
            { name: 'preset_id', value: { type: 2, integer_value: 5 } },
          ],
        },
        3000
      );
    });
  });
});
