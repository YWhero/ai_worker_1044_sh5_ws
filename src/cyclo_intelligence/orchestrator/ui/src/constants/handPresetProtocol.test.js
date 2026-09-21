import {
  buildCustomHandPresetRequest,
  buildHandPresetDeleteRequest,
  buildHandPresetRequest,
  buildHandPresetRestoreRequest,
  buildHandPresetUpdateRequest,
  parseHandPresetStatus,
} from './handPresetProtocol';

describe('hand preset ROS protocol', () => {
  test('builds the ai_worker service request without pose data', () => {
    expect(buildHandPresetRequest('both', 4)).toEqual({
      parameters: [
        {
          name: 'side',
          value: { type: 4, string_value: 'both' },
        },
        {
          name: 'preset_id',
          value: { type: 2, integer_value: 4 },
        },
      ],
    });
  });

  test.each([
    ['unknown', 1],
    ['left', -1],
    ['right', 1.5],
  ])('rejects an invalid request (%s, %s)', (side, presetId) => {
    expect(() => buildHandPresetRequest(side, presetId)).toThrow();
  });

  test('builds a custom preset save request with five normalized curls', () => {
    expect(buildCustomHandPresetRequest(
      'My grasp',
      'Thumb and index curl.',
      [1, 0.75, 0, 0, 0]
    )).toEqual({
      parameters: [
        { name: 'name', value: { type: 4, string_value: 'My grasp' } },
        {
          name: 'description',
          value: { type: 4, string_value: 'Thumb and index curl.' },
        },
        {
          name: 'curls',
          value: { type: 8, double_array_value: [1, 0.75, 0, 0, 0] },
        },
      ],
    });
  });

  test('rejects invalid custom finger controls', () => {
    expect(() => buildCustomHandPresetRequest('', '', [0, 0, 0, 0, 0])).toThrow();
    expect(() => buildCustomHandPresetRequest('Bad', '', [0, 0, 2, 0, 0])).toThrow();
  });

  test('builds preset metadata update and delete requests', () => {
    expect(buildHandPresetUpdateRequest(
      5,
      'Cup grasp',
      'For wide cups.',
      'Test with the blue cup.'
    )).toEqual({
      parameters: [
        { name: 'preset_id', value: { type: 2, integer_value: 5 } },
        { name: 'name', value: { type: 4, string_value: 'Cup grasp' } },
        { name: 'description', value: { type: 4, string_value: 'For wide cups.' } },
        { name: 'note', value: { type: 4, string_value: 'Test with the blue cup.' } },
      ],
    });
    expect(buildHandPresetDeleteRequest(5)).toEqual({
      parameters: [
        { name: 'preset_id', value: { type: 2, integer_value: 5 } },
      ],
    });
    expect(buildHandPresetRestoreRequest(5)).toEqual({
      parameters: [
        { name: 'preset_id', value: { type: 2, integer_value: 5 } },
      ],
    });
  });

  test('parses and normalizes ai_worker-advertised preset IDs', () => {
    expect(parseHandPresetStatus({
      data: JSON.stringify({
        preset_ids: [5, 1, 4, 1, 0],
        preview_joint_names: ['finger_r_joint1', 'finger_r_joint2'],
        presets: [
          {
            id: 1,
            name: 'Pinch Grasp',
            description: 'Thumb and index finger meet to pick up small objects.',
            visual: 'pinch',
            preview_open_positions: [0, 0],
            preview_positions: [0.6, 1.1],
          },
          {
            id: 4,
            name: 'Fist Grasp',
            description: 'Closes all fingers into the palm for a firm grip.',
            visual: 'fist',
          },
        ],
        enabled_sides: ['left', 'right'],
        left_preset_id: 5,
        right_preset_id: 1,
      }),
    })).toEqual({
      presetIds: [0, 1, 4, 5],
      presets: [
        {
          id: 0,
          name: 'Preset 0',
          description: '',
          note: '',
          visual: 'generic',
          custom: false,
          editable: true,
          deletable: false,
          deleteReason: '',
          editorCurls: [],
          editorCurlsExact: false,
          previewOpenPositions: [],
          previewPositions: [],
        },
        {
          id: 1,
          name: 'Pinch Grasp',
          description: 'Thumb and index finger meet to pick up small objects.',
          note: '',
          visual: 'pinch',
          custom: false,
          editable: true,
          deletable: false,
          deleteReason: '',
          editorCurls: [],
          editorCurlsExact: false,
          previewOpenPositions: [0, 0],
          previewPositions: [0.6, 1.1],
        },
        {
          id: 4,
          name: 'Fist Grasp',
          description: 'Closes all fingers into the palm for a firm grip.',
          note: '',
          visual: 'fist',
          custom: false,
          editable: true,
          deletable: false,
          deleteReason: '',
          editorCurls: [],
          editorCurlsExact: false,
          previewOpenPositions: [],
          previewPositions: [],
        },
        {
          id: 5,
          name: 'Preset 5',
          description: '',
          note: '',
          visual: 'generic',
          custom: false,
          editable: true,
          deletable: false,
          deleteReason: '',
          editorCurls: [],
          editorCurlsExact: false,
          previewOpenPositions: [],
          previewPositions: [],
        },
      ],
      previewJointNames: ['finger_r_joint1', 'finger_r_joint2'],
      customEditor: null,
      enabledSides: ['left', 'right'],
      leftPresetId: 5,
      rightPresetId: 1,
    });
  });

  test('sanitizes malformed metadata and keeps IDs authoritative', () => {
    const status = parseHandPresetStatus({
      data: JSON.stringify({
        preset_ids: [1],
        presets: [
          { id: 1, name: '  ', description: 123, visual: '' },
          { id: 9, name: 'Not registered', visual: 'fist' },
        ],
        enabled_sides: ['left'],
        left_preset_id: 1,
      }),
    });

    expect(status.presets).toEqual([
      {
        id: 1,
        name: 'Preset 1',
        description: '',
        note: '',
        visual: 'generic',
        custom: false,
        editable: true,
        deletable: false,
        deleteReason: '',
        editorCurls: [],
        editorCurlsExact: false,
        previewOpenPositions: [],
        previewPositions: [],
      },
    ]);
  });

  test('parses the ai_worker custom editor contract', () => {
    const jointNames = Array.from(
      { length: 20 },
      (_, index) => `finger_r_joint${index + 1}`
    );
    const open = Array(20).fill(0);
    const closed = Array(20).fill(1);
    const status = parseHandPresetStatus({
      data: JSON.stringify({
        preset_ids: [4],
        preview_joint_names: jointNames,
        presets: [{
          id: 4,
          name: 'Fist Grasp',
          preview_open_positions: open,
          preview_positions: closed,
        }],
        custom_editor: {
          control_labels: ['Thumb', 'Index', 'Middle', 'Ring', 'Little'],
          template_preset_id: 4,
          preview_open_positions: open,
          preview_closed_positions: closed,
        },
        enabled_sides: ['left', 'right'],
        left_preset_id: 4,
        right_preset_id: 4,
      }),
    });

    expect(status.customEditor).toEqual({
      controlLabels: ['Thumb', 'Index', 'Middle', 'Ring', 'Little'],
      templatePresetId: 4,
      previewOpenPositions: open,
      previewClosedPositions: closed,
    });
    expect(status.presets[0].previewPositions).toEqual(closed);
  });

  test('rejects malformed status instead of inventing preset IDs', () => {
    expect(parseHandPresetStatus({ data: 'not-json' })).toBeNull();
    expect(parseHandPresetStatus({ data: '{"preset_ids":[]}' })).toBeNull();
  });
});
