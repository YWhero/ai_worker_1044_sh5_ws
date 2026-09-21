export const HAND_PRESET_SERVICE = '/leader/hand_preset/set';
export const HAND_PRESET_SERVICE_TYPE = 'rcl_interfaces/srv/SetParametersAtomically';
export const HAND_PRESET_CUSTOM_SAVE_SERVICE = '/leader/hand_preset/custom/save';
export const HAND_PRESET_UPDATE_SERVICE = '/leader/hand_preset/manage/update';
export const HAND_PRESET_DELETE_SERVICE = '/leader/hand_preset/manage/delete';
export const HAND_PRESET_RESTORE_SERVICE = '/leader/hand_preset/manage/restore';
export const HAND_PRESET_STATUS_TOPIC = '/leader/hand_preset/status';
export const HAND_PRESET_STATUS_TYPE = 'std_msgs/msg/String';

const PARAMETER_INTEGER = 2;
const PARAMETER_STRING = 4;
const PARAMETER_DOUBLE_ARRAY = 8;
const VALID_SIDES = new Set(['left', 'right', 'both']);

function cleanText(value, fallback, maxLength) {
  if (typeof value !== 'string') return fallback;
  const cleaned = value.trim();
  return cleaned ? cleaned.slice(0, maxLength) : fallback;
}

export function buildHandPresetRequest(side, presetId) {
  const normalizedSide = String(side || '').trim().toLowerCase();
  const numericPresetId = Number(presetId);

  if (!VALID_SIDES.has(normalizedSide)) {
    throw new Error('Hand preset side must be left, right, or both');
  }
  if (!Number.isInteger(numericPresetId) || numericPresetId < 0 || numericPresetId > 255) {
    throw new Error('Hand preset ID must be an integer from 0 to 255');
  }

  return {
    parameters: [
      {
        name: 'side',
        value: {
          type: PARAMETER_STRING,
          string_value: normalizedSide,
        },
      },
      {
        name: 'preset_id',
        value: {
          type: PARAMETER_INTEGER,
          integer_value: numericPresetId,
        },
      },
    ],
  };
}

export function buildCustomHandPresetRequest(name, description, curls) {
  const normalizedName = String(name || '').trim();
  const normalizedDescription = String(description || '').trim();
  const normalizedCurls = Array.isArray(curls) ? curls.map(Number) : [];

  if (!normalizedName || normalizedName.length > 48) {
    throw new Error('Custom preset name must contain 1 to 48 characters');
  }
  if (normalizedDescription.length > 160) {
    throw new Error('Custom preset description must be 160 characters or fewer');
  }
  if (
    normalizedCurls.length !== 5
    || normalizedCurls.some((value) => !Number.isFinite(value) || value < 0 || value > 1)
  ) {
    throw new Error('Custom preset requires five finger curls from 0 to 1');
  }

  return {
    parameters: [
      {
        name: 'name',
        value: { type: PARAMETER_STRING, string_value: normalizedName },
      },
      {
        name: 'description',
        value: { type: PARAMETER_STRING, string_value: normalizedDescription },
      },
      {
        name: 'curls',
        value: { type: PARAMETER_DOUBLE_ARRAY, double_array_value: normalizedCurls },
      },
    ],
  };
}

export function buildHandPresetUpdateRequest(
  presetId,
  name,
  description,
  note
) {
  const numericPresetId = Number(presetId);
  const normalizedName = String(name || '').trim();
  const normalizedDescription = String(description || '').trim();
  const normalizedNote = String(note || '').trim();

  if (!Number.isInteger(numericPresetId) || numericPresetId < 0 || numericPresetId > 255) {
    throw new Error('Hand preset ID must be an integer from 0 to 255');
  }
  if (!normalizedName || normalizedName.length > 48) {
    throw new Error('Preset name must contain 1 to 48 characters');
  }
  if (normalizedDescription.length > 160) {
    throw new Error('Preset description must be 160 characters or fewer');
  }
  if (normalizedNote.length > 240) {
    throw new Error('Preset note must be 240 characters or fewer');
  }

  return {
    parameters: [
      {
        name: 'preset_id',
        value: { type: PARAMETER_INTEGER, integer_value: numericPresetId },
      },
      {
        name: 'name',
        value: { type: PARAMETER_STRING, string_value: normalizedName },
      },
      {
        name: 'description',
        value: { type: PARAMETER_STRING, string_value: normalizedDescription },
      },
      {
        name: 'note',
        value: { type: PARAMETER_STRING, string_value: normalizedNote },
      },
    ],
  };
}

export function buildHandPresetDeleteRequest(presetId) {
  const numericPresetId = Number(presetId);
  if (!Number.isInteger(numericPresetId) || numericPresetId < 0 || numericPresetId > 255) {
    throw new Error('Hand preset ID must be an integer from 0 to 255');
  }
  return {
    parameters: [
      {
        name: 'preset_id',
        value: { type: PARAMETER_INTEGER, integer_value: numericPresetId },
      },
    ],
  };
}

export function buildHandPresetRestoreRequest(presetId) {
  return buildHandPresetDeleteRequest(presetId);
}

function cleanNumberArray(value, expectedLength) {
  if (!Array.isArray(value) || value.length !== expectedLength) return [];
  const numbers = value.map(Number);
  return numbers.every(Number.isFinite) ? numbers : [];
}

function deriveEditorCurls(openPositions, closedPositions, targetPositions) {
  if (
    !openPositions.length
    || openPositions.length !== closedPositions.length
    || openPositions.length !== targetPositions.length
    || openPositions.length % 5 !== 0
  ) return [];
  const jointsPerFinger = openPositions.length / 5;
  return Array.from({ length: 5 }, (_, fingerIndex) => {
    const start = fingerIndex * jointsPerFinger;
    let numerator = 0;
    let denominator = 0;
    for (let index = start; index < start + jointsPerFinger; index += 1) {
      const templateDelta = closedPositions[index] - openPositions[index];
      numerator += (targetPositions[index] - openPositions[index]) * templateDelta;
      denominator += templateDelta * templateDelta;
    }
    if (denominator < 1e-12) return 0;
    return Math.max(0, Math.min(1, numerator / denominator));
  });
}

export function parseHandPresetStatus(message) {
  let payload;
  try {
    payload = JSON.parse(String(message?.data || ''));
  } catch (_error) {
    return null;
  }

  if (!payload || !Array.isArray(payload.preset_ids)) return null;

  const presetIds = [...new Set(payload.preset_ids
    .map(Number)
    .filter((value) => Number.isInteger(value) && value >= 0 && value <= 255))]
    .sort((left, right) => left - right);
  const enabledSides = Array.isArray(payload.enabled_sides)
    ? payload.enabled_sides.filter((side) => side === 'left' || side === 'right')
    : [];

  if (!presetIds.length || !enabledSides.length) return null;

  const activePresetId = (side) => {
    const value = Number(payload[`${side}_preset_id`]);
    return Number.isInteger(value) ? value : null;
  };
  const previewJointNames = Array.isArray(payload.preview_joint_names)
    ? payload.preview_joint_names
      .filter((name) => typeof name === 'string' && name.trim())
      .map((name) => name.trim())
      .slice(0, 64)
    : [];
  const metadataById = new Map();
  if (Array.isArray(payload.presets)) {
    payload.presets.forEach((preset) => {
      const id = Number(preset?.id);
      if (Number.isInteger(id) && presetIds.includes(id)) {
        metadataById.set(id, preset);
      }
    });
  }
  const presets = presetIds.map((id) => {
    const metadata = metadataById.get(id);
    return {
      id,
      name: cleanText(metadata?.name, `Preset ${id}`, 48),
      description: cleanText(metadata?.description, '', 160),
      note: cleanText(metadata?.note, '', 240),
      visual: cleanText(metadata?.visual, 'generic', 32).toLowerCase(),
      custom: metadata?.custom === true,
      editable: metadata?.editable !== false,
      deletable: metadata?.deletable === true,
      deleteReason: cleanText(metadata?.delete_reason, '', 200),
      editorCurls: cleanNumberArray(metadata?.editor_curls, 5),
      editorCurlsExact: metadata?.editor_curls_exact === true,
      previewOpenPositions: cleanNumberArray(
        metadata?.preview_open_positions,
        previewJointNames.length
      ),
      previewPositions: cleanNumberArray(
        metadata?.preview_positions,
        previewJointNames.length
      ),
    };
  });
  const editor = payload.custom_editor;
  const controlLabels = Array.isArray(editor?.control_labels)
    ? editor.control_labels
      .filter((label) => typeof label === 'string' && label.trim())
      .map((label) => label.trim().slice(0, 24))
      .slice(0, 5)
    : [];
  const editorOpenPositions = cleanNumberArray(
    editor?.preview_open_positions,
    previewJointNames.length
  );
  const editorClosedPositions = cleanNumberArray(
    editor?.preview_closed_positions,
    previewJointNames.length
  );
  const customEditor = (
    controlLabels.length === 5
    && previewJointNames.length > 0
    && previewJointNames.length % controlLabels.length === 0
    && editorOpenPositions.length === previewJointNames.length
    && editorClosedPositions.length === previewJointNames.length
  ) ? {
      controlLabels,
      templatePresetId: Number(editor.template_preset_id),
      previewOpenPositions: editorOpenPositions,
      previewClosedPositions: editorClosedPositions,
    } : null;

  const presetsWithEditorCurls = presets.map((preset) => ({
    ...preset,
    editorCurls: preset.editorCurls.length === 5
      ? preset.editorCurls
      : deriveEditorCurls(
        editorOpenPositions,
        editorClosedPositions,
        preset.previewPositions
      ),
    editorCurlsExact: preset.editorCurlsExact || preset.custom,
  }));

  return {
    presetIds,
    presets: presetsWithEditorCurls,
    previewJointNames,
    customEditor,
    enabledSides,
    leftPresetId: activePresetId('left'),
    rightPresetId: activePresetId('right'),
  };
}
