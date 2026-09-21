const API_BASE = '/api/bt/pose-presets';

async function readJsonResponse(response) {
  const text = await response.text();
  if (!text) return {};
  try {
    return JSON.parse(text);
  } catch {
    return { detail: text };
  }
}

function detailMessage(data, fallback) {
  const detail = data?.detail;
  if (detail && typeof detail === 'object') return detail.message || fallback;
  return detail || data?.message || fallback;
}

async function request(path, init) {
  const response = await fetch(`${API_BASE}${path}`, init);
  const data = await readJsonResponse(response);
  if (!response.ok) {
    const error = new Error(detailMessage(data, `Request failed (${response.status})`));
    error.status = response.status;
    error.detail = data?.detail;
    throw error;
  }
  return data;
}

export function listJointPosePresets(robotType) {
  return request(`?robot_type=${encodeURIComponent(robotType)}`);
}

export function saveJointPosePreset({ preset, overwrite = false }) {
  return request('', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ ...preset, overwrite }),
  });
}

export function deleteJointPosePreset(robotType, name) {
  return request(
    `/${encodeURIComponent(name)}?robot_type=${encodeURIComponent(robotType)}`,
    { method: 'DELETE' },
  );
}
