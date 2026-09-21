// Copyright 2026 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Seongwoo Kim

const runtimeConfig = (
  typeof window !== 'undefined' && window.__CYCLO_CONFIG__
) ? window.__CYCLO_CONFIG__ : {};

function getPort(name, defaultValue) {
  const port = Number(runtimeConfig[name]);
  return Number.isInteger(port) && port > 0 ? port : defaultValue;
}

export const CYCLO_UI_PORT = getPort('uiPort', 7080);
export const CYCLO_ROSBRIDGE_PORT = getPort('rosbridgePort', 7090);
export function normalizeRosbridgePath(value) {
  const path = typeof value === 'string' ? value.trim() : '';
  if (!/^\/([A-Za-z0-9_-]+\/)*[A-Za-z0-9_-]+\/?$/.test(path)) return '';
  return `${path.replace(/\/$/, '')}/`;
}

export const CYCLO_ROSBRIDGE_PATH = normalizeRosbridgePath(runtimeConfig.rosbridgePath);

/** Proxy profiles follow the current UI origin, including forwarded ports. */
export function buildRosbridgeUrl(host, location = (typeof window !== 'undefined' ? window.location : null)) {
  const protocol = location?.protocol === 'https:' ? 'wss:' : 'ws:';
  if (CYCLO_ROSBRIDGE_PATH) {
    return location?.host ? `${protocol}//${location.host}${CYCLO_ROSBRIDGE_PATH}` : '';
  }
  const hostname = typeof host === 'string' ? host.trim() : '';
  const authority = hostname.includes(':') && !hostname.startsWith('[') ? `[${hostname}]` : hostname;
  return authority ? `${protocol}//${authority}:${CYCLO_ROSBRIDGE_PORT}` : '';
}

export const CYCLO_VIDEO_SERVER_PORT = getPort('videoServerPort', 7082);
export const CYCLO_WEB_VIDEO_SERVER_PORT = getPort('webVideoServerPort', 7085);
export const CYCLO_CAMERA_PREVIEW_PORT = getPort('cameraPreviewPort', 7086);
export const CYCLO_SUPERVISOR_API_PORT = getPort('supervisorApiPort', 7100);
