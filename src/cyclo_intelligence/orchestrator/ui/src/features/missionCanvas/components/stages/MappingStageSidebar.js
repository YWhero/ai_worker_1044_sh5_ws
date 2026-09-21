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

import { useState } from "react";

import { TopicStatusPanel } from "../mapChrome";
import MappingSessionPanel from "../mapping/MappingSessionPanel";
import MappingTeleopPanel from "../mapping/MappingTeleopPanel";
import MappingUpperBodyJogPanel from "../mapping/MappingUpperBodyJogPanel";

export default function MappingStageSidebar({
  teleopDisabled,
  onPublishTeleop,
  onPublishRosTopic,
  onMessage,
  topicRows,
}) {
  const [mobileMotionActive, setMobileMotionActive] = useState(false);
  const [upperBodyBusy, setUpperBodyBusy] = useState(false);

  return (
    <>
      <MappingTeleopPanel
        disabled={teleopDisabled}
        motionBlocked={upperBodyBusy}
        onMotionActiveChange={setMobileMotionActive}
        onPublish={onPublishTeleop}
        onMessage={onMessage}
      />
      <MappingUpperBodyJogPanel
        disabled={teleopDisabled}
        mobileMotionActive={mobileMotionActive}
        onBusyChange={setUpperBodyBusy}
        onPublishRosTopic={onPublishRosTopic}
        onMessage={onMessage}
      />
      <MappingSessionPanel />
      <TopicStatusPanel topicRows={topicRows} />
    </>
  );
}
