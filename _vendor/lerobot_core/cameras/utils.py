#!/usr/bin/env python

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from .camera import Camera
from .configs import CameraConfig


def make_cameras_from_configs(camera_configs: dict[str, CameraConfig]) -> dict[str, Camera]:
    """Instantiate cameras from their configs.

    The SO follower used by lerobot_sdk runs without cameras (an empty config
    dict), so this returns an empty mapping in that case. Camera backends are
    not bundled with the vendored code; configuring one raises a clear error.
    """
    cameras: dict[str, Camera] = {}

    for key, cfg in camera_configs.items():
        raise NotImplementedError(
            f"Camera '{key}' (type '{cfg.type}') is not bundled with lerobot_sdk. "
            "The SO-100/SO-101 follower SDK is used without cameras; install and use "
            "the full `lerobot` package if you need camera support."
        )

    return cameras
