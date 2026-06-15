# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
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

"""Minimal type aliases extracted from ``lerobot.processor.core``.

The full ``lerobot.processor`` package pulls in heavy dependencies (e.g.
``torch``). The robot code vendored here only uses these two dictionary type
aliases, so they are reproduced verbatim to avoid the extra dependencies.
"""

from typing import Any

RobotAction = dict[str, Any]
RobotObservation = dict[str, Any]
