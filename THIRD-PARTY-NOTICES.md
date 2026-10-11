# Third-party notices

This project (`r1_robot`) is licensed under the MIT License (see `LICENSE`).
It vendors and links third-party components that carry their own licenses and
conditions. This file records them and reproduces the required notices.

---

## Unitree R1-7A SDK — `third_party/r1-7a_sdk`

* Source: <https://github.com/unitreerobotics/r1-7a_sdk>
* Pinned commit: `480cfc8c031aaeccddf562b4d789d46b64c6c543` (VERSION 0.1.0)
* License: **BSD-3-Clause**
* Copyright holder: HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics")

The SDK is consumed as a **read-only git submodule**. `r1_robot` never edits the
checkout in place; the two files it compiles are copied into the build tree and a
minimal, documented patch (`patches/0001-arm_motor_bus-feedback-stats.patch`) is
applied there. The pinned submodule is verified byte-clean at build time by the
`r1_sdk_cleanliness_check` target.

Full text of the upstream `LICENSE` (`third_party/r1-7a_sdk/LICENSE`):

```
BSD 3-Clause License

Copyright (c) 2026 HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics")

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

### Vendored `motor_controller` and design assets — license status

* `third_party/r1-7a_sdk/thirdparty/motor_controller/**` ships a prebuilt
  `libmotor_controller.so` and sources with **no license header of their own**.
  It is treated as covered by the SDK-root BSD-3-Clause above, pending written
  confirmation from Unitree Robotics. **Status: NOASSERTION.**
* The URDF / STEP / STL design assets under `third_party/r1-7a_sdk/urdf/` and
  `third_party/r1-7a_sdk/step/` carry only an unqualified `<license>BSD</license>`
  in `package.xml`. Their exact redistribution terms are not spelled out.
  **Status: NOASSERTION — confirm with Unitree before redistributing derivatives
  of the model geometry.**

---

## Python / build dependencies (not vendored)

These are external packages installed at build/run time; they are not redistributed
here. Their licenses apply to your environment:

| Package | License |
|---|---|
| `numpy` | BSD-3-Clause |
| `mujoco` | Apache-2.0 |
| `pinocchio` (`libpinocchio`) | BSD-2-Clause / CeCILL-C |
| `pytest` | MIT |
| CMake, GCC, `patch` | BSD / GPL / — (toolchain) |

Verify the exact versions and licenses in your environment with
`pip show <pkg>` before redistribution.

## Vendored CAD meshes under `sim/meshes/`

`sim/build_gripper_model.py` copies the STL files referenced by the two read-only
URDF sources into `sim/meshes/` so the generated model loads with relative paths
on any machine.  These are **redistributed binaries**, so their provenance is
recorded here.

| Family | Source | License | Status |
|---|---|---|---|
| `Link*_*.stl`, R1 arm meshes | the vendored Unitree SDK submodule (`third_party/r1-7a_sdk`, `LICENSE` present, BSD-3-Clause per that file) | BSD-3-Clause | attribution reproduced above |
| `*_finger*_*.stl`, gripper base / DM4310 meshes | reBot DevArm description package (`Rebot_Arm_description/DM/meshes/...`) | **CERN-OHL-W v2** (weakly reciprocal hardware licence) | `NOASSERTION` for software redistribution |

The CERN-OHL-W grant is written for hardware designs and their associated
documentation; it is not a standard OSI software licence, and applying it to STL
files compiled into a simulator is not something I have verified with anyone
competent to say.  Two practical notes before this leaves the research machine:

* the source tree is unmodified and remains the reference; the staged copies are
  regenerable, and `git rm -r sim/meshes` restores the previous state;
* if the arm or gripper is ever shipped or offered commercially, ask whether
  CERN-OHL-W requires published design source for the derived physical hardware.
  Nothing in this repository builds hardware.

`sim/meshes/manifest.json` (written by the generator) records each staged file's
original absolute path and SHA-256, so a review can prove which CAD a given
binary came from.
