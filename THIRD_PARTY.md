# Third-party software and models

AXIS-OS itself is licensed under the Apache License 2.0 (see `LICENSE`).
This page lists what AXIS-OS relies on, under which license, and whether it
is part of this repository. It is a summary, not legal advice: the license
published by each project is the one that applies.

**No model weights are stored in this repository**, and none are
redistributed by it. Weights are downloaded by the user, on their own
machine, under the terms of their authors.

## 1. Included in this repository

| Material | Location | Origin | License |
|----------|----------|--------|---------|
| Unitree G1 URDF files (one modified copy adds hand collisions) | `axis_os/hal/drivers/unitree_g1/urdf/` | [unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros) | BSD 3-Clause, `LICENSE_UNITREE_ROS.txt` |
| `vla.proto` (VLA serving protocol) | `axis_os/store/sdk/ai/proto/` | [VinRobotics/vla.cpp](https://github.com/VinRobotics/vla.cpp) | Apache 2.0 |
| CRC32 routine for Unitree DDS messages | `deploy/freshness_monitor/freshness_monitor.c` | [unitreerobotics/unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) | BSD 3-Clause, `deploy/freshness_monitor/LICENSE_UNITREE_SDK2.txt` |
| `LowCmd_` / `MotorCmd_` message layout | `deploy/freshness_monitor/idl/` | reconstructed from unitree_sdk2 and unitree_sdk2_python | BSD 3-Clause (Unitree) |

The required notices are reproduced in `NOTICE`.

## 2. Python dependencies (installed with pip, not included)

The core of the OS (HAL, safety guard, watchdog, sandbox) uses the Python
standard library only. The packages below are needed by optional features
or by the test suite. Licenses are taken from each package's metadata.

| Package | Used for | License |
|---------|----------|---------|
| numpy | perception, locomotion, tests | BSD 3-Clause |
| psutil | CPU / memory quotas of the app sandbox | BSD 3-Clause |
| cryptography | app signatures (Ed25519) | Apache 2.0 or BSD 3-Clause |
| pyhpke | app encryption (HPKE, RFC 9180) | MIT |
| pyzmq | VLA connectors (ZeroMQ) | BSD 3-Clause |
| protobuf | `VLACppConnector` | BSD 3-Clause |
| msgpack, msgpack-numpy | GR00T connector | Apache 2.0 / BSD 3-Clause |
| transformers | OWL-ViT detector, Whisper recognizer | Apache 2.0 |
| pillow | images for perception and tests | MIT-CMU (HPND) |
| onnxruntime | GR00T Whole-Body Control policy | MIT |
| torch | neural policies, perception | BSD 3-Clause |
| pandas, pyarrow, imageio, imageio-ffmpeg | recording VLA episodes (LeRobot format) | BSD 3-Clause / Apache 2.0 / BSD 2-Clause / BSD 2-Clause |
| unitree_sdk2py, cyclonedds | driving a real Unitree G1 over DDS | BSD 3-Clause / EPL 2.0 or BSD 3-Clause |

All of these licenses allow use from an Apache 2.0 project. Note that
`imageio-ffmpeg` downloads an FFmpeg binary, which has its own license
(LGPL or GPL depending on the build).

## 3. Models (downloaded at run time, never included)

| Model | Used by | License, as published by its authors |
|-------|---------|--------------------------------------|
| [google/owlvit-base-patch32](https://huggingface.co/google/owlvit-base-patch32), revision pinned | open-vocabulary object detection (`owlvit_detector.py`) | Apache 2.0 |
| [openai/whisper-small](https://huggingface.co/openai/whisper-small) | speech recognition (`whisper_recognizer.py`) | Apache 2.0 |
| Unitree locomotion policy (`g1_motion.pt`), from [unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym) | walking in simulation | BSD 3-Clause |
| GR00T Whole-Body Control policy, from [NVlabs/GR00T-WholeBodyControl](https://github.com/NVlabs/GR00T-WholeBodyControl) | balance and walking with load (29 DOF G1) | code Apache 2.0; **weights under the NVIDIA Open Model License** |
| [nvidia/GR00T-N1.7-3B](https://huggingface.co/nvidia/GR00T-N1.7-3B), with [NVIDIA/Isaac-GR00T](https://github.com/NVIDIA/Isaac-GR00T) | vision-language-action model, fine-tuned on simulated demonstrations | code Apache 2.0; **weights under an NVIDIA license**, see the model page |
| [nvidia/Cosmos-Reason2-2B](https://huggingface.co/nvidia/Cosmos-Reason2-2B) | required by the GR00T fine-tuning script | **NVIDIA license, to be accepted on Hugging Face before download** |
| [facebook/mms-tts-fra](https://huggingface.co/facebook/mms-tts-fra) | voice of one demo video script only (`video_v2_voix.py`) | **CC BY-NC 4.0: non-commercial use only** |

The models in bold are not under an open-source license. AXIS-OS does not
need them to run, and its test suite runs without any of them. Anyone who
uses them, or a model fine-tuned from them, must follow their terms; this is
in particular the case for a commercial deployment.

## 4. Simulation and runtime software (not included)

| Software | Used for | License |
|----------|----------|---------|
| NVIDIA Isaac Sim 6.1 (container image) | physics simulation of the G1 | NVIDIA proprietary license |
| [unitree_sim_isaaclab](https://github.com/unitreerobotics/unitree_sim_isaaclab) | reference for the Isaac Lab setup (documentation only) | Apache 2.0 |
| Docker | running Isaac Sim on the GPU machine | Apache 2.0 (Docker Engine) |
