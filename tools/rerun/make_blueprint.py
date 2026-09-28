#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Generates tools/rerun/default_blueprint.rbl, the viewer layout livox-mid360-rerun sends (#147).

Run from the repository root:  python3 tools/rerun/make_blueprint.py
Needs the Rerun Python SDK of the version the tool is built with (pip install rerun-sdk==0.38.1);
the output is committed and embedded at build time, so building the tool does not.

The layout: one 3D view of `lidar` with a fixed orbital eye, and a time panel on the `frame`
timeline that follows the newest data. Without a blueprint the 3D view's default eye tracks the
(smoothed) bounding box of the scene, which changes with every live frame, so the camera drifts.
The Rerun C++ SDK can log blueprint archetypes but cannot send the activation command that
makes the viewer use them; a .rbl saved by the Python SDK carries that command.
"""

from __future__ import annotations

import pathlib
import sys

import rerun.blueprint as rrb

APP_ID = 'livox_mid360'  # must match kApplicationId in viewer.cpp
OUT = pathlib.Path(__file__).resolve().parent / 'default_blueprint.rbl'


def main() -> int:
    blueprint = rrb.Blueprint(
        rrb.Spatial3DView(
            origin='lidar',
            name='lidar',
            eye_controls=rrb.EyeControls3D(
                kind=rrb.Eye3DKind.Orbital,
                position=[-9.0, -9.0, 7.0],
                look_target=[0.0, 0.0, 0.0],
                eye_up=[0.0, 0.0, 1.0],
            ),
        ),
        rrb.TimePanel(
            timeline='frame',
            play_state=rrb.components.PlayState.Following,
            loop_mode=rrb.components.LoopMode.Off,
        ),
        collapse_panels=False,
    )
    blueprint.save(APP_ID, OUT)
    print(f'wrote {OUT} ({OUT.stat().st_size} bytes)', file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
