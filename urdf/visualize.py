"""Visualize the aloha4franka URDF in viser.

Usage:
  uv run --with viser --with yourdfpy --with numpy python robots/aloha4franka/visualize.py
"""
from __future__ import annotations

import time
from pathlib import Path

import numpy as np
import yourdfpy

import viser
from viser.extras import ViserUrdf

URDF_PATH = Path(__file__).parent / "robot.urdf"


def main() -> None:
    server = viser.ViserServer()
    server.initial_camera.position = (0.3, 0.3, 0.3)

    assets_dir = URDF_PATH.parent

    def filename_handler(fname: str) -> str:
        if fname.startswith("package://"):
            fname = fname[len("package://"):]
        return str(assets_dir / fname)

    urdf = yourdfpy.URDF.load(
        str(URDF_PATH),
        load_meshes=True,
        build_scene_graph=True,
        load_collision_meshes=False,
        filename_handler=filename_handler,
    )
    viser_urdf = ViserUrdf(
        server,
        urdf_or_path=urdf,
    )

    # TF tree: add a frame for each link, updated whenever the cfg changes
    link_names = list(urdf.link_map.keys())
    frame_handles: dict[str, viser.FrameHandle] = {}

    def _rotmat_to_wxyz(R: np.ndarray) -> np.ndarray:
        # Robust conversion of a 3x3 rotation matrix to a wxyz quaternion
        tr = R[0, 0] + R[1, 1] + R[2, 2]
        if tr > 0:
            s = 2.0 * np.sqrt(1.0 + tr)
            w = 0.25 * s
            x = (R[2, 1] - R[1, 2]) / s
            y = (R[0, 2] - R[2, 0]) / s
            z = (R[1, 0] - R[0, 1]) / s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        return np.array([w, x, y, z])

    def update_tf_tree() -> None:
        for name in link_names:
            T = urdf.get_transform(name)
            position = T[:3, 3]
            wxyz = _rotmat_to_wxyz(T[:3, :3])
            if name in frame_handles:
                frame_handles[name].position = position
                frame_handles[name].wxyz = wxyz
            else:
                frame_handles[name] = server.scene.add_frame(
                    f"/tf/{name}",
                    position=position,
                    wxyz=wxyz,
                    axes_length=0.03,
                    axes_radius=0.0015,
                    origin_radius=0.004,
                )

    joint_limits = viser_urdf.get_actuated_joint_limits()
    slider_handles: list[viser.GuiInputHandle[float]] = []
    initial_config: list[float] = []

    def on_joint_change() -> None:
        cfg = np.array([s.value for s in slider_handles])
        viser_urdf.update_cfg(cfg)
        update_tf_tree()

    with server.gui.add_folder("Joint control"):
        for joint_name, (lower, upper) in joint_limits.items():
            lower = lower if lower is not None else -np.pi
            upper = upper if upper is not None else np.pi
            initial_pos = 0.0 if lower < -0.1 and upper > 0.1 else (lower + upper) / 2.0
            slider = server.gui.add_slider(
                label=joint_name,
                min=lower,
                max=upper,
                step=1e-4,
                initial_value=initial_pos,
            )
            slider.on_update(lambda _: on_joint_change())
            slider_handles.append(slider)
            initial_config.append(initial_pos)

    with server.gui.add_folder("Visibility"):
        tf_checkbox = server.gui.add_checkbox("Show TF tree", True)

        @tf_checkbox.on_update
        def _(_):
            for h in frame_handles.values():
                h.visible = tf_checkbox.value

    viser_urdf.update_cfg(np.array(initial_config))
    update_tf_tree()

    server.scene.add_grid("/grid", width=0.5, height=0.5, position=(0.0, 0.0, 0.0))

    reset_button = server.gui.add_button("Reset")

    @reset_button.on_click
    def _(_):
        for s, init_q in zip(slider_handles, initial_config):
            s.value = init_q

    while True:
        time.sleep(10.0)


if __name__ == "__main__":
    main()
