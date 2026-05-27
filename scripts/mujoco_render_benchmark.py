import argparse
import json
import os
import platform
import statistics
import subprocess
import sys
import time
from pathlib import Path

import mujoco

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mujoco_env.y_env2 import SimpleEnv2


XML_PATH = "./asset/example_scene_y2.xml"


def percentile(values, pct):
    if not values:
        return 0.0
    values = sorted(values)
    idx = min(len(values) - 1, round((len(values) - 1) * pct / 100.0))
    return values[idx]


def summarize_ms(values):
    if not values:
        return {"avg": 0.0, "p50": 0.0, "p95": 0.0, "max": 0.0}
    values_ms = [v * 1000.0 for v in values]
    return {
        "avg": statistics.fmean(values_ms),
        "p50": percentile(values_ms, 50),
        "p95": percentile(values_ms, 95),
        "max": max(values_ms),
    }


def run_cmd(cmd):
    try:
        return subprocess.check_output(cmd, text=True, stderr=subprocess.STDOUT).strip()
    except Exception as exc:
        return f"unavailable: {exc}"


def collect_machine_info():
    return {
        "python": platform.python_version(),
        "platform": platform.platform(),
        "processor": platform.processor(),
        "mujoco": mujoco.__version__,
        "mujoco_gl": os.environ.get("MUJOCO_GL", ""),
        "display": os.environ.get("DISPLAY", ""),
        "swap_interval": os.environ.get("MUJOCO_BENCH_SWAP_INTERVAL", "1"),
        "cpu": run_cmd(["bash", "-lc", "lscpu | sed -n '1,18p'"]),
        "gpu": run_cmd(["bash", "-lc", "nvidia-smi --query-gpu=name,driver_version,utilization.gpu --format=csv,noheader"]),
    }


def benchmark(args):
    needs_viewer = args.mode in {"camera", "viewer_fast", "viewer_full"}
    env = SimpleEnv2(XML_PATH, action_type="joint_angle", seed=0, initialize_viewer=needs_viewer)
    renderer = None
    if args.mode == "renderer":
        renderer = mujoco.Renderer(env.env.model, height=args.camera_height, width=args.camera_width)

    if needs_viewer and args.warmup_viewer_frames > 0:
        for _ in range(args.warmup_viewer_frames):
            env.env.render()

    timings = {
        "physics": [],
        "camera": [],
        "viewer": [],
        "total_loop": [],
    }

    start_wall = time.perf_counter()
    sim_dt = env.env.dt
    sim_steps = 0

    for i in range(args.loops):
        loop_start = time.perf_counter()

        t0 = time.perf_counter()
        env.step_env(nstep=args.physics_steps)
        timings["physics"].append(time.perf_counter() - t0)
        sim_steps += args.physics_steps

        if args.mode in {"camera", "viewer_full", "renderer"} and i % args.camera_every == 0:
            t0 = time.perf_counter()
            if renderer is not None:
                renderer.update_scene(env.env.data, camera="agentview")
                renderer.render()
                renderer.update_scene(env.env.data, camera="egocentric")
                renderer.render()
                if args.include_side_camera:
                    renderer.update_scene(env.env.data, camera="sideview")
                    renderer.render()
            elif args.include_side_camera:
                env.grab_image(include_side=True)
            else:
                env.grab_image_fast()
            timings["camera"].append(time.perf_counter() - t0)

        if args.mode == "viewer_fast":
            t0 = time.perf_counter()
            env.render(fast=True)
            timings["viewer"].append(time.perf_counter() - t0)
        elif args.mode == "viewer_full":
            t0 = time.perf_counter()
            env.render(fast=False, show_side_view=args.include_side_camera)
            timings["viewer"].append(time.perf_counter() - t0)

        timings["total_loop"].append(time.perf_counter() - loop_start)

    wall_time = time.perf_counter() - start_wall
    sim_time = sim_steps * sim_dt

    if renderer is not None:
        renderer.close()
    if needs_viewer:
        env.env.close_viewer()

    result = {
        "mode": args.mode,
        "loops": args.loops,
        "physics_steps_per_loop": args.physics_steps,
        "camera_every": args.camera_every,
        "include_side_camera": args.include_side_camera,
        "wall_time_s": wall_time,
        "sim_time_s": sim_time,
        "real_time_factor": sim_time / wall_time if wall_time > 0 else 0.0,
        "loop_hz": args.loops / wall_time if wall_time > 0 else 0.0,
        "physics_ms": summarize_ms(timings["physics"]),
        "camera_ms": summarize_ms(timings["camera"]),
        "viewer_ms": summarize_ms(timings["viewer"]),
        "total_loop_ms": summarize_ms(timings["total_loop"]),
        "machine": collect_machine_info(),
    }
    return result


def main():
    parser = argparse.ArgumentParser(description="Benchmark the real MuJoCo render paths used by model_sim.py.")
    parser.add_argument(
        "--mode",
        choices=["none", "camera", "renderer", "viewer_fast", "viewer_full"],
        default="none",
        help=(
            "none: physics only; camera: viewer-context fixed camera render/readback; "
            "renderer: mujoco.Renderer fixed camera render/readback; "
            "viewer_fast: viewer render without overlays; viewer_full: model_sim-like camera overlays + viewer render"
        ),
    )
    parser.add_argument("--loops", type=int, default=300)
    parser.add_argument("--physics-steps", type=int, default=8, help="8 steps ~= one 60 Hz frame for dt=0.002.")
    parser.add_argument("--camera-every", type=int, default=20, help="Render policy cameras every N loops.")
    parser.add_argument("--include-side-camera", action="store_true")
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--warmup-viewer-frames", type=int, default=15)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()

    result = benchmark(args)
    text = json.dumps(result, indent=2, ensure_ascii=False)
    print(text)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
