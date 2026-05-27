# MuJoCo Viewer / Camera 渲染瓶颈排查指南

## 目标

这里的目标不是直接优化 `model_sim.py`，而是先把慢拆开：

- MuJoCo physics 是否慢。
- 固定相机渲染和 `mjr_readPixels` 是否慢。
- 自定义 GLFW viewer 的 `render()` / `swap_buffers()` 是否慢。
- `model_sim.py` 的 overlay、三路相机、policy 推理是否把主循环拖慢。

当前项目的真实情况：

- 入口是 `model_sim.py`。
- 环境封装是 `mujoco_env.y_env2.SimpleEnv2`。
- viewer 是 `mujoco_env.mujoco_parser.MuJoCoMinimalViewer`，不是官方 `mujoco.viewer.launch_passive()`。
- viewer 初始化里默认 `glfw.swap_interval(1)`，也就是开启 VSync；远程桌面、虚拟显示、驱动异常或 CPU 显示链路都会让 `render()` 记录到的 wall time 变慢。
- 固定相机 `grab_image_fast()` 依赖同一个 viewer OpenGL context，因此本项目里的 camera benchmark 不是纯 headless EGL，而是“viewer GL context 下的 fixed camera render/readback”。

## 先确认运行环境

所有命令使用项目当前 conda 环境：

```bash
conda activate lerobot-mujoco-manipulation
```

确认 Python、MuJoCo、CPU、GPU、显示和 OpenGL 相关变量：

```bash
python -c "import mujoco, sys, os; print(sys.executable); print('mujoco', mujoco.__version__); print('DISPLAY=', os.environ.get('DISPLAY')); print('MUJOCO_GL=', os.environ.get('MUJOCO_GL'))"
lscpu | sed -n '1,18p'
nvidia-smi
```

如果 `nvidia-smi` 报 `couldn't communicate with the NVIDIA driver`，当前机器的 NVIDIA 驱动/容器透传不可用；viewer 和 offscreen render 很可能走 CPU/软件栈或远程显示链路。

如果安装了 `glxinfo`，继续确认 OpenGL renderer：

```bash
DISPLAY=:0 glxinfo -B
```

如果 `glxinfo` 不存在，可以先跳过；关键 benchmark 仍然能给出瓶颈位置。

## 实际 Benchmark 命令

脚本：

```bash
python scripts/mujoco_render_benchmark.py --help
```

### 1. Physics only

不创建 viewer，只测 MuJoCo step：

```bash
python scripts/mujoco_render_benchmark.py \
  --mode none \
  --loops 500 \
  --physics-steps 8 \
  --output outputs/benchmarks/mujoco_render/none.json
```

判读：

- `real_time_factor > 1`：physics 本身不是主瓶颈。
- `real_time_factor < 1` 且 `physics_ms.avg` 高：优先查 CPU、碰撞体、接触、solver、Python loop。

### 2. Fixed camera render/readback

创建 viewer GL context，但不显示主 viewer frame，只测 policy 使用的两路固定相机：

```bash
python scripts/mujoco_render_benchmark.py \
  --mode camera \
  --loops 300 \
  --physics-steps 8 \
  --camera-every 1 \
  --output outputs/benchmarks/mujoco_render/camera.json
```

加上 side camera，模拟 `model_sim.py` overlay 刷新时的三路相机：

```bash
python scripts/mujoco_render_benchmark.py \
  --mode camera \
  --loops 300 \
  --physics-steps 8 \
  --camera-every 1 \
  --include-side-camera \
  --output outputs/benchmarks/mujoco_render/camera_side.json
```

判读：

- `none` 很快，但 `camera_ms.avg` 高：瓶颈在 camera render / GPU / OpenGL readback / 分辨率。
- 两路相机明显快于三路相机：`model_sim.py` 里的 side overlay 应该降频或关闭。

### 2b. EGL Renderer fixed camera render/readback

不打开 GLFW viewer，使用 `mujoco.Renderer` 和 EGL 测固定相机：

```bash
MUJOCO_GL=egl python scripts/mujoco_render_benchmark.py \
  --mode renderer \
  --loops 300 \
  --physics-steps 8 \
  --camera-every 1 \
  --output outputs/benchmarks/mujoco_render/renderer_egl.json
```

判读：

- `camera` 慢但 `renderer` 快：当前慢点在 GLFW/X11 viewer context，不在 MuJoCo 渲染能力本身。
- `renderer` 也慢：继续查 GPU/EGL/驱动/软件渲染。

### 3. Viewer basic render

只测主 viewer render，不画 overlay 图像：

```bash
python scripts/mujoco_render_benchmark.py \
  --mode viewer_fast \
  --loops 300 \
  --physics-steps 8 \
  --output outputs/benchmarks/mujoco_render/viewer_fast_vsync_on.json
```

关闭 VSync 再测一次：

```bash
MUJOCO_BENCH_SWAP_INTERVAL=0 python scripts/mujoco_render_benchmark.py \
  --mode viewer_fast \
  --loops 300 \
  --physics-steps 8 \
  --output outputs/benchmarks/mujoco_render/viewer_fast_vsync_off.json
```

判读：

- VSync 开启时 `viewer_ms.avg` 接近 16.6 ms、33.3 ms 或更高，关闭后明显下降：瓶颈在 `glfw.swap_buffers()` / 显示同步链路，不是 physics。
- 关闭 VSync 后仍慢：查 GPU/驱动/远程桌面/OpenGL software rendering。

### 4. model_sim-like viewer full render

模拟 `model_sim.py` 的相机 overlay + viewer render：

```bash
python scripts/mujoco_render_benchmark.py \
  --mode viewer_full \
  --loops 300 \
  --physics-steps 8 \
  --camera-every 20 \
  --include-side-camera \
  --output outputs/benchmarks/mujoco_render/viewer_full.json
```

关闭 VSync 对比：

```bash
MUJOCO_BENCH_SWAP_INTERVAL=0 python scripts/mujoco_render_benchmark.py \
  --mode viewer_full \
  --loops 300 \
  --physics-steps 8 \
  --camera-every 20 \
  --include-side-camera \
  --output outputs/benchmarks/mujoco_render/viewer_full_vsync_off.json
```

判读：

- `viewer_fast` 快、`viewer_full` 慢：overlay、固定相机、图像拷贝是主瓶颈。
- `viewer_fast` 和 `viewer_full` 都慢：viewer/display/VSync/OpenGL 是主瓶颈。
- `camera` 慢而 `viewer_fast` 不慢：固定相机 render/readback 是主瓶颈。

## 对照 `model_sim.py`

`model_sim.py` 当前主循环特征：

- `RENDER_HZ = 60`，每帧都会 `PnPEnv.render(fast=False, show_side_view=True, idx=episode_idx)`。
- 每帧 physics 约 `round((1 / 60) / 0.002) = 8` step。
- policy 约 3 Hz，会调用 `grab_image_fast()` 渲染两路相机。
- overlay 约 3 Hz，会调用 `grab_image(include_side=True)` 渲染三路相机并做标题图像。
- viewer 默认 VSync，因此 render wall time 可能包含显示等待时间。

如果 benchmark 证明 viewer 是瓶颈，优先临时验证：

```bash
MUJOCO_BENCH_SWAP_INTERVAL=0 python model_sim.py
```

如果关闭 VSync 后 `model_sim.py` 明显变快，说明慢主要来自显示同步链路。  
如果关闭 VSync 仍然慢，说明不只是 `swap_interval(1)`，而是当前 OpenGL/window render 或远程显示路径整体慢。后续可以考虑：

- debug/teleop 降到 `RENDER_HZ = 20` 或 30。
- `PnPEnv.render(fast=True)` 做纯 viewer 验证。
- side overlay 默认关闭或降频。
- viewer 只做人看，采集/评测默认不打开 viewer。

当前修复后的推荐运行方式：

```bash
# 真实策略运行/计时：不打开 viewer，policy 图像走 EGL Renderer 快路径
MODEL_SIM_VIEWER=0 MUJOCO_GL=egl python model_sim.py

# 只想看 viewer：降低 viewer 负担
MODEL_SIM_RENDER_HZ=5 MODEL_SIM_RENDER_FAST=1 MODEL_SIM_SHOW_SIDE_VIEW=0 python model_sim.py
```

可调环境变量：

```text
MODEL_SIM_VIEWER=0/1
MODEL_SIM_RENDER_HZ=20
MODEL_SIM_POLICY_HZ=3
MODEL_SIM_OVERLAY_HZ=1
MODEL_SIM_RENDER_FAST=0/1
MODEL_SIM_SHOW_SIDE_VIEW=0/1
MODEL_SIM_CAMERA_WIDTH=640
MODEL_SIM_CAMERA_HEIGHT=480
MODEL_SIM_HEADLESS_MAX_FRAMES=0
```

## 本机初步排查结果

在当前机器、`lerobot-mujoco-manipulation` 环境、`DISPLAY=:0` 下，已跑过一组短 benchmark：

| mode | RTF | loop Hz | physics avg ms | camera avg ms | viewer avg ms | 结论 |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| none | 12.13 | 758.29 | 1.32 | 0 | 0 | physics 很快，CPU 物理步进不是主瓶颈 |
| viewer_fast, VSync on | 0.055 | 3.46 | 2.19 | 0 | 286.57 | 基础 viewer render 极慢 |
| viewer_fast, VSync off | 0.056 | 3.49 | 2.13 | 0 | 284.18 | 关闭 VSync 几乎无改善，排除单纯 VSync |
| camera, 2 cameras | 0.029 | 1.83 | 2.08 | 543.45 | 0 | 固定相机 render/readback 极慢 |
| renderer EGL, 2 cameras | 3.18 | 198.64 | 1.61 | 3.42 | 0 | 不开 viewer 时 EGL 相机很快 |

当前判断：

- 慢不来自 MuJoCo physics，也不主要来自 policy。
- 慢来自当前 GLFW/X11 viewer OpenGL 路径：viewer render 和 viewer-context fixed camera render/readback 都是几百毫秒级。
- 不打开 viewer、改用 `MUJOCO_GL=egl + mujoco.Renderer` 后，两路相机约 3.4 ms。
- `nvidia-smi` 能看到 RTX 3080 和驱动，但 benchmark 期间 GPU utilization 接近 0；这更像远程显示/GL context/软件渲染/读回路径异常，而不是 CUDA 算力问题。
- `model_sim.py` 里每帧 viewer render、3 Hz 两路 policy camera、3 Hz 三路 overlay camera，会把这个 OpenGL 慢路径反复放大。

下一步建议：

- 先用 `MUJOCO_BENCH_SWAP_INTERVAL=0 python model_sim.py` 验证实际观感；预计改善有限，因为 benchmark 里关闭 VSync 无明显收益。
- 评估/采集/真实计时时使用 `MODEL_SIM_VIEWER=0 MUJOCO_GL=egl python model_sim.py`。
- 需要观察时把 viewer/render 降到低频，或使用 `MODEL_SIM_RENDER_FAST=1`。
- 优先排查这台机器的 OpenGL renderer：安装/使用 `glxinfo -B`，确认不是 llvmpipe/software renderer。
- 如果必须远程看 viewer，优先尝试本地显示、VirtualGL、TurboVNC/NICE DCV 等真正走 GPU OpenGL 的方案。

## 结果记录模板

```markdown
## Run: <timestamp>

Machine:
- CPU:
- GPU / driver:
- OS:
- MuJoCo:
- Python:
- DISPLAY:
- MUJOCO_GL:
- MUJOCO_BENCH_SWAP_INTERVAL:

| mode | RTF | loop Hz | physics avg ms | camera avg ms | viewer avg ms | total avg ms | conclusion |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| none | | | | | | | |
| camera | | | | | | | |
| viewer_fast vsync on | | | | | | | |
| viewer_fast vsync off | | | | | | | |
| viewer_full | | | | | | | |

Conclusion:
- Bottleneck:
- Next action:
```

## 决策规则

- `none` 慢：优先查 CPU/physics/碰撞/接触/solver，不要先怪 viewer。
- `none` 快、`camera` 慢：优化相机数量、分辨率、readback 频率和 GPU/OpenGL。
- `none` 快、`camera` 快、`viewer_fast` 慢：优化 viewer/VSync/远程显示链路。
- `viewer_fast` 快、`viewer_full` 慢：优化 overlay、side camera、标题绘制、图像拷贝。
- 只有 headless/physics 确认无法满足需求时，再评估 MJX、Isaac Lab、ManiSkill。
