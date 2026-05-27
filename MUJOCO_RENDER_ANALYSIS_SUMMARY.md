# MuJoCo 渲染性能排查与修复总结

## 背景

在 `lerobot-mujoco-manipulation` conda 环境中执行 `python model_sim.py` 时，发现渲染很慢，并且程序记录到的 wall time 也很慢。由于同样代码在其他机器上表现正常，最初怀疑这台机器存在 CPU 性能瓶颈，尤其是 viewer 路径拖慢了仿真主循环。

本次排查目标是把慢拆成几类并逐一验证：

- MuJoCo physics step 是否慢。
- GLFW viewer 渲染是否慢。
- fixed camera render/readback 是否慢。
- VSync 是否导致 `swap_buffers()` 等待。
- 是否能通过 headless EGL 渲染绕开当前慢路径。

## 排查过程

### 1. 确认项目真实渲染路径

当前项目不是使用官方 `mujoco.viewer.launch_passive()`，而是自定义 viewer：

- 入口：`model_sim.py`
- 环境：`mujoco_env.y_env2.SimpleEnv2`
- viewer：`mujoco_env.mujoco_parser.MuJoCoMinimalViewer`
- 相机图像：`get_fixed_cam_rgb()`，内部使用当前 viewer 的 OpenGL context 做 `mjv_updateScene + mjr_render + mjr_readPixels`

关键发现：

```python
glfw.swap_interval(1)
```

viewer 默认开启 VSync，因此一开始需要验证是否是 `swap_buffers()` 被显示链路阻塞。

### 2. 增加真实 benchmark 脚本

新增脚本：

```text
scripts/mujoco_render_benchmark.py
```

它支持以下模式：

```text
none          只跑 physics，不创建 viewer
camera        创建 viewer context，只测 fixed camera render/readback
viewer_fast   只测基础 viewer render，不画 overlay
viewer_full   模拟 model_sim.py 的相机 overlay + viewer render
renderer      不打开 viewer，使用 mujoco.Renderer 测 EGL/offscreen 相机渲染
```

这样可以把 physics、viewer、camera、EGL renderer 分开测。

### 3. Physics-only 测试

命令：

```bash
DISPLAY=:0 conda run -n lerobot-mujoco-manipulation \
  python scripts/mujoco_render_benchmark.py \
  --mode none \
  --loops 50 \
  --physics-steps 8
```

结果：

```text
RTF: 12.13
loop Hz: 758.29
physics avg: 1.32 ms
```

结论：

MuJoCo physics 本身很快，CPU 物理步进不是主瓶颈。

### 4. Viewer basic render 测试

命令：

```bash
DISPLAY=:0 conda run -n lerobot-mujoco-manipulation \
  python scripts/mujoco_render_benchmark.py \
  --mode viewer_fast \
  --loops 80 \
  --physics-steps 8
```

结果：

```text
RTF: 0.055
loop Hz: 3.46
physics avg: 2.19 ms
viewer avg: 286.57 ms
```

结论：

基础 viewer render 已经是几百毫秒级，远远超过 60Hz 或 20Hz 的渲染预算。

### 5. 关闭 VSync 对比

命令：

```bash
DISPLAY=:0 MUJOCO_BENCH_SWAP_INTERVAL=0 \
conda run -n lerobot-mujoco-manipulation \
  python scripts/mujoco_render_benchmark.py \
  --mode viewer_fast \
  --loops 80 \
  --physics-steps 8
```

结果：

```text
RTF: 0.056
loop Hz: 3.49
physics avg: 2.13 ms
viewer avg: 284.18 ms
```

结论：

关闭 VSync 后几乎没有改善，因此不是单纯 `glfw.swap_interval(1)` 导致的等待。问题更像是当前 GLFW/X11/OpenGL viewer 路径整体异常慢。

### 6. Viewer-context fixed camera 测试

命令：

```bash
DISPLAY=:0 conda run -n lerobot-mujoco-manipulation \
  python scripts/mujoco_render_benchmark.py \
  --mode camera \
  --loops 80 \
  --physics-steps 8 \
  --camera-every 1
```

结果：

```text
RTF: 0.029
loop Hz: 1.83
physics avg: 2.08 ms
camera avg: 543.45 ms
```

结论：

fixed camera render/readback 也非常慢。由于该路径依赖 viewer OpenGL context，说明 `model_sim.py` 中 policy 图像抓取和 overlay 图像刷新都会被同一个慢路径拖住。

### 7. NVIDIA 状态检查

命令：

```bash
nvidia-smi
nvidia-smi pmon -c 1
```

结果显示：

- RTX 3080 和 NVIDIA driver 可见。
- viewer/camera 慢的时候 GPU utilization 接近 0。
- `glxinfo -B` 未安装，无法直接确认 OpenGL renderer。

结论：

这不像 CUDA 算力不足，更像远程显示、GLFW/X11 context、软件渲染或 readback 路径异常。

### 8. EGL Renderer 快路径验证

先不打开 viewer，直接使用项目真实 `SimpleEnv2` + `mujoco.Renderer`：

```bash
DISPLAY=:0 MUJOCO_GL=egl conda run -n lerobot-mujoco-manipulation \
  python -c "from mujoco_env.y_env2 import SimpleEnv2; import mujoco,time; env=SimpleEnv2('./asset/example_scene_y2.xml', action_type='joint_angle', seed=0, initialize_viewer=False); r=mujoco.Renderer(env.env.model,height=480,width=640); t=time.perf_counter(); imgs=[(env.step_env(8), r.update_scene(env.env.data,camera='agentview'), r.render())[2] for _ in range(20)]; print('elapsed', time.perf_counter()-t, imgs[-1].shape, int(imgs[-1].sum()))"
```

结果：

```text
20 frames elapsed: 0.156 s
约 7.8 ms/frame
```

随后用新增 benchmark 的 `renderer` 模式测两路相机：

```bash
MUJOCO_GL=egl DISPLAY=:0 conda run -n lerobot-mujoco-manipulation \
  python scripts/mujoco_render_benchmark.py \
  --mode renderer \
  --loops 50 \
  --physics-steps 8 \
  --camera-every 1
```

结果：

```text
RTF: 3.18
loop Hz: 198.64
physics avg: 1.61 ms
camera avg: 3.42 ms
GPU utilization: about 30%
```

结论：

MuJoCo 渲染能力本身没有问题。真正慢的是当前 GLFW/X11 viewer context。不开 viewer、走 `MUJOCO_GL=egl + mujoco.Renderer` 后，两路 policy 相机可以降到毫秒级。

### 9. 打开 viewer 后再使用 Renderer

进一步验证：同一进程中先打开 GLFW viewer，再创建 `mujoco.Renderer`。

结果：

```text
20 frames elapsed: 6.53 s
约 326 ms/frame
```

结论：

一旦打开当前 GLFW viewer，渲染路径会被拖回慢状态。因此真实策略运行和计时必须避免打开 viewer。

## 关键数据汇总

| mode | RTF | loop Hz | physics avg ms | camera avg ms | viewer avg ms | 结论 |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| none | 12.13 | 758.29 | 1.32 | 0 | 0 | physics 很快 |
| viewer_fast, VSync on | 0.055 | 3.46 | 2.19 | 0 | 286.57 | viewer 极慢 |
| viewer_fast, VSync off | 0.056 | 3.49 | 2.13 | 0 | 284.18 | 排除单纯 VSync |
| camera, viewer context | 0.029 | 1.83 | 2.08 | 543.45 | 0 | viewer-context camera 极慢 |
| renderer EGL, no viewer | 3.18 | 198.64 | 1.61 | 3.42 | 0 | EGL 快路径可用 |

## 最终判断

这台机器上的慢不是 CPU physics 瓶颈，也不是 policy 推理首先导致的瓶颈。

主要瓶颈是：

```text
当前 GLFW/X11 viewer OpenGL context 路径异常慢
```

它会同时拖慢：

- 主 viewer render。
- `get_fixed_cam_rgb()` 的 fixed camera render/readback。
- `model_sim.py` 中 policy 输入图像抓取。
- overlay 小窗图像刷新。

而 `MUJOCO_GL=egl + mujoco.Renderer` 在不开 viewer 时性能正常，因此修复方向是把真实策略运行切到 headless EGL 快路径。

## 已做修复

### 1. `SimpleEnv2` 增加 offscreen renderer

文件：

```text
mujoco_env/y_env2.py
```

新增能力：

- `init_offscreen_renderer(width, height)`
- `close_offscreen_renderer()`
- `get_fixed_cam_rgb_offscreen(cam_name)`

并让：

- `grab_image_fast()`
- `grab_image()`

在 offscreen renderer 存在时优先使用 `mujoco.Renderer`。

### 2. `model_sim.py` 支持 headless 快路径

新增环境变量：

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

推荐真实运行：

```bash
MODEL_SIM_VIEWER=0 MUJOCO_GL=egl python model_sim.py
```

只想观察 viewer 时：

```bash
MODEL_SIM_RENDER_HZ=5 MODEL_SIM_RENDER_FAST=1 MODEL_SIM_SHOW_SIDE_VIEW=0 python model_sim.py
```

### 3. `mujoco_parser.py` 增加 VSync 开关

文件：

```text
mujoco_env/mujoco_parser.py
```

变更：

```python
glfw.swap_interval(int(os.environ.get("MUJOCO_BENCH_SWAP_INTERVAL", "1")))
```

用于 benchmark 和排查。

### 4. `utils.py` 支持无 DISPLAY import

文件：

```text
mujoco_env/utils.py
```

把 `pyautogui` 改为 `get_monitor_size()` 内懒加载。headless EGL 运行时不再因为 import 阶段缺少 DISPLAY 直接失败。

### 5. 文档和 benchmark 更新

新增/更新：

```text
scripts/mujoco_render_benchmark.py
MUJOCO_RENDER_BENCHMARK.md
```

## 验证结果

语法检查：

```bash
conda run -n lerobot-mujoco-manipulation \
  python -m py_compile \
  model_sim.py \
  scripts/mujoco_render_benchmark.py \
  mujoco_env/y_env2.py \
  mujoco_env/utils.py
```

结果：通过。

EGL renderer benchmark：

```text
RTF 3.18
camera avg 3.42 ms
```

`model_sim.py` headless 冒烟测试：

```bash
MODEL_SIM_VIEWER=0 MUJOCO_GL=egl MODEL_SIM_HEADLESS_MAX_FRAMES=5 \
conda run -n lerobot-mujoco-manipulation python model_sim.py
```

结果：能加载 policy、初始化 MuJoCo、初始化 offscreen renderer，并正常退出。

## 后续建议

- 真实策略运行、计时、采集、评估默认使用：

```bash
MODEL_SIM_VIEWER=0 MUJOCO_GL=egl python model_sim.py
```

- viewer 只用于人眼调试，不再作为性能评估路径。
- 若必须远程看高帧率 viewer，继续排查 OpenGL renderer：

```bash
DISPLAY=:0 glxinfo -B
```

- 如果 `glxinfo` 显示 llvmpipe/software renderer，应切换到 VirtualGL、TurboVNC、NICE DCV 或本地显示等真正走 GPU OpenGL 的方案。
- 如果后续还要保存视频，建议在 headless EGL 路径下按低频缓存帧，避免同步视频编码阻塞主循环。
