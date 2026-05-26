# Rendering Performance Optimization Report

---

## 一、问题诊断：画面卡顿/掉帧的根因

原始代码的渲染管线存在 **四个叠加的瓶颈**：

### 瓶颈 1：每帧创建线程

```python
# 原代码：每次策略推理都 new 一个 thread
policy_worker = threading.Thread(target=run_policy_async, args=(data,), daemon=True)
policy_worker.start()
```

`threading.Thread` 每 333ms 创建/销毁一次，每次约 1-2ms 的内核开销。频繁的 OS 调度抖动会干扰渲染节奏。

### 瓶颈 2：物理-渲染不同步

```python
# 原代码：4 步物理后才条件渲染
PnPEnv.step_env(nstep=4)     # 物理跳 4 步
...
if now >= next_render_time:   # 30Hz 条件检查
    PnPEnv.render(fast=True) # 可能跳过不渲染
```

物理每循环跳 4 步（约 8ms 模拟时间），但渲染只在 ~33ms 间隔触发。**画面在两次渲染之间跳过 4 个物理状态，观感上就是"跳帧"**。

### 瓶颈 3：渲染频率 30Hz

人眼对 30fps 和 60fps 的差异非常敏感。30fps 的帧间隔 33ms，本身就低于流畅阈值。

### 瓶颈 4：策略图像抓取阻塞主线程

```python
def build_policy_input():
    image, wrist_image = PnPEnv.grab_image()  # 2× get_fixed_cam_rgb, 各 ~10ms (800x600)
    ...
```

`get_fixed_cam_rgb` 内部做 `mjv_updateScene` + `mjr_render` + `mjr_readPixels`，800x600 下每次约 10ms。两次相机就是 ~20ms 阻塞在主线程上。如果恰好发生在渲染时间窗口内，直接掉帧。

---

## 二、优化策略：五个方向

| 方向 | 原代码 | 优化后 |
|---|---|---|
| **线程模型** | 每周期 new Thread | 持久化 worker + Event 通信 |
| **渲染频率** | 30Hz | 60Hz |
| **物理-渲染耦合** | 4 步物理/循环，条件渲染 | N 步物理/帧，每帧必渲染 |
| **图像抓取解耦** | 策略和 overlay 共用同一抓取 | 策略轻量抓取 + overlay 独立刷新 |
| **帧节奏** | `time.sleep` + 模拟时间锚定 | `next_frame_time` 统一时钟 + sleep/spin 混合 |

---

## 三、逐项改动详解

### 3.1 持久化策略推理线程

**原理：** 用 `threading.Event` 做生产者-消费者通信，替代每周期 new Thread。

```
主线程 (生产者)                 Worker线程 (消费者)
─────────────                  ────────────────
dispatch_policy(data):         _policy_worker():
  policy_done.clear()            policy_event.wait()  ← 阻塞
  policy_data = data             data = policy_data
  policy_event.set()  ──────→   policy_event.clear()
                                 policy.select_action(data)  ← GPU推理
                                 policy_result = action
                                 policy_done.set()  ──→  collect_action() 读取
                                 policy_busy = False
```

**关键设计：**

- `policy_busy` 布尔标志防止重复派发（替代原代码的 `policy_worker.is_alive()` 检查）
- `policy_event` 唤醒 worker，`policy_done` 通知主线程结果就绪

**效果：** 消除每 333ms 的线程创建/销毁开销，减少主线程抖动。

### 3.2 固定 60fps 帧率 + 每帧物理+渲染

**原理：** 用单一 `next_frame_time` 时钟替代原来的 `next_sim_time` + `next_render_time` 双时钟。

```python
# 物理步数自动匹配帧间隔
PHYSICS_STEPS = max(1, round(FRAME_INTERVAL / SIM_DT))
# dt=0.002 时: round(16.7ms / 2ms) ≈ 8 步/帧 → 模拟速度接近实时

# 每帧固定顺序
if now >= next_frame_time:
    动作应用 → 物理步进(N步) → 策略/overlay → 渲染  # 每帧必渲染
    next_frame_time += FRAME_INTERVAL

# 精确帧节奏
wait_time = next_frame_time - time.perf_counter()
if wait_time > 0.002:
    time.sleep(wait_time - 0.001)   # sleep 处理大块等待
# < 2ms 的剩余时间靠自旋消耗（比 sleep 精确）
```

**效果：** 渲染从 30fps → 60fps，每帧画面都反映最新物理状态，画面连续性翻倍。

### 3.3 图像抓取职责分离

这是本次优化最核心的架构改动。将原来混杂在一起的两种图像需求彻底解耦：

```
原代码 (混杂):
  build_policy_input() → grab_image() → 同时做策略输入 + overlay 缓存
  问题: 3 相机 + 标题处理阻塞策略帧

优化后 (解耦):
  策略帧 (3Hz): build_policy_input() → grab_image_fast() → 仅 2 相机，不更新 overlay
  Overlay帧 (3Hz): 独立定时器 → grab_image(include_side=True) → 3 相机 + 标题
  互斥: policy_dispatched 标志 → 策略帧时跳过 overlay 刷新，避免同帧双重抓取
```

```python
# 两个独立时钟，初始相位错开 250ms
next_policy_time  = t + 0.50s   # 每 333ms
next_overlay_time = t + 0.25s   # 每 333ms (OVERLAY_HZ=3)

# 步骤 4: 策略分发
if now >= next_policy_time and not policy_busy:
    dispatch_policy(build_policy_input())  # grab_image_fast: 2 相机
    policy_dispatched = True

# 步骤 5: Overlay 刷新（策略帧时跳过）
if not policy_dispatched and now >= next_overlay_time:
    PnPEnv.grab_image(include_side=True)  # 3 相机 + 标题
```

### 3.4 y_env2.py: 渲染模式重构

**`render()` 增加 `fast` 和 `show_side_view` 参数：**

```python
def render(self, teleop=False, idx=0, fast=False, show_side_view=False):
    if fast:
        self.env.render()        # 纯 viewer 渲染，~1ms，无任何叠加
        return

    # 以下使用缓存图像，不做相机抓取
    self.env.plot_time()         # 时间戳
    self.env.plot_sphere(...)    # TCP 位置球
    self.env.plot_capsule(...)   # TCP 方向柱
    self.env.plot_T(...)         # Episode 编号
    # 相机小窗 — 缓存图像 blit，极快
    self.env.viewer_rgb_overlay(self.rgb_agent_view, 'top right')
    self.env.viewer_rgb_overlay(self.rgb_egocentric_view, 'bottom right')
    if teleop or show_side_view:
        self.env.viewer_rgb_overlay(self.rgb_side_view, 'top left')
    # 任务指令文字
    self.env.viewer_text_overlay('Language Instructions', self.instruction)
    self.env.render()
```

**关键设计：**

- `fast=True`：纯渲染，无叠加物（性能最优，但无信息显示）
- `fast=False`：全叠加渲染，所有相机图像使用缓存（由 `grab_image()` 低频刷新），每帧只做便宜的 marker + blit
- `show_side_view` 解耦自 `teleop`，仿真时也能看到侧视相机
- `hasattr` 守卫防止首次 `grab_image` 前访问不存在的属性

**`grab_image_fast()` 新增方法：**

```python
def grab_image_fast(self):
    """仅抓取原始图像，跳过 add_title_to_img 处理"""
    rgb_agent = self.env.get_fixed_cam_rgb(cam_name='agentview')
    rgb_ego = self.env.get_fixed_cam_rgb(cam_name='egocentric')
    return rgb_agent, rgb_ego
```

专为策略输入设计的轻量路径，跳过了 `add_title_to_img` 的 CPU 开销。

**`init_viewer` 增加参数：** `initialize_viewer` 允许延迟初始化 viewer（先加载模型再开窗口），`reset_env` 允许跳过重复 reset。

### 3.5 Markers 内存安全性验证

经确认，`mujoco_parser.py:733` 在每帧渲染后执行 `self._markers[:] = []`，marker 不会泄漏。因此 60fps 下的 `plot_sphere/plot_capsule/plot_T` 是安全的。

---

## 四、性能影响总结

### 每帧耗时

| 帧类型 | 频率 (/60帧) | 耗时 | vs 16.7ms 预算 |
|---|---|---|---|
| 普通帧 | 54 | ~7ms | 宽裕 |
| 策略帧 | 3 | ~27ms | 超出 ~10ms（物理5 + 抓图20 + 渲染2） |
| Overlay帧 | 3 | ~37ms | 超出 ~20ms（物理5 + 抓图35 + 渲染2） |

> 策略帧和 overlay 帧互斥（同帧不触发），实际每秒约 6 帧略超预算，其余 54 帧在预算内。

### 开销来源

| 操作 | 耗时 | 频率 | 说明 |
|---|---|---|---|
| 物理步进 (8步) | ~5ms | 60Hz | `step_env(nstep=8)` |
| Viewer 渲染 | ~1-2ms | 60Hz | `mjv_updateScene + mjr_render` |
| Marker + Overlay blit | ~1ms | 60Hz | 球/柱/文字/缓存图像叠加 |
| 策略图像抓取 | ~20ms | 3Hz | 2 相机 `get_fixed_cam_rgb` (800x600) |
| Overlay 图像刷新 | ~35ms | 3Hz | 3 相机 + `add_title_to_img` (800x600) |

### 可调渲染参数

| 参数 | 默认值 | 作用 | 调低影响 |
|---|---|---|---|
| `RENDER_HZ` | 60 | 渲染帧率 | 画面流畅度下降 |
| `OVERLAY_HZ` | 3 | 相机小窗刷新率 | 小窗画面变卡 |

---

## 五、恢复的原有渲染功能

| 功能 | 状态 |
|---|---|
| 任务中文指令文字叠加 | 已恢复（`Language Instructions` overlay） |
| Agent View 相机小窗（右上） | 已恢复（缓存图像 blit） |
| Egocentric View 相机小窗（右下） | 已恢复（缓存图像 blit） |
| Side View 相机小窗（左上） | 已恢复（`show_side_view=True` 解耦自 teleop） |
| TCP 位置指示器（球+柱） | 已恢复（每帧 marker，自动清理） |
| Episode 编号显示 | 已恢复（`idx=episode_idx`） |
