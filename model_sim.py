from lerobot.common.datasets.lerobot_dataset import LeRobotDataset, LeRobotDatasetMetadata
import numpy as np
from lerobot.common.datasets.utils import write_json, serialize_dict
from lerobot.configs.types import FeatureType
from lerobot.common.datasets.factory import resolve_delta_timestamps
from lerobot.common.datasets.utils import dataset_to_policy_features
import torch
from PIL import Image
import threading
import time
from mujoco_env.y_env2 import SimpleEnv2
xml_path = './asset/example_scene_y2.xml'

# --- Model selection ---
MODEL_TYPE = 'pi0'  # 'smolvla' or 'pi0'

if MODEL_TYPE == 'smolvla':
    from lerobot.common.policies.smolvla.configuration_smolvla import SmolVLAConfig as PolicyConfig
    from lerobot.common.policies.smolvla.modeling_smolvla import SmolVLAPolicy as Policy
    CKPT_PATH = './ckpt/smolvla_omy/checkpoints/last/pretrained_model'
elif MODEL_TYPE == 'pi0':
    from lerobot.common.policies.pi0.configuration_pi0 import PI0Config as PolicyConfig
    from lerobot.common.policies.pi0.modeling_pi0 import PI0Policy as Policy
    CKPT_PATH = './ckpt/pi0_omy/checkpoints/last/pretrained_model'
else:
    raise ValueError(f"Unknown MODEL_TYPE: {MODEL_TYPE}")

device = 'cuda'
torch.backends.cudnn.benchmark = True

try:
    dataset_metadata = LeRobotDatasetMetadata("omy_pnp_language", root='./demo_data_language')
except:
    dataset_metadata = LeRobotDatasetMetadata("omy_pnp_language", root='./omy_pnp_language')
features = dataset_to_policy_features(dataset_metadata.features)
output_features = {key: ft for key, ft in features.items() if ft.type is FeatureType.ACTION}
input_features = {key: ft for key, ft in features.items() if key not in output_features}
cfg = PolicyConfig(input_features=input_features, output_features=output_features, chunk_size=5, n_action_steps=5)
delta_timestamps = resolve_delta_timestamps(cfg, dataset_metadata)

print(f'Loading {MODEL_TYPE} policy...', flush=True)
policy = Policy.from_pretrained(CKPT_PATH, dataset_stats=dataset_metadata.stats)
# You can load the trained policy from hub if you don't have the resources to train it.
# policy = Policy.from_pretrained("Jeongeun/omy_pnp_pi0", config=cfg, dataset_stats=dataset_metadata.stats)
policy.to(device)
print('Policy loaded. Initializing MuJoCo environment...', flush=True)

from torchvision import transforms
# Approach 1: Using torchvision.transforms
def get_default_transform(image_size: int = 224):
    """
    Returns a torchvision transform that:
     Converts to a FloatTensor and scales pixel values [0,255] -> [0.0,1.0]
    """
    return transforms.Compose([
        transforms.ToTensor(),  # PIL [0–255] -> FloatTensor [0.0–1.0], shape C×H×W
    ])

PnPEnv = SimpleEnv2(xml_path, action_type='joint_angle', seed=0, initialize_viewer=False)
policy.reset()
policy.eval()
print('Opening MuJoCo viewer...', flush=True)
PnPEnv.init_viewer(reset_env=False)
for _ in range(15):
    PnPEnv.env.render()
    time.sleep(1.0 / 60.0)
print('Simulation ready.', flush=True)
IMG_TRANSFORM = get_default_transform()

# --- Timing configuration ---
SIM_DT = PnPEnv.env.dt            # MuJoCo physics timestep (e.g. 0.002s)
RENDER_HZ = 60                     # target render framerate
POLICY_HZ = 3                      # policy inference rate
OVERLAY_HZ = 3                     # camera overlay refresh rate (independent of policy)
MAX_EPISODE_STEPS = 150            # max policy actions per episode (~50s at 3Hz)
FRAME_INTERVAL = 1.0 / RENDER_HZ   # ~16.7ms per frame
# Physics steps per frame so simulation runs near real-time
PHYSICS_STEPS = max(1, round(FRAME_INTERVAL / SIM_DT))

# Initial overlay image capture (so overlays are visible from frame 1)
PnPEnv.grab_image(include_side=True)

# --- Persistent policy worker (avoids per-cycle thread creation overhead) ---
policy_data = None
policy_result = None
policy_lock = threading.Lock()
policy_event = threading.Event()
policy_done = threading.Event()
policy_busy = False
policy_running = True

def _policy_worker():
    global policy_result, policy_busy
    while policy_running:
        policy_event.wait()
        if not policy_running:
            break
        policy_busy = True
        data = policy_data
        policy_event.clear()
        with torch.inference_mode():
            action = policy.select_action(data)
        action = action[0, :7].cpu().numpy()
        with policy_lock:
            policy_result = action
        policy_done.set()
        policy_busy = False

policy_thread = threading.Thread(target=_policy_worker, daemon=True)
policy_thread.start()

def build_policy_input():
    state = PnPEnv.get_joint_state()[:6]
    image, wrist_image = PnPEnv.grab_image_fast()
    image = Image.fromarray(image)
    wrist_image = Image.fromarray(wrist_image)
    if MODEL_TYPE == 'pi0':
        # pi0 training pipeline: 800x600 raw → 256x256 resize → model
        image = image.resize((256, 256))
        wrist_image = wrist_image.resize((256, 256))
    return {
        'observation.state': torch.from_numpy(np.asarray(state, dtype=np.float32)).unsqueeze(0).to(device, non_blocking=True),
        'observation.image': IMG_TRANSFORM(image).unsqueeze(0).to(device, non_blocking=True),
        'observation.wrist_image': IMG_TRANSFORM(wrist_image).unsqueeze(0).to(device, non_blocking=True),
        'task': [PnPEnv.instruction],
    }

def dispatch_policy(data):
    global policy_data, policy_result
    policy_done.clear()
    policy_result = None
    policy_data = data
    policy_event.set()

def collect_action():
    global policy_result
    if policy_done.is_set():
        with policy_lock:
            action = policy_result
            policy_result = None
        policy_done.clear()
        return action
    return None

# --- Main loop: fixed-framerate, physics+render every frame ---
next_frame_time = time.perf_counter()
next_policy_time = time.perf_counter() + 0.5
# Offset overlay refresh from policy timing so they rarely overlap
next_overlay_time = time.perf_counter() + 0.25
action = None
episode_step = 0
episode_idx = 0

def reset_episode(reason=''):
    """Reset environment and policy for a new episode."""
    global episode_step, episode_idx
    episode_idx += 1
    if reason:
        print(f'Episode {episode_idx}: {reason} (steps={episode_step})', flush=True)
    policy.reset()
    PnPEnv.reset()
    episode_step = 0
    with policy_lock:
        policy_result = None
    policy_done.clear()

while PnPEnv.env.is_viewer_alive():
    now = time.perf_counter()

    if now >= next_frame_time:

        # 1. Collect & apply latest action BEFORE physics (reduces 1-frame latency)
        action_applied = False
        new_action = collect_action()
        if new_action is not None:
            action = new_action
        if action is not None:
            PnPEnv.step(action)
            action = None
            action_applied = True
            episode_step += 1

        # 2. Physics stepping
        PnPEnv.step_env(nstep=PHYSICS_STEPS)

        # 3. Check episode end conditions (after action + physics)
        episode_done = False
        if action_applied:
            if PnPEnv.check_success():
                reset_episode('SUCCESS')
                episode_done = True
            elif episode_step >= MAX_EPISODE_STEPS:
                reset_episode('TIMEOUT')
                episode_done = True

        # 4. Dispatch policy inference when due and worker is idle
        #    Uses grab_image_fast() — 2 cameras, ~10ms, no overlay side-effect
        policy_dispatched = False
        if now >= next_policy_time and not policy_busy:
            if not episode_done and PnPEnv.check_success():
                reset_episode('SUCCESS')
            dispatch_policy(build_policy_input())
            next_policy_time += 1.0 / POLICY_HZ
            if next_policy_time < now:
                next_policy_time = now + 1.0 / POLICY_HZ
            policy_dispatched = True

        # 5. Refresh camera overlay images at independent rate
        #    Skip if policy was just dispatched (avoid double capture in one frame)
        if not policy_dispatched and now >= next_overlay_time:
            PnPEnv.grab_image(include_side=True)
            next_overlay_time += 1.0 / OVERLAY_HZ
            if next_overlay_time < now:
                next_overlay_time = now + 1.0 / OVERLAY_HZ

        # 6. Render every frame with overlays (uses cached images, cheap blit)
        PnPEnv.render(fast=False, show_side_view=True, idx=episode_idx)

        next_frame_time += FRAME_INTERVAL
        if next_frame_time < now:
            next_frame_time = now + FRAME_INTERVAL

    # Precise frame pacing: sleep for bulk, spin-wait remainder
    wait_time = next_frame_time - time.perf_counter()
    if wait_time > 0.002:
        time.sleep(wait_time - 0.001)
    elif wait_time < -FRAME_INTERVAL:
        next_frame_time = time.perf_counter()

# Cleanup
policy_running = False
policy_event.set()
policy_thread.join(timeout=1.0)
