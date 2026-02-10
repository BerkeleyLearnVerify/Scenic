import gymnasium as gym
import numpy as np
from stable_baselines3 import PPO, SAC
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.vec_env import VecNormalize
import torch
import os

# Create directories
os.makedirs("pusher_models", exist_ok=True)
os.makedirs("pusher_logs", exist_ok=True)
os.makedirs("pusher_tensorboard", exist_ok=True)

def make_env():
    return gym.make("Pusher-v5", render_mode="rgb_array")

# Create vectorized environment with normalization for better training
vec_env = make_vec_env(make_env, n_envs=8)  # More parallel environments
vec_env = VecNormalize(vec_env, norm_obs=True, norm_reward=True, clip_obs=10.)

# Create evaluation environment
eval_env = make_vec_env(make_env, n_envs=1)
eval_env = VecNormalize(eval_env, norm_obs=True, norm_reward=True, clip_obs=10., training=False)

# Callbacks
checkpoint_callback = CheckpointCallback(
    save_freq=50000,
    save_path="./pusher_models/",
    name_prefix="pusher_improved_checkpoint"
)

eval_callback = EvalCallback(
    eval_env,
    best_model_save_path="./pusher_models/",
    log_path="./pusher_logs/",
    eval_freq=25000,
    deterministic=True,
    render=False,
    n_eval_episodes=10
)

# Option 1: Improved PPO with better hyperparameters
print("Training with improved PPO...")
model = PPO(
    "MlpPolicy", 
    vec_env,
    learning_rate=1e-4,      # Lower learning rate for stability
    n_steps=4096,            # More steps per update
    batch_size=128,          # Smaller batch size
    n_epochs=20,             # More epochs per update
    gamma=0.995,             # Higher discount factor
    gae_lambda=0.98,         # Higher GAE lambda
    clip_range=0.2,          # Standard clip range
    normalize_advantage=True,
    ent_coef=0.05,           # Higher entropy for more exploration
    vf_coef=0.5,
    max_grad_norm=0.5,
    verbose=1,
    tensorboard_log="./pusher_tensorboard/",
    policy_kwargs=dict(
        net_arch=dict(pi=[512, 512, 256], vf=[512, 512, 256]),  # Deeper networks
        activation_fn=torch.nn.ReLU,
    )
)

# Train for much longer
total_timesteps = 5_000_000  # 5 million timesteps

print(f"Starting training for {total_timesteps:,} timesteps...")
try:
    model.learn(
        total_timesteps=total_timesteps,
        callback=[checkpoint_callback, eval_callback],
        progress_bar=True
    )
    print("Training completed!")
except KeyboardInterrupt:
    print("Training interrupted by user")

# Save the model and normalization statistics
model.save("pusher/PPO_pusher_improved_v2.zip")
vec_env.save("pusher/vec_normalize.pkl")

# Test the model
print("\nTesting the improved model...")
test_env = gym.make("Pusher-v5", render_mode="rgb_array")

episode_rewards = []
success_count = 0
num_test_episodes = 10

for episode in range(num_test_episodes):
    obs, info = test_env.reset()
    episode_reward = 0
    
    # Track object and goal positions
    initial_obj_pos = None
    final_obj_pos = None
    goal_pos = None
    
    for step in range(500):  # Shorter episodes for testing
        # Normalize observation the same way as during training
        if hasattr(vec_env, 'normalize_obs'):
            obs = vec_env.normalize_obs(obs)
            
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = test_env.step(action)
        episode_reward += reward
        
        # Extract positions for success measurement (approximate indices)
        if step == 0 and len(obs) >= 23:
            initial_obj_pos = obs[17:19]  # Object x,y position
            goal_pos = obs[20:22]  # Goal x,y position
        
        if step == 499 or terminated or truncated:
            if len(obs) >= 23:
                final_obj_pos = obs[17:19]
            break
    
    # Check if successful (object within 0.1 units of goal)
    if initial_obj_pos is not None and final_obj_pos is not None and goal_pos is not None:
        initial_distance = np.linalg.norm(initial_obj_pos - goal_pos)
        final_distance = np.linalg.norm(final_obj_pos - goal_pos)
        
        if final_distance < 0.1:  # Success threshold
            success_count += 1
            success_marker = "✓"
        else:
            success_marker = "✗"
            
        print(f"Episode {episode+1}: Reward={episode_reward:.3f}, "
              f"Initial dist={initial_distance:.3f}, Final dist={final_distance:.3f} {success_marker}")
    else:
        print(f"Episode {episode+1}: Reward={episode_reward:.3f}")
    
    episode_rewards.append(episode_reward)

print(f"\n=== Results ===")
print(f"Success rate: {success_count}/{num_test_episodes} ({100*success_count/num_test_episodes:.1f}%)")
print(f"Average reward: {np.mean(episode_rewards):.3f} ± {np.std(episode_rewards):.3f}")
print(f"Best episode: {np.max(episode_rewards):.3f}")

test_env.close()
vec_env.close()
eval_env.close()

print("\nModel saved as pusher/PPO_pusher_improved_v2.zip")
print("Don't forget to update your pusher_bridge.py to use the new model!")