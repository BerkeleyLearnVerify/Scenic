import gymnasium as gym
from stable_baselines3 import PPO

# Use Pusher-v5 which is compatible with MuJoCo 3.x
env = gym.make("Pusher-v5", render_mode="rgb_array")

model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=1_000_000)

# Test the trained model
vec_env = model.get_env()
obs = vec_env.reset()
for i in range(1000):
    action, _state = model.predict(obs, deterministic=True)
    obs, reward, done, info = vec_env.step(action)

model.save("PPO_pusher.zip")
print("Model saved as PPO_pusher.zip")