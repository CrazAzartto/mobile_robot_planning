#!/usr/bin/env python3
"""
train_rl.py
===========
Standalone training script for the RL navigation policy.
Uses the simplified RobotNavEnv gymnasium environment for fast training
without requiring Gazebo.

Usage:
  pip install stable-baselines3 gymnasium
  python3 train_rl.py --algorithm ppo --total-timesteps 500000 --output models/nav_policy

The trained model can then be loaded by rl_node.py for inference in the
live ROS 2 simulation.
"""

import argparse
import os
import sys
import time


def main():
    parser = argparse.ArgumentParser(
        description='Train RL navigation policy')
    parser.add_argument('--algorithm', type=str, default='ppo',
                        choices=['ppo', 'sac'],
                        help='RL algorithm (default: ppo)')
    parser.add_argument('--total-timesteps', type=int, default=500_000,
                        help='Total training timesteps (default: 500000)')
    parser.add_argument('--output', type=str, default='models/nav_policy',
                        help='Output model path (default: models/nav_policy)')
    parser.add_argument('--learning-rate', type=float, default=3e-4,
                        help='Learning rate (default: 3e-4)')
    parser.add_argument('--n-envs', type=int, default=4,
                        help='Number of parallel environments (default: 4)')
    parser.add_argument('--eval-freq', type=int, default=10_000,
                        help='Evaluation frequency (default: 10000)')
    parser.add_argument('--seed', type=int, default=42,
                        help='Random seed (default: 42)')
    args = parser.parse_args()

    # Import dependencies
    try:
        import numpy as np
        from stable_baselines3 import PPO, SAC
        from stable_baselines3.common.env_util import make_vec_env
        from stable_baselines3.common.callbacks import (
            EvalCallback, CheckpointCallback)
        from stable_baselines3.common.monitor import Monitor
    except ImportError:
        print('ERROR: Required packages not installed.')
        print('Run: pip install stable-baselines3 gymnasium')
        sys.exit(1)

    # Import our custom environment
    # Add parent directory to path so we can import rl_env
    script_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.dirname(script_dir)
    if parent_dir not in sys.path:
        sys.path.insert(0, parent_dir)

    from rl_planner.rl_env import RobotNavEnv

    # Create output directory
    output_dir = os.path.dirname(args.output) or '.'
    os.makedirs(output_dir, exist_ok=True)
    log_dir = os.path.join(output_dir, 'logs')
    os.makedirs(log_dir, exist_ok=True)

    print(f'Training RL policy with {args.algorithm.upper()}')
    print(f'  Total timesteps: {args.total_timesteps:,}')
    print(f'  Parallel envs:   {args.n_envs}')
    print(f'  Learning rate:   {args.learning_rate}')
    print(f'  Output:          {args.output}')
    print()

    # Create vectorised environments for parallel training
    env = make_vec_env(
        RobotNavEnv,
        n_envs=args.n_envs,
        seed=args.seed,
    )

    # Create evaluation environment
    eval_env = Monitor(RobotNavEnv())

    # Callbacks
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=output_dir,
        log_path=log_dir,
        eval_freq=args.eval_freq // args.n_envs,
        n_eval_episodes=10,
        deterministic=True,
        verbose=1,
    )

    checkpoint_callback = CheckpointCallback(
        save_freq=50_000 // args.n_envs,
        save_path=output_dir,
        name_prefix='checkpoint',
    )

    # Create model
    if args.algorithm == 'ppo':
        model = PPO(
            'MlpPolicy',
            env,
            learning_rate=args.learning_rate,
            n_steps=2048,
            batch_size=64,
            n_epochs=10,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            ent_coef=0.01,
            vf_coef=0.5,
            max_grad_norm=0.5,
            verbose=1,
            seed=args.seed,
            tensorboard_log=log_dir,
            policy_kwargs=dict(
                net_arch=dict(pi=[256, 256], vf=[256, 256])
            ),
        )
    else:  # SAC
        model = SAC(
            'MlpPolicy',
            env,
            learning_rate=args.learning_rate,
            buffer_size=100_000,
            learning_starts=1000,
            batch_size=256,
            tau=0.005,
            gamma=0.99,
            verbose=1,
            seed=args.seed,
            tensorboard_log=log_dir,
            policy_kwargs=dict(
                net_arch=dict(pi=[256, 256], qf=[256, 256])
            ),
        )

    # Train
    print(f'\nStarting training at {time.strftime("%H:%M:%S")}...\n')
    t_start = time.time()

    model.learn(
        total_timesteps=args.total_timesteps,
        callback=[eval_callback, checkpoint_callback],
        progress_bar=True,
    )

    elapsed = time.time() - t_start
    print(f'\nTraining complete in {elapsed:.1f}s ({elapsed/60:.1f} min)')

    # Save final model
    model.save(args.output)
    print(f'Model saved to: {args.output}.zip')

    # Quick evaluation
    print('\n--- Final Evaluation (20 episodes) ---')
    successes = 0
    collisions = 0
    total_reward = 0
    total_steps = 0

    test_env = RobotNavEnv()
    for ep in range(20):
        obs, _ = test_env.reset()
        ep_reward = 0
        done = False
        steps = 0
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = test_env.step(action)
            ep_reward += reward
            steps += 1
            done = terminated or truncated
        total_reward += ep_reward
        total_steps += steps
        if info.get('reached', False):
            successes += 1
        if info.get('collision', False):
            collisions += 1

    print(f'  Success rate:   {successes}/20 ({100*successes/20:.0f}%)')
    print(f'  Collision rate: {collisions}/20 ({100*collisions/20:.0f}%)')
    print(f'  Avg reward:     {total_reward/20:.1f}')
    print(f'  Avg steps:      {total_steps/20:.1f}')
    print(f'\nTo use in ROS 2:')
    print(f'  ros2 launch rl_planner rl.launch.py model_path:={os.path.abspath(args.output)}.zip')


if __name__ == '__main__':
    main()
