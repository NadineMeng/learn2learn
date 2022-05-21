#!/usr/bin/env python3

from gym.envs.registration import register

from .subproc_vec_env import SubprocVecEnv

# 2D Navigation
# ----------------------------------------

register(
    'Particles2D-v1',
    entry_point='learn2learn.gym.envs.particles.particles_2d:Particles2DEnv',
    max_episode_steps=100
)



# Classic Control
# ------------------------------------------
register(
    id='CartPole-Dis-v1',
    entry_point='learn2learn.gym.envs.classic_control.cartpole:MetaCartPoleEnv',
    max_episode_steps=200,
    #reward_threshold=195.0,
)

register(
    id='Cartpole-Conti-v1',
    entry_point='learn2learn.gym.envs.classic_control.cartpole_continuous:MetaContinuousCartPoleEnv',
    max_episode_steps=200,
    #reward_threshold=195.0,
)




# Mujoco
# ----------------------------------------

register(
    'HalfCheetahForwardBackward-v1',
    entry_point='learn2learn.gym.envs.mujoco.halfcheetah_forward_backward:HalfCheetahForwardBackwardEnv',
    max_episode_steps=100,
)

register(
    'HalfCheetahGra-v1',
    entry_point='learn2learn.gym.envs.mujoco.halfcheetah_gra:HalfCheetahGraEnv',
    max_episode_steps=100,
)

register(
    'AntForwardBackward-v1',
    entry_point='learn2learn.gym.envs.mujoco.ant_forward_backward:AntForwardBackwardEnv',
    max_episode_steps=100,
)

register(
    'AntDirection-v1',
    entry_point='learn2learn.gym.envs.mujoco.ant_direction:AntDirectionEnv',
    max_episode_steps=100,
)

register(
    'HumanoidForwardBackward-v1',
    entry_point='learn2learn.gym.envs.mujoco.humanoid_forward_backward:HumanoidForwardBackwardEnv',
    max_episode_steps=200,
)

register(
    'HumanoidDirection-v1',
    entry_point='learn2learn.gym.envs.mujoco.humanoid_direction:HumanoidDirectionEnv',
    max_episode_steps=200,
)

register(
    'WalkerVel-v1',
    entry_point='learn2learn.gym.envs.mujoco.walker2d_rand_vel:Walker2DRandVelEnv',
    max_episode_steps=200,
)
