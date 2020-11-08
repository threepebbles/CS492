from gym.envs.registration import register

register(
    id='ur5-v0',
    entry_point='cs492_gym.envs:UR5Env',
)

