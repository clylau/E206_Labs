from gym.envs.registration import register

register(
    id='fetch-v0',
    entry_point='gym_fetch.envs:FetchEnv',
)
