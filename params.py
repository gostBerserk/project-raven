"""Single place for the tuning constants the simulation and the camera script share
(the same idea as one parameter file driving a whole Simulink model)."""

HORIZON_S = 1.2        # only react to balls that will pass closest within this many seconds
DANGER_FACTOR = 2.0    # a ball passing within DANGER_FACTOR x platform radius is a threat
DEAD_ON_FRAC = 0.1     # closest approach under this fraction of the platform radius = direct hit
TRACK_HISTORY = 10     # frames in the least-squares velocity fit (tuned in simulation: 6 was too noisy)
CAMERA_FPS = 30        # control and tracking rate
