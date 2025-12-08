"""
Third-party lerobot plugin that exposes Pingti robots to lerobot's discovery.

When `register_third_party_plugins()` scans for modules prefixed with
`lerobot_robot_`, importing this package will pull in the Pingti robot
definitions so their configs are registered.
"""

# Import the pingti robot modules so their RobotConfig subclasses register.
from pingti.robots import pingti_follower, bi_pingti_follower  # noqa: F401

