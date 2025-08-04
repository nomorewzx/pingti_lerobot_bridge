import logging
import pickle  # nosec
import threading
import time
from collections.abc import Callable
from dataclasses import asdict
from pprint import pformat
from queue import Queue
from typing import Any

import draccus
import grpc
import torch
from lerobot.configs.policies import PreTrainedConfig
from pingti.robots import pingti_follower # noqa: F401 # pylint: disable=unused-import # necessary for draccus
from lerobot.scripts.server.configs import RobotClientConfig
from lerobot.scripts.server.robot_client import RobotClient
from lerobot.scripts.server.helpers import (
    FPSTracker,
    RemotePolicyConfig,
    map_robot_keys_to_lerobot_features,
    validate_robot_cameras_for_policy,
    visualize_action_queue_size,
)

from lerobot.transport import (
    async_inference_pb2,  # type: ignore
    async_inference_pb2_grpc,  # type: ignore
)
from lerobot.transport.utils import grpc_channel_options

from pingti.robots.utils import make_robot_from_config
from pingti.scripts.server.constants import SUPPORTED_ROBOTS

class PingtiRobotClient(RobotClient):
    """Subclass of RobotClient that only overrides the make_robot_from_config function.
    
    All other parameters and functions are inherited from RobotClient.
    """
    
    def __init__(self, config: RobotClientConfig):
        """Initialize CustomRobotClient with unified configuration.
        
        Args:
            config: RobotClientConfig containing all configuration parameters
        """
        # Store configuration
        self.config = config
        # Use custom robot creation instead of the default
        self.robot = make_robot_from_config(config.robot)
        self.robot.connect()

        # Reuse the rest of the parent class initialization by calling it manually
        # since we can't call super().__init__() due to the robot creation difference
        lerobot_features = map_robot_keys_to_lerobot_features(self.robot)

        if config.verify_robot_cameras:
            # Load policy config for validation
            policy_config = PreTrainedConfig.from_pretrained(config.pretrained_name_or_path)
            policy_image_features = policy_config.image_features

            # The cameras specified for inference must match the one supported by the policy chosen
            validate_robot_cameras_for_policy(lerobot_features, policy_image_features)

        # Use environment variable if server_address is not provided in config
        self.server_address = config.server_address

        self.policy_config = RemotePolicyConfig(
            config.policy_type,
            config.pretrained_name_or_path,
            lerobot_features,
            config.actions_per_chunk,
            config.policy_device,
        )
        self.channel = grpc.insecure_channel(
            self.server_address, grpc_channel_options(initial_backoff=f"{config.environment_dt:.4f}s")
        )
        self.stub = async_inference_pb2_grpc.AsyncInferenceStub(self.channel)
        self.logger.info(f"Initializing client to connect to server at {self.server_address}")

        self._running_event = threading.Event()

        # Initialize client side variables
        self.latest_action_lock = threading.Lock()
        self.latest_action = -1
        self.action_chunk_size = -1

        self._chunk_size_threshold = config.chunk_size_threshold

        self.action_queue = Queue()
        self.action_queue_lock = threading.Lock()  # Protect queue operations
        self.action_queue_size = []
        self.start_barrier = threading.Barrier(2)  # 2 threads: action receiver, control loop

        # FPS measurement
        self.fps_tracker = FPSTracker(target_fps=self.config.fps)

        self.logger.info("Robot connected and ready")

        # Use an event for thread-safe coordination
        self.must_go = threading.Event()
        self.must_go.set()  # Initially set - observations qualify for direct processing

@draccus.wrap()
def async_client(cfg: RobotClientConfig):
    logging.info(pformat(asdict(cfg)))

    if cfg.robot.type not in SUPPORTED_ROBOTS:
        raise ValueError(f"Robot {cfg.robot.type} not yet supported!")

    client = PingtiRobotClient(cfg)

    if client.start():
        client.logger.info("Starting action receiver thread...")

        # Create and start action receiver thread
        action_receiver_thread = threading.Thread(target=client.receive_actions, daemon=True)

        # Start action receiver thread
        action_receiver_thread.start()

        try:
            # The main thread runs the control loop
            client.control_loop(task=cfg.task)

        finally:
            client.stop()
            action_receiver_thread.join()
            if cfg.debug_visualize_queue_size:
                visualize_action_queue_size(client.action_queue_size)
            client.logger.info("Client stopped")


if __name__ == "__main__":
    async_client()  # run the client
