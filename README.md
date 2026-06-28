## Introduction
`pingti_lerobot_bridge` enables the use of Lerobot for calibration, teleoperation, data collection, and other tasks on the PingTi Follower Arm and SO-ARM100 Leader Arm.

## Robot Variants

This package provides two single-arm PingTi follower variants:

- `pingti_follower`: standard PingTi Arm with higher-torque `sts3250` shoulder lift motors.
- `pingti_lite_follower`: Lite PingTi Arm variant with lower-torque `sts3215` shoulder lift motors.

Use the variant through LeRobot CLI's `--robot.type` argument, for example `--robot.type=pingti_lite_follower`.

## Docs
- [pingti_lerobot_bridge tutorial](./docs/pingti_lerobot_bridge_tutorial.md)

## Development

### Running Unit Tests

The project includes comprehensive unit tests to ensure code quality and functionality. All tests are written using Python's built-in `unittest` framework.

#### Prerequisites

Use Python 3.12 and make sure you have all dependencies installed. This installs LeRobot from PyPI:
```bash
pip install -e .
```

#### Running All Tests

To run all unit tests in the project:
```bash
python -m unittest discover tests -v
```

#### Running Specific Test Files

To run tests from a specific test file:
```bash
# Run all tests in test_pingti_follower.py
python -m unittest tests.test_pingti_follower -v

# Run all tests in test_bi_pingti_follower.py
python -m unittest tests.test_bi_pingti_follower -v

# Run external tests
python -m unittest tests.external.test_NongMobileManipulator -v
```
