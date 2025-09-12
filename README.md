## Introduction
`pingti_lerobot_bridge` enables the use of Lerobot for calibration, teleoperation, data collection, and other tasks on the PingTi Follower Arm and SO-ARM100 Leader Arm.

## Docs
- [pingti_lerobot_bridge tutorial](./docs/pingti_lerobot_bridge_tutorial.md)

## Development

### Running Unit Tests

The project includes comprehensive unit tests to ensure code quality and functionality. All tests are written using Python's built-in `unittest` framework.

#### Prerequisites

Make sure you have all dependencies installed:
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
