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

#### Running Individual Test Classes

To run a specific test class:
```bash
# Run only TestBiPingtiFollowerMotorsFt tests
python -m unittest tests.test_bi_pingti_follower.TestBiPingtiFollowerMotorsFt -v
```

#### Running Individual Test Methods

To run a specific test method:
```bash
# Run only the test_motors_ft_filters_out_secondary_motors test
python -m unittest tests.test_bi_pingti_follower.TestBiPingtiFollowerMotorsFt.test_motors_ft_filters_out_secondary_motors -v
```

#### Test Coverage

The test suite covers:
- **PingtiFollower**: Basic robot functionality, motor control, and mirror joint behavior
- **BiPingtiFollower**: Bimanual robot functionality, motor filtering, and feature extraction
- **External Components**: NongMobileManipulator and other external integrations

#### Test Output

Tests run with verbose output (`-v` flag) to show:
- Test names and descriptions
- Pass/fail status
- Execution time
- Any error details for failed tests

Example output:
```
test_motors_ft_filters_out_secondary_motors (tests.test_bi_pingti_follower.TestBiPingtiFollowerMotorsFt)
Test that _motors_ft property filters out motors ending with 'secondary' ... ok
test_motors_ft_with_only_primary_motors (tests.test_bi_pingti_follower.TestBiPingtiFollowerMotorsFt)
Test _motors_ft when there are no secondary motors ... ok
----------------------------------------------------------------------
Ran 2 tests in 0.004s
OK
```