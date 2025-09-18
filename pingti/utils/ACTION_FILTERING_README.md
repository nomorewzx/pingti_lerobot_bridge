# Action Filtering for SO100Follower Robot

This document describes the action filtering functionality added to the SO100Follower robot to reduce jitter and improve control smoothness.

## Overview

The SO100Follower robot now includes built-in action filtering capabilities that can smooth out noisy or jittery actions before sending them to the motors. This helps reduce mechanical vibrations and provides more stable robot control.

## Features

- **Multiple Filter Types**: Low-pass, moving average, and adaptive filters
- **Configurable Parameters**: Adjustable smoothing factors and thresholds
- **Easy Integration**: Seamlessly integrated into the existing robot interface
- **Backward Compatible**: Can be disabled by setting filter type to "none"

## Filter Types

### 1. Low-Pass Filter
Applies exponential smoothing to reduce high-frequency noise while preserving action trends.

**Configuration:**
```python
config = SO100FollowerConfig(
    port="/dev/ttyUSB0",
    action_filter_type="lowpass",
    action_filter_alpha=0.3  # 0.1 = more smoothing, 0.9 = less smoothing
)
```

**Best for:** General-purpose smoothing with predictable behavior

### 2. Moving Average Filter
Maintains a sliding window of recent actions and returns the average for each motor.

**Configuration:**
```python
config = SO100FollowerConfig(
    port="/dev/ttyUSB0",
    action_filter_type="moving_average",
    action_filter_window_size=3  # Number of recent actions to average
)
```

**Best for:** Simple smoothing with minimal computational overhead

### 3. Adaptive Filter
Combines low-pass and moving average filtering, automatically adjusting smoothing based on action magnitude.

**Configuration:**
```python
config = SO100FollowerConfig(
    port="/dev/ttyUSB0",
    action_filter_type="adaptive",
    action_filter_alpha=0.3,  # Base smoothing factor
    action_filter_adaptation_threshold=0.1  # Threshold for adaptation
)
```

**Best for:** Intelligent smoothing that adapts to different action patterns

### 4. No Filter
Disables filtering (default behavior).

**Configuration:**
```python
config = SO100FollowerConfig(
    port="/dev/ttyUSB0",
    action_filter_type="none"
)
```

## Usage Examples

### Basic Usage

```python
from lerobot.robots.so100_follower.config_so100_follower import SO100FollowerConfig
from lerobot.robots.so100_follower.so100_follower import SO100Follower

# Create robot with filtering
config = SO100FollowerConfig(
    port="/dev/ttyUSB0",
    action_filter_type="lowpass",
    action_filter_alpha=0.3
)

robot = SO100Follower(config)
robot.connect()

# Actions are automatically filtered
action = {
    "shoulder_pan.pos": 0.5,
    "shoulder_lift.pos": 0.3,
    "elbow_flex.pos": 0.2,
    "wrist_flex.pos": 0.1,
    "wrist_roll.pos": 0.0,
    "gripper.pos": 50.0
}

sent_action = robot.send_action(action)  # Filtering happens here
```

### Resetting Filter State

```python
# Reset filter at the start of a new task
robot.reset_action_filter()
```

### Policy Integration

```python
# When using with policies, filtering is transparent
observation = robot.get_observation()
action = policy.select_action(observation)
robot.send_action(action)  # Automatically filtered
```

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `action_filter_type` | str | "none" | Filter type: "none", "lowpass", "moving_average", "adaptive" |
| `action_filter_alpha` | float | 0.3 | Smoothing factor for low-pass and adaptive filters (0 < α ≤ 1) |
| `action_filter_window_size` | int | 3 | Window size for moving average filter |
| `action_filter_adaptation_threshold` | float | 0.1 | Threshold for adaptive filter adaptation |

## Performance Impact

- **Computational Overhead**: Minimal (O(1) for low-pass, O(n) for moving average where n=window_size)
- **Memory Usage**: Small (stores previous values and action history)
- **Latency**: Negligible (microseconds per action)

## Recommendations

### For Different Use Cases

1. **General Robotics Tasks**
   ```python
   action_filter_type="lowpass"
   action_filter_alpha=0.3
   ```

2. **High-Precision Manipulation**
   ```python
   action_filter_type="adaptive"
   action_filter_alpha=0.2
   action_filter_adaptation_threshold=0.05
   ```

3. **Simple Smoothing**
   ```python
   action_filter_type="moving_average"
   action_filter_window_size=3
   ```

4. **Maximum Smoothness**
   ```python
   action_filter_type="lowpass"
   action_filter_alpha=0.1
   ```

### Tuning Guidelines

1. **Start Conservative**: Begin with α=0.3 for low-pass filters
2. **Monitor Performance**: Watch for oversmoothing (delayed response) or undersmoothing (still jittery)
3. **Task-Specific Tuning**: Adjust based on your specific robot and task requirements
4. **Reset Between Tasks**: Always call `reset_action_filter()` when starting new tasks

## Troubleshooting

### Robot Still Jittery
- Decrease `action_filter_alpha` (try 0.1-0.2)
- Increase `action_filter_window_size` for moving average
- Check if `max_relative_target` is too high

### Robot Response Too Slow
- Increase `action_filter_alpha` (try 0.5-0.7)
- Decrease `action_filter_window_size`
- Consider using adaptive filter

### Filter Not Working
- Verify `action_filter_type` is not "none"
- Check that actions contain `.pos` keys
- Ensure filter is not being reset between actions

## Testing

Run the test script to see filter performance:

```bash
python -m tests.utils.test_action_filters
```

This will generate comparison plots showing the effectiveness of different filter types.

## Implementation Details

The filtering is implemented in:
- `pingti/utils/action_filters.py`: Filter classes and factory function
- `pingti/robots/so100_follower/so100_follower.py`: Integration with robot
- `pingti/robots/so100_follower/config_so100_follower.py`: Configuration options

The filtering happens in the `send_action` method before the action is sent to the motors, ensuring all actions are consistently filtered.
