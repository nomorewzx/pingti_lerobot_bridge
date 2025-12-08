# Data Collection & Visualization Guide for LeRobot

This guide covers common scenarios and solutions when collecting data using LeRobot's recording functionality.

## Basic Recording Controls

If you want to use the Hugging Face hub features for uploading your dataset and you haven't previously done it, make sure you've logged in using a write-access token, which can be generated from the [Hugging Face settings](https://huggingface.co/settings/tokens):
```bash
huggingface-cli login --token ${HUGGINGFACE_TOKEN} --add-to-git-credential
```

Store your Hugging Face repository name in a variable to run these commands:
```bash
HF_USER=$(huggingface-cli whoami | head -n 1)
echo $HF_USER
```

Run command below for recording:

```bash
lerobot-record \
    --robot.type=pingti_follower \
    --robot.port=/dev/ttyUSB0 \
    --robot.cameras="{ front: {type: opencv, index_or_path: 4, width: 640, height: 480, fps: 30}}" \
    --robot.id=my_pingti_follower \
    --dataset.repo_id=zhenxuan/datasetv3_test \
    --dataset.num_episodes=2 \
    --dataset.single_task="Grab the cube" \
    --display_data=true \
    --teleop.type=so100_leader \
    --teleop.port=/dev/ttyACM0 \
    --teleop.id=blue
```

### Keyboard Shortcuts
- `→` (Right Arrow): End current episode early and proceed to environment reset
- `←` (Left Arrow): End current episode and re-record it
- `ESC`: Stop the entire recording process
- `CTRL+C`: Emergency stop (not recommended for normal use)

### Episode Duration
- Set by `--control.episode_time_s` parameter
- Can be shorter if ended early with `→` key
- Actual duration is properly saved in the dataset

## Common Scenarios and Solutions

### 1. Quality Control During Recording

#### Scenario: Current Episode is Not Good
Press `←` to immediately stop and restart the episode

#### Scenario: Need to Take a Break
- Press `ESC` to stop recording
- Use `--control.resume=true` when continuing later


### 2. Best Practices Before Starting
- Plan your total number of episodes ( --control.num_episodes )
- Set appropriate episode duration ( --control.episode_time_s )
- Consider setting longer duration and using → to end episodes early During Recording
- Monitor recording quality in real-time
- Use ← for immediate re-recording if needed
- Take breaks between episodes during reset phase After Recording

## Replay an episode

Run command below for replaying episode

```bash
lerobot-replay \
    --robot.type=pingti_follower \
    --robot.port=/dev/ttyUSB0 \
    --robot.id=my_pingti_follower \
    --dataset.repo_id=zhenxuan/datasetv3_test \
    --dataset.episode=0
```

## Visualization

Visualize the repository using the following command 
```
lerobot-dataset-viz     --repo-id zhenxuan/datasetv3_test     --episode-index 1
```
