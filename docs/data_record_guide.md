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
Note: use `push_to_hub=true` to upload the dataset to the Hugging Face hub.
```bash
python pingti/scripts/control_pingti_robot.py \
  --robot.type=pingti \
  --control.type=record \
  --control.fps=30 \
  --control.single_task="Put the remote controller into the basket" \
  --control.repo_id=${HF_USER}/pingti_test \
  --control.tags='["pingti","tutorial"]' \
  --control.warmup_time_s=5 \
  --control.episode_time_s=60 \
  --control.reset_time_s=30 \
  --control.num_episodes=10 \
  --control.push_to_hub=false
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

### 2. Managing Recorded Episodes

#### Resuming Recording
To continue recording where you left off:

```bash
python lerobot/scripts/control_robot.py \
    --robot.type=so100 \
    --control.type=record \
    --control.fps=30 \
    --control.repo_id=$USER/dataset_name \
    --control.resume=true \
    [other original parameters]
 ```

### 3. Best Practices Before Starting
- Plan your total number of episodes ( --control.num_episodes )
- Set appropriate episode duration ( --control.episode_time_s )
- Consider setting longer duration and using → to end episodes early During Recording
- Monitor recording quality in real-time
- Use ← for immediate re-recording if needed
- Take breaks between episodes during reset phase After Recording

## Visualization

Visualize the repository using the following command (`cd` to `lerobot` dir before running below command):
```
python lerobot/scripts/visualize_dataset_html.py \
    --repo-id $USER/dataset_name
```
Go to below link to visualize the dataset

http://localhost:9090

You will see something like below:
![image](../media/visualization_repo.jpg)
