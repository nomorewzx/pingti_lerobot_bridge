
`git clone {}`

`conda create -y -n pingti_arm python=3.10`

`conda activate pingti_arm`


Install Lerobot
`git clone https://github.com/huggingface/lerobot.git`
`cd lerobot`
`pip install -e ".[feetech]"`

Install Pingti library
`cd ../`
`pip install -e .`

Calibrate PingTi follower arm and SO-ARM100 leader arm

Find ports

Update the port number in `pingti/common/configs.py`

Calibration command

```
python pingti/scripts/control_pingti_robot.py \
  --robot.type=pingti \
  --robot.cameras='{}' \
  --control.type=calibrate \
  --control.fps=30

```

Run teleoperation
```
python pingti/scripts/control_pingti_robot.py \
  --robot.type=pingti \
  --robot.cameras='{}' \
  --control.type=teleoperate \
  --control.fps=30
```