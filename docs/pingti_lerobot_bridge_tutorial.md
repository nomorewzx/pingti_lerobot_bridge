## 1. Create a Python 3.12 environment

`pingti_lerobot_bridge` depends on the PyPI release of LeRobot. You no longer need a local editable checkout of `huggingface/lerobot`.

Using `venv`:

```bash
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
```

Or using conda:

```bash
conda create -y -n pingti_lerobot python=3.12
conda activate pingti_lerobot
python -m pip install --upgrade pip
```

## 2. Install pingti_lerobot_bridge

### A. Clone repo

`cd` to workspace dir:

```
cd ../
```

Clone repo

```
git clone https://github.com/nomorewzx/pingti_lerobot_bridge.git
```

### B. Install pingti_lerobot_bridge

Make sure you are in the Python 3.12 environment. Then run command below:

```
cd ./pingti_lerobot_bridge
pip install .
```

This installs `lerobot[async,feetech,transformers-dep]==0.5.1` from PyPI.

## 3. Calibration

>**Note**: You need to identify the port number of control board of PingTi Arm and control board of SO-ARM100. See [Lerobot tutorial for finding port](https://github.com/huggingface/lerobot/blob/main/examples/10_use_so100.md#c-configure-the-motors)

### A. Calibrate SO100_Leader

Following [Lerobot SO100 Leader calibration](https://huggingface.co/docs/lerobot/main/en/so100#leader)


```
lerobot-calibrate \
    --teleop.type=so100_leader \
    --teleop.port=/dev/ttyACM0 \
    --teleop.id=blue
```

See video of calibration process can be found [here](https://huggingface.co/docs/lerobot/en/so101#calibration-video)

### B. Calibrate Pingti_Follower

Run command below, replace port number with your own port number.

```
lerobot-calibrate \
    --robot.type=pingti_follower \
    --robot.port=/dev/ttyUSB0 \
    --robot.id=my_pingti_follower  
```

Also move the joints of pingti arm similar to the video of so101 [here](https://huggingface.co/docs/lerobot/en/so101#calibration-video)

For the lower-torque shoulder variant, use `--robot.type=pingti_lite_follower`.


## 4. Run teleoperation

Run teleoperation (replace port with your own port)

```bash
lerobot-teleoperate \
  --robot.type=pingti_follower \
  --robot.port=/dev/ttyUSB0 \
  --robot.id=my_pingti_follower \
  --teleop.type=so100_leader \
  --teleop.port=/dev/ttyACM0 \
  --teleop.id=blue \
  --display_data=false
```

And then you can run teleopearation with camera using command below, which is helpful to visualize whether camera captures the teleoperation scene properly before actually collecting dataset

```bash
lerobot-teleoperate \
    --robot.type=pingti_follower \
    --robot.port=/dev/ttyUSB0 \
    --robot.cameras="{ front: {type: opencv, index_or_path: 4, width: 640, height: 480, fps: 30}}" \
    --robot.id=my_pingti_follower \
    --teleop.type=so100_leader \
    --teleop.port=/dev/ttyACM0 \
    --teleop.id=blue \
    --display_data=true
```

You can refer [here](https://huggingface.co/docs/lerobot/main/en/cameras#finding-your-camera) to find your available cameras

## 5. Record dataset & Visualize Dataset

Follow [Data Record Guideline](./data_record_guide.md) for recording dataset

## 6. Policy training

You can follow [Lerobot Tutorial](https://huggingface.co/docs/lerobot/getting_started_real_world_robot#train-a-policy) for training and evaluation of policies:


## 8. Async Inference

Using below command to run async inference like [Lerobot Asynchronous Inference on SO-100/101 arms](https://huggingface.co/docs/lerobot/en/async)

### Robot Server
```bash
python -m lerobot.async_inference.policy_server \
    --host=127.0.0.1 \
    --port=8080
```

### Robot Client

Below command starts a `act` model on server

```bash
python pingti/scripts/server/pingti_robot_client.py \
    --server_address=127.0.0.1:8080 \
    --robot.type=pingti_follower \
    --robot.port=/dev/ttyUSB0 \
    --robot.id=my_pingti_follower \
    --robot.cameras="{front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, overhead: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30}}" \
    --task="Put blue battery into small storage basket" \
    --policy_type=act \
    --pretrained_name_or_path=${HF_USER}/model_to_eval \
    --policy_device=mps \
    --client_device=cpu \
    --actions_per_chunk=50 \
    --chunk_size_threshold=0.5 \
    --aggregate_fn_name=weighted_average \
    --debug_visualize_queue_size=True
```
