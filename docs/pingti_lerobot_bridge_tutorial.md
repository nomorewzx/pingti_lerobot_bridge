## 1. (Prerequsite) Install Lerobot
If you have not installed lerobot, pls follow steps below, you can also see [Lerobot Tutorial](pingti/scripts/control_pingti_robot.py) for more in detail explaination
### A. Clone Lerobot
```
git clone https://github.com/huggingface/lerobot.git
```
### B. Create virtual environment
```
conda create -y -n lerobot python=3.10
```
### C. Install Lerobot with feetech sdk
`conda activate lerobot`

`cd ~/lerobot && pip install -e ".[feetech]"`

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

The directory structure should be like below:

```
your_workspace_dir/
    lerobot/
    pingti_lerobot_bridge/
```

### B. Install pingti_lerobot_bridge

Make sure you are in the python virtual env `lerobot`. Then run command below

```
cd ./pingti_lerobot_bridge && pip install -e .
```

## 3. Calibration

>**Note**: You need to identify the port number of control board of PingTi Arm and control board of SO-ARM100. See [Lerobot tutorial for finding port](https://github.com/huggingface/lerobot/blob/main/examples/10_use_so100.md#c-configure-the-motors)

### A. Calibrate SO100_Leader

Following [Lerobot SO100 Leader calibration](https://huggingface.co/docs/lerobot/main/en/so100#leader)

Run command below and replace the port number with your own port

```
python -m lerobot.calibrate \
    --teleop.type=so100_leader \
    --teleop.port=/dev/tty.usbmodem58A60699971 \
    --teleop.id=blue
```

See video of calibration process can be found [here](https://huggingface.co/docs/lerobot/en/so101#calibration-video)

### B. Calibrate Pingti_Follower

Run command below, replace port number with your own port number

```
python -m pingti.calibrate \                                               
    --robot.type=pingti_follower \
    --robot.port=/dev/tty.usbserial-A50285BI \
    --robot.id=my_pingti_follower  
```

Also move the joints of pingti arm similar to the video of so101 [here](https://huggingface.co/docs/lerobot/en/so101#calibration-video)


## 4. Run teleoperation

Run teleoperation (replace port with your own port)

```bash
python -m pingti.teleoperate \                                              
    --robot.type=pingti_follower \
    --robot.port=/dev/tty.usbserial-A50285BI \
    --robot.id=my_pingti_follower \
    --teleop.type=so100_leader \
    --teleop.port=/dev/tty.usbmodem58A60699971 \
    --teleop.id=blue
```

And then you can run teleopearation with camera using command below, which is helpful to visualize whether camera captures the teleoperation scene properly before actually collecting dataset

```bash
python -m pingti.teleoperate \
    --robot.type=pingti_follower \
    --robot.port=/dev/tty.usbserial-A50285BI \
    --robot.id=my_pingti_follower \
    --robot.cameras="{ front: {type: opencv, index_or_path: 0, width: 1920, height: 1080, fps: 30}}" \
    --teleop.type=so100_leader \
    --teleop.port=/dev/tty.usbmodem58A60699971 \
    --teleop.id=blue \
    --display_data=true
```

You can refer [here](https://huggingface.co/docs/lerobot/main/en/cameras#finding-your-camera) to find your available cameras

## 5. Record dataset & Visualize Dataset

Follow [Data Record Guideline](./data_record_guide.md) for recording dataset

## 6. Policy training

You can follow [Lerobot Tutorial](https://huggingface.co/docs/lerobot/getting_started_real_world_robot#train-a-policy) for below tasks:

To train a diffusion model 

```bash
python -m lerobot.scripts.train \
  --dataset.repo_id=${HF_USER}/put_battery_into_basket \
  --policy.type=diffusion \
  --output_dir=outputs/train/duffusion_battery \
  --job_name=diffusion_battery \
  --policy.device=cuda \
  --wandb.enable=true \
  --policy.repo_id=${HF_USER}/diffusion_battery
```

## 7. Eval a Trained Policy

SImilar to [Lerobot SO101: Evaluate your policy](https://huggingface.co/docs/lerobot/getting_started_real_world_robot#evaluate-your-policy), we use `record` command to evaluate a trained policy for PingTi arm

```bash
python -m pingti.record  \
  --robot.type=pingti_follower \
  --robot.port=/dev/tty.usbserial-A50285BI \
  --robot.cameras="{front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, overhead: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30}}" \
  --robot.id=my_pingti_follower \
  --display_data=true \
  --dataset.num_episodes=10 \
  --dataset.repo_id=${HF_USER}/eval_diffusion_put_battery_into_basket \
  --dataset.single_task="Put blue battery into small storage basket" \
  --policy.path=${HF_USER}/diffusion_battery
```
## 8. Async Inference

Using below command to run async inference like [Lerobot Asynchronous Inference on SO-100/101 arms](https://huggingface.co/docs/lerobot/en/async)

### Robot Server
```bash
python src/lerobot/scripts/server/policy_server.py \
    --host=127.0.0.1 \          
    --port=8080
```

### Robot Client

Below command starts a `act` model on server

```bash
python pingti/scripts/server/pingti_robot_client.py \
    --server_address=127.0.0.1:8080 \
    --robot.type=pingti_follower \
    --robot.port=/dev/tty.usbserial-A50285BI \
    --robot.id=my_pingti_follower \
    --robot.cameras="{front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, overhead: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30}}" \
    --task="Put blue battery into small storage basket" \
    --policy_type=act \
    --pretrained_name_or_path= ${HF_USER}/model_to_eval \ # Pretrained model name or path
    --policy_device=mps \
    --actions_per_chunk=50 \
    --chunk_size_threshold=0.5 \
    --aggregate_fn_name=weighted_average \
    --debug_visualize_queue_size=True
```