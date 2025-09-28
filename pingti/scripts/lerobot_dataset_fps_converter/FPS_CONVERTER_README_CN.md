# LeRobot FPS 转换工具

## 功能
将 LeRobot 数据集从 30 FPS 转换为 15 FPS，保持数据同步性。

## 使用方法

### 基本用法
```bash
python lerobot_fps_converter.py \
    --input_dataset_path /path/to/your/30fps/dataset \
    --output_dataset_path /path/to/your/15fps/dataset \
    --source_fps 30 \
    --target_fps 15 \
    --max_workers 4
```

### 参数说明
- `--input_dataset_path`: 输入数据集路径（30 FPS）
- `--output_dataset_path`: 输出数据集路径（15 FPS）
- `--source_fps`: 源 FPS（默认：30）
- `--target_fps`: 目标 FPS（默认：15）
- `--max_workers`: 并行工作线程数（默认：4）

## 转换内容
- ✅ 视频文件：使用 FFmpeg 下采样
- ✅ Parquet 文件：下采样数据并重新计算时间戳
- ✅ 元数据：更新 info.json 和 episodes.jsonl
- ✅ 索引连续性：确保 index 列全局连续

## 注意事项
- 确保 FFmpeg 已安装：`sudo apt install ffmpeg`
- 确保 Python 依赖已安装：`pip install pandas numpy`
- 转换过程中会保持数据完整性

## 示例

### 转换数据集
```bash
# 转换数据集
python lerobot_fps_converter.py \
    --input_dataset_path /home/user/dataset_30fps \
    --output_dataset_path /home/user/dataset_15fps \
    --source_fps 30 \
    --target_fps 15 \
    --max_workers 4
```

### 验证转换结果
```bash
# 验证转换结果
python -c "
from lerobot.datasets.lerobot_dataset import LeRobotDataset
dataset = LeRobotDataset('/path/to/15fps/dataset')
print(f'FPS: {dataset.fps}')
print(f'Total frames: {dataset.num_frames}')
print('数据集加载成功！')
"
```

## 支持的转换
- 30 FPS → 15 FPS（2:1 下采样）
- 30 FPS → 10 FPS（3:1 下采样）
- 60 FPS → 30 FPS（2:1 下采样）
- 其他整数倍下采样

## 故障排除

### 常见问题
1. **FFmpeg 未找到**
   ```bash
   sudo apt update && sudo apt install ffmpeg
   ```

2. **Python 依赖缺失**
   ```bash
   pip install pandas numpy
   ```

3. **权限错误**
   ```bash
   chmod +x lerobot_fps_converter.py
   ```

### 性能优化
- 增加 `--max_workers` 参数以利用多核 CPU
- 确保有足够的磁盘空间存储转换后的数据集
- 使用 SSD 存储以提高 I/O 性能
