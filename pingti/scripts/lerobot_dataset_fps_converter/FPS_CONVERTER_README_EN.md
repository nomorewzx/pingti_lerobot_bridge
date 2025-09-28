# LeRobot FPS Converter

## Features
Convert LeRobot datasets from 30 FPS to 15 FPS while maintaining data synchronization.

## Usage

### Basic Usage
```bash
python lerobot_fps_converter.py \
    --input_dataset_path /path/to/your/30fps/dataset \
    --output_dataset_path /path/to/your/15fps/dataset \
    --source_fps 30 \
    --target_fps 15 \
    --max_workers 4
```

### Parameters
- `--input_dataset_path`: Input dataset path (30 FPS)
- `--output_dataset_path`: Output dataset path (15 FPS)
- `--source_fps`: Source FPS (default: 30)
- `--target_fps`: Target FPS (default: 15)
- `--max_workers`: Number of parallel workers (default: 4)

## Conversion Contents
- ✅ Video files: Downsampling using FFmpeg
- ✅ Parquet files: Downsample data and recalculate timestamps
- ✅ Metadata: Update info.json and episodes.jsonl
- ✅ Index continuity: Ensure global continuous index column

## Notes
- Ensure FFmpeg is installed: `sudo apt install ffmpeg`
- Ensure Python dependencies are installed: `pip install pandas numpy`
- Data integrity is maintained during conversion

## Examples

### Convert Dataset
```bash
# Convert dataset
python lerobot_fps_converter.py \
    --input_dataset_path /home/user/dataset_30fps \
    --output_dataset_path /home/user/dataset_15fps \
    --source_fps 30 \
    --target_fps 15 \
    --max_workers 4
```

### Verify Conversion Results
```bash
# Verify conversion results
python -c "
from lerobot.datasets.lerobot_dataset import LeRobotDataset
dataset = LeRobotDataset('/path/to/15fps/dataset')
print(f'FPS: {dataset.fps}')
print(f'Total frames: {dataset.num_frames}')
print('Dataset loaded successfully!')
"
```

## Supported Conversions
- 30 FPS → 15 FPS (2:1 downsampling)
- 30 FPS → 10 FPS (3:1 downsampling)
- 60 FPS → 30 FPS (2:1 downsampling)
- Other integer ratio downsampling

## Troubleshooting

### Common Issues
1. **FFmpeg not found**
   ```bash
   sudo apt update && sudo apt install ffmpeg
   ```

2. **Missing Python dependencies**
   ```bash
   pip install pandas numpy
   ```

3. **Permission errors**
   ```bash
   chmod +x lerobot_fps_converter.py
   ```

### Performance Optimization
- Increase `--max_workers` parameter to utilize multi-core CPU
- Ensure sufficient disk space for converted dataset
- Use SSD storage for better I/O performance
