#!/usr/bin/env python3
"""
LeRobot Dataset FPS Converter
Converts LeRobot datasets from one FPS to another while maintaining data synchronization.
Uses FFmpeg for video conversion and Python for parquet data and metadata processing.
"""

import argparse
import json
import logging
import shutil
import subprocess
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed
import time
import pandas as pd
import numpy as np

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


def run_ffmpeg_conversion(input_file: Path, output_file: Path, downsample_factor: int, target_fps: int) -> tuple[bool, str]:
    """Run FFmpeg conversion for a single video"""
    
    # Ensure output directory exists
    output_file.parent.mkdir(parents=True, exist_ok=True)
    
    # Build FFmpeg command
    cmd = [
        'ffmpeg', '-i', str(input_file),
        '-vf', f'select=not(mod(n\\,{downsample_factor}))',
        '-r', str(target_fps),
        '-c:v', 'libx264',
        '-preset', 'medium',
        '-crf', '23',
        '-c:a', 'aac',
        '-b:a', '128k',
        '-y',  # Overwrite output file
        str(output_file)
    ]
    
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, check=True)
        return True, ""
    except subprocess.CalledProcessError as e:
        return False, e.stderr


def convert_single_video(args) -> dict:
    """Convert a single video file (for multithreading)"""
    input_file, output_file, downsample_factor, target_fps, index, total = args
    
    logger.info(f"[{index}/{total}] Converting: {input_file.name}")
    
    success, error = run_ffmpeg_conversion(input_file, output_file, downsample_factor, target_fps)
    
    return {
        'input_file': input_file,
        'output_file': output_file,
        'success': success,
        'error': error,
        'index': index
    }


def update_dataset_metadata(input_dir: Path, output_dir: Path, source_fps: int, target_fps: int, actual_episode_lengths: list):
    """Update dataset metadata"""
    
    downsample_factor = source_fps // target_fps
    
    # Update info.json
    info_path = input_dir / "meta" / "info.json"
    output_info_path = output_dir / "meta" / "info.json"
    
    if info_path.exists():
        with open(info_path, 'r') as f:
            info = json.load(f)
        
        info['fps'] = target_fps
        info['total_frames'] = sum(actual_episode_lengths)
        
        with open(output_info_path, 'w') as f:
            json.dump(info, f, indent=2)
        
        logger.info(f"Updated info.json: FPS {source_fps} -> {target_fps}, total frames: {info['total_frames']}")
    
    # Update episodes.jsonl
    episodes_path = input_dir / "meta" / "episodes.jsonl"
    output_episodes_path = output_dir / "meta" / "episodes.jsonl"
    
    if episodes_path.exists():
        episodes = []
        with open(episodes_path, 'r') as f:
            for line in f:
                episode = json.loads(line.strip())
                episodes.append(episode)
        
        # Update length using actual row counts
        for i, episode in enumerate(episodes):
            if i < len(actual_episode_lengths):
                episode['length'] = actual_episode_lengths[i]
        
        with open(output_episodes_path, 'w') as f:
            for episode in episodes:
                f.write(json.dumps(episode) + '\n')
        
        logger.info(f"Updated episodes.jsonl: using actual row counts {actual_episode_lengths}")


def convert_parquet_file(parquet_path: Path, source_fps: int, target_fps: int, global_start_index: int):
    """Convert timestamps in parquet file"""
    
    logger.info(f"Converting parquet: {parquet_path.name}")
    
    # Read parquet file
    df = pd.read_parquet(parquet_path)
    
    # Calculate downsampling factor
    downsample_factor = source_fps // target_fps
    
    # Downsample data
    downsampled_df = df.iloc[::downsample_factor].copy()
    
    # Recalculate timestamps and frame indices, maintaining relative time relationships
    downsampled_df['frame_index'] = np.arange(len(downsampled_df))
    downsampled_df['timestamp'] = downsampled_df['frame_index'] / target_fps
    
    # Recalculate globally continuous index
    downsampled_df['index'] = np.arange(global_start_index, global_start_index + len(downsampled_df))
    
    # Save updated file
    downsampled_df.to_parquet(parquet_path, index=False)
    
    logger.info(f"Parquet conversion completed: {len(df)} -> {len(downsampled_df)} rows, index range: {downsampled_df['index'].min()} - {downsampled_df['index'].max()}")
    
    return len(downsampled_df)


def batch_convert_dataset(input_dir: Path, output_dir: Path, source_fps: int, target_fps: int, max_workers: int = 4):
    """Batch convert dataset"""
    
    logger.info(f"Starting batch conversion: {source_fps} FPS -> {target_fps} FPS")
    
    downsample_factor = source_fps // target_fps
    if source_fps % target_fps != 0:
        raise ValueError(f"Cannot downsample from {source_fps} FPS to {target_fps} FPS (not integer ratio)")
    
    # Create output directory
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Copy meta files
    meta_src = input_dir / "meta"
    meta_dst = output_dir / "meta"
    if meta_src.exists():
        shutil.copytree(meta_src, meta_dst, dirs_exist_ok=True)
        logger.info("Copied meta directory")
    
    # Copy and convert data files
    data_src = input_dir / "data"
    data_dst = output_dir / "data"
    if data_src.exists():
        shutil.copytree(data_src, data_dst, dirs_exist_ok=True)
        
        parquet_files = list(data_dst.rglob("*.parquet"))
        logger.info(f"Found {len(parquet_files)} parquet files")
        
        # Calculate global start index for each episode
        episodes_path = input_dir / "meta" / "episodes.jsonl"
        with open(episodes_path, 'r') as f:
            episodes = [json.loads(line.strip()) for line in f]
        
        # Calculate start index for each episode
        global_start_index = 0
        episode_start_indices = []
        for episode in episodes:
            episode_start_indices.append(global_start_index)
            global_start_index += episode['length'] // downsample_factor
        
        # Convert each parquet file and collect actual row counts
        actual_episode_lengths = [0] * len(episodes)
        for parquet_file in parquet_files:
            try:
                # Extract episode index from filename
                episode_index = int(parquet_file.stem.split('_')[-1])
                start_index = episode_start_indices[episode_index]
                actual_length = convert_parquet_file(parquet_file, source_fps, target_fps, start_index)
                actual_episode_lengths[episode_index] = actual_length
            except Exception as e:
                logger.error(f"Failed to convert parquet {parquet_file}: {e}")
    else:
        logger.warning("Data directory not found")
    
    # Find all video files
    videos_dir = input_dir / "videos"
    if not videos_dir.exists():
        logger.warning("Videos directory not found")
        return
    
    video_files = list(videos_dir.rglob("*.mp4"))
    logger.info(f"Found {len(video_files)} video files")
    
    if not video_files:
        logger.warning("No video files found")
        return
    
    # Prepare conversion tasks
    tasks = []
    for i, video_file in enumerate(video_files, 1):
        rel_path = video_file.relative_to(videos_dir)
        output_file = output_dir / "videos" / rel_path
        tasks.append((video_file, output_file, downsample_factor, target_fps, i, len(video_files)))
    
    # Execute conversion
    start_time = time.time()
    success_count = 0
    failed_count = 0
    failed_files = []
    
    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        # Submit all tasks
        future_to_task = {executor.submit(convert_single_video, task): task for task in tasks}
        
        # Process completed tasks
        for future in as_completed(future_to_task):
            result = future.result()
            
            if result['success']:
                success_count += 1
                logger.info(f"✅ [{result['index']}/{len(video_files)}] {result['input_file'].name}")
            else:
                failed_count += 1
                failed_files.append(result['input_file'])
                logger.error(f"❌ [{result['index']}/{len(video_files)}] {result['input_file'].name}: {result['error']}")
    
    # Update metadata
    update_dataset_metadata(input_dir, output_dir, source_fps, target_fps, actual_episode_lengths)
    
    # Output results
    elapsed_time = time.time() - start_time
    logger.info("=" * 60)
    logger.info("Batch conversion completed!")
    logger.info(f"Total files: {len(video_files)}")
    logger.info(f"Success: {success_count}")
    logger.info(f"Failed: {failed_count}")
    logger.info(f"Time elapsed: {elapsed_time:.2f} seconds")
    logger.info(f"Average speed: {len(video_files)/elapsed_time:.2f} files/second")
    
    if failed_files:
        logger.warning("Failed files:")
        for file in failed_files:
            logger.warning(f"  - {file}")
    
    logger.info("=" * 60)


def main():
    parser = argparse.ArgumentParser(description="Convert LeRobot dataset FPS while maintaining data synchronization")
    parser.add_argument("--input_dataset_path", type=str, required=True,
                       help="Input dataset path")
    parser.add_argument("--output_dataset_path", type=str, required=True,
                       help="Output dataset path")
    parser.add_argument("--source_fps", type=int, default=30,
                       help="Source FPS")
    parser.add_argument("--target_fps", type=int, default=15,
                       help="Target FPS")
    parser.add_argument("--max_workers", type=int, default=4,
                       help="Maximum number of parallel workers")
    
    args = parser.parse_args()
    
    input_dir = Path(args.input_dataset_path)
    output_dir = Path(args.output_dataset_path)
    
    if not input_dir.exists():
        raise FileNotFoundError(f"Input dataset does not exist: {input_dir}")
    
    # Check if FFmpeg is installed
    try:
        subprocess.run(['ffmpeg', '-version'], capture_output=True, check=True)
    except (subprocess.CalledProcessError, FileNotFoundError):
        logger.error("FFmpeg is not installed, please install it first: sudo apt install ffmpeg")
        return
    
    batch_convert_dataset(input_dir, output_dir, args.source_fps, args.target_fps, args.max_workers)


if __name__ == "__main__":
    main()
