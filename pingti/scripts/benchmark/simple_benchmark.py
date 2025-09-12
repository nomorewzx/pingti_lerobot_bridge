#!/usr/bin/env python

"""
Simple benchmark script to compare inference speed between ACT and SmolVLA policies.

This script creates synthetic data that matches the expected input format for both policies
and measures their inference times.
"""

import time
import warnings
from typing import Dict, Any, Optional

import torch
from torch import Tensor

# Suppress warnings for cleaner output
warnings.filterwarnings("ignore")


def create_synthetic_batch(device: str, batch_size: int = 1) -> Dict[str, Tensor]:
    """Create synthetic batch data for testing."""
    batch = {
        "observation.state": torch.randn(batch_size, 14, device=device),  # Robot state
        "observation.environment_state": torch.randn(batch_size, 4, device=device),  # Env state
        "observation.image": torch.randn(batch_size, 3, 224, 224, device=device),  # Image
        "task": ["manipulation task"] * batch_size,  # Task description for SmolVLA
    }
    return batch


def create_act_config():
    """Create ACT config with proper input features."""
    from lerobot.configs.types import FeatureType, PolicyFeature
    from lerobot.policies.act.configuration_act import ACTConfig
    
    config = ACTConfig()
    
    # Add required input features
    config.input_features = {
        "observation.state": PolicyFeature(
            shape=(14,),
            type=FeatureType.STATE,
        ),
        "observation.environment_state": PolicyFeature(
            shape=(4,),
            type=FeatureType.ENV,
        ),
        "observation.image": PolicyFeature(
            shape=(3, 224, 224),
            type=FeatureType.VISUAL,
        ),
    }
    
    # Add required output features
    config.output_features = {
        "action": PolicyFeature(
            shape=(14,),
            type=FeatureType.ACTION,
        ),
    }
    
    return config


def create_dummy_dataset_stats(device: str):
    """Create dummy dataset stats for normalization.
    Includes both mean/std and min/max where appropriate to satisfy different normalization modes.
    """
    return {
        "observation.state": {
            "mean": torch.zeros(14, device=device),
            "std": torch.ones(14, device=device),
            "min": -torch.ones(14, device=device),
            "max": torch.ones(14, device=device),
        },
        "observation.environment_state": {
            "mean": torch.zeros(4, device=device),
            "std": torch.ones(4, device=device),
            "min": -torch.ones(4, device=device),
            "max": torch.ones(4, device=device),
        },
        "observation.image": {
            "mean": torch.zeros(3, 1, 1, device=device),
            "std": torch.ones(3, 1, 1, device=device),
        },
        "action": {
            "mean": torch.zeros(14, device=device),
            "std": torch.ones(14, device=device),
            "min": -torch.ones(14, device=device),
            "max": torch.ones(14, device=device),
        },
    }


def create_smolvla_config():
    """Create SmolVLA config with proper input features."""
    from lerobot.configs.types import FeatureType, PolicyFeature
    from lerobot.policies.smolvla.configuration_smolvla import SmolVLAConfig
    
    config = SmolVLAConfig()
    
    config.input_features = {
        "observation.state": PolicyFeature(
            shape=(14,),
            type=FeatureType.STATE,
        ),
        "observation.environment_state": PolicyFeature(
            shape=(4,),
            type=FeatureType.ENV,
        ),
        "observation.image": PolicyFeature(
            shape=(3, 224, 224),
            type=FeatureType.VISUAL,
        ),
    }
    
    config.output_features = {
        "action": PolicyFeature(
            shape=(14,),
            type=FeatureType.ACTION,
        ),
    }
    
    return config


def create_diffusion_config():
    """Create Diffusion policy config with proper input/output features.
    Uses the same synthetic feature spec as ACT/SmolVLA for fair comparison.
    """
    from lerobot.configs.types import FeatureType, PolicyFeature
    try:
        from lerobot.policies.diffusion.configuration_diffusion import DiffusionConfig
    except Exception as e:
        raise ImportError(f"DiffusionConfig not available: {e}")

    config = DiffusionConfig()

    config.input_features = {
        "observation.state": PolicyFeature(
            shape=(14,),
            type=FeatureType.STATE,
        ),
        "observation.environment_state": PolicyFeature(
            shape=(4,),
            type=FeatureType.ENV,
        ),
        "observation.image": PolicyFeature(
            shape=(3, 224, 224),
            type=FeatureType.VISUAL,
        ),
    }

    config.output_features = {
        "action": PolicyFeature(
            shape=(14,),
            type=FeatureType.ACTION,
        ),
    }

    return config


def benchmark_policy(policy, policy_name: str, device: str, num_runs: int = 100, warmup_runs: int = 10):
    """Benchmark a single policy."""
    print(f"Benchmarking {policy_name}...")
    
    # Warmup
    print(f"  Warming up with {warmup_runs} runs...")
    with torch.no_grad():
        for i in range(warmup_runs):
            batch = create_synthetic_batch(device)
            try:
                policy.reset()
                _ = policy.select_action(batch)
            except Exception as e:
                print(f"  Warning: Warmup error {i}: {e}")
    
    # Actual benchmarking
    print(f"  Running {num_runs} inference tests...")
    times = []
    successful_runs = 0
    
    with torch.no_grad():
        for i in range(num_runs):
            batch = create_synthetic_batch(device)
            try:
                policy.reset()
                start_time = time.perf_counter()
                _ = policy.select_action(batch)
                end_time = time.perf_counter()
                
                inference_time = end_time - start_time
                times.append(inference_time)
                successful_runs += 1
                
                if i % 20 == 0:
                    print(f"    Run {i+1}/{num_runs}: {inference_time*1000:.2f}ms")
                    
            except Exception as e:
                print(f"    Warning: Run {i} error: {e}")
                continue
    
    if not times:
        raise RuntimeError(f"No successful runs for {policy_name}")
    
    times_tensor = torch.tensor(times)
    stats = {
        "mean_time": times_tensor.mean().item(),
        "std_time": times_tensor.std().item(),
        "min_time": times_tensor.min().item(),
        "max_time": times_tensor.max().item(),
        "median_time": times_tensor.median().item(),
        "successful_runs": successful_runs,
        "total_runs": num_runs,
        "mode": "cold_start_each_step",
    }
    
    return stats


def benchmark_policy_steady_state(policy, policy_name: str, device: str, num_runs: int = 100, warmup_runs: int = 10):
    """Benchmark in steady-state: reset once; time sequential select_action calls.
    Also separates regular steps vs. refills (when the action queue is empty before the call).
    """
    print(f"Benchmarking {policy_name} (steady-state)...")

    times_all = []
    times_step = []
    times_refill = []
    successful_runs = 0

    policy.reset()

    with torch.no_grad():
        if warmup_runs > 0:
            print(f"  Warming up with {warmup_runs} sequential runs...")
            for i in range(warmup_runs):
                batch = create_synthetic_batch(device)
                try:
                    _ = policy.select_action(batch)
                except Exception as e:
                    print(f"  Warning: Warmup error {i}: {e}")

        print(f"  Running {num_runs} steady-state inference tests...")
        for i in range(num_runs):
            batch = create_synthetic_batch(device)
            try:
                # Detect refill opportunity if internal action queue exists and is empty
                before_len = None
                q = getattr(policy, "_queues", None)
                if isinstance(q, dict) and "action" in q:
                    try:
                        before_len = len(q["action"])  # deque
                    except Exception:
                        before_len = None

                start_time = time.perf_counter()
                _ = policy.select_action(batch)
                end_time = time.perf_counter()

                dt = end_time - start_time
                times_all.append(dt)
                successful_runs += 1

                if before_len == 0:
                    times_refill.append(dt)
                else:
                    times_step.append(dt)

                if i % 20 == 0:
                    print(f"    Run {i+1}/{num_runs}: {dt*1000:.2f}ms")
            except Exception as e:
                print(f"    Warning: Run {i} error: {e}")
                continue

    if not times_all:
        raise RuntimeError(f"No successful runs for {policy_name} (steady-state)")

    t_all = torch.tensor(times_all)
    stats: Dict[str, Any] = {
        "mean_time": t_all.mean().item(),
        "std_time": t_all.std().item(),
        "min_time": t_all.min().item(),
        "max_time": t_all.max().item(),
        "median_time": t_all.median().item(),
        "successful_runs": successful_runs,
        "total_runs": num_runs,
        "mode": "steady_state",
    }

    if times_step:
        t_step = torch.tensor(times_step)
        stats.update({
            "step_mean_time": t_step.mean().item(),
            "step_median_time": t_step.median().item(),
            "step_count": len(times_step),
        })
    if times_refill:
        t_refill = torch.tensor(times_refill)
        stats.update({
            "refill_mean_time": t_refill.mean().item(),
            "refill_median_time": t_refill.median().item(),
            "refill_count": len(times_refill),
        })

    return stats


def print_results(act_stats: Dict[str, Any], smolvla_stats: Dict[str, Any], diffusion_stats: Optional[Dict[str, Any]] = None):
    """Print benchmark results. If diffusion_stats is provided, include it."""
    print("\n" + "="*70)
    print("INFERENCE SPEED BENCHMARK RESULTS")
    print("="*70)
    
    if diffusion_stats is None:
        print(f"{'Metric':<20} {'ACT (ms)':<12} {'SmolVLA (ms)':<12} {'Speedup':<12}")
    else:
        print(f"{'Metric':<20} {'ACT (ms)':<12} {'SmolVLA (ms)':<12} {'Diffusion (ms)':<16} {'ACT/SmolVLA':<12} {'Diff/SmolVLA':<12}")
    print("-"*70)
    
    act_mean_ms = act_stats["mean_time"] * 1000
    smolvla_mean_ms = smolvla_stats["mean_time"] * 1000
    if diffusion_stats is not None:
        diffusion_mean_ms = diffusion_stats["mean_time"] * 1000
    speedup = act_mean_ms / smolvla_mean_ms if smolvla_mean_ms > 0 else float('inf')
    
    if diffusion_stats is None:
        print(f"{'Mean Time':<20} {act_mean_ms:<12.2f} {smolvla_mean_ms:<12.2f} {speedup:<12.2f}x")
    else:
        diff_speedup = diffusion_mean_ms / smolvla_mean_ms if smolvla_mean_ms > 0 else float('inf')
        print(f"{'Mean Time':<20} {act_mean_ms:<12.2f} {smolvla_mean_ms:<12.2f} {diffusion_mean_ms:<16.2f} {speedup:<12.2f}x {diff_speedup:<12.2f}x")
    
    act_median_ms = act_stats["median_time"] * 1000
    smolvla_median_ms = smolvla_stats["median_time"] * 1000
    if diffusion_stats is not None:
        diffusion_median_ms = diffusion_stats["median_time"] * 1000
    median_speedup = act_median_ms / smolvla_median_ms if smolvla_median_ms > 0 else float('inf')
    
    if diffusion_stats is None:
        print(f"{'Median Time':<20} {act_median_ms:<12.2f} {smolvla_median_ms:<12.2f} {median_speedup:<12.2f}x")
    else:
        diff_median_speedup = diffusion_median_ms / smolvla_median_ms if smolvla_median_ms > 0 else float('inf')
        print(f"{'Median Time':<20} {act_median_ms:<12.2f} {smolvla_median_ms:<12.2f} {diffusion_median_ms:<16.2f} {median_speedup:<12.2f}x {diff_median_speedup:<12.2f}x")
    
    act_min_ms = act_stats["min_time"] * 1000
    smolvla_min_ms = smolvla_stats["min_time"] * 1000
    if diffusion_stats is not None:
        diffusion_min_ms = diffusion_stats["min_time"] * 1000
    min_speedup = act_min_ms / smolvla_min_ms if smolvla_min_ms > 0 else float('inf')
    
    if diffusion_stats is None:
        print(f"{'Min Time':<20} {act_min_ms:<12.2f} {smolvla_min_ms:<12.2f} {min_speedup:<12.2f}x")
    else:
        diff_min_speedup = diffusion_min_ms / smolvla_min_ms if smolvla_min_ms > 0 else float('inf')
        print(f"{'Min Time':<20} {act_min_ms:<12.2f} {smolvla_min_ms:<12.2f} {diffusion_min_ms:<16.2f} {min_speedup:<12.2f}x {diff_min_speedup:<12.2f}x")
    
    act_max_ms = act_stats["max_time"] * 1000
    smolvla_max_ms = smolvla_stats["max_time"] * 1000
    if diffusion_stats is not None:
        diffusion_max_ms = diffusion_stats["max_time"] * 1000
    max_speedup = act_max_ms / smolvla_max_ms if smolvla_max_ms > 0 else float('inf')
    
    if diffusion_stats is None:
        print(f"{'Max Time':<20} {act_max_ms:<12.2f} {smolvla_max_ms:<12.2f} {max_speedup:<12.2f}x")
    else:
        diff_max_speedup = diffusion_max_ms / smolvla_max_ms if smolvla_max_ms > 0 else float('inf')
        print(f"{'Max Time':<20} {act_max_ms:<12.2f} {smolvla_max_ms:<12.2f} {diffusion_max_ms:<16.2f} {max_speedup:<12.2f}x {diff_max_speedup:<12.2f}x")
    
    # Show step vs refill if detected
    def maybe_line(name: str, key: str, stats: Dict[str, Any]) -> str:
        return f"{stats[key]*1000:.2f}ms" if key in stats else "-"

    print("-"*70)
    if diffusion_stats is None:
        print(f"{'Successful Runs':<20} {act_stats['successful_runs']:<12} {smolvla_stats['successful_runs']:<12}")
        print(f"{'Step Mean (ms)':<20} {maybe_line('act','step_mean_time', act_stats):<12} {maybe_line('smol','step_mean_time', smolvla_stats):<12}")
        print(f"{'Refill Mean (ms)':<20} {maybe_line('act','refill_mean_time', act_stats):<12} {maybe_line('smol','refill_mean_time', smolvla_stats):<12}")
        print(f"{'Step/Refill Count':<20} {act_stats.get('step_count','-')}/{act_stats.get('refill_count','-'):<12} {smolvla_stats.get('step_count','-')}/{smolvla_stats.get('refill_count','-'):<12}")
    else:
        print(f"{'Successful Runs':<20} {act_stats['successful_runs']:<12} {smolvla_stats['successful_runs']:<12} {diffusion_stats['successful_runs']:<16}")
        print(f"{'Step Mean (ms)':<20} {maybe_line('act','step_mean_time', act_stats):<12} {maybe_line('smol','step_mean_time', smolvla_stats):<12} {maybe_line('diff','step_mean_time', diffusion_stats):<16}")
        print(f"{'Refill Mean (ms)':<20} {maybe_line('act','refill_mean_time', act_stats):<12} {maybe_line('smol','refill_mean_time', smolvla_stats):<12} {maybe_line('diff','refill_mean_time', diffusion_stats):<16}")
        print(f"{'Step/Refill Count':<20} {act_stats.get('step_count','-')}/{act_stats.get('refill_count','-'):<12} {smolvla_stats.get('step_count','-')}/{smolvla_stats.get('refill_count','-'):<12} {diffusion_stats.get('step_count','-')}/{diffusion_stats.get('refill_count','-'):<16}")
    
    print("\n" + "="*70)
    print("SUMMARY")
    print("="*70)
    
    if diffusion_stats is None:
        if speedup > 1:
            print(f"ACT is {speedup:.2f}x SLOWER than SmolVLA on average")
        elif speedup < 1:
            print(f"ACT is {1/speedup:.2f}x FASTER than SmolVLA on average")
        else:
            print("ACT and SmolVLA have similar inference speeds")
        print(f"ACT mean inference time: {act_mean_ms:.2f}ms")
        print(f"SmolVLA mean inference time: {smolvla_mean_ms:.2f}ms")
    else:
        diff_speedup = diffusion_mean_ms / smolvla_mean_ms if smolvla_mean_ms > 0 else float('inf')
        def summarize(name: str, ms: float):
            print(f"{name} mean inference time: {ms:.2f}ms")
        print(f"ACT vs SmolVLA: {'slower' if speedup>1 else 'faster' if speedup<1 else 'similar'} ({(speedup if speedup>1 else 1/speedup if speedup>0 else float('inf')):.2f}x)")
        print(f"Diffusion vs SmolVLA: {'slower' if diff_speedup>1 else 'faster' if diff_speedup<1 else 'similar'} ({(diff_speedup if diff_speedup>1 else 1/diff_speedup if diff_speedup>0 else float('inf')):.2f}x)")
        summarize("ACT", act_mean_ms)
        summarize("SmolVLA", smolvla_mean_ms)
        summarize("Diffusion", diffusion_mean_ms)



def main():
    device = "cuda" if torch.cuda.is_available() else "mps"
    print(f"Using device: {device}")
    print()
    
    try:
        from lerobot.policies.act.modeling_act import ACTPolicy
        from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
    except ImportError as e:
        print(f"Failed to import ACT/SmolVLA policies: {e}")
        return

    diffusion_available = True
    try:
        from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy
    except Exception as e:
        diffusion_available = False
        print(f"Warning: Diffusion policy not available ({e}). Skipping Diffusion benchmark.")
    
    print("Creating policies...")
    dummy_stats = create_dummy_dataset_stats(device)
    
    try:
        act_config = create_act_config()
        act_policy = ACTPolicy(act_config, dataset_stats=dummy_stats).to(device)
        act_policy.eval()
        print("✓ ACT policy created")
    except Exception as e:
        print(f"✗ Failed to create ACT policy: {e}")
        return
    
    try:
        smolvla_config = create_smolvla_config()
        smolvla_policy = SmolVLAPolicy(smolvla_config, dataset_stats=dummy_stats).to(device)
        smolvla_policy.eval()
        print("✓ SmolVLA policy created")
    except Exception as e:
        print(f"✗ Failed to create SmolVLA policy: {e}")
        return

    diffusion_policy = None
    if diffusion_available:
        try:
            diffusion_config = create_diffusion_config()
            diffusion_policy = DiffusionPolicy(diffusion_config, dataset_stats=dummy_stats).to(device)
            diffusion_policy.eval()
            print("✓ Diffusion policy created")
        except Exception as e:
            print(f"✗ Failed to create Diffusion policy: {e}")
            diffusion_policy = None
    
    print()
    
    num_runs = 500
    warmup_runs = 5
    
    try:
        act_stats = benchmark_policy_steady_state(act_policy, "ACT", device, num_runs, warmup_runs)
    except Exception as e:
        print(f"✗ ACT benchmarking failed: {e}")
        return
    
    print()
    
    try:
        smolvla_stats = benchmark_policy_steady_state(smolvla_policy, "SmolVLA", device, num_runs, warmup_runs)
    except Exception as e:
        print(f"✗ SmolVLA benchmarking failed: {e}")
        return

    diffusion_stats = None
    if diffusion_policy is not None:
        print()
        try:
            diffusion_stats = benchmark_policy_steady_state(diffusion_policy, "Diffusion", device, num_runs, warmup_runs)
        except Exception as e:
            print(f"✗ Diffusion benchmarking failed: {e}")
            diffusion_stats = None
    
    print_results(act_stats, smolvla_stats, diffusion_stats)


if __name__ == "__main__":
    main()
