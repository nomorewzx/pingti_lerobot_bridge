#!/usr/bin/env python

"""
Updated test script to verify that the ACT and SmolVLA policies can be created and run inference successfully.
"""

import torch
import warnings

# Suppress warnings for cleaner output
warnings.filterwarnings("ignore")

def test_act_creation():
    """Test ACT policy creation and inference."""
    print("Testing ACT policy creation...")
    
    try:
        from lerobot.policies.act.modeling_act import ACTPolicy
        from lerobot.policies.act.configuration_act import ACTConfig
        from lerobot.configs.types import FeatureType, PolicyFeature
        
        # Create config with proper features (use environment state to avoid image path)
        config = ACTConfig()
        config.input_features = {
            "observation.state": PolicyFeature(
                shape=(14,),
                type=FeatureType.STATE,
            ),
            "observation.environment_state": PolicyFeature(
                shape=(4,),
                type=FeatureType.ENV,
            ),
        }
        config.output_features = {
            "action": PolicyFeature(
                shape=(14,),
                type=FeatureType.ACTION,
            ),
        }
        
        # Create dummy dataset stats for normalization
        device = "cuda" if torch.cuda.is_available() else "mps"
        dummy_stats = {
            "observation.state": {
                "mean": torch.zeros(14, device=device),
                "std": torch.ones(14, device=device),
            },
            "observation.environment_state": {
                "mean": torch.zeros(4, device=device),
                "std": torch.ones(4, device=device),
            },
            "action": {
                "mean": torch.zeros(14, device=device),
                "std": torch.ones(14, device=device),
            },
        }
        
        # Create policy with stats
        policy = ACTPolicy(config, dataset_stats=dummy_stats).to(device)
        policy.eval()
        
        print("✓ ACT policy created successfully")
        
        # Test inference (no image path, provide env state)
        batch = {
            "observation.state": torch.randn(1, 14, device=device),
            "observation.environment_state": torch.randn(1, 4, device=device),
        }
        
        with torch.no_grad():
            action = policy.select_action(batch)
            print(f"✓ ACT inference successful, action shape: {action.shape}")
        
        return True
        
    except Exception as e:
        print(f"✗ ACT policy creation failed: {e}")
        return False


def test_smolvla_creation():
    """Test SmolVLA policy creation and inference."""
    print("Testing SmolVLA policy creation...")
    
    try:
        from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
        from lerobot.policies.smolvla.configuration_smolvla import SmolVLAConfig
        from lerobot.configs.types import FeatureType, PolicyFeature
        
        # Create config with proper features
        config = SmolVLAConfig()
        config.input_features = {
            "observation.state": PolicyFeature(
                shape=(14,),
                type=FeatureType.STATE,
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
        
        # Create dummy dataset stats for normalization
        device = "cuda" if torch.cuda.is_available() else "mps"
        dummy_stats = {
            "observation.state": {
                "mean": torch.zeros(14, device=device),
                "std": torch.ones(14, device=device),
            },
            "observation.image": {
                "mean": torch.zeros(3, device=device),
                "std": torch.ones(3, device=device),
            },
            "action": {
                "mean": torch.zeros(14, device=device),
                "std": torch.ones(14, device=device),
            },
        }
        
        # Create policy with stats
        policy = SmolVLAPolicy(config, dataset_stats=dummy_stats).to(device)
        policy.eval()
        
        print("✓ SmolVLA policy created successfully")
        
        # Test inference
        batch = {
            "observation.state": torch.randn(1, 14, device=device),
            "observation.image": torch.randn(1, 3, 224, 224, device=device),
            "task": "manipulation task",
        }
        
        with torch.no_grad():
            action = policy.select_action(batch)
            print(f"✓ SmolVLA inference successful, action shape: {action.shape}")
        
        return True
        
    except Exception as e:
        print(f"✗ SmolVLA policy creation failed: {e}")
        return False


def test_benchmark_scripts():
    """Test that benchmark scripts can be imported and run basic functions."""
    print("Testing benchmark script functions...")
    
    try:
        # Test simple_benchmark functions
        from simple_benchmark import create_act_config, create_smolvla_config, create_dummy_dataset_stats
        
        device = "cuda" if torch.cuda.is_available() else "mps"
        
        # Test config creation
        act_config = create_act_config()
        smolvla_config = create_smolvla_config()
        dummy_stats = create_dummy_dataset_stats(device)
        
        print("✓ Benchmark script functions imported successfully")
        print("✓ Config creation functions work")
        print("✓ Dataset stats creation works")
        
        return True
        
    except Exception as e:
        print(f"✗ Benchmark script test failed: {e}")
        return False


def main():
    print("Testing policy creation fixes (v2)...")
    print("=" * 50)
    
    device = "cuda" if torch.cuda.is_available() else "mps"
    print(f"Using device: {device}")
    print()
    
    # Test ACT
    act_success = test_act_creation()
    print()
    
    # Test SmolVLA
    smolvla_success = test_smolvla_creation()
    print()
    
    # Test benchmark scripts
    script_success = test_benchmark_scripts()
    print()
    
    # Summary
    print("=" * 50)
    print("SUMMARY")
    print("=" * 50)
    
    if act_success and smolvla_success and script_success:
        print("✓ All tests passed!")
        print("✓ ACT policy creation and inference successful")
        print("✓ SmolVLA policy creation and inference successful")
        print("✓ Benchmark scripts are ready to use")
        print()
        print("You can now run the benchmark scripts:")
        print("  python simple_benchmark.py")
        print("  python real_dataset_benchmark.py --num_samples 10")
    else:
        print("✗ Some tests failed.")
        if not act_success:
            print("  - ACT policy test failed")
        if not smolvla_success:
            print("  - SmolVLA policy test failed")
        if not script_success:
            print("  - Benchmark script test failed")


if __name__ == "__main__":
    main()
