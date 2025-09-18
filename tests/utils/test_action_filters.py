#!/usr/bin/env python3
"""
Test script to demonstrate action filtering functionality for SO100Follower robot.
This script shows how different filter types can reduce jitter in robot actions.
"""

import numpy as np
import matplotlib.pyplot as plt
from pingti.utils.action_filters import create_action_filter


def generate_noisy_actions(num_steps=100, noise_level=0.1):
    """Generate synthetic noisy actions for testing."""
    # Generate smooth trajectory
    t = np.linspace(0, 4*np.pi, num_steps)
    base_trajectory = np.sin(t) * 0.5 + 0.5  # Range [0, 1]
    
    # Add noise
    noise = np.random.normal(0, noise_level, num_steps)
    noisy_trajectory = base_trajectory + noise
    
    # Generate actions for all motors
    actions = []
    for i in range(num_steps):
        action = {
            "shoulder_pan.pos": noisy_trajectory[i],
            "shoulder_lift.pos": noisy_trajectory[i] * 0.8,
            "elbow_flex.pos": noisy_trajectory[i] * 0.6,
            "wrist_flex.pos": noisy_trajectory[i] * 0.4,
            "wrist_roll.pos": noisy_trajectory[i] * 0.2,
            "gripper.pos": 50.0,  # Fixed gripper position
        }
        actions.append(action)
    
    return actions, base_trajectory


def test_filters():
    """Test different action filters and compare their performance."""
    print("Testing Action Filters for SO100Follower Robot")
    print("=" * 50)
    
    # Generate test data
    actions, true_trajectory = generate_noisy_actions(num_steps=100, noise_level=0.15)
    
    # Test different filter types
    filter_configs = [
        ("none", "No Filter", {}),
        ("lowpass", "Low-Pass Filter (α=0.3)", {"alpha": 0.3}),
        ("lowpass", "Low-Pass Filter (α=0.1)", {"alpha": 0.1}),
        ("moving_average", "Moving Average (w=3)", {"window_size": 3}),
        ("moving_average", "Moving Average (w=5)", {"window_size": 5}),
        ("adaptive", "Adaptive Filter", {"base_alpha": 0.3, "adaptation_threshold": 0.1}),
    ]
    
    results = {}
    
    for filter_type, name, config in filter_configs:
        print(f"\nTesting {name}...")
        
        # Create filter
        action_filter = create_action_filter(filter_type, **config)
        
        # Apply filter to actions
        filtered_actions = []
        for action in actions:
            filtered_action = action_filter.filter(action)
            filtered_actions.append(filtered_action)
        
        # Extract shoulder_pan position for analysis
        filtered_trajectory = [action["shoulder_pan.pos"] for action in filtered_actions]
        
        # Calculate metrics
        mse = np.mean((np.array(filtered_trajectory) - true_trajectory) ** 2)
        noise_reduction = np.std(actions[0]["shoulder_pan.pos"] - true_trajectory[0]) / np.std(np.array(filtered_trajectory) - true_trajectory)
        
        results[name] = {
            "trajectory": filtered_trajectory,
            "mse": mse,
            "noise_reduction": noise_reduction
        }
        
        print(f"  MSE: {mse:.4f}")
        print(f"  Noise Reduction: {noise_reduction:.2f}x")
    
    # Plot results
    plt.figure(figsize=(15, 10))
    
    # Plot 1: All trajectories
    plt.subplot(2, 2, 1)
    plt.plot(true_trajectory, 'k-', linewidth=2, label='True Trajectory')
    plt.plot([action["shoulder_pan.pos"] for action in actions], 'r--', alpha=0.7, label='Noisy Input')
    
    colors = ['blue', 'green', 'orange', 'purple', 'brown']
    for i, (name, data) in enumerate(results.items()):
        if name != "No Filter":
            plt.plot(data["trajectory"], color=colors[i % len(colors)], alpha=0.8, label=name)
    
    plt.title('Action Filtering Comparison')
    plt.xlabel('Time Step')
    plt.ylabel('Position')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Plot 2: MSE comparison
    plt.subplot(2, 2, 2)
    names = list(results.keys())
    mses = [results[name]["mse"] for name in names]
    bars = plt.bar(names, mses, color=['red', 'blue', 'green', 'orange', 'purple', 'brown'])
    plt.title('Mean Squared Error Comparison')
    plt.ylabel('MSE')
    plt.xticks(rotation=45)
    plt.grid(True, alpha=0.3)
    
    # Add value labels on bars
    for bar, mse in zip(bars, mses):
        plt.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.001, 
                f'{mse:.3f}', ha='center', va='bottom')
    
    # Plot 3: Noise reduction comparison
    plt.subplot(2, 2, 3)
    noise_reductions = [results[name]["noise_reduction"] for name in names]
    bars = plt.bar(names, noise_reductions, color=['red', 'blue', 'green', 'orange', 'purple', 'brown'])
    plt.title('Noise Reduction Factor')
    plt.ylabel('Noise Reduction (x)')
    plt.xticks(rotation=45)
    plt.grid(True, alpha=0.3)
    
    # Add value labels on bars
    for bar, reduction in zip(bars, noise_reductions):
        plt.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01, 
                f'{reduction:.1f}x', ha='center', va='bottom')
    
    # Plot 4: Filter response to step input
    plt.subplot(2, 2, 4)
    step_actions = [{"shoulder_pan.pos": 1.0 if i < 50 else 0.0} for i in range(100)]
    
    for filter_type, name, config in filter_configs[:4]:  # Show first 4 filters
        action_filter = create_action_filter(filter_type, **config)
        step_response = []
        for action in step_actions:
            filtered = action_filter.filter(action)
            step_response.append(filtered["shoulder_pan.pos"])
        
        plt.plot(step_response, label=name, linewidth=2)
    
    plt.title('Step Response Comparison')
    plt.xlabel('Time Step')
    plt.ylabel('Position')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('./tests/utils/action_filter_comparison.png', dpi=150, bbox_inches='tight')
    plt.show()
    
    # Print summary
    print("\n" + "=" * 50)
    print("FILTER PERFORMANCE SUMMARY")
    print("=" * 50)
    print(f"{'Filter Name':<25} {'MSE':<10} {'Noise Reduction':<15}")
    print("-" * 50)
    for name, data in results.items():
        print(f"{name:<25} {data['mse']:<10.4f} {data['noise_reduction']:<15.2f}x")
    
    print("\nRecommendations:")
    print("- For maximum smoothing: Use Low-Pass Filter with α=0.1")
    print("- For balanced performance: Use Adaptive Filter")
    print("- For simple smoothing: Use Moving Average with window=3")
    print("- For real-time applications: Use Low-Pass Filter with α=0.3")


def test_robot_integration():
    """Test how filters would work with actual robot configuration."""
    print("\n" + "=" * 50)
    print("ROBOT INTEGRATION TEST")
    print("=" * 50)
    
    # Simulate robot configuration
    from pingti.robots.pingti_follower.config_pingti_follower import PingtiFollowerConfig
    
    configs = [
        PingtiFollowerConfig(
            port="/dev/ttyUSB0",
            action_filter_type="none"
        ),
        PingtiFollowerConfig(
            port="/dev/ttyUSB0",
            action_filter_type="lowpass",
            action_filter_alpha=0.3
        ),
        PingtiFollowerConfig(
            port="/dev/ttyUSB0",
            action_filter_type="adaptive",
            action_filter_alpha=0.3,
            action_filter_adaptation_threshold=0.1
        )
    ]
    
    for i, config in enumerate(configs):
        print(f"\nConfiguration {i+1}:")
        print(f"  Filter Type: {config.action_filter_type}")
        if config.action_filter_type != "none":
            print(f"  Alpha: {config.action_filter_alpha}")
            if config.action_filter_type == "adaptive":
                print(f"  Adaptation Threshold: {config.action_filter_adaptation_threshold}")
    
    print("\nTo use action filtering in your robot:")
    print("1. Set action_filter_type in your robot configuration")
    print("2. Adjust filter parameters as needed")
    print("3. Call robot.reset_action_filter() when starting new tasks")


if __name__ == "__main__":
    test_filters()
    test_robot_integration()
