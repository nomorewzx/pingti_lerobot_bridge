"""
Action filtering utilities for smoothing robot actions to reduce jitter and improve control stability.
"""

import logging
from abc import ABC, abstractmethod
from collections import deque
from typing import Any, Dict

import numpy as np

logger = logging.getLogger(__name__)


class ActionFilter(ABC):
    """Abstract base class for action filters."""
    
    @abstractmethod
    def filter(self, action: Dict[str, Any]) -> Dict[str, Any]:
        """
        Apply filtering to the input action.
        
        Args:
            action: Dictionary containing action values for each motor
            
        Returns:
            Filtered action dictionary
        """
        pass
    
    def reset(self) -> None:
        """Reset the filter state."""
        pass


class LowPassFilter(ActionFilter):
    """
    Low-pass filter for smoothing actions.
    
    This filter applies exponential smoothing to reduce high-frequency noise
    while preserving the overall action trend.
    """
    
    def __init__(self, alpha: float = 0.3, initial_values: Dict[str, float] | None = None):
        """
        Initialize the low-pass filter.
        
        Args:
            alpha: Smoothing factor (0 < alpha <= 1). Lower values = more smoothing
            initial_values: Initial values for each motor (optional)
        """
        if not 0 < alpha <= 1:
            raise ValueError("Alpha must be between 0 and 1")
        
        self.alpha = alpha
        self.previous_values = initial_values or {}
        
    def filter(self, action: Dict[str, Any]) -> Dict[str, Any]:
        """Apply low-pass filtering to the action."""
        filtered_action = {}
        
        for key, value in action.items():
            if key.endswith('.pos'):  # Only filter position values
                motor_name = key.removesuffix('.pos')
                
                if motor_name in self.previous_values:
                    # Apply exponential smoothing
                    filtered_value = self.alpha * value + (1 - self.alpha) * self.previous_values[motor_name]
                else:
                    # First value, no filtering
                    filtered_value = value
                
                self.previous_values[motor_name] = filtered_value
                filtered_action[key] = filtered_value
            else:
                # Non-position values pass through unchanged
                filtered_action[key] = value
                
        return filtered_action
    
    def reset(self) -> None:
        """Reset the filter state."""
        self.previous_values = {}


class MovingAverageFilter(ActionFilter):
    """
    Moving average filter for smoothing actions.
    
    This filter maintains a sliding window of recent actions and returns
    the average of the window for each motor.
    """
    
    def __init__(self, window_size: int = 3, initial_values: Dict[str, float] | None = None):
        """
        Initialize the moving average filter.
        
        Args:
            window_size: Size of the sliding window
            initial_values: Initial values for each motor (optional)
        """
        if window_size < 1:
            raise ValueError("Window size must be at least 1")
        
        self.window_size = window_size
        self.action_history = {key: deque(maxlen=window_size) for key in (initial_values or {})}
        
    def filter(self, action: Dict[str, Any]) -> Dict[str, Any]:
        """Apply moving average filtering to the action."""
        filtered_action = {}
        
        for key, value in action.items():
            if key.endswith('.pos'):  # Only filter position values
                motor_name = key.removesuffix('.pos')
                
                # Initialize history for new motors
                if motor_name not in self.action_history:
                    self.action_history[motor_name] = deque(maxlen=self.window_size)
                
                # Add current value to history
                self.action_history[motor_name].append(value)
                
                # Calculate average
                if len(self.action_history[motor_name]) > 0:
                    filtered_value = np.mean(self.action_history[motor_name])
                else:
                    filtered_value = value
                
                filtered_action[key] = filtered_value
            else:
                # Non-position values pass through unchanged
                filtered_action[key] = value
                
        return filtered_action
    
    def reset(self) -> None:
        """Reset the filter state."""
        for history in self.action_history.values():
            history.clear()


class AdaptiveFilter(ActionFilter):
    """
    Adaptive filter that combines low-pass and moving average filtering.
    
    This filter automatically adjusts its smoothing based on the action magnitude
    to provide more smoothing for small changes and less smoothing for large changes.
    """
    
    def __init__(self, 
                 base_alpha: float = 0.3,
                 window_size: int = 3,
                 adaptation_threshold: float = 0.1,
                 initial_values: Dict[str, float] | None = None):
        """
        Initialize the adaptive filter.
        
        Args:
            base_alpha: Base smoothing factor for low-pass component
            window_size: Window size for moving average component
            adaptation_threshold: Threshold for action magnitude adaptation
            initial_values: Initial values for each motor (optional)
        """
        self.base_alpha = base_alpha
        self.window_size = window_size
        self.adaptation_threshold = adaptation_threshold
        self.previous_values = initial_values or {}
        self.action_history = {key: deque(maxlen=window_size) for key in (initial_values or {})}
        
    def filter(self, action: Dict[str, Any]) -> Dict[str, Any]:
        """Apply adaptive filtering to the action."""
        filtered_action = {}
        
        for key, value in action.items():
            if key.endswith('.pos'):  # Only filter position values
                motor_name = key.removesuffix('.pos')
                
                # Initialize history for new motors
                if motor_name not in self.action_history:
                    self.action_history[motor_name] = deque(maxlen=self.window_size)
                
                # Add current value to history
                self.action_history[motor_name].append(value)
                
                if motor_name in self.previous_values:
                    # Calculate action magnitude
                    action_magnitude = abs(value - self.previous_values[motor_name])
                    
                    # Adapt alpha based on action magnitude
                    if action_magnitude > self.adaptation_threshold:
                        # Large action, use less smoothing
                        alpha = min(0.8, self.base_alpha * 2)
                    else:
                        # Small action, use more smoothing
                        alpha = self.base_alpha
                    
                    # Apply exponential smoothing
                    filtered_value = alpha * value + (1 - alpha) * self.previous_values[motor_name]
                else:
                    # First value, no filtering
                    filtered_value = value
                
                self.previous_values[motor_name] = filtered_value
                filtered_action[key] = filtered_value
            else:
                # Non-position values pass through unchanged
                filtered_action[key] = value
                
        return filtered_action
    
    def reset(self) -> None:
        """Reset the filter state."""
        self.previous_values = {}
        for history in self.action_history.values():
            history.clear()


def create_action_filter(filter_type: str, **kwargs) -> ActionFilter:
    """
    Factory function to create action filters.
    
    Args:
        filter_type: Type of filter ('lowpass', 'moving_average', 'adaptive', or 'none')
        **kwargs: Additional arguments for the specific filter type
        
    Returns:
        ActionFilter instance
    """
    if filter_type == 'none' or filter_type is None:
        return NoOpFilter()
    elif filter_type == 'lowpass':
        return LowPassFilter(**kwargs)
    elif filter_type == 'moving_average':
        return MovingAverageFilter(**kwargs)
    elif filter_type == 'adaptive':
        return AdaptiveFilter(**kwargs)
    else:
        raise ValueError(f"Unknown filter type: {filter_type}")


class NoOpFilter(ActionFilter):
    """No-operation filter that passes actions through unchanged."""
    
    def filter(self, action: Dict[str, Any]) -> Dict[str, Any]:
        """Return action unchanged."""
        return action
