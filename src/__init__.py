"""Force Control for Robots - A modern implementation for research and education.

This package provides comprehensive force control implementations for robot manipulation
tasks, including PID control, impedance control, and hybrid position-force control.

DISCLAIMER: This is for research and educational purposes only.
Do not use on real hardware without expert review and proper safety measures.
"""

__version__ = "1.0.0"
__author__ = "AI Assistant"
__email__ = "ai@example.com"

# Import main classes for easy access
from .controllers import (
    ForceController,
    PIDForceController,
    ImpedanceController,
    HybridPositionForceController,
    ForceControllerFactory,
    ControlState,
    ControlLimits
)

from .simulation import (
    RobotSimulation,
    SimulationConfig,
    RobotConfig
)

from .evaluation import (
    ForceControlEvaluator,
    ControlMetrics,
    PerformanceLeaderboard
)

from .visualization import (
    ForceControlVisualizer,
    SimulationVisualizer
)

from .utils import (
    set_random_seed,
    load_config,
    save_config,
    validate_force_limits,
    validate_velocity_limits,
    Timer,
    DataLogger
)

__all__ = [
    # Controllers
    "ForceController",
    "PIDForceController", 
    "ImpedanceController",
    "HybridPositionForceController",
    "ForceControllerFactory",
    "ControlState",
    "ControlLimits",
    
    # Simulation
    "RobotSimulation",
    "SimulationConfig", 
    "RobotConfig",
    
    # Evaluation
    "ForceControlEvaluator",
    "ControlMetrics",
    "PerformanceLeaderboard",
    
    # Visualization
    "ForceControlVisualizer",
    "SimulationVisualizer",
    
    # Utils
    "set_random_seed",
    "load_config",
    "save_config", 
    "validate_force_limits",
    "validate_velocity_limits",
    "Timer",
    "DataLogger",
]
