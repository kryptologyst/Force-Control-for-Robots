# Force Control for Robots

**DISCLAIMER: This project is for research and educational purposes only. Do not use on real hardware without expert review and proper safety measures.**

## Overview

This project implements modern force control systems for robot manipulation tasks, including PID control, impedance control, and hybrid position-force control. The system is designed for research and educational purposes with comprehensive evaluation metrics and interactive demonstrations.

## Features

- **Multiple Control Methods**: PID, Impedance Control, Hybrid Position-Force Control
- **Physics Simulation**: PyBullet integration with URDF robot models
- **Comprehensive Evaluation**: Tracking metrics, stability analysis, and performance benchmarks
- **Interactive Demos**: Streamlit-based visualization and control interface
- **Safety Features**: Force limits, emergency stops, and simulation-first approach
- **Modern Architecture**: Type hints, comprehensive testing, and CI/CD pipeline

## Quick Start

### Installation

```bash
# Clone the repository
git clone https://github.com/kryptologyst/Force-Control-for-Robots.git
cd Force-Control-for-Robots

# Install in development mode
pip install -e ".[dev]"

# Optional: Install ROS 2 dependencies
pip install -e ".[ros2]"
```

### Basic Usage

```python
from src.controllers import PIDForceController, ImpedanceController
from src.simulation import RobotSimulation
from src.evaluation import ForceControlEvaluator

# Create a force controller
controller = PIDForceController(
    desired_force=5.0,
    kp=0.1, ki=0.05, kd=0.01,
    max_force=10.0
)

# Run simulation
sim = RobotSimulation()
evaluator = ForceControlEvaluator()

# Execute control loop
results = sim.run_force_control(controller, duration=10.0)
metrics = evaluator.evaluate(results)

print(f"Force tracking RMSE: {metrics['force_rmse']:.3f} N")
```

### Interactive Demo

```bash
# Launch Streamlit demo
streamlit run demo/force_control_demo.py
```

## Project Structure

```
src/
├── controllers/          # Force control implementations
├── simulation/           # Physics simulation and robot models
├── evaluation/           # Metrics and performance analysis
├── utils/               # Common utilities and helpers
└── visualization/       # Plotting and visualization tools

robots/
├── urdf/               # Robot description files
├── meshes/             # 3D robot meshes
└── config/             # Robot-specific configurations

config/
├── controllers.yaml    # Controller parameters
├── simulation.yaml     # Simulation settings
└── evaluation.yaml     # Evaluation metrics

tests/
├── unit/               # Unit tests
├── integration/        # Integration tests
└── fixtures/           # Test data and fixtures

demo/
├── force_control_demo.py    # Interactive Streamlit demo
├── ros2_launch/             # ROS 2 launch files
└── notebooks/               # Jupyter notebooks

assets/
├── results/            # Evaluation results and plots
├── videos/             # Demo videos
└── configs/            # Saved configurations
```

## Control Methods

### 1. PID Force Control
Classical proportional-integral-derivative control for force tracking.

### 2. Impedance Control
Implements impedance control with configurable stiffness and damping.

### 3. Hybrid Position-Force Control
Combines position and force control in different task space directions.

## Evaluation Metrics

- **Force Tracking**: RMSE, overshoot, settling time
- **Stability**: Phase margin, gain margin, stability margins
- **Performance**: Control effort, jerk, smoothness
- **Safety**: Force limit violations, emergency stops

## Safety Features

- Force and velocity limits
- Emergency stop mechanisms
- Simulation-first approach
- Comprehensive error handling
- Safety disclaimers and warnings

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests for new functionality
5. Run the test suite: `pytest`
6. Submit a pull request

## License

MIT License - see LICENSE file for details.

## Citation

If you use this project in your research, please cite:

```bibtex
@software{force_control_robots,
  title={Force Control for Robots: A Modern Implementation},
  author={Kryptologyst},
  year={2026},
  url={https://github.com/kryptologyst/Force-Control-for-Robots}
}
```
# Force-Control-for-Robots
