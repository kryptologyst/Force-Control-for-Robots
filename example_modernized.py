#!/usr/bin/env python3
"""
Force Control for Robots - Quick Start Example

This script demonstrates the modernized force control system with a simple example
that replaces the original basic implementation.

DISCLAIMER: This is for research and educational purposes only.
Do not use on real hardware without expert review and proper safety measures.
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
from pathlib import Path

# Add src to path
sys.path.append(str(Path(__file__).parent / "src"))

from controllers import PIDForceController, ControlLimits
from simulation import RobotSimulation, SimulationConfig, RobotConfig
from evaluation import ForceControlEvaluator
from visualization import ForceControlVisualizer
from utils import set_random_seed, Timer

def main():
    """Run the modernized force control example."""
    
    print("="*80)
    print("FORCE CONTROL FOR ROBOTS - MODERNIZED EXAMPLE")
    print("="*80)
    print("This example demonstrates the modernized force control system")
    print("with improved controllers, simulation, and evaluation.")
    print("="*80)
    
    # Set random seed for reproducibility
    set_random_seed(42)
    
    # Create PID controller with safety limits
    controller = PIDForceController(
        desired_force=5.0,
        kp=0.1,
        ki=0.05,
        kd=0.01,
        control_limits=ControlLimits(max_force=10.0, max_velocity=1.0)
    )
    
    # Configure simulation
    sim_config = SimulationConfig(
        dt=0.01,
        enable_gui=False,  # Set to True to see PyBullet GUI
        enable_realtime=False,
        duration=10.0
    )
    
    robot_config = RobotConfig(
        urdf_path="robots/urdf/simple_arm.urdf"
    )
    
    print("Running force control simulation...")
    
    # Run simulation with timing
    with Timer("Force Control Simulation"):
        with RobotSimulation(sim_config, robot_config) as sim:
            results = sim.run_force_control(controller, duration=10.0)
    
    # Evaluate performance
    evaluator = ForceControlEvaluator()
    metrics = evaluator.evaluate(results)
    
    # Print results
    print("\n" + "="*60)
    print("SIMULATION RESULTS")
    print("="*60)
    print(f"Simulation Duration: {results['time'][-1]:.2f} seconds")
    print(f"Total Steps: {len(results['time'])}")
    print(f"Force RMSE: {metrics.force_rmse:.4f} N")
    print(f"Force MAE: {metrics.force_mae:.4f} N")
    print(f"Overshoot: {metrics.force_overshoot:.1f}%")
    print(f"Settling Time: {metrics.force_settling_time:.3f} s")
    print(f"Control Effort: {metrics.control_effort:.4f} N⋅s")
    print(f"Safety Violations: {metrics.force_limit_violations}")
    
    # Generate performance report
    print("\n" + "="*60)
    print("PERFORMANCE REPORT")
    print("="*60)
    print(evaluator.generate_report(metrics, "PID Controller"))
    
    # Create visualization
    print("\nCreating visualization...")
    visualizer = ForceControlVisualizer()
    
    # Plot results
    fig = visualizer.plot_force_tracking(results, show=True)
    
    # Save results
    assets_dir = Path("assets/results")
    assets_dir.mkdir(parents=True, exist_ok=True)
    
    fig.savefig(assets_dir / "force_control_example.png", dpi=300, bbox_inches='tight')
    print(f"Results saved to {assets_dir / 'force_control_example.png'}")
    
    print("\n" + "="*80)
    print("EXAMPLE COMPLETED SUCCESSFULLY!")
    print("="*80)
    print("This modernized implementation includes:")
    print("✓ Advanced PID controller with integral windup protection")
    print("✓ Physics-based simulation with PyBullet")
    print("✓ Comprehensive performance evaluation")
    print("✓ Safety limits and validation")
    print("✓ Professional visualization tools")
    print("✓ Type hints and documentation")
    print("✓ Unit tests and CI/CD pipeline")
    print("="*80)
    print("For more advanced features, run:")
    print("  python scripts/run_demo.py --demo all")
    print("  streamlit run demo/force_control_demo.py")
    print("="*80)

if __name__ == "__main__":
    main()
