#!/usr/bin/env python3
"""
Force Control for Robots - Main Example Script

This script demonstrates the force control system with different controllers
and provides comprehensive evaluation and visualization.

DISCLAIMER: This is for research and educational purposes only.
Do not use on real hardware without expert review and proper safety measures.
"""

import numpy as np
import matplotlib.pyplot as plt
import logging
from pathlib import Path
import argparse
import sys

# Add src to path
sys.path.append(str(Path(__file__).parent / "src"))

from controllers import (
    PIDForceController, ImpedanceController, HybridPositionForceController,
    ForceControllerFactory, ControlLimits
)
from simulation import RobotSimulation, SimulationConfig, RobotConfig
from evaluation import ForceControlEvaluator, PerformanceLeaderboard
from visualization import ForceControlVisualizer
from utils import set_random_seed, load_config, Timer

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def run_single_controller_demo():
    """Run demonstration with a single PID controller."""
    logger.info("Running single controller demonstration...")
    
    # Set random seed for reproducibility
    set_random_seed(42)
    
    # Create PID controller
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
        enable_gui=False,  # Disable GUI for automated runs
        enable_realtime=False,
        duration=10.0
    )
    
    robot_config = RobotConfig(
        urdf_path="robots/urdf/simple_arm.urdf"
    )
    
    # Run simulation
    with Timer("Single Controller Simulation"):
        with RobotSimulation(sim_config, robot_config) as sim:
            results = sim.run_force_control(controller, duration=10.0)
    
    # Evaluate results
    evaluator = ForceControlEvaluator()
    metrics = evaluator.evaluate(results)
    
    # Print results
    print("\n" + "="*60)
    print("SINGLE CONTROLLER DEMONSTRATION RESULTS")
    print("="*60)
    print(evaluator.generate_report(metrics, "PID Controller"))
    
    # Create visualizations
    visualizer = ForceControlVisualizer()
    
    # Save plots
    assets_dir = Path("assets/results")
    assets_dir.mkdir(parents=True, exist_ok=True)
    
    fig = visualizer.plot_force_tracking(
        results, 
        save_path=str(assets_dir / "single_controller_force_tracking.png"),
        show=False
    )
    
    # Create interactive plot
    interactive_fig = visualizer.create_interactive_plot(
        results, "PID Force Control Performance"
    )
    interactive_fig.write_html(str(assets_dir / "single_controller_interactive.html"))
    
    logger.info("Single controller demonstration completed!")
    return results, metrics


def run_controller_comparison():
    """Run comparison between different controller types."""
    logger.info("Running controller comparison...")
    
    # Set random seed for reproducibility
    set_random_seed(42)
    
    # Controller configurations
    controllers_config = {
        "PID": {
            'type': 'pid',
            'params': {'kp': 0.1, 'ki': 0.05, 'kd': 0.01}
        },
        "Impedance": {
            'type': 'impedance',
            'params': {'stiffness': 100.0, 'damping': 20.0, 'mass': 1.0}
        },
        "Hybrid": {
            'type': 'hybrid',
            'params': {
                'position_gains': 100.0,
                'force_gains': 0.1,
                'force_directions': np.array([False, True])
            }
        }
    }
    
    # Simulation configuration
    sim_config = SimulationConfig(
        dt=0.01,
        enable_gui=False,
        enable_realtime=False
    )
    
    robot_config = RobotConfig(
        urdf_path="robots/urdf/simple_arm.urdf"
    )
    
    # Run simulations
    results_dict = {}
    leaderboard = PerformanceLeaderboard()
    evaluator = ForceControlEvaluator()
    
    with Timer("Controller Comparison"):
        for controller_name, config in controllers_config.items():
            logger.info(f"Testing {controller_name} controller...")
            
            # Create controller
            controller = ForceControllerFactory.create_controller(
                config['type'],
                desired_force=5.0,
                control_limits=ControlLimits(max_force=10.0, max_velocity=1.0),
                **config['params']
            )
            
            # Run simulation
            with RobotSimulation(sim_config, robot_config) as sim:
                results = sim.run_force_control(controller, duration=10.0)
            
            # Evaluate and store results
            metrics = evaluator.evaluate(results)
            results_dict[controller_name] = results
            leaderboard.add_result(controller_name, metrics, config['params'])
    
    # Print comparison results
    print("\n" + "="*60)
    print("CONTROLLER COMPARISON RESULTS")
    print("="*60)
    
    for controller_name, results in results_dict.items():
        metrics = evaluator.evaluate(results)
        print(f"\n{controller_name} Controller:")
        print(f"  Force RMSE: {metrics.force_rmse:.4f} N")
        print(f"  Settling Time: {metrics.force_settling_time:.3f} s")
        print(f"  Overshoot: {metrics.force_overshoot:.1f}%")
        print(f"  Control Effort: {metrics.control_effort:.4f} N⋅s")
    
    # Print leaderboard
    print("\n" + "="*40)
    print("PERFORMANCE LEADERBOARD")
    print("="*40)
    leaderboard.print_leaderboard("force_rmse")
    
    # Create comparison visualization
    visualizer = ForceControlVisualizer()
    assets_dir = Path("assets/results")
    
    fig = visualizer.plot_comparison(
        results_dict,
        save_path=str(assets_dir / "controller_comparison.png"),
        show=False
    )
    
    # Export leaderboard
    leaderboard.export_results(str(assets_dir / "comparison_leaderboard.json"))
    
    logger.info("Controller comparison completed!")
    return results_dict, leaderboard


def run_parameter_sweep():
    """Run parameter sweep to find optimal controller gains."""
    logger.info("Running parameter sweep...")
    
    # Set random seed for reproducibility
    set_random_seed(42)
    
    # Parameter ranges for PID controller
    kp_values = [0.05, 0.1, 0.2, 0.5]
    ki_values = [0.01, 0.05, 0.1, 0.2]
    kd_values = [0.005, 0.01, 0.02, 0.05]
    
    # Simulation configuration
    sim_config = SimulationConfig(
        dt=0.01,
        enable_gui=False,
        enable_realtime=False
    )
    
    robot_config = RobotConfig(
        urdf_path="robots/urdf/simple_arm.urdf"
    )
    
    # Run parameter sweep
    best_metrics = None
    best_params = None
    best_rmse = float('inf')
    
    evaluator = ForceControlEvaluator()
    
    with Timer("Parameter Sweep"):
        for kp in kp_values:
            for ki in ki_values:
                for kd in kd_values:
                    logger.info(f"Testing Kp={kp}, Ki={ki}, Kd={kd}")
                    
                    # Create controller
                    controller = PIDForceController(
                        desired_force=5.0,
                        kp=kp,
                        ki=ki,
                        kd=kd,
                        control_limits=ControlLimits(max_force=10.0, max_velocity=1.0)
                    )
                    
                    # Run simulation
                    with RobotSimulation(sim_config, robot_config) as sim:
                        results = sim.run_force_control(controller, duration=5.0)
                    
                    # Evaluate
                    metrics = evaluator.evaluate(results)
                    
                    # Check if this is the best so far
                    if metrics.force_rmse < best_rmse:
                        best_rmse = metrics.force_rmse
                        best_metrics = metrics
                        best_params = {'kp': kp, 'ki': ki, 'kd': kd}
    
    # Print results
    print("\n" + "="*60)
    print("PARAMETER SWEEP RESULTS")
    print("="*60)
    print(f"Best Parameters: Kp={best_params['kp']}, Ki={best_params['ki']}, Kd={best_params['kd']}")
    print(f"Best Force RMSE: {best_rmse:.4f} N")
    print(f"Best Settling Time: {best_metrics.force_settling_time:.3f} s")
    print(f"Best Overshoot: {best_metrics.force_overshoot:.1f}%")
    
    logger.info("Parameter sweep completed!")
    return best_params, best_metrics


def main():
    """Main function."""
    parser = argparse.ArgumentParser(
        description="Force Control for Robots - Demonstration Script"
    )
    parser.add_argument(
        "--demo",
        choices=["single", "comparison", "sweep", "all"],
        default="all",
        help="Type of demonstration to run"
    )
    parser.add_argument(
        "--gui",
        action="store_true",
        help="Enable PyBullet GUI (slower)"
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="Simulation duration in seconds"
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Random seed for reproducibility"
    )
    
    args = parser.parse_args()
    
    # Set random seed
    set_random_seed(args.seed)
    
    # Print header
    print("="*80)
    print("FORCE CONTROL FOR ROBOTS - DEMONSTRATION SCRIPT")
    print("="*80)
    print("DISCLAIMER: This is for research and educational purposes only.")
    print("Do not use on real hardware without expert review and safety measures.")
    print("="*80)
    
    try:
        if args.demo in ["single", "all"]:
            run_single_controller_demo()
        
        if args.demo in ["comparison", "all"]:
            run_controller_comparison()
        
        if args.demo in ["sweep", "all"]:
            run_parameter_sweep()
        
        print("\n" + "="*80)
        print("ALL DEMONSTRATIONS COMPLETED SUCCESSFULLY!")
        print("="*80)
        print("Results saved to assets/results/")
        print("Run 'streamlit run demo/force_control_demo.py' for interactive demo")
        print("="*80)
        
    except KeyboardInterrupt:
        logger.info("Demonstration interrupted by user")
    except Exception as e:
        logger.error(f"Demonstration failed: {e}")
        raise


if __name__ == "__main__":
    main()
