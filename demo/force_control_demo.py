"""Interactive Streamlit demo for force control system."""

import streamlit as st
import numpy as np
import matplotlib.pyplot as plt
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import pandas as pd
from pathlib import Path
import sys
import logging

# Add src to path for imports
sys.path.append(str(Path(__file__).parent.parent.parent / "src"))

from controllers import PIDForceController, ImpedanceController, HybridPositionForceController, ForceControllerFactory
from simulation import RobotSimulation, SimulationConfig, RobotConfig
from evaluation import ForceControlEvaluator, PerformanceLeaderboard
from visualization import ForceControlVisualizer
from utils import load_config, set_random_seed, Timer

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Page configuration
st.set_page_config(
    page_title="Force Control for Robots",
    page_icon="🤖",
    layout="wide",
    initial_sidebar_state="expanded"
)

# Custom CSS
st.markdown("""
<style>
    .main-header {
        font-size: 2.5rem;
        font-weight: bold;
        text-align: center;
        margin-bottom: 2rem;
        color: #1f77b4;
    }
    .warning-box {
        background-color: #fff3cd;
        border: 1px solid #ffeaa7;
        border-radius: 0.5rem;
        padding: 1rem;
        margin: 1rem 0;
    }
    .metric-card {
        background-color: #f8f9fa;
        border: 1px solid #dee2e6;
        border-radius: 0.5rem;
        padding: 1rem;
        margin: 0.5rem 0;
    }
</style>
""", unsafe_allow_html=True)

def main():
    """Main Streamlit application."""
    
    # Header
    st.markdown('<h1 class="main-header">🤖 Force Control for Robots</h1>', unsafe_allow_html=True)
    
    # Safety disclaimer
    st.markdown("""
    <div class="warning-box">
        <h4>⚠️ Safety Disclaimer</h4>
        <p><strong>This is a research and educational tool only.</strong> 
        Do not use on real hardware without expert review and proper safety measures.</p>
    </div>
    """, unsafe_allow_html=True)
    
    # Sidebar configuration
    st.sidebar.header("Configuration")
    
    # Controller selection
    controller_type = st.sidebar.selectbox(
        "Controller Type",
        ["PID", "Impedance", "Hybrid Position-Force"],
        help="Select the type of force controller to use"
    )
    
    # Controller parameters
    st.sidebar.subheader("Controller Parameters")
    
    if controller_type == "PID":
        kp = st.sidebar.slider("Proportional Gain (Kp)", 0.01, 1.0, 0.1, 0.01)
        ki = st.sidebar.slider("Integral Gain (Ki)", 0.0, 0.2, 0.05, 0.01)
        kd = st.sidebar.slider("Derivative Gain (Kd)", 0.0, 0.1, 0.01, 0.01)
        integral_limit = st.sidebar.slider("Integral Limit", 0.1, 5.0, 1.0, 0.1)
        
        controller_params = {
            'kp': kp,
            'ki': ki,
            'kd': kd,
            'integral_limit': integral_limit
        }
        
    elif controller_type == "Impedance":
        stiffness = st.sidebar.slider("Stiffness", 10.0, 1000.0, 100.0, 10.0)
        damping = st.sidebar.slider("Damping", 1.0, 100.0, 20.0, 1.0)
        mass = st.sidebar.slider("Mass", 0.1, 10.0, 1.0, 0.1)
        
        controller_params = {
            'stiffness': stiffness,
            'damping': damping,
            'mass': mass
        }
        
    else:  # Hybrid
        position_gains = st.sidebar.slider("Position Gains", 10.0, 1000.0, 100.0, 10.0)
        force_gains = st.sidebar.slider("Force Gains", 0.01, 1.0, 0.1, 0.01)
        force_directions = st.sidebar.multiselect(
            "Force Controlled Directions",
            ["X", "Y", "Z"],
            default=["Y"],
            help="Select which directions are force controlled"
        )
        
        # Convert to boolean array
        force_mask = [False, False, False]
        for direction in force_directions:
            if direction == "X":
                force_mask[0] = True
            elif direction == "Y":
                force_mask[1] = True
            elif direction == "Z":
                force_mask[2] = True
        
        controller_params = {
            'position_gains': position_gains,
            'force_gains': force_gains,
            'force_directions': np.array(force_mask)
        }
    
    # Simulation parameters
    st.sidebar.subheader("Simulation Parameters")
    
    desired_force = st.sidebar.slider("Desired Force (N)", -10.0, 10.0, 5.0, 0.1)
    simulation_duration = st.sidebar.slider("Duration (s)", 1.0, 30.0, 10.0, 0.5)
    enable_gui = st.sidebar.checkbox("Enable GUI", value=False, help="Show PyBullet GUI (slower)")
    enable_realtime = st.sidebar.checkbox("Real-time", value=False, help="Run in real-time")
    
    # Safety limits
    st.sidebar.subheader("Safety Limits")
    max_force = st.sidebar.slider("Max Force (N)", 1.0, 20.0, 10.0, 0.5)
    max_velocity = st.sidebar.slider("Max Velocity (m/s)", 0.1, 5.0, 1.0, 0.1)
    
    # Random seed
    random_seed = st.sidebar.number_input("Random Seed", value=42, min_value=0, max_value=10000)
    
    # Main content area
    col1, col2 = st.columns([2, 1])
    
    with col1:
        st.header("Force Control Simulation")
        
        # Run simulation button
        if st.button("🚀 Run Simulation", type="primary"):
            run_simulation(
                controller_type.lower(),
                controller_params,
                desired_force,
                simulation_duration,
                enable_gui,
                enable_realtime,
                max_force,
                max_velocity,
                random_seed
            )
    
    with col2:
        st.header("Quick Info")
        st.info(f"""
        **Controller:** {controller_type}
        
        **Target Force:** {desired_force} N
        
        **Duration:** {simulation_duration} s
        
        **Safety Limits:**
        - Max Force: {max_force} N
        - Max Velocity: {max_velocity} m/s
        """)
    
    # Results section
    if 'simulation_results' in st.session_state:
        display_results()
    
    # Leaderboard section
    if 'leaderboard' in st.session_state:
        display_leaderboard()


def run_simulation(
    controller_type: str,
    controller_params: dict,
    desired_force: float,
    duration: float,
    enable_gui: bool,
    enable_realtime: bool,
    max_force: float,
    max_velocity: float,
    random_seed: int
):
    """Run force control simulation."""
    
    # Set random seed
    set_random_seed(random_seed)
    
    # Create progress bar
    progress_bar = st.progress(0)
    status_text = st.empty()
    
    try:
        # Initialize simulation
        status_text.text("Initializing simulation...")
        progress_bar.progress(10)
        
        sim_config = SimulationConfig(
            dt=0.01,
            enable_gui=enable_gui,
            enable_realtime=enable_realtime
        )
        
        robot_config = RobotConfig(
            urdf_path="robots/urdf/simple_arm.urdf"
        )
        
        # Create controller
        status_text.text("Creating controller...")
        progress_bar.progress(20)
        
        controller_params['desired_force'] = desired_force
        controller_params['control_limits'] = None  # Will use default
        
        controller = ForceControllerFactory.create_controller(
            controller_type, **controller_params
        )
        
        # Run simulation
        status_text.text("Running simulation...")
        progress_bar.progress(30)
        
        with RobotSimulation(sim_config, robot_config) as sim:
            results = sim.run_force_control(controller, duration)
        
        progress_bar.progress(80)
        
        # Evaluate results
        status_text.text("Evaluating results...")
        evaluator = ForceControlEvaluator()
        metrics = evaluator.evaluate(results)
        
        progress_bar.progress(100)
        status_text.text("Simulation completed!")
        
        # Store results in session state
        st.session_state['simulation_results'] = {
            'results': results,
            'metrics': metrics,
            'controller_type': controller_type,
            'controller_params': controller_params,
            'config': {
                'desired_force': desired_force,
                'duration': duration,
                'max_force': max_force,
                'max_velocity': max_velocity
            }
        }
        
        # Add to leaderboard
        if 'leaderboard' not in st.session_state:
            st.session_state['leaderboard'] = PerformanceLeaderboard()
        
        controller_name = f"{controller_type}_{len(st.session_state['leaderboard'].results) + 1}"
        st.session_state['leaderboard'].add_result(
            controller_name, metrics, controller_params
        )
        
        st.success("Simulation completed successfully!")
        
    except Exception as e:
        st.error(f"Simulation failed: {str(e)}")
        logger.error(f"Simulation error: {e}")


def display_results():
    """Display simulation results."""
    
    results_data = st.session_state['simulation_results']
    results = results_data['results']
    metrics = results_data['metrics']
    controller_type = results_data['controller_type']
    
    st.header("📊 Simulation Results")
    
    # Metrics summary
    col1, col2, col3, col4 = st.columns(4)
    
    with col1:
        st.metric("Force RMSE", f"{metrics.force_rmse:.3f} N")
    with col2:
        st.metric("Force MAE", f"{metrics.force_mae:.3f} N")
    with col3:
        st.metric("Overshoot", f"{metrics.force_overshoot:.1f}%")
    with col4:
        st.metric("Settling Time", f"{metrics.force_settling_time:.2f} s")
    
    # Performance grade
    grade = "A" if metrics.force_rmse < 0.1 else "B" if metrics.force_rmse < 0.5 else "C" if metrics.force_rmse < 1.0 else "D"
    st.metric("Performance Grade", grade)
    
    # Plots
    st.subheader("📈 Performance Plots")
    
    # Create visualizer
    visualizer = ForceControlVisualizer()
    
    # Force tracking plot
    fig = visualizer.plot_force_tracking(results, show=False)
    st.pyplot(fig)
    
    # Interactive plot
    st.subheader("🔍 Interactive Analysis")
    
    interactive_fig = visualizer.create_interactive_plot(
        results, f"{controller_type.title()} Controller Performance"
    )
    st.plotly_chart(interactive_fig, use_container_width=True)
    
    # Detailed metrics
    st.subheader("📋 Detailed Metrics")
    
    metrics_data = {
        "Metric": [
            "Force RMSE", "Force MAE", "Force Overshoot", "Settling Time",
            "Control Effort", "Control Smoothness", "Energy Consumption",
            "Force Limit Violations", "Velocity Limit Violations", "Emergency Stops"
        ],
        "Value": [
            f"{metrics.force_rmse:.4f} N",
            f"{metrics.force_mae:.4f} N",
            f"{metrics.force_overshoot:.2f}%",
            f"{metrics.force_settling_time:.3f} s",
            f"{metrics.control_effort:.4f} N⋅s",
            f"{metrics.control_smoothness:.4f} N/s³",
            f"{metrics.energy_consumption:.4f} N²⋅s",
            f"{metrics.force_limit_violations}",
            f"{metrics.velocity_limit_violations}",
            f"{metrics.emergency_stops}"
        ]
    }
    
    df = pd.DataFrame(metrics_data)
    st.dataframe(df, use_container_width=True)
    
    # Performance report
    st.subheader("📄 Performance Report")
    
    evaluator = ForceControlEvaluator()
    report = evaluator.generate_report(metrics, controller_type.title())
    st.text(report)


def display_leaderboard():
    """Display performance leaderboard."""
    
    leaderboard = st.session_state['leaderboard']
    
    st.header("🏆 Performance Leaderboard")
    
    # Select metric for ranking
    metric_options = [
        "force_rmse", "force_mae", "force_overshoot", "force_settling_time",
        "control_effort", "control_smoothness", "energy_consumption"
    ]
    
    selected_metric = st.selectbox(
        "Rank by Metric",
        metric_options,
        index=0,
        help="Select metric to rank controllers by"
    )
    
    # Get leaderboard
    rankings = leaderboard.get_leaderboard(selected_metric, ascending=True)
    
    if rankings:
        # Create leaderboard dataframe
        leaderboard_data = {
            "Rank": range(1, len(rankings) + 1),
            "Controller": [name for name, _ in rankings],
            f"{selected_metric.replace('_', ' ').title()}": [f"{score:.4f}" for _, score in rankings]
        }
        
        df = pd.DataFrame(leaderboard_data)
        st.dataframe(df, use_container_width=True)
        
        # Export button
        if st.button("📥 Export Leaderboard"):
            leaderboard.export_results("assets/results/leaderboard.json")
            st.success("Leaderboard exported to assets/results/leaderboard.json")
    else:
        st.info("No results available yet. Run some simulations to populate the leaderboard!")


if __name__ == "__main__":
    main()
