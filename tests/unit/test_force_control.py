"""Unit tests for force control system."""

import pytest
import numpy as np
from unittest.mock import Mock, patch
import sys
from pathlib import Path

# Add src to path for imports
sys.path.append(str(Path(__file__).parent.parent / "src"))

from controllers import (
    PIDForceController, ImpedanceController, HybridPositionForceController,
    ForceControllerFactory, ControlState, ControlLimits
)
from simulation import RobotSimulation, SimulationConfig, RobotConfig
from evaluation import ForceControlEvaluator, ControlMetrics, PerformanceLeaderboard
from utils import (
    set_random_seed, load_config, save_config, validate_force_limits,
    validate_velocity_limits, normalize_angle, Timer
)


class TestPIDForceController:
    """Test PID force controller."""
    
    def test_init(self):
        """Test PID controller initialization."""
        controller = PIDForceController(
            desired_force=5.0,
            kp=0.1,
            ki=0.05,
            kd=0.01
        )
        
        assert controller.desired_force == 5.0
        assert controller.kp == 0.1
        assert controller.ki == 0.05
        assert controller.kd == 0.01
        assert controller.is_enabled
    
    def test_compute_control(self):
        """Test PID control computation."""
        controller = PIDForceController(
            desired_force=5.0,
            kp=0.1,
            ki=0.05,
            kd=0.01,
            dt=0.01
        )
        
        # Create test state
        state = ControlState(
            position=np.array([0.0]),
            velocity=np.array([0.0]),
            force=np.array([3.0]),  # 2N error
            torque=np.array([0.0]),
            timestamp=0.0
        )
        
        # Compute control
        control = controller.compute_control(state)
        
        # Check that control is computed
        assert isinstance(control, np.ndarray)
        assert len(control) == 1
        
        # Check proportional term (should be positive for positive error)
        assert control[0] > 0
    
    def test_integral_windup_protection(self):
        """Test integral windup protection."""
        controller = PIDForceController(
            desired_force=5.0,
            kp=0.1,
            ki=1.0,  # High integral gain
            kd=0.01,
            integral_limit=1.0,
            dt=0.01
        )
        
        # Simulate large error for multiple steps
        for _ in range(100):
            state = ControlState(
                position=np.array([0.0]),
                velocity=np.array([0.0]),
                force=np.array([0.0]),  # Large error
                torque=np.array([0.0]),
                timestamp=0.0
            )
            controller.compute_control(state)
        
        # Check that integral term is limited
        assert abs(controller.integral[0]) <= controller.integral_limit
    
    def test_force_limits(self):
        """Test force limit application."""
        controller = PIDForceController(
            desired_force=5.0,
            kp=10.0,  # High gain to exceed limits
            ki=0.0,
            kd=0.0,
            control_limits=ControlLimits(max_force=1.0)
        )
        
        state = ControlState(
            position=np.array([0.0]),
            velocity=np.array([0.0]),
            force=np.array([0.0]),  # Large error
            torque=np.array([0.0]),
            timestamp=0.0
        )
        
        control = controller.compute_control(state)
        
        # Check that control is limited
        assert abs(control[0]) <= 1.0
    
    def test_reset(self):
        """Test controller reset."""
        controller = PIDForceController(desired_force=5.0)
        
        # Run controller to accumulate state
        state = ControlState(
            position=np.array([0.0]),
            velocity=np.array([0.0]),
            force=np.array([3.0]),
            torque=np.array([0.0]),
            timestamp=0.0
        )
        controller.compute_control(state)
        
        # Reset and check state is cleared
        controller.reset()
        assert np.all(controller.prev_error == 0)
        assert np.all(controller.integral == 0)


class TestImpedanceController:
    """Test impedance controller."""
    
    def test_init(self):
        """Test impedance controller initialization."""
        controller = ImpedanceController(
            desired_force=5.0,
            stiffness=100.0,
            damping=20.0,
            mass=1.0
        )
        
        assert controller.desired_force == 5.0
        assert controller.stiffness == 100.0
        assert controller.damping == 20.0
        assert controller.mass == 1.0
    
    def test_compute_control(self):
        """Test impedance control computation."""
        controller = ImpedanceController(
            desired_force=5.0,
            stiffness=100.0,
            damping=20.0,
            mass=1.0
        )
        
        current_state = ControlState(
            position=np.array([0.0]),
            velocity=np.array([0.0]),
            force=np.array([0.0]),
            torque=np.array([0.0]),
            timestamp=0.0
        )
        
        desired_state = ControlState(
            position=np.array([0.1]),  # Position error
            velocity=np.array([0.0]),
            force=np.array([0.0]),
            torque=np.array([0.0]),
            timestamp=0.0
        )
        
        control = controller.compute_control(current_state, desired_state)
        
        assert isinstance(control, np.ndarray)
        assert len(control) == 1
        
        # Should have stiffness term (positive for positive position error)
        assert control[0] > 0


class TestHybridPositionForceController:
    """Test hybrid position-force controller."""
    
    def test_init(self):
        """Test hybrid controller initialization."""
        controller = HybridPositionForceController(
            desired_force=5.0,
            desired_position=np.array([0.0, 0.0]),
            force_directions=np.array([False, True])
        )
        
        assert controller.desired_force == 5.0
        assert np.array_equal(controller.desired_position, np.array([0.0, 0.0]))
        assert np.array_equal(controller.force_directions, np.array([False, True]))
    
    def test_compute_control(self):
        """Test hybrid control computation."""
        controller = HybridPositionForceController(
            desired_force=5.0,
            desired_position=np.array([0.0, 0.0]),
            force_directions=np.array([False, True]),
            position_gains=100.0,
            force_gains=0.1
        )
        
        state = ControlState(
            position=np.array([0.1, 0.0]),  # Position error in x
            velocity=np.array([0.0, 0.0]),
            force=np.array([0.0, 3.0]),  # Force error in y
            torque=np.array([0.0, 0.0]),
            timestamp=0.0
        )
        
        control = controller.compute_control(state)
        
        assert isinstance(control, np.ndarray)
        assert len(control) == 2
        
        # X direction should have position control (positive for positive error)
        assert control[0] > 0
        # Y direction should have force control (positive for positive error)
        assert control[1] > 0


class TestForceControllerFactory:
    """Test force controller factory."""
    
    def test_create_pid_controller(self):
        """Test PID controller creation."""
        controller = ForceControllerFactory.create_controller(
            'pid',
            desired_force=5.0,
            kp=0.1,
            ki=0.05,
            kd=0.01
        )
        
        assert isinstance(controller, PIDForceController)
        assert controller.desired_force == 5.0
    
    def test_create_impedance_controller(self):
        """Test impedance controller creation."""
        controller = ForceControllerFactory.create_controller(
            'impedance',
            desired_force=5.0,
            stiffness=100.0,
            damping=20.0
        )
        
        assert isinstance(controller, ImpedanceController)
        assert controller.desired_force == 5.0
    
    def test_create_hybrid_controller(self):
        """Test hybrid controller creation."""
        controller = ForceControllerFactory.create_controller(
            'hybrid',
            desired_force=5.0,
            desired_position=np.array([0.0, 0.0]),
            force_directions=np.array([False, True])
        )
        
        assert isinstance(controller, HybridPositionForceController)
        assert controller.desired_force == 5.0
    
    def test_invalid_controller_type(self):
        """Test invalid controller type raises error."""
        with pytest.raises(ValueError):
            ForceControllerFactory.create_controller('invalid_type')
    
    def test_get_available_controllers(self):
        """Test getting available controller types."""
        controllers = ForceControllerFactory.get_available_controllers()
        assert 'pid' in controllers
        assert 'impedance' in controllers
        assert 'hybrid' in controllers


class TestForceControlEvaluator:
    """Test force control evaluator."""
    
    def test_init(self):
        """Test evaluator initialization."""
        evaluator = ForceControlEvaluator(
            settling_threshold=0.05,
            overshoot_threshold=0.1
        )
        
        assert evaluator.settling_threshold == 0.05
        assert evaluator.overshoot_threshold == 0.1
    
    def test_evaluate_perfect_tracking(self):
        """Test evaluation with perfect force tracking."""
        evaluator = ForceControlEvaluator()
        
        # Create perfect tracking results
        time = np.linspace(0, 10, 100)
        desired_force = np.ones(100) * 5.0
        force = np.ones(100) * 5.0  # Perfect tracking
        control_force = np.zeros(100)
        velocity = np.zeros(100)
        
        results = {
            'time': time,
            'force': force,
            'desired_force': desired_force,
            'control_force': control_force,
            'velocity': velocity
        }
        
        metrics = evaluator.evaluate(results)
        
        assert metrics.force_rmse == 0.0
        assert metrics.force_mae == 0.0
        assert metrics.force_overshoot == 0.0
        assert metrics.force_settling_time == 0.0
    
    def test_evaluate_with_error(self):
        """Test evaluation with force tracking error."""
        evaluator = ForceControlEvaluator()
        
        # Create results with constant error
        time = np.linspace(0, 10, 100)
        desired_force = np.ones(100) * 5.0
        force = np.ones(100) * 4.0  # 1N error
        control_force = np.ones(100) * 1.0
        velocity = np.zeros(100)
        
        results = {
            'time': time,
            'force': force,
            'desired_force': desired_force,
            'control_force': control_force,
            'velocity': velocity
        }
        
        metrics = evaluator.evaluate(results)
        
        assert metrics.force_rmse == 1.0
        assert metrics.force_mae == 1.0
        assert metrics.force_overshoot == 0.0
    
    def test_overshoot_calculation(self):
        """Test overshoot calculation."""
        evaluator = ForceControlEvaluator()
        
        # Create results with overshoot
        time = np.linspace(0, 10, 100)
        desired_force = np.ones(100) * 5.0
        force = np.concatenate([
            np.linspace(0, 7, 50),  # Overshoot to 7N
            np.linspace(7, 5, 50)   # Settle to 5N
        ])
        control_force = np.zeros(100)
        velocity = np.zeros(100)
        
        results = {
            'time': time,
            'force': force,
            'desired_force': desired_force,
            'control_force': control_force,
            'velocity': velocity
        }
        
        metrics = evaluator.evaluate(results)
        
        # Should detect overshoot (7N vs 5N desired = 40% overshoot)
        assert metrics.force_overshoot > 0
    
    def test_generate_report(self):
        """Test report generation."""
        evaluator = ForceControlEvaluator()
        
        metrics = ControlMetrics(
            force_rmse=0.1,
            force_mae=0.08,
            force_overshoot=5.0,
            force_settling_time=1.5,
            control_effort=2.0,
            control_smoothness=0.5,
            energy_consumption=1.0,
            force_limit_violations=0,
            velocity_limit_violations=0,
            emergency_stops=0
        )
        
        report = evaluator.generate_report(metrics, "Test Controller")
        
        assert "Test Controller" in report
        assert "0.1000" in report  # RMSE
        assert "5.00%" in report   # Overshoot


class TestPerformanceLeaderboard:
    """Test performance leaderboard."""
    
    def test_init(self):
        """Test leaderboard initialization."""
        leaderboard = PerformanceLeaderboard()
        assert len(leaderboard.results) == 0
    
    def test_add_result(self):
        """Test adding result to leaderboard."""
        leaderboard = PerformanceLeaderboard()
        
        metrics = ControlMetrics(
            force_rmse=0.1,
            force_mae=0.08,
            force_overshoot=5.0,
            force_settling_time=1.5,
            control_effort=2.0,
            control_smoothness=0.5,
            energy_consumption=1.0,
            force_limit_violations=0,
            velocity_limit_violations=0,
            emergency_stops=0
        )
        
        leaderboard.add_result("Test Controller", metrics, {"kp": 0.1})
        
        assert "Test Controller" in leaderboard.results
        assert leaderboard.results["Test Controller"]["metrics"].force_rmse == 0.1
    
    def test_get_leaderboard(self):
        """Test getting sorted leaderboard."""
        leaderboard = PerformanceLeaderboard()
        
        # Add multiple results
        metrics1 = ControlMetrics(force_rmse=0.2, force_mae=0.0, force_overshoot=0.0, 
                               force_settling_time=0.0, control_effort=0.0, 
                               control_smoothness=0.0, energy_consumption=0.0,
                               force_limit_violations=0, velocity_limit_violations=0, 
                               emergency_stops=0)
        metrics2 = ControlMetrics(force_rmse=0.1, force_mae=0.0, force_overshoot=0.0, 
                               force_settling_time=0.0, control_effort=0.0, 
                               control_smoothness=0.0, energy_consumption=0.0,
                               force_limit_violations=0, velocity_limit_violations=0, 
                               emergency_stops=0)
        
        leaderboard.add_result("Controller 1", metrics1)
        leaderboard.add_result("Controller 2", metrics2)
        
        rankings = leaderboard.get_leaderboard("force_rmse", ascending=True)
        
        assert len(rankings) == 2
        assert rankings[0][0] == "Controller 2"  # Lower RMSE should be first
        assert rankings[1][0] == "Controller 1"


class TestUtils:
    """Test utility functions."""
    
    def test_set_random_seed(self):
        """Test random seed setting."""
        set_random_seed(42)
        
        # Generate some random numbers
        rand1 = np.random.random()
        rand2 = np.random.random()
        
        # Reset seed and generate again
        set_random_seed(42)
        rand3 = np.random.random()
        rand4 = np.random.random()
        
        # Should be the same
        assert rand1 == rand3
        assert rand2 == rand4
    
    def test_validate_force_limits(self):
        """Test force limit validation."""
        # Valid force
        force = np.array([5.0, 3.0, 2.0])
        assert validate_force_limits(force, 10.0) == True
        
        # Invalid force
        force = np.array([15.0, 3.0, 2.0])
        assert validate_force_limits(force, 10.0) == False
    
    def test_validate_velocity_limits(self):
        """Test velocity limit validation."""
        # Valid velocity
        velocity = np.array([0.5, 0.3, 0.2])
        assert validate_velocity_limits(velocity, 1.0) == True
        
        # Invalid velocity
        velocity = np.array([1.5, 0.3, 0.2])
        assert validate_velocity_limits(velocity, 1.0) == False
    
    def test_normalize_angle(self):
        """Test angle normalization."""
        # Test various angles
        assert normalize_angle(np.pi) == np.pi
        assert normalize_angle(3 * np.pi) == np.pi
        assert normalize_angle(-np.pi) == -np.pi
        assert normalize_angle(-3 * np.pi) == -np.pi
    
    def test_timer_context_manager(self):
        """Test timer context manager."""
        timer = Timer("Test Operation")
        
        with timer:
            import time
            time.sleep(0.01)  # Small delay
        
        assert timer.elapsed > 0


class TestSimulationConfig:
    """Test simulation configuration."""
    
    def test_default_config(self):
        """Test default simulation configuration."""
        config = SimulationConfig()
        
        assert config.dt == 0.01
        assert config.gravity == -9.81
        assert config.enable_gui == True
        assert config.enable_realtime == False
    
    def test_custom_config(self):
        """Test custom simulation configuration."""
        config = SimulationConfig(
            dt=0.005,
            gravity=-10.0,
            enable_gui=False,
            enable_realtime=True
        )
        
        assert config.dt == 0.005
        assert config.gravity == -10.0
        assert config.enable_gui == False
        assert config.enable_realtime == True


class TestRobotConfig:
    """Test robot configuration."""
    
    def test_default_config(self):
        """Test default robot configuration."""
        config = RobotConfig()
        
        assert config.base_position == (0, 0, 0)
        assert config.base_orientation == (0, 0, 0, 1)
        assert config.joint_damping == 0.1
        assert config.joint_friction == 0.1
    
    def test_custom_config(self):
        """Test custom robot configuration."""
        config = RobotConfig(
            urdf_path="custom_robot.urdf",
            base_position=(1, 2, 3),
            base_orientation=(0, 0, 0, 1),
            joint_damping=0.2,
            joint_friction=0.2
        )
        
        assert config.urdf_path == "custom_robot.urdf"
        assert config.base_position == (1, 2, 3)
        assert config.joint_damping == 0.2
        assert config.joint_friction == 0.2


if __name__ == "__main__":
    pytest.main([__file__])
