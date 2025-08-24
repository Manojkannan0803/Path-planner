#!/usr/bin/env python3
"""
Motion Primitive Loader for CATALYST A* Plugin

This module loads and processes motion primitives from MATLAB .mat files
and provides vehicle dynamics calculations based on the ACADO-generated
primitives.
"""

import os
import math
import numpy as np
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass
import logging

try:
    from scipy.io import loadmat
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False
    logging.warning("SciPy not available - using mock motion primitive data")


@dataclass
class MotionPrimitive:
    """
    Motion primitive data structure.
    
    Based on ACADO-generated motion primitives from MATLAB system.
    """
    id: int
    theta_start: float  # degrees
    gamma_start: float  # degrees  
    theta_end: float    # degrees
    gamma_end: float    # degrees
    direction: int      # 1=forward, -1=reverse
    
    # Trajectory data
    x_trajectory: List[float]
    y_trajectory: List[float]
    theta_trajectory: List[float]
    gamma_trajectory: List[float]
    delta_trajectory: List[float]  # steering angle
    
    # Costs and metadata
    path_length: float
    execution_time: float
    curvature_cost: float
    
    def __post_init__(self):
        """Validate motion primitive data."""
        if not self.x_trajectory or len(self.x_trajectory) < 2:
            raise ValueError("Motion primitive must have at least 2 trajectory points")
        
        # Ensure all trajectory arrays have same length
        lengths = [
            len(self.x_trajectory),
            len(self.y_trajectory), 
            len(self.theta_trajectory),
            len(self.gamma_trajectory)
        ]
        if not all(l == lengths[0] for l in lengths):
            raise ValueError("All trajectory arrays must have same length")


@dataclass
class VehicleParameters:
    """Articulated vehicle parameters from MATLAB system."""
    
    # Vehicle dimensions (from MATLAB files)
    L_0f: float = 3.8      # Tractor wheelbase [m]
    L_0b: float = 0.3      # Distance of king-pin to tractor drive axle [m]
    L_1f: float = 8.475    # Trailer wheelbase [m]
    
    # Vehicle dimensions for collision detection
    oh_0f: float = 1.5     # Tractor front overhang [m]
    oh_0b: float = 0.94    # Tractor rear overhang [m]
    oh_1f: float = 9.475   # Trailer front overhang [m] (L_1f + 1)
    oh_1b: float = 5.0     # Trailer rear overhang [m]
    width: float = 2.5     # Vehicle width [m]
    
    # Motion parameters
    v0: float = 1.0        # Drive axle velocity [m/s]
    max_delta: float = math.radians(30)  # Max steering angle [rad]
    max_gamma: float = math.radians(60)  # Max articulation angle [rad]


class MotionPrimitiveLoader:
    """
    Loads and manages motion primitives for A* path planning.
    
    Converts MATLAB .mat files to Python motion primitive objects
    and provides vehicle dynamics calculations.
    """
    
    def __init__(self, primitives_dir: str = None):
        """
        Initialize motion primitive loader.
        
        Args:
            primitives_dir: Directory containing motion primitive .mat files
        """
        self.logger = logging.getLogger(__name__)
        self.primitives_dir = primitives_dir
        self.vehicle_params = VehicleParameters()
        
        # Motion primitive storage
        self.primitives: Dict[str, MotionPrimitive] = {}
        self.primitive_index: Dict[Tuple[int, int], List[MotionPrimitive]] = {}
        
        # Discretization (from MATLAB)
        self.theta_discrete = list(range(0, 360, 9))  # 0, 9, 18, ..., 351 degrees
        self.gamma_discrete = [-21, 0, 21]  # degrees
        self.direction_options = [1, -1]  # forward, reverse
        
        self.logger.info("Motion primitive loader initialized")
    
    def load_motion_primitives(self) -> bool:
        """
        Load motion primitives from MATLAB .mat files.
        
        Returns:
            Success status
        """
        if not SCIPY_AVAILABLE:
            self.logger.warning("SciPy not available - generating mock primitives")
            return self._generate_mock_primitives()
        
        if not self.primitives_dir or not os.path.exists(self.primitives_dir):
            self.logger.warning("Primitives directory not found - generating mock primitives")
            return self._generate_mock_primitives()
        
        success_count = 0
        total_files = 0
        
        # Load primitives from different theta directories
        for theta_start in [0, 180]:  # Based on MATLAB folder structure
            theta_dir = os.path.join(
                self.primitives_dir, 
                f"Motion_primitivescheck1_{theta_start}"
            )
            
            if not os.path.exists(theta_dir):
                continue
            
            # Load .mat files
            for filename in os.listdir(theta_dir):
                if filename.endswith('.mat'):
                    total_files += 1
                    filepath = os.path.join(theta_dir, filename)
                    
                    try:
                        primitive = self._load_mat_file(filepath, theta_start)
                        if primitive:
                            self._add_primitive(primitive)
                            success_count += 1
                    except Exception as e:
                        self.logger.warning(f"Failed to load {filename}: {str(e)}")
        
        self.logger.info(f"Loaded {success_count}/{total_files} motion primitives")
        
        if success_count == 0:
            self.logger.warning("No primitives loaded - generating mock primitives")
            return self._generate_mock_primitives()
        
        return success_count > 0
    
    def _load_mat_file(self, filepath: str, theta_start: int) -> Optional[MotionPrimitive]:
        """Load motion primitive from MATLAB .mat file."""
        try:
            # Extract theta_end from filename (e.g., "mp_180_1.mat" -> theta_end=1)
            filename = os.path.basename(filepath)
            parts = filename.replace('.mat', '').split('_')
            if len(parts) < 3:
                return None
            
            theta_end = int(parts[2])
            gamma_end = 0  # From MATLAB - all primitives end with gamma=0
            
            # Load MATLAB data
            mat_data = loadmat(filepath)
            
            # Extract trajectory data (structure may vary)
            # Look for common variable names in ACADO output
            traj_vars = ['states', 'x', 'DifferentialStates', 'output']
            trajectory_data = None
            
            for var_name in traj_vars:
                if var_name in mat_data:
                    trajectory_data = mat_data[var_name]
                    break
            
            if trajectory_data is None:
                self.logger.warning(f"No trajectory data found in {filepath}")
                return self._generate_mock_primitive(theta_start, theta_end, gamma_end)
            
            # Parse trajectory data (assuming ACADO format: [x1, y1, theta1, gamma, delta, omega])
            if trajectory_data.shape[1] >= 4:
                x_traj = trajectory_data[:, 0].tolist()
                y_traj = trajectory_data[:, 1].tolist()
                theta_traj = [math.degrees(t) for t in trajectory_data[:, 2]]
                gamma_traj = [math.degrees(g) for g in trajectory_data[:, 3]]
                delta_traj = [math.degrees(d) for d in trajectory_data[:, 4]] if trajectory_data.shape[1] > 4 else [0.0] * len(x_traj)
            else:
                return self._generate_mock_primitive(theta_start, theta_end, gamma_end)
            
            # Calculate path length
            path_length = self._calculate_path_length(x_traj, y_traj)
            
            primitive = MotionPrimitive(
                id=len(self.primitives),
                theta_start=theta_start,
                gamma_start=0,
                theta_end=theta_end,
                gamma_end=gamma_end,
                direction=1,  # Assume forward (can be enhanced)
                x_trajectory=x_traj,
                y_trajectory=y_traj,
                theta_trajectory=theta_traj,
                gamma_trajectory=gamma_traj,
                delta_trajectory=delta_traj,
                path_length=path_length,
                execution_time=len(x_traj) * 0.1,  # Assume 0.1s per step
                curvature_cost=self._calculate_curvature_cost(delta_traj)
            )
            
            return primitive
            
        except Exception as e:
            self.logger.error(f"Error loading {filepath}: {str(e)}")
            return None
    
    def _generate_mock_primitives(self) -> bool:
        """Generate mock motion primitives for testing."""
        self.logger.info("Generating mock motion primitives")
        
        primitive_id = 0
        
        # Generate primitives for key theta combinations
        theta_combinations = [
            (0, 0), (0, 45), (0, 90), (0, 180), (0, 270),
            (180, 0), (180, 45), (180, 90), (180, 270)
        ]
        
        for theta_start, theta_end in theta_combinations:
            for gamma_end in self.gamma_discrete:
                for direction in self.direction_options:
                    primitive = self._generate_mock_primitive(
                        theta_start, theta_end, gamma_end, direction, primitive_id
                    )
                    self._add_primitive(primitive)
                    primitive_id += 1
        
        self.logger.info(f"Generated {len(self.primitives)} mock motion primitives")
        return True
    
    def _generate_mock_primitive(self, theta_start: int, theta_end: int, 
                                gamma_end: int, direction: int = 1, 
                                primitive_id: int = None) -> MotionPrimitive:
        """Generate a mock motion primitive."""
        if primitive_id is None:
            primitive_id = len(self.primitives)
        
        # Generate smooth trajectory
        num_steps = 20
        
        # Calculate target position based on angles
        length = 5.0 * direction  # 5 meter primitive
        
        # Simple trajectory generation
        x_traj = []
        y_traj = []
        theta_traj = []
        gamma_traj = []
        delta_traj = []
        
        for i in range(num_steps + 1):
            t = i / num_steps
            
            # Smooth interpolation
            theta_current = theta_start + t * (theta_end - theta_start)
            gamma_current = t * gamma_end
            
            # Position based on integrated motion
            x = length * t * math.cos(math.radians(theta_current))
            y = length * t * math.sin(math.radians(theta_current))
            
            x_traj.append(x)
            y_traj.append(y)
            theta_traj.append(theta_current)
            gamma_traj.append(gamma_current)
            delta_traj.append(0.0)  # Simplified steering
        
        path_length = abs(length)
        
        return MotionPrimitive(
            id=primitive_id,
            theta_start=theta_start,
            gamma_start=0,
            theta_end=theta_end,
            gamma_end=gamma_end,
            direction=direction,
            x_trajectory=x_traj,
            y_trajectory=y_traj,
            theta_trajectory=theta_traj,
            gamma_trajectory=gamma_traj,
            delta_trajectory=delta_traj,
            path_length=path_length,
            execution_time=2.0,  # 2 seconds
            curvature_cost=abs(theta_end - theta_start) * 0.1
        )
    
    def _add_primitive(self, primitive: MotionPrimitive):
        """Add primitive to storage and index."""
        # Store by ID
        self.primitives[str(primitive.id)] = primitive
        
        # Index by start angles (discretized)
        theta_key = self._discretize_angle(primitive.theta_start)
        gamma_key = self._discretize_gamma(primitive.gamma_start)
        
        index_key = (theta_key, gamma_key)
        if index_key not in self.primitive_index:
            self.primitive_index[index_key] = []
        
        self.primitive_index[index_key].append(primitive)
    
    def get_applicable_primitives(self, current_theta: float, 
                                 current_gamma: float) -> List[MotionPrimitive]:
        """
        Get motion primitives applicable from current state.
        
        Args:
            current_theta: Current trailer orientation [degrees]
            current_gamma: Current articulation angle [degrees]
            
        Returns:
            List of applicable motion primitives
        """
        # Discretize current state
        theta_key = self._discretize_angle(current_theta)
        gamma_key = self._discretize_gamma(current_gamma)
        
        index_key = (theta_key, gamma_key)
        
        return self.primitive_index.get(index_key, [])
    
    def calculate_end_state(self, start_x: float, start_y: float,
                           start_theta: float, start_gamma: float,
                           primitive: MotionPrimitive) -> Tuple[float, float, float, float]:
        """
        Calculate end state after applying motion primitive.
        
        Args:
            start_x, start_y: Start position
            start_theta: Start trailer orientation [degrees]
            start_gamma: Start articulation angle [degrees]
            primitive: Motion primitive to apply
            
        Returns:
            (end_x, end_y, end_theta, end_gamma)
        """
        if not primitive.x_trajectory:
            return start_x, start_y, start_theta, start_gamma
        
        # Apply relative motion from primitive
        dx = primitive.x_trajectory[-1] - primitive.x_trajectory[0]
        dy = primitive.y_trajectory[-1] - primitive.y_trajectory[0]
        
        # Rotate relative motion by current orientation
        cos_theta = math.cos(math.radians(start_theta))
        sin_theta = math.sin(math.radians(start_theta))
        
        rotated_dx = dx * cos_theta - dy * sin_theta
        rotated_dy = dx * sin_theta + dy * cos_theta
        
        end_x = start_x + rotated_dx
        end_y = start_y + rotated_dy
        end_theta = primitive.theta_end
        end_gamma = primitive.gamma_end
        
        return end_x, end_y, end_theta, end_gamma
    
    def _discretize_angle(self, angle: float) -> int:
        """Discretize angle to nearest discrete value."""
        # Normalize to 0-360
        angle = angle % 360
        
        # Find nearest discrete value
        return min(self.theta_discrete, key=lambda x: abs(x - angle))
    
    def _discretize_gamma(self, gamma: float) -> int:
        """Discretize articulation angle."""
        return min(self.gamma_discrete, key=lambda x: abs(x - gamma))
    
    def _calculate_path_length(self, x_traj: List[float], y_traj: List[float]) -> float:
        """Calculate total path length."""
        if len(x_traj) < 2:
            return 0.0
        
        total_length = 0.0
        for i in range(1, len(x_traj)):
            dx = x_traj[i] - x_traj[i-1]
            dy = y_traj[i] - y_traj[i-1]
            total_length += math.sqrt(dx*dx + dy*dy)
        
        return total_length
    
    def _calculate_curvature_cost(self, delta_traj: List[float]) -> float:
        """Calculate curvature cost from steering angles."""
        if not delta_traj:
            return 0.0
        
        return sum(abs(d) for d in delta_traj) / len(delta_traj)
    
    def get_vehicle_corners(self, x: float, y: float, theta: float, 
                           gamma: float) -> Tuple[List[float], List[float]]:
        """
        Calculate vehicle corner positions for collision detection.
        
        Based on staticobs_check.m from MATLAB.
        
        Args:
            x, y: Trailer axle position
            theta: Trailer orientation [radians]
            gamma: Articulation angle [radians]
            
        Returns:
            (x_corners, y_corners) - 8 corners total (4 trailer + 4 tractor)
        """
        # Vehicle parameters
        L_1f = self.vehicle_params.L_1f
        L_0f = self.vehicle_params.L_0f
        L_0b = self.vehicle_params.L_0b
        oh_1b = self.vehicle_params.oh_1b
        oh_1f = self.vehicle_params.oh_1f
        oh_0f = self.vehicle_params.oh_0f
        oh_0b = self.vehicle_params.oh_0b
        w_1 = self.vehicle_params.width
        
        # Trailer corners
        lv12_1 = math.sqrt(oh_1b**2 + (w_1/2)**2)
        lv34_1 = math.sqrt((w_1/2)**2 + oh_1f**2)
        
        # Trailer corner angles
        av1 = theta + math.pi/2 + math.atan(oh_1b/(w_1/2))
        av2 = theta - math.pi/2 - math.atan(oh_1b/(w_1/2))
        av3 = theta - math.atan((w_1/2)/oh_1f)
        av4 = theta + math.atan((w_1/2)/oh_1f)
        
        # Trailer corners
        xv1_1 = x + lv12_1 * math.cos(av1)
        yv1_1 = y + lv12_1 * math.sin(av1)
        xv2_1 = x + lv12_1 * math.cos(av2)
        yv2_1 = y + lv12_1 * math.sin(av2)
        xv3_1 = x + lv34_1 * math.cos(av3)
        yv3_1 = y + lv34_1 * math.sin(av3)
        xv4_1 = x + lv34_1 * math.cos(av4)
        yv4_1 = y + lv34_1 * math.sin(av4)
        
        # Tractor calculations
        x_1f = x + L_1f * math.cos(theta)  # King pin position
        y_1f = y + L_1f * math.sin(theta)
        theta_0 = theta + gamma  # Tractor orientation
        x_0 = x_1f - L_0b * math.cos(theta_0)  # Tractor drive axle
        y_0 = y_1f - L_0b * math.sin(theta_0)
        
        # Tractor corner distances
        lv12_0 = math.sqrt(oh_0b**2 + (w_1/2)**2)
        lv34_0 = math.sqrt((w_1/2)**2 + (oh_0f + L_0f)**2)
        
        # Tractor corner angles
        av1_0 = theta_0 + math.pi/2 + math.atan(oh_0b/(w_1/2))
        av2_0 = theta_0 - math.pi/2 - math.atan(oh_0b/(w_1/2))
        av3_0 = theta_0 - math.atan((w_1/2)/(oh_0f + L_0f))
        av4_0 = theta_0 + math.atan((w_1/2)/(oh_0f + L_0f))
        
        # Tractor corners
        xv1_0 = x_0 + lv12_0 * math.cos(av1_0)
        yv1_0 = y_0 + lv12_0 * math.sin(av1_0)
        xv2_0 = x_0 + lv12_0 * math.cos(av2_0)
        yv2_0 = y_0 + lv12_0 * math.sin(av2_0)
        xv3_0 = x_0 + lv34_0 * math.cos(av3_0)
        yv3_0 = y_0 + lv34_0 * math.sin(av3_0)
        xv4_0 = x_0 + lv34_0 * math.cos(av4_0)
        yv4_0 = y_0 + lv34_0 * math.sin(av4_0)
        
        # Combine all corners
        x_corners = [xv1_1, xv2_1, xv3_1, xv4_1, xv1_0, xv2_0, xv3_0, xv4_0]
        y_corners = [yv1_1, yv2_1, yv3_1, yv4_1, yv1_0, yv2_0, yv3_0, yv4_0]
        
        return x_corners, y_corners


if __name__ == "__main__":
    # Test motion primitive loader
    logging.basicConfig(level=logging.INFO)
    
    print("Testing Motion Primitive Loader")
    print("="*50)
    
    loader = MotionPrimitiveLoader()
    success = loader.load_motion_primitives()
    
    print(f"Load success: {success}")
    print(f"Total primitives: {len(loader.primitives)}")
    print(f"Index keys: {len(loader.primitive_index)}")
    
    # Test primitive retrieval
    applicable = loader.get_applicable_primitives(0.0, 0.0)
    print(f"Primitives for (0°, 0°): {len(applicable)}")
    
    if applicable:
        primitive = applicable[0]
        print(f"Sample primitive: {primitive.theta_start}° → {primitive.theta_end}°")
        print(f"Path length: {primitive.path_length:.2f}m")
        
        # Test end state calculation
        end_x, end_y, end_theta, end_gamma = loader.calculate_end_state(
            0.0, 0.0, 0.0, 0.0, primitive
        )
        print(f"End state: ({end_x:.2f}, {end_y:.2f}) θ={end_theta:.1f}° γ={end_gamma:.1f}°")
        
        # Test vehicle corners
        x_corners, y_corners = loader.get_vehicle_corners(0.0, 0.0, 0.0, 0.0)
        print(f"Vehicle corners: {len(x_corners)} points")
    
    print("✅ Motion primitive loader test completed!")
