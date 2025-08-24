#!/usr/bin/env python3
"""
Collision Detection Module for CATALYST A* Plugin

This module implements collision detection for articulated vehicles
based on the MATLAB staticobs_check.m function.
"""

import math
import numpy as np
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass
import logging


@dataclass
class ObstaclePolygon:
    """Represents a polygonal obstacle."""
    id: int
    x_coords: List[float]
    y_coords: List[float]
    
    def __post_init__(self):
        """Validate obstacle data."""
        if len(self.x_coords) != len(self.y_coords):
            raise ValueError("X and Y coordinates must have same length")
        if len(self.x_coords) < 3:
            raise ValueError("Obstacle must have at least 3 vertices")


@dataclass
class Environment:
    """Environment boundaries and obstacles."""
    length_dc: float = 328.0  # From MATLAB
    width_dc: float = 200.0   # From MATLAB
    obstacles: List[ObstaclePolygon] = None
    
    def __post_init__(self):
        if self.obstacles is None:
            self.obstacles = []


class CollisionDetector:
    """
    Collision detection for articulated vehicles.
    
    Based on MATLAB staticobs_check.m function with support for:
    - Boundary checking
    - Polygon obstacle collision
    - Articulated vehicle geometry
    """
    
    def __init__(self):
        """Initialize collision detector."""
        self.logger = logging.getLogger(__name__)
        
        # Vehicle parameters (from MATLAB staticobs_check.m)
        self.L_1f = 8.475  # Trailer wheelbase [m]
        self.L_0f = 3.8    # Tractor wheelbase [m]  
        self.L_0b = 0.3    # King-pin to tractor drive axle [m]
        self.oh_1b = 5.0   # Trailer rear overhang [m]
        self.oh_1f = 9.475 # Trailer front overhang [m] (L_1f + 1)
        self.oh_0f = 1.5   # Tractor front overhang [m]
        self.oh_0b = 0.94  # Tractor rear overhang [m]
        self.w_1 = 2.5     # Vehicle width [m]
        
        # Environment boundaries (from MATLAB)
        self.max_x = 286.0  # From MATLAB boundary check
        self.max_y = 200.0  # From MATLAB boundary check
        self.min_x = 0.0
        self.min_y = 0.0
        
        self.logger.info("Collision detector initialized")
    
    def check_collision(self, x: float, y: float, theta: float, gamma: float,
                       environment: Environment, current_xa: float = 0.0, 
                       current_ya: float = 0.0) -> bool:
        """
        Check collision for articulated vehicle at given pose.
        
        Based on MATLAB staticobs_check.m function.
        
        Args:
            x, y: Trailer axle position
            theta: Trailer orientation [radians]
            gamma: Articulation angle [radians]  
            environment: Environment with obstacles
            current_xa, current_ya: Additional offset (from MATLAB cur.xa, cur.ya)
            
        Returns:
            True if collision-free, False if collision detected
        """
        try:
            # Get vehicle corner positions
            x_corners, y_corners = self._get_vehicle_corners(
                x, y, theta, gamma, current_xa, current_ya
            )
            
            # Check boundary collisions
            if self._check_boundary_collision(x_corners, y_corners, environment):
                return False
            
            # Check obstacle collisions
            if self._check_obstacle_collisions(x_corners, y_corners, environment):
                return False
            
            return True
            
        except Exception as e:
            self.logger.error(f"Collision check failed: {str(e)}")
            return False
    
    def _get_vehicle_corners(self, x: float, y: float, theta: float, gamma: float,
                           xa_offset: float = 0.0, ya_offset: float = 0.0) -> Tuple[List[float], List[float]]:
        """
        Calculate all 8 vehicle corners (4 trailer + 4 tractor).
        
        Direct conversion from MATLAB staticobs_check.m
        """
        # Trailer corner calculations
        lv12_1 = math.sqrt(self.oh_1b**2 + (self.w_1/2)**2)  # Vector 1,2 length
        lv34_1 = math.sqrt((self.w_1/2)**2 + self.oh_1f**2)  # Vector 3,4 length
        
        # Trailer corner angles (converted from MATLAB degrees to radians)
        av1 = theta + math.pi/2 + math.atan(self.oh_1b/(self.w_1/2))
        av2 = theta - math.pi/2 - math.atan(self.oh_1b/(self.w_1/2))
        av3 = theta - math.atan((self.w_1/2)/self.oh_1f)
        av4 = theta + math.atan((self.w_1/2)/self.oh_1f)
        
        # Trailer corner positions
        xv1_1 = x + lv12_1 * math.cos(av1) + xa_offset
        yv1_1 = y + lv12_1 * math.sin(av1) + ya_offset
        xv2_1 = x + lv12_1 * math.cos(av2) + xa_offset
        yv2_1 = y + lv12_1 * math.sin(av2) + ya_offset
        xv3_1 = x + lv34_1 * math.cos(av3) + xa_offset
        yv3_1 = y + lv34_1 * math.sin(av3) + ya_offset
        xv4_1 = x + lv34_1 * math.cos(av4) + xa_offset
        yv4_1 = y + lv34_1 * math.sin(av4) + ya_offset
        
        # Tractor calculations (from MATLAB)
        x_1f = x + self.L_1f * math.cos(theta)  # King pin position
        y_1f = y + self.L_1f * math.sin(theta)
        theta_0 = theta + gamma  # Tractor orientation
        x_0 = x_1f - self.L_0b * math.cos(theta_0)  # Tractor drive axle center
        y_0 = y_1f - self.L_0b * math.sin(theta_0)
        
        # Tractor corner calculations
        lv12_0 = math.sqrt(self.oh_0b**2 + (self.w_1/2)**2)
        lv34_0 = math.sqrt((self.w_1/2)**2 + (self.L_0f + self.oh_0f)**2)
        
        # Tractor corner angles
        av1_0 = theta_0 + math.pi/2 + math.atan(self.oh_0b/(self.w_1/2))
        av2_0 = theta_0 - math.pi/2 - math.atan(self.oh_0b/(self.w_1/2))
        av3_0 = theta_0 - math.atan((self.w_1/2)/(self.oh_0f + self.L_0f))
        av4_0 = theta_0 + math.atan((self.w_1/2)/(self.oh_0f + self.L_0f))
        
        # Tractor corner positions
        xv1_0 = x_0 + lv12_0 * math.cos(av1_0) + xa_offset
        yv1_0 = y_0 + lv12_0 * math.sin(av1_0) + ya_offset
        xv2_0 = x_0 + lv12_0 * math.cos(av2_0) + xa_offset
        yv2_0 = y_0 + lv12_0 * math.sin(av2_0) + ya_offset
        xv3_0 = x_0 + lv34_0 * math.cos(av3_0) + xa_offset
        yv3_0 = y_0 + lv34_0 * math.sin(av3_0) + ya_offset
        xv4_0 = x_0 + lv34_0 * math.cos(av4_0) + xa_offset
        yv4_0 = y_0 + lv34_0 * math.sin(av4_0) + ya_offset
        
        # Combine all corners (from MATLAB)
        x_corners = [xv1_1, xv2_1, xv3_1, xv4_1, xv1_0, xv2_0, xv3_0, xv4_0]
        y_corners = [yv1_1, yv2_1, yv3_1, yv4_1, yv1_0, yv2_0, yv3_0, yv4_0]
        
        return x_corners, y_corners
    
    def _check_boundary_collision(self, x_corners: List[float], y_corners: List[float],
                                 environment: Environment) -> bool:
        """
        Check if vehicle corners are outside environment boundaries.
        
        From MATLAB: cibxmax, cibymax, cibxmin, cibymin checks
        """
        # Use environment boundaries if provided, otherwise use defaults
        max_x = environment.length_dc if hasattr(environment, 'length_dc') else self.max_x
        max_y = environment.width_dc if hasattr(environment, 'width_dc') else self.max_y
        
        # Check boundary violations (from MATLAB logic)
        cibxmax = any(x >= max_x for x in x_corners)
        cibymax = any(y >= max_y for y in y_corners)
        cibxmin = any(x <= self.min_x for x in x_corners)
        cibymin = any(y <= self.min_y for y in y_corners)
        
        return cibxmax or cibymax or cibxmin or cibymin
    
    def _check_obstacle_collisions(self, x_corners: List[float], y_corners: List[float],
                                  environment: Environment) -> bool:
        """
        Check collision with polygonal obstacles.
        
        From MATLAB: InPolygon check for each obstacle
        """
        if not environment.obstacles:
            return False
        
        for obstacle in environment.obstacles:
            # Remove zero values (from MATLAB logic)
            obs_x = [x for x in obstacle.x_coords if x != 0]
            obs_y = [y for y in obstacle.y_coords if y != 0]
            
            if len(obs_x) < 3 or len(obs_y) < 3:
                continue
            
            # Check if any vehicle corner is inside obstacle polygon
            for x_corner, y_corner in zip(x_corners, y_corners):
                if self._point_in_polygon(x_corner, y_corner, obs_x, obs_y):
                    return True
        
        return False
    
    def _point_in_polygon(self, x: float, y: float, poly_x: List[float], 
                         poly_y: List[float]) -> bool:
        """
        Check if point is inside polygon using ray casting algorithm.
        
        Equivalent to MATLAB InPolygon function.
        """
        if len(poly_x) != len(poly_y) or len(poly_x) < 3:
            return False
        
        n = len(poly_x)
        inside = False
        
        p1x, p1y = poly_x[0], poly_y[0]
        for i in range(1, n + 1):
            p2x, p2y = poly_x[i % n], poly_y[i % n]
            
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            
            p1x, p1y = p2x, p2y
        
        return inside
    
    def create_test_environment(self) -> Environment:
        """Create test environment with sample obstacles."""
        # Sample rectangular obstacles
        obstacles = [
            ObstaclePolygon(
                id=1,
                x_coords=[50, 70, 70, 50, 50],
                y_coords=[30, 30, 50, 50, 30]
            ),
            ObstaclePolygon(
                id=2,
                x_coords=[120, 140, 140, 120, 120],
                y_coords=[80, 80, 100, 100, 80]
            ),
            ObstaclePolygon(
                id=3,
                x_coords=[200, 220, 220, 200, 200],
                y_coords=[40, 40, 60, 60, 40]
            )
        ]
        
        return Environment(
            length_dc=328.0,
            width_dc=200.0,
            obstacles=obstacles
        )
    
    def visualize_collision_check(self, x: float, y: float, theta: float, gamma: float,
                                 environment: Environment) -> Dict:
        """
        Return collision check data for visualization.
        
        Args:
            x, y: Vehicle position
            theta, gamma: Vehicle angles [radians]
            environment: Environment
            
        Returns:
            Dictionary with collision data for plotting
        """
        x_corners, y_corners = self._get_vehicle_corners(x, y, theta, gamma)
        collision_free = self.check_collision(x, y, theta, gamma, environment)
        
        # Calculate vehicle outline polygons
        trailer_corners = [(x_corners[i], y_corners[i]) for i in range(4)]
        tractor_corners = [(x_corners[i], y_corners[i]) for i in range(4, 8)]
        
        return {
            'collision_free': collision_free,
            'vehicle_corners': list(zip(x_corners, y_corners)),
            'trailer_polygon': trailer_corners,
            'tractor_polygon': tractor_corners,
            'obstacles': [(obs.x_coords, obs.y_coords) for obs in environment.obstacles]
        }


def test_collision_detection():
    """Test collision detection functionality."""
    print("Testing Collision Detection")
    print("="*50)
    
    detector = CollisionDetector()
    environment = detector.create_test_environment()
    
    print(f"Environment: {environment.length_dc}m × {environment.width_dc}m")
    print(f"Obstacles: {len(environment.obstacles)}")
    
    # Test cases
    test_cases = [
        # (x, y, theta_deg, gamma_deg, expected_result)
        (25.0, 25.0, 0, 0, True),      # Free space
        (60.0, 40.0, 0, 0, False),     # Inside obstacle 1
        (10.0, 10.0, 45, 0, True),     # Free space at angle
        (130.0, 90.0, 90, 15, False),  # Inside obstacle 2
        (300.0, 100.0, 0, 0, False),   # Outside boundary
        (-10.0, 50.0, 0, 0, False),    # Outside boundary (negative)
    ]
    
    print("\nTest Cases:")
    for i, (x, y, theta_deg, gamma_deg, expected) in enumerate(test_cases):
        theta_rad = math.radians(theta_deg)
        gamma_rad = math.radians(gamma_deg)
        
        result = detector.check_collision(x, y, theta_rad, gamma_rad, environment)
        status = "✅ PASS" if result == expected else "❌ FAIL"
        
        print(f"  {i+1}: ({x:5.1f}, {y:5.1f}) θ={theta_deg:3.0f}° γ={gamma_deg:3.0f}° → "
              f"{'FREE' if result else 'COLLISION'} {status}")
    
    # Test vehicle corner calculation
    print(f"\nVehicle Corner Test:")
    x_corners, y_corners = detector._get_vehicle_corners(100.0, 100.0, 0.0, 0.0)
    print(f"  Vehicle at (100, 100): {len(x_corners)} corners calculated")
    print(f"  Trailer corners: {list(zip(x_corners[:4], y_corners[:4]))[:2]}...")
    print(f"  Tractor corners: {list(zip(x_corners[4:], y_corners[4:]))[:2]}...")
    
    # Test boundary detection
    print(f"\nBoundary Test:")
    boundary_test = detector._check_boundary_collision([350.0], [100.0], environment)
    print(f"  Point (350, 100) boundary collision: {boundary_test}")
    
    print("\n✅ Collision detection test completed!")
    return True


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    test_collision_detection()
