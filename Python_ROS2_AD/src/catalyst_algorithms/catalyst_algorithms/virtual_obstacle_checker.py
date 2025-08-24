#!/usr/bin/env python3
"""
Virtual Obstacle Check - Converted from MATLAB virtualobs_check.m

This module provides intelligent motion primitive selection based on
obstacle proximity. It helps reduce computational time by selecting
appropriate angular ranges for motion primitives based on the presence
of obstacles in the vehicle's forward path.

Original MATLAB file: virtualobs_check.m
Author: Manojpriyadharson Kannan 

Features:
- Forward-looking obstacle detection
- Adaptive motion primitive selection
- Reduced computational overhead
- Real-life driving behavior simulation
"""

import math
import numpy as np
from typing import Tuple, List, Optional
from dataclasses import dataclass
import logging

from .collision_detection import Environment, ObstaclePolygon


@dataclass
class VehicleState:
    """Current vehicle state for virtual obstacle checking."""
    x: float           # Semi-trailer x position [m]
    y: float           # Semi-trailer y position [m]
    theta: float       # Semi-trailer orientation [rad]
    gamma: float       # Articulation angle [rad]
    

@dataclass
class VehicleGeometry:
    """Vehicle geometry parameters."""
    L_1f: float = 8.475   # Wheelbase of semitrailer [m]
    L_0f: float = 3.8     # Wheel base of the tractor [m]
    L_0b: float = 0.3     # Distance of 1st king-pin to tractor drive axle [m]
    oh_1b: float = 5.0    # Longitudinal distance from trailer axle to end [m]
    oh_1f: float = 9.475  # Longitudinal distance from trailer axle to front [m]
    w_1: float = 2.5      # Width of trailer [m]
    oh_0f: float = 1.5    # Frontal overhang of truck [m]
    oh_0b: float = 0.94   # Distance from drive axle to end of tractor [m]


@dataclass
class MotionPrimitiveRanges:
    """Motion primitive angular ranges."""
    positive_range: List[float]  # Forward rotation angles
    negative_range: List[float]  # Backward rotation angles


class VirtualObstacleChecker:
    """
    Virtual obstacle detection for intelligent motion primitive selection.
    
    This class implements forward-looking obstacle detection to help
    the A* algorithm select appropriate motion primitives, reducing
    computational overhead while maintaining path quality.
    """
    
    def __init__(self, geometry: Optional[VehicleGeometry] = None):
        """
        Initialize virtual obstacle checker.
        
        Args:
            geometry: Vehicle geometry parameters
        """
        self.logger = logging.getLogger(__name__)
        self.geometry = geometry or VehicleGeometry()
        self.logger.info("Virtual Obstacle Checker initialized")
    
    def check_virtual_obstacles(self, current_state: VehicleState, 
                              environment: Environment,
                              current_theta_deg: float) -> MotionPrimitiveRanges:
        """
        Check for virtual obstacles and return appropriate motion primitive ranges.
        
        Args:
            current_state: Current vehicle state
            environment: Environment with obstacles
            current_theta_deg: Current orientation in degrees
            
        Returns:
            Motion primitive ranges based on obstacle detection
        """
        # Calculate tractor corners and forward detection area
        detection_polygon = self._calculate_forward_detection_area(current_state)
        
        # Check for obstacle collision in forward area
        obstacle_detected = self._check_obstacle_collision(detection_polygon, environment)
        
        # Select motion primitive ranges based on detection
        if obstacle_detected:
            # Wide range for obstacle avoidance
            positive_range = list(range(int(current_theta_deg), int(current_theta_deg + 91), 9))
            negative_range = list(range(int(current_theta_deg - 90), int(current_theta_deg + 1), 9))
        else:
            # Narrow range for efficient forward progress
            positive_range = list(range(int(current_theta_deg), int(current_theta_deg + 37), 9))
            negative_range = list(range(int(current_theta_deg - 36), int(current_theta_deg + 1), 9))
        
        return MotionPrimitiveRanges(
            positive_range=positive_range,
            negative_range=negative_range
        )
    
    def _calculate_forward_detection_area(self, state: VehicleState) -> List[Tuple[float, float]]:
        """
        Calculate forward detection polygon for virtual obstacle checking.
        
        Args:
            state: Current vehicle state
            
        Returns:
            List of (x, y) coordinates defining detection polygon
        """
        # Calculate tractor position and orientation
        x_kingpin = state.x + self.geometry.L_1f * math.cos(state.theta)
        y_kingpin = state.y + self.geometry.L_1f * math.sin(state.theta)
        
        theta_tractor = state.theta + state.gamma
        
        x_drive_axle = x_kingpin - self.geometry.L_0b * math.cos(theta_tractor)
        y_drive_axle = y_kingpin - self.geometry.L_0b * math.sin(theta_tractor)
        
        # Calculate tractor front corners
        front_corners = self._calculate_tractor_corners(x_drive_axle, y_drive_axle, theta_tractor)
        
        # Create forward detection area (extended in front of vehicle)
        detection_distance = 5.0  # Look ahead distance [m]
        detection_width = 2.0     # Additional width for detection [m]
        
        # Get front corners (corners 3 and 4)
        front_left = front_corners[2]   # xv3, yv3
        front_right = front_corners[3]  # xv4, yv4
        
        # Extend forward from front corners
        extended_front_left = (
            front_left[0] + detection_distance * math.cos(theta_tractor),
            front_left[1] + detection_distance * math.sin(theta_tractor)
        )
        
        extended_front_right = (
            front_right[0] + detection_distance * math.cos(theta_tractor),
            front_right[1] + detection_distance * math.sin(theta_tractor)
        )
        
        # Create detection polygon
        detection_polygon = [
            front_right,
            front_left,
            extended_front_left,
            extended_front_right
        ]
        
        return detection_polygon
    
    def _calculate_tractor_corners(self, x_center: float, y_center: float, 
                                 theta: float) -> List[Tuple[float, float]]:
        """
        Calculate tractor corner positions.
        
        Args:
            x_center: X coordinate of drive axle center
            y_center: Y coordinate of drive axle center
            theta: Tractor orientation [rad]
            
        Returns:
            List of corner coordinates [(x1,y1), (x2,y2), (x3,y3), (x4,y4)]
        """
        # Vector lengths to corners
        lv12 = math.hypot(self.geometry.oh_0b, self.geometry.w_1/2)
        lv34 = math.hypot(self.geometry.w_1/2, self.geometry.L_0f + self.geometry.oh_0f)
        
        # Convert theta to degrees for angle calculations
        theta_deg = math.degrees(theta)
        
        # Angles to corners
        av1 = theta_deg + 90 + math.degrees(math.atan(self.geometry.oh_0b / (self.geometry.w_1/2)))
        av2 = theta_deg - 90 - math.degrees(math.atan(self.geometry.oh_0b / (self.geometry.w_1/2)))
        av3 = theta_deg - math.degrees(math.atan((self.geometry.w_1/2) / (self.geometry.oh_0f + self.geometry.L_0f)))
        av4 = theta_deg + math.degrees(math.atan((self.geometry.w_1/2) / (self.geometry.oh_0f + self.geometry.L_0f)))
        
        # Calculate corner positions
        corners = [
            (x_center + lv12 * math.cos(math.radians(av1)),
             y_center + lv12 * math.sin(math.radians(av1))),  # Corner 1
            (x_center + lv12 * math.cos(math.radians(av2)),
             y_center + lv12 * math.sin(math.radians(av2))),  # Corner 2
            (x_center + lv34 * math.cos(math.radians(av3)),
             y_center + lv34 * math.sin(math.radians(av3))),  # Corner 3
            (x_center + lv34 * math.cos(math.radians(av4)),
             y_center + lv34 * math.sin(math.radians(av4)))   # Corner 4
        ]
        
        return corners
    
    def _check_obstacle_collision(self, detection_polygon: List[Tuple[float, float]], 
                                environment: Environment) -> bool:
        """
        Check if detection polygon intersects with any obstacles.
        
        Args:
            detection_polygon: Forward detection area polygon
            environment: Environment with obstacles
            
        Returns:
            True if obstacle detected in forward area
        """
        for obstacle in environment.obstacles:
            if self._polygons_intersect(detection_polygon, 
                                      list(zip(obstacle.x_coords, obstacle.y_coords))):
                return True
        
        return False
    
    def _polygons_intersect(self, poly1: List[Tuple[float, float]], 
                          poly2: List[Tuple[float, float]]) -> bool:
        """
        Check if two polygons intersect using separating axis theorem.
        
        Args:
            poly1: First polygon vertices
            poly2: Second polygon vertices
            
        Returns:
            True if polygons intersect
        """
        # Simple bounding box check first
        if not self._bounding_boxes_intersect(poly1, poly2):
            return False
        
        # Check if any vertices of poly1 are inside poly2
        for point in poly1:
            if self._point_in_polygon(point[0], point[1], poly2):
                return True
        
        # Check if any vertices of poly2 are inside poly1
        for point in poly2:
            if self._point_in_polygon(point[0], point[1], poly1):
                return True
        
        # Check edge intersections
        return self._edges_intersect(poly1, poly2)
    
    def _bounding_boxes_intersect(self, poly1: List[Tuple[float, float]], 
                                poly2: List[Tuple[float, float]]) -> bool:
        """Check if bounding boxes of two polygons intersect."""
        min_x1, max_x1 = min(p[0] for p in poly1), max(p[0] for p in poly1)
        min_y1, max_y1 = min(p[1] for p in poly1), max(p[1] for p in poly1)
        
        min_x2, max_x2 = min(p[0] for p in poly2), max(p[0] for p in poly2)
        min_y2, max_y2 = min(p[1] for p in poly2), max(p[1] for p in poly2)
        
        return not (max_x1 < min_x2 or max_x2 < min_x1 or max_y1 < min_y2 or max_y2 < min_y1)
    
    def _point_in_polygon(self, x: float, y: float, polygon: List[Tuple[float, float]]) -> bool:
        """Check if point is inside polygon using ray casting."""
        n = len(polygon)
        inside = False
        
        j = n - 1
        for i in range(n):
            if ((polygon[i][1] > y) != (polygon[j][1] > y)) and \
               (x < (polygon[j][0] - polygon[i][0]) * (y - polygon[i][1]) / 
                (polygon[j][1] - polygon[i][1]) + polygon[i][0]):
                inside = not inside
            j = i
        
        return inside
    
    def _edges_intersect(self, poly1: List[Tuple[float, float]], 
                        poly2: List[Tuple[float, float]]) -> bool:
        """Check if any edges of the two polygons intersect."""
        def ccw(A, B, C):
            return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])
        
        def segments_intersect(A, B, C, D):
            return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)
        
        n1, n2 = len(poly1), len(poly2)
        
        for i in range(n1):
            edge1 = (poly1[i], poly1[(i + 1) % n1])
            for j in range(n2):
                edge2 = (poly2[j], poly2[(j + 1) % n2])
                if segments_intersect(edge1[0], edge1[1], edge2[0], edge2[1]):
                    return True
        
        return False
    
    def get_detection_info(self, state: VehicleState, current_theta_deg: float) -> dict:
        """
        Get detection information for debugging and visualization.
        
        Args:
            state: Current vehicle state
            current_theta_deg: Current orientation in degrees
            
        Returns:
            Dictionary with detection information
        """
        detection_polygon = self._calculate_forward_detection_area(state)
        tractor_corners = self._calculate_tractor_corners(
            state.x + self.geometry.L_1f * math.cos(state.theta) - 
            self.geometry.L_0b * math.cos(state.theta + state.gamma),
            state.y + self.geometry.L_1f * math.sin(state.theta) - 
            self.geometry.L_0b * math.sin(state.theta + state.gamma),
            state.theta + state.gamma
        )
        
        return {
            'vehicle_state': {
                'x': state.x,
                'y': state.y,
                'theta_deg': math.degrees(state.theta),
                'gamma_deg': math.degrees(state.gamma)
            },
            'detection_polygon': detection_polygon,
            'tractor_corners': tractor_corners,
            'current_theta_deg': current_theta_deg
        }


def test_virtual_obstacle_checker():
    """Test virtual obstacle checker implementation."""
    print("Testing Virtual Obstacle Checker")
    print("="*50)
    
    checker = VirtualObstacleChecker()
    
    # Create test environment with simple obstacle
    from .dpd_environment import DPDEnvironmentBuilder
    env_builder = DPDEnvironmentBuilder()
    environment = env_builder.build_dpd_environment()
    
    # Test states
    test_cases = [
        # (state, description)
        (VehicleState(50.0, 100.0, math.radians(0), math.radians(0)), 
         "Heading east, no immediate obstacles"),
        (VehicleState(100.0, 60.0, math.radians(90), math.radians(10)), 
         "Heading north near main building"),
        (VehicleState(200.0, 120.0, math.radians(180), math.radians(-5)), 
         "Heading west in open area"),
        (VehicleState(30.0, 60.0, math.radians(45), math.radians(0)), 
         "Heading northeast near obstacles"),
    ]
    
    for i, (state, description) in enumerate(test_cases):
        print(f"\nTest Case {i+1}: {description}")
        print(f"Position: ({state.x:.1f}, {state.y:.1f})")
        print(f"Orientation: {math.degrees(state.theta):.1f}°")
        print(f"Articulation: {math.degrees(state.gamma):.1f}°")
        
        # Check virtual obstacles
        current_theta_deg = math.degrees(state.theta)
        mp_ranges = checker.check_virtual_obstacles(state, environment, current_theta_deg)
        
        print(f"Motion Primitive Ranges:")
        print(f"  Positive: {mp_ranges.positive_range[0]}° to {mp_ranges.positive_range[-1]}°")
        print(f"  Negative: {mp_ranges.negative_range[0]}° to {mp_ranges.negative_range[-1]}°")
        print(f"  Total angles: {len(mp_ranges.positive_range) + len(mp_ranges.negative_range)}")
        
        # Get detection info
        detection_info = checker.get_detection_info(state, current_theta_deg)
        print(f"Detection polygon vertices: {len(detection_info['detection_polygon'])}")
        
        # Check if narrow or wide range selected
        total_range = mp_ranges.positive_range[-1] - mp_ranges.negative_range[0]
        strategy = "OBSTACLE AVOIDANCE" if total_range > 100 else "EFFICIENT FORWARD"
        print(f"Strategy: {strategy} (range: {total_range}°)")
    
    print(f"\n✅ Virtual Obstacle Checker test completed!")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    test_virtual_obstacle_checker()
