#!/usr/bin/env python3
"""
Rectangular Heuristics - Converted from MATLAB rectheur.m and rectheur1.m

This module provides enhanced heuristic calculations for A* path planning
that consider obstacle constraints, specifically the main DC building.

Original MATLAB files: rectheur.m, rectheur1.m
Author: Manojpriyadharson Kannan (Student number: 638628)
Converted by: CATALYST Team

Heuristic Features:
- Obstacle-aware path cost estimation
- Zone-based routing around main DC building
- Multiple path strategies based on position
- Manhattan distance with obstacle penalties
"""

import math
from typing import Tuple, Optional
from dataclasses import dataclass
import logging


@dataclass
class HeuristicState:
    """State information for heuristic calculation."""
    x: float
    y: float
    theta: float = 0.0


@dataclass
class HeuristicLimits:
    """Boundary limits for heuristic zones."""
    xlim1: float = 25.0   # X boundary limit
    ylim1: float = 146.0  # Y boundary limit 1
    ylim2: float = 50.0   # Y boundary limit 2


class RectangularHeuristics:
    """
    Enhanced heuristic calculator considering obstacle constraints.
    
    Provides obstacle-aware heuristic calculations for A* path planning,
    implementing the logic from MATLAB rectheur.m and rectheur1.m files.
    """
    
    def __init__(self):
        """Initialize rectangular heuristics calculator."""
        self.logger = logging.getLogger(__name__)
        self.limits = HeuristicLimits()
        self.logger.info("Rectangular Heuristics initialized")
    
    def calculate_heuristic_main(self, current: HeuristicState, goal: HeuristicState) -> float:
        """
        Calculate heuristic cost considering main DC building obstacle.
        
        Based on MATLAB rectheur.m function.
        
        Args:
            current: Current state (x, y, theta)
            goal: Goal state (x, y, theta)
            
        Returns:
            Heuristic cost considering obstacle constraints
        """
        # Main DC building boundary polygon
        xv_1 = [58.6, 28, 28, 58.6]
        yv_1 = [53, 53, 144, 144]
        
        # Check if current position is inside main building polygon
        in_polygon = self._point_in_polygon(current.x, current.y, xv_1, yv_1)
        
        # Zone conditions
        zone_a = current.y >= 144 and current.x >= 28
        zone_b = current.y >= 119.5 and current.y <= 144 and current.x >= 87
        zone_c = current.x < 28 and current.y > 54
        
        if zone_a:
            # Route around top of building
            length = (abs(current.x - self.limits.xlim1) + 
                     abs(current.y - self.limits.ylim2) + 
                     abs(self.limits.xlim1 - goal.x) + 
                     abs(self.limits.ylim2 - goal.y)) * 3.0
        elif zone_b:
            # Route around right side of building
            length = (abs(current.y - self.limits.ylim1) + 
                     abs(current.x - self.limits.xlim1) + 
                     abs(self.limits.ylim1 - self.limits.ylim2) + 
                     abs(self.limits.xlim1 - goal.x) + 
                     abs(self.limits.ylim2 - goal.y)) * 3.0
        elif zone_c:
            # Route around left side of building
            length = (abs(current.y - self.limits.ylim2) + 
                     abs(self.limits.ylim2 - goal.y) + 
                     abs(current.x - goal.x)) * 2.8
        elif in_polygon:
            # Inside obstacle - infinite cost
            length = float('inf')
        else:
            # Direct Manhattan distance
            length = (abs(current.y - goal.y) + abs(current.x - goal.x)) * 2.5
        
        return length
    
    def calculate_heuristic_alternative(self, current: HeuristicState, goal: HeuristicState) -> float:
        """
        Calculate alternative heuristic cost for different routing strategy.
        
        Based on MATLAB rectheur1.m function.
        
        Args:
            current: Current state (x, y, theta)
            goal: Goal state (x, y, theta)
            
        Returns:
            Alternative heuristic cost
        """
        # Zone conditions (different from main heuristic)
        zone_a = current.y <= 53 and current.x >= 28
        zone_b = current.y > 53 and current.y < 144 and current.x < 28
        zone_c = current.y <= 61 and current.y >= 53 and current.x > 108
        
        if zone_a:
            # Route below building
            length = (abs(current.x - self.limits.xlim1) + 
                     abs(current.y - self.limits.ylim1) + 
                     abs(self.limits.xlim1 - goal.x) + 
                     abs(self.limits.ylim1 - goal.y)) * 2.5
        elif zone_b:
            # Route left of building
            length = (abs(current.x - goal.x) + 
                     abs(current.y - self.limits.ylim1) + 
                     abs(self.limits.ylim1 - goal.y)) * 2.8
        elif zone_c:
            # Route around narrow passage
            length = (abs(current.y - self.limits.ylim2) + 
                     abs(current.x - self.limits.xlim1) + 
                     abs(self.limits.ylim1 - self.limits.ylim2) + 
                     abs(self.limits.xlim1 - goal.x) + 
                     abs(self.limits.ylim1 - goal.y)) * 3.0
        else:
            # Direct Manhattan distance
            length = (abs(current.y - goal.y) + abs(current.x - goal.x)) * 2.5
        
        return length
    
    def calculate_adaptive_heuristic(self, current: HeuristicState, goal: HeuristicState, 
                                   strategy: str = "main") -> float:
        """
        Calculate adaptive heuristic based on strategy selection.
        
        Args:
            current: Current state
            goal: Goal state
            strategy: "main", "alternative", or "combined"
            
        Returns:
            Calculated heuristic cost
        """
        if strategy == "main":
            return self.calculate_heuristic_main(current, goal)
        elif strategy == "alternative":
            return self.calculate_heuristic_alternative(current, goal)
        elif strategy == "combined":
            # Use minimum of both strategies
            main_cost = self.calculate_heuristic_main(current, goal)
            alt_cost = self.calculate_heuristic_alternative(current, goal)
            return min(main_cost, alt_cost)
        else:
            raise ValueError(f"Unknown strategy: {strategy}")
    
    def _point_in_polygon(self, x: float, y: float, xv: list, yv: list) -> bool:
        """
        Check if point is inside polygon using ray casting algorithm.
        
        Equivalent to MATLAB InPolygon function used in rectheur.m
        
        Args:
            x: Point x coordinate
            y: Point y coordinate
            xv: Polygon x vertices
            yv: Polygon y vertices
            
        Returns:
            True if point is inside polygon
        """
        n = len(xv)
        inside = False
        
        j = n - 1
        for i in range(n):
            if ((yv[i] > y) != (yv[j] > y)) and \
               (x < (xv[j] - xv[i]) * (y - yv[i]) / (yv[j] - yv[i]) + xv[i]):
                inside = not inside
            j = i
        
        return inside
    
    def get_zone_info(self, state: HeuristicState) -> dict:
        """
        Get zone information for debugging and visualization.
        
        Args:
            state: Current state
            
        Returns:
            Dictionary with zone classification and costs
        """
        # Main DC building polygon
        xv_1 = [58.6, 28, 28, 58.6]
        yv_1 = [53, 53, 144, 144]
        
        in_polygon = self._point_in_polygon(state.x, state.y, xv_1, yv_1)
        
        # Main heuristic zones
        zone_a_main = state.y >= 144 and state.x >= 28
        zone_b_main = state.y >= 119.5 and state.y <= 144 and state.x >= 87
        zone_c_main = state.x < 28 and state.y > 54
        
        # Alternative heuristic zones
        zone_a_alt = state.y <= 53 and state.x >= 28
        zone_b_alt = state.y > 53 and state.y < 144 and state.x < 28
        zone_c_alt = state.y <= 61 and state.y >= 53 and state.x > 108
        
        return {
            'position': (state.x, state.y),
            'inside_building': in_polygon,
            'main_zones': {
                'zone_a': zone_a_main,
                'zone_b': zone_b_main,
                'zone_c': zone_c_main
            },
            'alternative_zones': {
                'zone_a': zone_a_alt,
                'zone_b': zone_b_alt,
                'zone_c': zone_c_alt
            }
        }
    
    def validate_heuristic_consistency(self, start: HeuristicState, goal: HeuristicState) -> dict:
        """
        Validate heuristic consistency (admissibility check).
        
        Args:
            start: Start state
            goal: Goal state
            
        Returns:
            Validation results
        """
        # Calculate direct Euclidean distance
        euclidean_dist = math.sqrt((goal.x - start.x)**2 + (goal.y - start.y)**2)
        
        # Calculate Manhattan distance
        manhattan_dist = abs(goal.x - start.x) + abs(goal.y - start.y)
        
        # Calculate heuristic costs
        main_heuristic = self.calculate_heuristic_main(start, goal)
        alt_heuristic = self.calculate_heuristic_alternative(start, goal)
        
        return {
            'euclidean_distance': euclidean_dist,
            'manhattan_distance': manhattan_dist,
            'main_heuristic': main_heuristic,
            'alternative_heuristic': alt_heuristic,
            'heuristic_admissible': main_heuristic >= euclidean_dist,
            'ratio_to_euclidean': main_heuristic / euclidean_dist if euclidean_dist > 0 else float('inf')
        }


def test_rectangular_heuristics():
    """Test rectangular heuristics implementation."""
    print("Testing Rectangular Heuristics")
    print("="*50)
    
    heuristics = RectangularHeuristics()
    
    # Test states
    test_cases = [
        # (current, goal, description)
        (HeuristicState(10, 10), HeuristicState(300, 180), "Far bottom-left to top-right"),
        (HeuristicState(50, 150), HeuristicState(200, 60), "Above building to right side"),
        (HeuristicState(15, 80), HeuristicState(250, 100), "Left side to right side"),
        (HeuristicState(120, 55), HeuristicState(180, 170), "Right side to top"),
        (HeuristicState(40, 100), HeuristicState(100, 80), "Inside building area"),
    ]
    
    for i, (current, goal, description) in enumerate(test_cases):
        print(f"\nTest Case {i+1}: {description}")
        print(f"Current: ({current.x:.1f}, {current.y:.1f})")
        print(f"Goal: ({goal.x:.1f}, {goal.y:.1f})")
        
        # Calculate costs
        main_cost = heuristics.calculate_heuristic_main(current, goal)
        alt_cost = heuristics.calculate_heuristic_alternative(current, goal)
        combined_cost = heuristics.calculate_adaptive_heuristic(current, goal, "combined")
        
        print(f"Main heuristic: {main_cost:.2f}")
        print(f"Alternative heuristic: {alt_cost:.2f}")
        print(f"Combined heuristic: {combined_cost:.2f}")
        
        # Get zone information
        zone_info = heuristics.get_zone_info(current)
        print(f"Zone info: {zone_info}")
        
        # Validate consistency
        validation = heuristics.validate_heuristic_consistency(current, goal)
        print(f"Validation: Admissible={validation['heuristic_admissible']}, "
              f"Ratio={validation['ratio_to_euclidean']:.2f}")
    
    print(f"\n✅ Rectangular Heuristics test completed!")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    test_rectangular_heuristics()
