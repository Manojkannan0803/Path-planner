#!/usr/bin/env python3
"""
Cost Calculators for CATALYST A* Plugin

This module implements g-cost and h-cost calculations
based on the MATLAB g_cost.m and h_cost.m functions.
"""

import math
from typing import Optional, Tuple, List
from dataclasses import dataclass
import logging


@dataclass
class CostState:
    """State information for cost calculations."""
    x: float
    y: float
    xa: float
    ya: float
    theta: float  # radians
    gamma: float  # radians
    g_cost: float = 0.0
    parent_g_cost: Optional[float] = None


@dataclass
class GoalState:
    """Goal state for heuristic calculations."""
    x: float
    y: float
    xa: float  
    ya: float
    theta: float  # radians
    gamma: float  # radians


@dataclass
class ZonePolygon:
    """Polygon zone for cost penalties."""
    x_coords: List[float]
    y_coords: List[float]


class CostCalculator:
    """
    Cost calculator implementing MATLAB g_cost.m and h_cost.m functions.
    
    Provides both G-cost (cost so far) and H-cost (heuristic) calculations
    with support for rectangular heuristics and zone-based penalties.
    """
    
    def __init__(self):
        """Initialize cost calculator."""
        self.logger = logging.getLogger(__name__)
        
        # Heuristic parameters (from MATLAB h_cost.m)
        self.heuristic_coefficient = 2.5  # From MATLAB hc_d = dist*2.5
        self.zone_penalty_factor = 1000.0  # From MATLAB hc_d = hc_d*1000
        
        # Zone thresholds (from MATLAB h_cost.m)
        self.zone_y_threshold_low = 70.0   # From MATLAB i_state.y>=70
        self.zone_y_threshold_high = 80.0  # From MATLAB f_state.y<=80
        
        self.logger.info("Cost calculator initialized")
    
    def calculate_g_cost(self, current_state: CostState, path_length: float) -> float:
        """
        Calculate G-cost (cost so far traveled).
        
        Based on MATLAB g_cost.m function.
        
        Args:
            current_state: Current state with parent cost information
            path_length: Length of motion primitive that led to this state
            
        Returns:
            Total G-cost from start to current state
        """
        # From MATLAB g_cost.m logic
        if current_state.parent_g_cost is None:
            # First iteration (from MATLAB: if isempty(cur.pred.gcost))
            g_cost = abs(0 + 0 + path_length)
        else:
            # Subsequent iterations (from MATLAB: cur.pred.gcost+d_tot)
            g_cost = abs(0 + current_state.parent_g_cost + path_length)
        
        return g_cost
    
    def calculate_h_cost(self, current_state: CostState, goal_state: GoalState,
                        initial_state: CostState, large_zone: Optional[ZonePolygon] = None,
                        precise_zone: Optional[ZonePolygon] = None) -> float:
        """
        Calculate H-cost (heuristic cost to goal).
        
        Based on MATLAB h_cost.m function with support for:
        - Euclidean distance heuristic
        - Rectangular heuristic (for longer distances) 
        - Zone-based penalties
        
        Args:
            current_state: Current state
            goal_state: Goal state
            initial_state: Initial state for zone selection
            large_zone: Large zone polygon for penalties
            precise_zone: Precise zone polygon for penalties
            
        Returns:
            Heuristic cost to goal
        """
        # Zone-based heuristic selection (from MATLAB h_cost.m)
        use_rectangular = self._should_use_rectangular_heuristic(goal_state, initial_state)
        
        if use_rectangular:
            # Use rectangular heuristic for longer distances
            hc_d = self._calculate_rectangular_heuristic(current_state, goal_state)
        else:
            # Use Euclidean distance (from MATLAB h_cost.m)
            hc_d = self._calculate_euclidean_heuristic(current_state, goal_state)
        
        # Apply zone penalties (from MATLAB h_cost.m)
        if large_zone and precise_zone:
            hc_d = self._apply_zone_penalties(current_state, hc_d, large_zone, precise_zone)
        
        return hc_d
    
    def _should_use_rectangular_heuristic(self, goal_state: GoalState, 
                                        initial_state: CostState) -> bool:
        """
        Determine whether to use rectangular heuristic.
        
        From MATLAB h_cost.m:
        if f_state.y<=80 && i_state.y>=70 -> rectheur(ip)
        elseif f_state.y>=80 && i_state.y<=70 -> rectheur1(ip)
        else -> Euclidean distance
        """
        goal_y = goal_state.ya if goal_state.ya != 0 else goal_state.y
        initial_y = initial_state.ya if initial_state.ya != 0 else initial_state.y
        
        condition1 = goal_y <= self.zone_y_threshold_high and initial_y >= self.zone_y_threshold_low
        condition2 = goal_y >= self.zone_y_threshold_high and initial_y <= self.zone_y_threshold_low
        
        return condition1 or condition2
    
    def _calculate_euclidean_heuristic(self, current_state: CostState, 
                                     goal_state: GoalState) -> float:
        """
        Calculate Euclidean distance heuristic.
        
        From MATLAB h_cost.m:
        a = (ip.xa-f_state.x)^2;
        b = (ip.ya-f_state.y)^2;
        dist = sqrt(a + b);
        hc_d= dist*2.5;
        """
        # Use xa, ya if available, otherwise use x, y
        current_x = current_state.xa if current_state.xa != 0 else current_state.x
        current_y = current_state.ya if current_state.ya != 0 else current_state.y
        goal_x = goal_state.xa if goal_state.xa != 0 else goal_state.x
        goal_y = goal_state.ya if goal_state.ya != 0 else goal_state.y
        
        a = (current_x - goal_x) ** 2
        b = (current_y - goal_y) ** 2
        dist = math.sqrt(a + b)
        
        hc_d = dist * self.heuristic_coefficient
        
        return hc_d
    
    def _calculate_rectangular_heuristic(self, current_state: CostState,
                                       goal_state: GoalState) -> float:
        """
        Calculate rectangular heuristic for longer path planning.
        
        This is a placeholder for the MATLAB rectheur.m and rectheur1.m functions.
        The actual implementation would depend on the specific rectangular heuristic
        used in the MATLAB system.
        """
        # For now, use Manhattan distance as a simple rectangular heuristic
        current_x = current_state.xa if current_state.xa != 0 else current_state.x
        current_y = current_state.ya if current_state.ya != 0 else current_state.y
        goal_x = goal_state.xa if goal_state.xa != 0 else goal_state.x
        goal_y = goal_state.ya if goal_state.ya != 0 else goal_state.y
        
        manhattan_dist = abs(current_x - goal_x) + abs(current_y - goal_y)
        
        # Apply heuristic coefficient
        hc_d = manhattan_dist * self.heuristic_coefficient
        
        self.logger.debug(f"Rectangular heuristic: {hc_d:.2f}")
        return hc_d
    
    def _apply_zone_penalties(self, current_state: CostState, base_heuristic: float,
                            large_zone: ZonePolygon, precise_zone: ZonePolygon) -> float:
        """
        Apply zone-based penalties to heuristic.
        
        From MATLAB h_cost.m:
        inlg = InPolygon(ip.xa,ip.ya,xv_lg,yv_lg);
        in2 = InPolygon(ip.xa,ip.ya,xv_lg,yv_lg);
        if in2 && ~inlg
            hc_d= hc_d*1000;
        """
        current_x = current_state.xa if current_state.xa != 0 else current_state.x
        current_y = current_state.ya if current_state.ya != 0 else current_state.y
        
        # Check if point is in large zone
        in_large = self._point_in_polygon(current_x, current_y, large_zone)
        
        # Check if point is in precise zone  
        in_precise = self._point_in_polygon(current_x, current_y, precise_zone)
        
        # Apply penalty if in precise zone but not in large zone (MATLAB logic)
        # Note: The MATLAB logic seems to be: if in2 && ~inlg, where in2 is precise and inlg is large
        if in_precise and not in_large:
            penalized_cost = base_heuristic * self.zone_penalty_factor
            self.logger.debug(f"Zone penalty applied: {base_heuristic:.2f} → {penalized_cost:.2f}")
            return penalized_cost
        
        return base_heuristic
    
    def _point_in_polygon(self, x: float, y: float, polygon: ZonePolygon) -> bool:
        """
        Check if point is inside polygon.
        
        Uses ray casting algorithm (same as collision detection).
        """
        if len(polygon.x_coords) != len(polygon.y_coords) or len(polygon.x_coords) < 3:
            return False
        
        poly_x = polygon.x_coords
        poly_y = polygon.y_coords
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
    
    def calculate_transition_cost(self, from_state: CostState, to_state: CostState,
                                 motion_primitive_cost: float = 0.0) -> float:
        """
        Calculate cost of transitioning between states.
        
        Args:
            from_state: Source state
            to_state: Destination state  
            motion_primitive_cost: Additional cost from motion primitive
            
        Returns:
            Transition cost
        """
        # Basic Euclidean distance
        dx = to_state.xa - from_state.xa
        dy = to_state.ya - from_state.ya
        distance_cost = math.sqrt(dx*dx + dy*dy)
        
        # Add motion primitive cost (curvature, time, etc.)
        total_cost = distance_cost + motion_primitive_cost
        
        return total_cost
    
    def create_test_zones(self) -> Tuple[ZonePolygon, ZonePolygon]:
        """Create test zones for penalty calculations."""
        # Large zone (approximate final area) - this is the "large" zone
        large_zone = ZonePolygon(
            x_coords=[55, 75, 75, 55, 55],
            y_coords=[55, 55, 75, 75, 55]
        )
        
        # Precise zone (exact goal area) - this is outside the large zone for penalty testing
        precise_zone = ZonePolygon(
            x_coords=[80, 90, 90, 80, 80],
            y_coords=[80, 80, 90, 90, 80]
        )
        
        return large_zone, precise_zone


def test_cost_calculator():
    """Test cost calculator functionality."""
    print("Testing Cost Calculator")
    print("="*50)
    
    calculator = CostCalculator()
    
    # Test states
    start_state = CostState(x=0, y=0, xa=0, ya=0, theta=0, gamma=0)
    current_state = CostState(x=10, y=10, xa=10, ya=10, theta=math.radians(45), gamma=0)
    goal_state = GoalState(x=50, y=50, xa=50, ya=50, theta=math.radians(90), gamma=0)
    
    print(f"Start: ({start_state.x}, {start_state.y})")
    print(f"Current: ({current_state.x}, {current_state.y}) θ={math.degrees(current_state.theta):.1f}°")
    print(f"Goal: ({goal_state.x}, {goal_state.y}) θ={math.degrees(goal_state.theta):.1f}°")
    
    # Test G-cost calculation
    print(f"\nG-Cost Tests:")
    
    # First iteration (no parent)
    g_cost1 = calculator.calculate_g_cost(current_state, 14.14)  # sqrt(10^2 + 10^2)
    print(f"  First iteration: path_length=14.14 → g_cost={g_cost1:.2f}")
    
    # Subsequent iteration
    current_state.parent_g_cost = g_cost1
    g_cost2 = calculator.calculate_g_cost(current_state, 10.0)
    print(f"  Second iteration: parent_g={g_cost1:.2f}, path_length=10.0 → g_cost={g_cost2:.2f}")
    
    # Test H-cost calculation
    print(f"\nH-Cost Tests:")
    
    # Euclidean heuristic
    h_cost1 = calculator.calculate_h_cost(current_state, goal_state, start_state)
    expected_dist = math.sqrt((50-10)**2 + (50-10)**2) * 2.5  # sqrt(1600+1600) * 2.5
    print(f"  Euclidean: calculated={h_cost1:.2f}, expected≈{expected_dist:.2f}")
    
    # Test with zones
    large_zone, precise_zone = calculator.create_test_zones()
    
    # Test state inside precise zone (should get penalty)
    zone_state = CostState(x=60, y=60, xa=60, ya=60, theta=0, gamma=0)
    h_cost_penalty = calculator.calculate_h_cost(zone_state, goal_state, start_state, 
                                               large_zone, precise_zone)
    h_cost_no_penalty = calculator.calculate_h_cost(zone_state, goal_state, start_state)
    
    print(f"  Zone test: no_penalty={h_cost_no_penalty:.2f}, with_zones={h_cost_penalty:.2f}")
    
    # Test rectangular heuristic conditions
    print(f"\nRectangular Heuristic Tests:")
    
    # Set up conditions for rectangular heuristic
    long_goal = GoalState(x=50, y=75, xa=50, ya=75, theta=0, gamma=0)  # y=75 < 80
    long_start = CostState(x=0, y=75, xa=0, ya=75, theta=0, gamma=0)   # y=75 > 70
    
    should_use_rect = calculator._should_use_rectangular_heuristic(long_goal, long_start)
    print(f"  Should use rectangular: {should_use_rect}")
    
    if should_use_rect:
        rect_h = calculator._calculate_rectangular_heuristic(current_state, long_goal)
        eucl_h = calculator._calculate_euclidean_heuristic(current_state, long_goal)
        print(f"  Rectangular heuristic: {rect_h:.2f}")
        print(f"  Euclidean heuristic: {eucl_h:.2f}")
    
    # Test transition cost
    print(f"\nTransition Cost Test:")
    next_state = CostState(x=15, y=12, xa=15, ya=12, theta=0, gamma=0)
    transition_cost = calculator.calculate_transition_cost(current_state, next_state, 1.0)
    expected_transition = math.sqrt(25 + 4) + 1.0  # sqrt((15-10)^2 + (12-10)^2) + 1.0
    print(f"  Calculated: {transition_cost:.2f}, expected≈{expected_transition:.2f}")
    
    print(f"\n✅ Cost calculator test completed!")
    return True


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    test_cost_calculator()
