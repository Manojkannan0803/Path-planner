#!/usr/bin/env python3
"""
DPD Scenario Environment Model - Converted from MATLAB DPDscenario.m

This module defines the exact DPD (Distribution Center) scenario environment
with all static obstacles as defined in the original MATLAB system.

Original MATLAB file: DPDscenario.m
Author: Manojpriyadharson Kannan (Student number: 638628)
Converted by: CATALYST Team

Environment specifications:
- Distribution center: 328m × 200m
- 6 static obstacles with complex polygonal shapes
- Exact coordinates from MATLAB implementation
"""

import math
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass
import logging

from .collision_detection import Environment, ObstaclePolygon


@dataclass
class DPDEnvironmentConfig:
    """Configuration for DPD scenario environment."""
    length_dc: float = 328.0  # Distribution center length [m]
    width_dc: float = 200.0   # Distribution center width [m]
    scale: float = 1.0        # Grid scale [m]
    obstacle_count: int = 6   # Number of static obstacles


class DPDEnvironmentBuilder:
    """
    Builds the exact DPD scenario environment from MATLAB DPDscenario.m
    
    Creates the Distribution Center environment with all static obstacles
    positioned exactly as in the original MATLAB implementation.
    """
    
    def __init__(self):
        """Initialize DPD environment builder."""
        self.logger = logging.getLogger(__name__)
        self.config = DPDEnvironmentConfig()
        self.logger.info("DPD Environment Builder initialized")
    
    def build_dpd_environment(self) -> Environment:
        """
        Build complete DPD environment with all obstacles.
        
        Returns:
            Environment object with DPD scenario setup
        """
        environment = Environment()
        environment.length_dc = self.config.length_dc
        environment.width_dc = self.config.width_dc
        
        # Build all obstacles from MATLAB data
        obstacles = self._create_all_obstacles()
        environment.obstacles = obstacles
        
        self.logger.info(f"Built DPD environment with {len(obstacles)} obstacles")
        return environment
    
    def _create_all_obstacles(self) -> List[ObstaclePolygon]:
        """Create all 6 obstacles from MATLAB DPDscenario.m"""
        obstacles = []
        
        # Obstacle 1 - Main complex building (largest)
        obstacle1 = self._create_obstacle_1()
        obstacles.append(obstacle1)
        
        # Obstacle 2 - Secondary building
        obstacle2 = self._create_obstacle_2()
        obstacles.append(obstacle2)
        
        # Obstacle 3 - Third building
        obstacle3 = self._create_obstacle_3()
        obstacles.append(obstacle3)
        
        # Obstacle 4 - Fourth building
        obstacle4 = self._create_obstacle_4()
        obstacles.append(obstacle4)
        
        # Obstacle 5 - Fifth building
        obstacle5 = self._create_obstacle_5()
        obstacles.append(obstacle5)
        
        # Obstacle 6 - Sixth building
        obstacle6 = self._create_obstacle_6()
        obstacles.append(obstacle6)
        
        return obstacles
    
    def _create_obstacle_1(self) -> ObstaclePolygon:
        """
        Create Obstacle 1 - Main complex building.
        
        From MATLAB:
        obsx(1,:) = [28,58.6,58.6,61.4,61.4,108,108,200.5,200.5,204.25,204.25,228,228,285,285,228,228,111.4,111.4,108.6,108.6,108,108,91.4,91.4,88.6,88.6,28];
        obsy(1,:) = [53,53,69.5,69.5,53,53,61,61,77.5,77.5,61,61,57,57,140,140,136,136,119.5,119.5,136,136,144,144,127.5,127.5,144,144];
        """
        x_coords = [
            28, 58.6, 58.6, 61.4, 61.4, 108, 108, 200.5, 200.5, 204.25, 
            204.25, 228, 228, 285, 285, 228, 228, 111.4, 111.4, 108.6, 
            108.6, 108, 108, 91.4, 91.4, 88.6, 88.6, 28
        ]
        
        y_coords = [
            53, 53, 69.5, 69.5, 53, 53, 61, 61, 77.5, 77.5, 
            61, 61, 57, 57, 140, 140, 136, 136, 119.5, 119.5, 
            136, 136, 144, 144, 127.5, 127.5, 144, 144
        ]
        
        return ObstaclePolygon(id=1, x_coords=x_coords, y_coords=y_coords)
    
    def _create_obstacle_2(self) -> ObstaclePolygon:
        """
        Create Obstacle 2 - Secondary building.
        
        From MATLAB:
        assind1 = [72,240,255,122,110,105,115,87,72]
        temp1(assind1) = [72,240,255,122,110,105,115,87,72]
        assind2 = [13,13,33,33,21.33,18.94,33,33,13]  
        temp1(assind2) = [13,13,33,33,21.33,23.3099,33,33,13]
        """
        # Extract non-zero coordinates
        x_coords = [72, 240, 255, 122, 110, 105, 115, 87, 72]
        y_coords = [13, 13, 33, 33, 21.33, 23.3099, 33, 33, 13]
        
        return ObstaclePolygon(id=2, x_coords=x_coords, y_coords=y_coords)
    
    def _create_obstacle_3(self) -> ObstaclePolygon:
        """
        Create Obstacle 3 - Third building.
        
        From MATLAB:
        assind3 = [27,243,260,44]
        temp1(assind3) = [27,243,260,44]
        assind4 = [168,168,178,178]
        temp1(assind4) = [168,168,178,178]
        """
        x_coords = [27, 243, 260, 44, 27]  # Close polygon
        y_coords = [168, 168, 178, 178, 168]  # Close polygon
        
        return ObstaclePolygon(id=3, x_coords=x_coords, y_coords=y_coords)
    
    def _create_obstacle_4(self) -> ObstaclePolygon:
        """
        Create Obstacle 4 - Fourth building.
        
        From MATLAB:
        assind5 = [100,150,150,100]
        temp1(assind5) = [100,150,150,100]
        assind6 = [195,195,192.5,192.5] -> [195,195,190,190]
        temp1(assind6) = [195,195,190,190]
        """
        x_coords = [100, 150, 150, 100, 100]  # Close polygon
        y_coords = [195, 195, 190, 190, 195]  # Close polygon (corrected values)
        
        return ObstaclePolygon(id=4, x_coords=x_coords, y_coords=y_coords)
    
    def _create_obstacle_5(self) -> ObstaclePolygon:
        """
        Create Obstacle 5 - Fifth building.
        
        From MATLAB:
        assind7 = [162.6,180,180,162.6]
        temp1(assind7) = [162.6,180,180,162.6]
        assind8 = [195,195,192.5,192.5] -> [195,195,190,190]
        temp1(assind8) = [195,195,190,190]
        """
        x_coords = [162.6, 180, 180, 162.6, 162.6]  # Close polygon
        y_coords = [195, 195, 190, 190, 195]  # Close polygon
        
        return ObstaclePolygon(id=5, x_coords=x_coords, y_coords=y_coords)
    
    def _create_obstacle_6(self) -> ObstaclePolygon:
        """
        Create Obstacle 6 - Sixth building.
        
        From MATLAB:
        assind9 = [28,50,50,28]
        temp1(assind9) = [28,50,50,28]
        assind10 = [53,53,51,51]
        temp1(assind10) = [53,53,51,51]
        """
        x_coords = [28, 50, 50, 28, 28]  # Close polygon
        y_coords = [53, 53, 51, 51, 53]  # Close polygon
        
        return ObstaclePolygon(id=6, x_coords=x_coords, y_coords=y_coords)
    
    def get_obstacle_summary(self) -> Dict:
        """Get summary of all obstacles."""
        environment = self.build_dpd_environment()
        
        summary = {
            'environment_size': f"{environment.length_dc}m × {environment.width_dc}m",
            'obstacle_count': len(environment.obstacles),
            'obstacles': []
        }
        
        for obs in environment.obstacles:
            obs_info = {
                'id': obs.id,
                'vertices': len(obs.x_coords),
                'area': self._calculate_polygon_area(obs.x_coords, obs.y_coords),
                'bounds': {
                    'x_min': min(obs.x_coords),
                    'x_max': max(obs.x_coords),
                    'y_min': min(obs.y_coords),
                    'y_max': max(obs.y_coords)
                }
            }
            summary['obstacles'].append(obs_info)
        
        return summary
    
    def _calculate_polygon_area(self, x_coords: List[float], y_coords: List[float]) -> float:
        """Calculate polygon area using shoelace formula."""
        if len(x_coords) < 3:
            return 0.0
        
        n = len(x_coords)
        area = 0.0
        
        for i in range(n):
            j = (i + 1) % n
            area += x_coords[i] * y_coords[j]
            area -= x_coords[j] * y_coords[i]
        
        return abs(area) / 2.0
    
    def validate_environment(self) -> Tuple[bool, List[str]]:
        """
        Validate the DPD environment for correctness.
        
        Returns:
            (is_valid, error_messages)
        """
        errors = []
        environment = self.build_dpd_environment()
        
        # Check environment bounds
        if environment.length_dc <= 0 or environment.width_dc <= 0:
            errors.append("Environment dimensions must be positive")
        
        # Check obstacles
        for obs in environment.obstacles:
            if len(obs.x_coords) != len(obs.y_coords):
                errors.append(f"Obstacle {obs.id}: mismatched coordinate arrays")
            
            if len(obs.x_coords) < 3:
                errors.append(f"Obstacle {obs.id}: insufficient vertices for polygon")
            
            # Check if obstacle is within environment bounds
            if (any(x < 0 or x > environment.length_dc for x in obs.x_coords) or
                any(y < 0 or y > environment.width_dc for y in obs.y_coords)):
                errors.append(f"Obstacle {obs.id}: vertices outside environment bounds")
        
        return len(errors) == 0, errors
    
    def create_test_points(self) -> Dict[str, List[Tuple[float, float]]]:
        """
        Create test points for collision detection validation.
        
        Returns:
            Dictionary with test point categories
        """
        return {
            'free_space': [
                (10.0, 10.0),    # Bottom left free area
                (300.0, 50.0),   # Right side free area
                (150.0, 180.0),  # Top center free area
                (50.0, 100.0),   # Left center free area
            ],
            'inside_obstacles': [
                (60.0, 60.0),    # Inside obstacle 1
                (150.0, 25.0),   # Inside obstacle 2
                (40.0, 173.0),   # Inside obstacle 3
                (125.0, 192.0),  # Inside obstacle 4
                (171.0, 192.0),  # Inside obstacle 5
                (35.0, 52.0),    # Inside obstacle 6
            ],
            'boundary_test': [
                (0.0, 0.0),      # Environment corner
                (328.0, 200.0),  # Environment corner
                (400.0, 100.0),  # Outside environment
                (-10.0, 50.0),   # Outside environment
            ]
        }


def test_dpd_environment():
    """Test DPD environment creation and validation."""
    print("Testing DPD Environment Builder")
    print("="*50)
    
    builder = DPDEnvironmentBuilder()
    
    # Build environment
    environment = builder.build_dpd_environment()
    print(f"Environment: {environment.length_dc}m × {environment.width_dc}m")
    print(f"Obstacles: {len(environment.obstacles)}")
    
    # Validate environment
    is_valid, errors = builder.validate_environment()
    print(f"Validation: {'✅ PASSED' if is_valid else '❌ FAILED'}")
    if errors:
        for error in errors:
            print(f"  - {error}")
    
    # Get obstacle summary
    summary = builder.get_obstacle_summary()
    print(f"\nObstacle Summary:")
    for i, obs_info in enumerate(summary['obstacles']):
        print(f"  Obstacle {obs_info['id']}: {obs_info['vertices']} vertices, "
              f"area={obs_info['area']:.1f}m², "
              f"bounds=({obs_info['bounds']['x_min']:.1f},{obs_info['bounds']['y_min']:.1f})"
              f"-({obs_info['bounds']['x_max']:.1f},{obs_info['bounds']['y_max']:.1f})")
    
    # Test collision detection with sample points
    from .collision_detection import CollisionDetector
    detector = CollisionDetector()
    
    test_points = builder.create_test_points()
    print(f"\nCollision Detection Test:")
    
    for category, points in test_points.items():
        print(f"  {category.replace('_', ' ').title()}:")
        for x, y in points:
            collision_free = detector.check_collision(x, y, 0.0, 0.0, environment)
            result = "FREE" if collision_free else "COLLISION"
            print(f"    ({x:6.1f}, {y:6.1f}): {result}")
    
    print(f"\n✅ DPD Environment test completed!")
    return is_valid


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    test_dpd_environment()
