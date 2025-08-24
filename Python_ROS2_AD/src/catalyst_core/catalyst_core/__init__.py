# CATALYST Core Package - Platform Layer
"""
This package implements the Platform Layer of the XX (work)-inspired architecture.

Architecture Layers:
1. Platform Layer (catalyst_core) - THIS PACKAGE
   - Plugin management and orchestration
   - Configuration management  
   - Simulation execution control
   - Performance monitoring

2. Interface Layer (catalyst_interfaces)
   - Message definitions
   - Service definitions
   - Data interceptors

3. Algorithm Layer (catalyst_algorithms) 
   - Path planning plugins (A*, RRT*, etc.)
   - Cost calculators
   - Collision detection

4. Model Layer (catalyst_models)
   - Vehicle dynamics models
   - Environment models
   - Motion primitive libraries

This follows XX (work)'s modular plugin architecture for autonomous vehicle systems.
"""

__version__ = "1.0.0"
__author__ = "CATALYST Team"
