# +Planner Package Regression Test Suite

## Overview

This document describes the comprehensive regression test suite for the +Planner package used in the Autonomous Valet Parking System. The test suite ensures code quality, prevents regressions, and validates functionality across all package components.

## Test Structure

```
+Planner/
├── +Testing/
│   ├── VehicleParamsTest.m           # Original VehicleParams unit tests
│   ├── PlannerTestSuite.m            # Comprehensive regression tests
│   ├── TestDataManager.m             # Test data management utilities
│   ├── runPlannerRegressionTests.m   # Main test runner function
│   └── TestData/                     # Test configuration files
└── quickRegressionTest.m             # Quick test runner script
```

## Test Categories

### 1. VehicleParams Tests (`VehicleParamsTest.m`)
- **Purpose**: Unit tests for vehicle configuration management
- **Coverage**: 13 individual test methods
- **Focus Areas**:
  - Configuration loading and validation
  - Vehicle dimension properties
  - Steering characteristics
  - Performance limits and safety margins
  - Enhanced API methods (planning, environment, simulation parameters)
  - YAML parser functionality
  - Error handling and edge cases

### 2. Regression Test Suite (`PlannerTestSuite.m`)
- **Purpose**: Comprehensive regression testing framework
- **Test Tags**: 
  - `VehicleParams`: Core vehicle parameter functionality
  - `Core`: Essential functionality tests
  - `Regression`: Regression prevention tests
  - `ErrorHandling`: Error and edge case testing
  - `Performance`: Performance benchmarking
  - `Integration`: Cross-component integration (future)
  - `Future`: Placeholder tests for upcoming features

### 3. Performance Tests
- **VehicleParams instantiation benchmarking**
- **Property access performance validation**
- **Memory usage monitoring**
- **Regression detection for performance critical paths**

## Running Tests

### Quick Regression Test
```matlab
% Run basic functionality tests
quickRegressionTest
```

### Full Regression Test Suite
```matlab
% Run all tests with default settings
results = Planner.Testing.runPlannerRegressionTests()

% Run with detailed output
results = Planner.Testing.runPlannerRegressionTests('Verbosity', 3)

% Run specific test category
results = Planner.Testing.runPlannerRegressionTests('Tag', 'VehicleParams')

% Generate HTML report
results = Planner.Testing.runPlannerRegressionTests('GenerateReport', true)

% Run with coverage analysis
results = Planner.Testing.runPlannerRegressionTests('GenerateReport', true, 'Verbosity', 3)
```

### Individual Test Classes
```matlab
% Run VehicleParams tests only
runtests('Planner.Testing.VehicleParamsTest')

% Run regression suite only  
runtests('Planner.Testing.PlannerTestSuite')

% Run specific test method
runtests('Planner.Testing.VehicleParamsTest', 'ProcedureName', 'testVehicleDimensions')
```

## Test Data Management

The `TestDataManager` class provides utilities for:
- **Creating temporary configuration files**
- **Generating valid/invalid test configurations**
- **Managing test data lifecycle**
- **Cleanup operations**

```matlab
% Get test data manager instance
manager = Planner.Testing.TestDataManager.getInstance();

% Create test configuration
filename = manager.createTempConfigFile(content, 'test_config');

% Cleanup temporary files
manager.cleanupTempFiles();
```

## Test Configuration

### Valid Test Configuration
The test suite uses standardized vehicle configurations:
- **Length**: 4.07m, **Width**: 1.75m, **Wheelbase**: 2.55m
- **Max Steering**: 44°, **Min Turning Radius**: 2.64m (physics-consistent)
- **Complete YAML structure** with all required sections

### Error Testing Scenarios
- **Missing configuration files**
- **Invalid vehicle dimensions** (negative values, inconsistent parameters)
- **Malformed YAML structure**
- **Edge case values** (minimal valid configurations)

## Regression Prevention

### Current Coverage
- ✅ **VehicleParams Configuration System**
- ✅ **YAML Parser with Nested Structures**
- ✅ **Enhanced API Methods**
- ✅ **Physics Consistency Validation**
- ✅ **Error Handling and Edge Cases**

### Future Expansion Points
- 🔲 **Bus Structure Definitions** (when implemented)
- 🔲 **Motion Primitive Infrastructure** (when implemented)
- 🔲 **Simulink Integration Tests** (when implemented)
- 🔲 **End-to-End Path Planning** (integration phase)

## Continuous Integration

### Automated Testing Workflow
1. **Pre-commit**: Run quick regression tests
2. **Pull Request**: Full regression suite with coverage
3. **Release**: Performance benchmarks and integration tests
4. **Nightly**: Extended test suite with stress testing

### Success Criteria
- **All core tests must pass** (100% pass rate)
- **Performance tests within tolerances** (no regression > 10%)
- **Code coverage targets** (aim for >90% line coverage)
- **No new warnings or errors**

## Debugging Failed Tests

### Common Issues
1. **Configuration File Path Issues**
   - Ensure working directory is correct
   - Check `+Planner/+Config/vehicle_config.yaml` exists

2. **MATLAB Path Issues**  
   - Verify `+Planner` package is on MATLAB path
   - Check package structure is intact

3. **Test Environment Issues**
   - Clear workspace: `clear all; clc`
   - Reset test environment: `TestDataManager.cleanupAllTestData()`

### Debugging Commands
```matlab
% Check VehicleParams instantiation
vp = Planner.Src.VehicleParams()

% Verify configuration loading
vp.isLoaded()
vp.validateConfig()

% Test YAML parser directly
config = vp.parseYAML('+Planner/+Config/vehicle_config.yaml')

% Check test environment
manager = Planner.Testing.TestDataManager.getInstance()
manager.ensureTestDataDir()
```

## Performance Benchmarks

### Target Performance
- **VehicleParams instantiation**: < 1.0 seconds
- **Property access**: < 1ms per access
- **Configuration validation**: < 100ms
- **YAML parsing**: < 500ms for typical config

### Performance Regression Detection
Tests automatically fail if performance degrades beyond acceptable thresholds, ensuring the package maintains responsiveness for real-time applications.

## Report Generation

### HTML Test Reports
```matlab
% Generate comprehensive HTML report
results = Planner.Testing.runPlannerRegressionTests(...
    'GenerateReport', true, ...
    'Verbosity', 3, ...
    'OutputDir', 'TestReports')
```

Reports include:
- **Test execution summary**
- **Individual test results and timing**
- **Code coverage analysis** (if available)
- **Performance benchmark results**
- **Failure analysis and stack traces**

## Maintenance

### Adding New Tests
1. **For VehicleParams**: Add methods to `VehicleParamsTest.m`
2. **For new components**: Create new test class in `+Testing/`
3. **Update test runner**: Add new test suite to `runPlannerRegressionTests.m`
4. **Tag appropriately**: Use consistent test tags for filtering

### Test Data Updates
When configuration structure changes:
1. **Update `TestDataManager`** with new valid configurations
2. **Update existing test expectations**
3. **Add new error test scenarios** if needed
4. **Verify backward compatibility** where appropriate

---

*Last Updated: 2025-09-23*  
*Package: +Planner (Autonomous Valet Parking System)*