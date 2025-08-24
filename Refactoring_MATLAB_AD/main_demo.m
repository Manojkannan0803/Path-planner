%% MAIN_DEMO - Demonstration of Refactored Path Planning System
% This script replaces the original monolithic Pathplanning_Astar.m
% and shows how to use the new layered architecture

function main_demo()
    fprintf('=== REFACTORED PATH PLANNING SYSTEM DEMO ===\n');
    fprintf('Layered Architecture with Modular Algorithm Support\n\n');
    
    %% 1. BASIC USAGE - Single Path Planning
    fprintf('1. BASIC USAGE - Single Path Planning\n');
    fprintf('------------------------------------\n');
    
    % Initialize planning session
    session = PlanningSession('scenarios/DPDScenario.json');
    
    % Define start and goal states (same as original)
    start_state = struct('x', 75, 'y', 45, 'theta', 180, 'gamma', 0);
    goal_state = struct('x', 60, 'y', 64, 'theta', 270, 'gamma', 0);
    
    % Execute planning with default A* algorithm
    [path, success, stats] = session.execute_single_planning(start_state, goal_state);
    
    %% 2. ALGORITHM MODULARITY - Dynamic Algorithm Switching
    fprintf('\n2. ALGORITHM MODULARITY - Dynamic Algorithm Switching\n');
    fprintf('-----------------------------------------------------\n');
    
    % Switch to different algorithms dynamically
    algorithms_to_test = {'astar', 'dijkstra'};
    
    for i = 1:length(algorithms_to_test)
        algorithm_name = algorithms_to_test{i};
        fprintf('\nTesting algorithm: %s\n', algorithm_name);
        
        % Change algorithm at runtime
        session.set_algorithm(algorithm_name);
        
        % Execute planning with new algorithm
        [path, success, stats] = session.execute_single_planning(start_state, goal_state);
    end
    
    %% 3. ALGORITHM COMPARISON
    fprintf('\n3. ALGORITHM COMPARISON\n');
    fprintf('----------------------\n');
    
    % Compare multiple algorithms on same problem
    comparison_algorithms = {'astar', 'dijkstra'};
    session.compare_algorithms(comparison_algorithms, start_state, goal_state);
    
    %% 4. BATCH PROCESSING
    fprintf('\n4. BATCH PROCESSING\n');
    fprintf('------------------\n');
    
    % Define multiple test cases
    test_cases = create_test_cases();
    
    % Execute batch planning
    batch_results = session.execute_batch_planning(test_cases);
    
    %% 5. CONFIGURATION FLEXIBILITY
    fprintf('\n5. CONFIGURATION FLEXIBILITY\n');
    fprintf('----------------------------\n');
    
    % Demonstrate configuration changes
    demonstrate_configuration_flexibility(session, start_state, goal_state);
    
    %% 6. PERFORMANCE BENCHMARKING
    fprintf('\n6. PERFORMANCE BENCHMARKING\n');
    fprintf('---------------------------\n');
    
    % Run performance benchmark
    benchmark_test_suite = create_benchmark_suite();
    session.benchmark_performance(benchmark_test_suite);
    
    %% 7. SESSION MANAGEMENT
    fprintf('\n7. SESSION MANAGEMENT\n');
    fprintf('--------------------\n');
    
    % Get session summary
    summary = session.get_session_summary();
    display_session_summary(summary);
    
    % Save session for later analysis
    session.save_session('demo_session_results.mat');
    
    fprintf('\n=== DEMO COMPLETED ===\n');
    
    %% 8. ARCHITECTURE COMPARISON
    fprintf('\n8. ARCHITECTURE COMPARISON\n');
    fprintf('-------------------------\n');
    compare_with_original_architecture();
end

function test_cases = create_test_cases()
    % Create multiple test cases for batch processing
    test_cases = [
        struct('start_state', struct('x', 75, 'y', 45, 'theta', 180, 'gamma', 0), ...
               'goal_state', struct('x', 60, 'y', 64, 'theta', 270, 'gamma', 0)), ...
        struct('start_state', struct('x', 50, 'y', 30, 'theta', 0, 'gamma', 0), ...
               'goal_state', struct('x', 80, 'y', 70, 'theta', 90, 'gamma', 0)), ...
        struct('start_state', struct('x', 100, 'y', 50, 'theta', 270, 'gamma', 0), ...
               'goal_state', struct('x', 40, 'y', 80, 'theta', 180, 'gamma', 0))
    ];
end

function benchmark_suite = create_benchmark_suite()
    % Create comprehensive benchmark test suite
    benchmark_suite = [
        struct('start_state', struct('x', 75, 'y', 45, 'theta', 180, 'gamma', 0), ...
               'goal_state', struct('x', 60, 'y', 64, 'theta', 270, 'gamma', 0)), ...
        struct('start_state', struct('x', 30, 'y', 30, 'theta', 45, 'gamma', 0), ...
               'goal_state', struct('x', 90, 'y', 90, 'theta', 225, 'gamma', 0))
    ];
end

function demonstrate_configuration_flexibility(session, start_state, goal_state)
    % Demonstrate runtime configuration changes
    
    % 1. Change cost function weights
    fprintf('Changing cost function weights...\n');
    new_cost_config = struct('cost_weights', struct(...
        'distance', 1.0, 'direction_change', 0.5, 'reverse_penalty', 2.0));
    session.planning_engine.set_cost_function(new_cost_config);
    
    [path1, success1, stats1] = session.execute_single_planning(start_state, goal_state);
    
    % 2. Change heuristic function
    fprintf('Changing heuristic function...\n');
    new_heuristic_config = struct('heuristic_type', 'manhattan', 'admissible', true);
    session.planning_engine.set_heuristic_function(new_heuristic_config);
    
    [path2, success2, stats2] = session.execute_single_planning(start_state, goal_state);
    
    % 3. Change scenario
    fprintf('Attempting to change scenario...\n');
    % session.change_scenario('WarehouseScenario');  % Would work if file exists
end

function display_session_summary(summary)
    % Display session summary
    fprintf('Session Summary:\n');
    % fprintf('Current Algorithm: %s\n', summary.current_algorithm.name);
    % fprintf('Current Scenario: %s\n', summary.current_scenario.name);
    fprintf('Summary object created.\n');
end

function compare_with_original_architecture()
    % Compare original vs refactored architecture
    fprintf('\n=== ARCHITECTURE COMPARISON ===\n');
    
    fprintf('\nORIGINAL MONOLITHIC APPROACH:\n');
    fprintf('- Single file: Pathplanning_Astar.m (364 lines)\n');
    fprintf('- Everything mixed together\n');
    fprintf('- Global variables everywhere\n');
    fprintf('- Hard to test, extend, or maintain\n');
    fprintf('- Fixed algorithm (only A*)\n');
    fprintf('- Configuration hardcoded\n');
    
    fprintf('\nREFACTORED LAYERED APPROACH:\n');
    fprintf('- 5 Clear architectural layers\n');
    fprintf('- Modular algorithm support (A*, Dijkstra, RRT, etc.)\n');
    fprintf('- Runtime algorithm switching\n');
    fprintf('- Configuration-driven behavior\n');
    fprintf('- Comprehensive testing support\n');
    fprintf('- Clean separation of concerns\n');
    
    fprintf('\nKEY BENEFITS ACHIEVED:\n');
    fprintf('✓ MODULARITY: Each component has single responsibility\n');
    fprintf('✓ EXTENSIBILITY: Easy to add new algorithms\n');
    fprintf('✓ TESTABILITY: Layer-by-layer testing possible\n');
    fprintf('✓ MAINTAINABILITY: Changes isolated to specific layers\n');
    fprintf('✓ REUSABILITY: Components can be shared across projects\n');
    fprintf('✓ CONFIGURABILITY: Runtime behavior modification\n');
    
    fprintf('\nARCHITECTURAL LAYERS:\n');
    fprintf('Layer 5: Applications (PlanningSession, scenarios)\n');
    fprintf('Layer 4: Algorithms (Modular algorithm engine)\n');
    fprintf('Layer 3: Services (Cost, heuristic, collision detection)\n');
    fprintf('Layer 2: Framework (State management, data structures)\n');
    fprintf('Layer 1: Foundation (Data loading, math utilities)\n');
    
    fprintf('\n================================\n');
end

%% Entry Point
if isempty(dbstack())
    main_demo();
end
