function results = runPlannerRegressionTests(varargin)
    % runPlannerRegressionTests - Execute comprehensive regression test suite
    %
    % This function runs all regression tests for the +Planner package,
    % including individual component tests and integration tests.
    %
    % Usage:
    %   results = runPlannerRegressionTests()
    %   results = runPlannerRegressionTests('Verbosity', 3)
    %   results = runPlannerRegressionTests('Tag', 'VehicleParams')
    %   results = runPlannerRegressionTests('GenerateReport', true)
    %
    % Optional Parameters:
    %   'Verbosity'      - Test output verbosity (0-4, default: 2)
    %   'Tag'           - Run only tests with specific tag
    %   'GenerateReport' - Generate HTML test report (default: false)
    %   'Parallel'      - Run tests in parallel (default: false)
    %   'OutputDir'     - Directory for test reports (default: 'TestReports')
    %
    % Returns:
    %   results - Array of test results from matlab.unittest framework
    %
    % Example:
    %   % Run all tests with detailed output
    %   results = runPlannerRegressionTests('Verbosity', 3, 'GenerateReport', true)
    %
    %   % Run only VehicleParams tests
    %   results = runPlannerRegressionTests('Tag', 'VehicleParams')
    %
    %   % Run performance tests only
    %   results = runPlannerRegressionTests('Tag', 'Performance')
    
    %% Parse input arguments
    p = inputParser;
    addParameter(p, 'Verbosity', 2, @(x) isnumeric(x) && x >= 0 && x <= 4);
    addParameter(p, 'Tag', '', @ischar);
    addParameter(p, 'GenerateReport', false, @islogical);
    addParameter(p, 'Parallel', false, @islogical);
    addParameter(p, 'OutputDir', 'TestReports', @ischar);
    addParameter(p, 'StopOnFailure', false, @islogical);
    
    parse(p, varargin{:});
    options = p.Results;
    
    %% Setup test environment
    fprintf('\n');
    fprintf('===============================================\n');
    fprintf('  +Planner Package Regression Test Suite\n');
    fprintf('===============================================\n');
    fprintf('Date: %s\n', datestr(now));
    fprintf('MATLAB Version: %s\n', version);
    fprintf('Test Options:\n');
    fprintf('  Verbosity: %d\n', options.Verbosity);
    if ~isempty(options.Tag)
        fprintf('  Tag Filter: %s\n', options.Tag);
    end
    fprintf('  Generate Report: %s\n', string(options.GenerateReport));
    fprintf('  Parallel Execution: %s\n', string(options.Parallel));
    fprintf('\n');
    
    %% Create test suite
    import matlab.unittest.TestSuite
    import matlab.unittest.TestRunner
    import matlab.unittest.plugins.*
    
    % Collect all test classes
    testSuites = {};
    
    % Individual component tests
    fprintf('Collecting test suites...\n');
    
    % VehicleParams tests (existing)
    try
        vehicleParamsTests = TestSuite.fromClass(?Planner.Testing.VehicleParamsTest);
        testSuites{end+1} = vehicleParamsTests;
        fprintf('  ✓ VehicleParamsTest (%d tests)\n', length(vehicleParamsTests));
    catch ME
        fprintf('  ⚠ VehicleParamsTest not available: %s\n', ME.message);
    end
    
    % Comprehensive regression suite
    try
        regressionTests = TestSuite.fromClass(?Planner.Testing.PlannerTestSuite);
        testSuites{end+1} = regressionTests;
        fprintf('  ✓ PlannerTestSuite (%d tests)\n', length(regressionTests));
    catch ME
        fprintf('  ⚠ PlannerTestSuite not available: %s\n', ME.message);
    end
    
    % Combine all test suites
    if isempty(testSuites)
        error('No test suites found. Ensure test classes exist in +Planner/+Testing/');
    end
    
    fullSuite = [testSuites{:}];
    
    % Apply tag filtering if specified
    if ~isempty(options.Tag)
        fullSuite = fullSuite.selectIf('Tag', options.Tag);
        fprintf('Filtered to %d tests with tag "%s"\n', length(fullSuite), options.Tag);
    end
    
    fprintf('Total tests to run: %d\n\n', length(fullSuite));
    
    %% Create test runner with plugins
    runner = TestRunner.withTextOutput('Verbosity', options.Verbosity);
    
    % Add report generation plugin
    if options.GenerateReport
        if ~isfolder(options.OutputDir)
            mkdir(options.OutputDir);
        end
        
        reportFile = fullfile(options.OutputDir, sprintf('PlannerTestReport_%s.html', ...
            datestr(now, 'yyyy-mm-dd_HH-MM-SS')));
        
        try
            htmlPlugin = HTMLPlugin.producingHTMLReport(reportFile);
            runner.addPlugin(htmlPlugin);
            fprintf('HTML report will be generated: %s\n', reportFile);
        catch ME
            warning('Planner:TestRunner:HTMLPlugin', 'Could not add HTML plugin: %s', ME.message);
        end
    end
    
    % Add code coverage plugin if available
    try
        coverageReport = fullfile(options.OutputDir, 'coverage');
        if ~isfolder(coverageReport)
            mkdir(coverageReport);
        end
        
        coveragePlugin = CodeCoveragePlugin.forFolder('+Planner', ...
            'IncludingSubfolders', true, ...
            'Producing', CoverageReport(coverageReport));
        runner.addPlugin(coveragePlugin);
        fprintf('Code coverage will be measured\n');
    catch ME
        % Coverage plugin might not be available in all MATLAB versions
        if options.Verbosity >= 2
            fprintf('Code coverage not available: %s\n', ME.message);
        end
    end
    
    % Add stop on failure plugin if requested
    if options.StopOnFailure
        runner.addPlugin(StopOnFailuresPlugin);
        fprintf('Will stop on first failure\n');
    end
    
    fprintf('\n');
    
    %% Run tests
    startTime = tic;
    
    if options.Parallel && license('test', 'Distrib_Computing_Toolbox')
        try
            % Run tests in parallel if Parallel Computing Toolbox available
            fprintf('Running tests in parallel...\n\n');
            results = runner.runInParallel(fullSuite);
        catch ME
            fprintf('Parallel execution failed, falling back to serial: %s\n', ME.message);
            results = runner.run(fullSuite);
        end
    else
        fprintf('Running tests serially...\n\n');
        results = runner.run(fullSuite);
    end
    
    executionTime = toc(startTime);
    
    %% Generate summary report
    fprintf('\n');
    fprintf('===============================================\n');
    fprintf('              TEST SUMMARY\n');
    fprintf('===============================================\n');
    
    totalTests = length(results);
    passedTests = sum([results.Passed]);
    failedTests = sum([results.Failed]);
    incompleteTests = sum([results.Incomplete]);
    
    fprintf('Total Tests:      %d\n', totalTests);
    fprintf('Passed:           %d\n', passedTests);
    fprintf('Failed:           %d\n', failedTests);
    fprintf('Incomplete:       %d\n', incompleteTests);
    fprintf('Success Rate:     %.1f%%\n', (passedTests/totalTests)*100);
    fprintf('Execution Time:   %.2f seconds\n', executionTime);
    
    if failedTests > 0
        fprintf('\nFAILED TESTS:\n');
        failedResults = results([results.Failed]);
        for i = 1:length(failedResults)
            fprintf('  ✗ %s\n', failedResults(i).Name);
            if ~isempty(failedResults(i).Details.DiagnosticRecord)
                fprintf('    Reason: %s\n', failedResults(i).Details.DiagnosticRecord(1).DiagnosticText);
            end
        end
    end
    
    if incompleteTests > 0
        fprintf('\nINCOMPLETE TESTS:\n');
        incompleteResults = results([results.Incomplete]);
        for i = 1:length(incompleteResults)
            fprintf('  ⚠ %s\n', incompleteResults(i).Name);
        end
    end
    
    %% Performance summary
    if any(contains({results.Name}, 'Performance'))
        fprintf('\nPERFORMACE TESTS:\n');
        perfResults = results(contains({results.Name}, 'Performance'));
        for i = 1:length(perfResults)
            if perfResults(i).Passed
                fprintf('  ✓ %s\n', perfResults(i).Name);
            else
                fprintf('  ✗ %s (Performance regression detected)\n', perfResults(i).Name);
            end
        end
    end
    
    %% Final status
    fprintf('\n');
    if failedTests == 0 && incompleteTests == 0
        fprintf('🎉 ALL TESTS PASSED! 🎉\n');
        fprintf('The +Planner package is ready for use.\n');
    elseif failedTests == 0
        fprintf('⚠️  TESTS COMPLETED WITH WARNINGS ⚠️\n');
        fprintf('Some tests were incomplete but no failures occurred.\n');
    else
        fprintf('❌ TEST FAILURES DETECTED ❌\n');
        fprintf('Please review and fix failing tests before deployment.\n');
    end
    
    if options.GenerateReport && exist('reportFile', 'var')
        fprintf('\nDetailed HTML report available at:\n%s\n', reportFile);
    end
    
    fprintf('===============================================\n\n');
end