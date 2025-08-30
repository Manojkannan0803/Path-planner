function rounded_value = ppround_1(value)
    % PPROUND_1 - Precision rounding function
    % Refactored from original ppround_1.m to foundation layer
    % 
    % This function rounds values to fit the scale of the state space
    % Used for discretizing continuous coordinates to grid coordinates
    %
    % Input:
    %   value - Continuous coordinate value to be rounded
    %
    % Output:
    %   rounded_value - Discretized coordinate value
    
    % Round to nearest integer (maintaining original functionality)
    rounded_value = round(value);
end
