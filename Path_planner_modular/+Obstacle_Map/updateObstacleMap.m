function [obsx_out, obsy_out, status] = updateObstacleMap(obsx_static, obsy_static, obsx_dyn, obsy_dyn, update_flag)

% Initialize output status
status = 0; % Default to failure

% If update_flag is true, update the obstacle map with dynamic obstacles
if update_flag
    % Combine static and dynamic obstacles
    obsx_out = [obsx_static, obsx_dyn];
    obsy_out = [obsy_static, obsy_dyn];

    % set status to success
    status = 1;
else
    % If no update, return only static obstacles
    obsx_out = obsx_static;
    obsy_out = obsy_static;

    % set status to success
    status = 1;
end

end