function [in, on] = InPolygon(x, y, xv, yv)
    % INPOLYGON - Point-in-polygon test
    % Refactored from original InPolygon to foundation layer
    %
    % Tests whether points are inside or on the boundary of a polygon
    %
    % Inputs:
    %   x, y - Coordinates of query points
    %   xv, yv - Vertices of the polygon
    %
    % Outputs:
    %   in - Logical array indicating points inside polygon
    %   on - Logical array indicating points on polygon boundary
    
    % Use MATLAB's built-in inpolygon function for robustness
    if nargout > 1
        [in, on] = inpolygon(x, y, xv, yv);
    else
        in = inpolygon(x, y, xv, yv);
    end
end
