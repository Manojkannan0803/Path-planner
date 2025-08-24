classdef GeometryUtils
    % GEOMETRYUTILS - Low-level geometric calculations
    % Foundation layer utility class for geometric operations
    
    methods (Static)
        function distance = euclidean_distance(x1, y1, x2, y2)
            % Calculate Euclidean distance between two points
            distance = sqrt((x2 - x1)^2 + (y2 - y1)^2);
        end
        
        function angle = angle_between_points(x1, y1, x2, y2)
            % Calculate angle between two points in degrees
            angle = atan2d(y2 - y1, x2 - x1);
        end
        
        function [x_rot, y_rot] = rotate_point(x, y, angle_deg, cx, cy)
            % Rotate point around center by angle in degrees
            if nargin < 5
                cx = 0; cy = 0; % Default to origin
            end
            
            angle_rad = deg2rad(angle_deg);
            x_shifted = x - cx;
            y_shifted = y - cy;
            
            x_rot = x_shifted * cos(angle_rad) - y_shifted * sin(angle_rad) + cx;
            y_rot = x_shifted * sin(angle_rad) + y_shifted * cos(angle_rad) + cy;
        end
        
        function normalized_angle = normalize_angle(angle_deg)
            % Normalize angle to [0, 360) range
            normalized_angle = mod(angle_deg, 360);
        end
        
        function diff = angle_difference(angle1, angle2)
            % Calculate minimum difference between two angles
            diff = abs(angle1 - angle2);
            if diff > 180
                diff = 360 - diff;
            end
        end
        
        function area = polygon_area(xv, yv)
            % Calculate area of polygon using shoelace formula
            n = length(xv);
            area = 0;
            for i = 1:n
                j = mod(i, n) + 1;
                area = area + (xv(i) * yv(j) - xv(j) * yv(i));
            end
            area = abs(area) / 2;
        end
    end
end
