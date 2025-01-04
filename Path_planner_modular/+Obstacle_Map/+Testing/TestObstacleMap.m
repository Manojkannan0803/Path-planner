classdef TestObstacleMap < matlab.unittest.TestCase

    methods (Test)
        % Test methods

        function testInitialization(obj)
            % Test Case for the constructor and obstacle map initialization
            % Inserting DPD Scenario
            obsx_static = [28,58.6,58.6,61.4,61.4,108,108,148.6625,148.6625,151.3375,151.3375,152.6675,152.6675,155.3425,155.3425,228,228,285,285,228,228,108,108,28];
            obsy_static = [53,53,69.5,69.5,53,53,61,61,77.5,77.5,61,61,77.5,77.5,61,61,57,57,140,140,136,136,144,144];

            % Create Obstaclemap object
            map = ObstacleMap(obsx_static,obsy_static);

            % Verfiy that the object are initialized correctly
            obj.verifyEqual(map.obsx_static, obsx_static);
            obj.verifyEqual(map.obsy_static, obsy_static);
            obj.verifyEmpty(map.obsx_update_static);
            obj.verifyEmpty(map.obsy_update_static);
            obj.verifyEmpty(map.obsx_dyn);
            obj.verifyEmpty(map.obsy_dyn);
            obj.verifyEmpty(map.vx_dyn);
            obj.verifyEmpty(map.vy_dyn);
        end

        function testUpdatewithFlagtrue(obj)
            % Test updating obstacle map when update_flag is true
            % Inserting DPD Scenario
            obsx_static = [28,58.6,58.6,61.4,61.4,108,108,148.6625,148.6625,151.3375,151.3375,152.6675,152.6675,155.3425,155.3425,228,228,285,285,228,228,108,108,28];
            obsy_static = [53,53,69.5,69.5,53,53,61,61,77.5,77.5,61,61,77.5,77.5,61,61,57,57,140,140,136,136,144,144];

            update_obsx = [72,240,255,157,145,140,150,87,72]; 
            update_obsy = [13,13,33,33,21,23,33,33,13];
            
            % Create Obstaclemap object
            map = ObstacleMap(obsx_static,obsy_static);
            
            % Call updateMap with update_flag = true
            update_flag = true;
            [obsx_out, obsy_out, status] = map.updateMap(update_obsx, update_obsy, update_flag);

            % Verfiy updated map and status
            expected_obsx = [obsx_static, update_obsx];
            expected_obsy = [obsy_static, update_obsy];
            obj.verifyEqual(obsx_out, expected_obsx);
            obj.verifyEqual(obsy_out, expected_obsy);
            obj.verifyTrue(status);
            obj.verifyEqual(map.obsx_update_static, update_obsx);
            obj.verifyEqual(map.obsy_update_static, update_obsy);
        end

        function testUpdatewithFlagFlase(obj)
            % Test case for not updating the obstacle map (only static!)
            % Inserting DPD Scenario
            obsx_static = [28,58.6,58.6,61.4,61.4,108,108,148.6625,148.6625,151.3375,151.3375,152.6675,152.6675,155.3425,155.3425,228,228,285,285,228,228,108,108,28];
            obsy_static = [53,53,69.5,69.5,53,53,61,61,77.5,77.5,61,61,77.5,77.5,61,61,57,57,140,140,136,136,144,144];

            update_obsx = [72,240,255,157,145,140,150,87,72]; 
            update_obsy = [13,13,33,33,21,23,33,33,13];

            % Create Obstaclemap object
            map = ObstacleMap(obsx_static,obsy_static);
            
            % Call updateMap with update_flag = false
            update_flag = false;
            [obsx_out, obsy_out, status] = map.updateMap(update_obsx, update_obsy, update_flag);

            % Verfiy updated map and status
            expected_obsx = obsx_static;
            expected_obsy = obsy_static;
            obj.verifyEqual(obsx_out, expected_obsx);
            obj.verifyEqual(obsy_out, expected_obsy);
            obj.verifyFalse(status);
        end

        function testVisualization(obj)
            % Test visualization to ensure correct plotting
            obsx_static = [28,58.6,58.6,61.4,61.4,108,108,148.6625,148.6625,151.3375,151.3375,152.6675,152.6675,155.3425,155.3425,228,228,285,285,228,228,108,108,28];
            obsy_static = [53,53,69.5,69.5,53,53,61,61,77.5,77.5,61,61,77.5,77.5,61,61,57,57,140,140,136,136,144,144];

            update_obsx = [72,240,255,157,145,140,150,87,72];
            update_obsy = [13,13,33,33,21,23,33,33,13];

            map = ObstacleMap(obsx_static,obsy_static);
            map.updateMap(update_obsx, update_obsy, true);

            map.visualizeobstaclemap();

            update_obsx_1 = [162.6,180,180,162.6];
            update_obsy_1 = [195,195,190,190];

            map.updateMap(update_obsx_1, update_obsy_1, true);

            map.visualizeobstaclemap();

            % No assertion is inserted; visualization should be inspected
            % manually
        end

        function testEdgeCaseNoStaticObstacle(obj)
            % Test initializing empty static obstacles
            obsx_static = [];
            obsy_static = [];
            map = ObstacleMap(obsx_static,obsy_static);

            % Verify Properties
            obj.verifyEmpty(map.obsx_static);
            obj.verifyEmpty(map.obsy_static);
            obj.verifyEqual(map.Initialstatic_count, 0);
        end

        function testEdgeCaseEmptyNewStaticObstacle(obj)
            % Test updating with empty new static obstacles

            obsx_static = [28,58.6,58.6,61.4,61.4,108,108,148.6625,148.6625,151.3375,151.3375,152.6675,152.6675,155.3425,155.3425,228,228,285,285,228,228,108,108,28];
            obsy_static = [53,53,69.5,69.5,53,53,61,61,77.5,77.5,61,61,77.5,77.5,61,61,57,57,140,140,136,136,144,144];

            update_obsx = [];
            update_obsy = [];
            map = ObstacleMap(obsx_static,obsy_static);
            % Call updateMap with update_flag = true
            update_flag = true;
            [obsx_out, obsy_out, status] = map.updateMap(update_obsx, update_obsy, update_flag);

            % Verify no change in map
            obj.verifyEqual(obsx_out, obsx_static);
            obj.verifyEqual(obsy_out, obsy_static);
            obj.verifyTrue(status);
        end
                
    end

end