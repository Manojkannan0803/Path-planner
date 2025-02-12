function bus_defs = create_model_buses()

bus_defs.SAV_velocity = Simulink.Bus;
bus_defs.SAV_velocity.Elements(end+1) = utilities.create_bus_elements('v_1',1,'double'); % velocity of semi-axle
bus_defs.SAV_velocity.Elements(end+1) = utilities.create_bus_elements('v_0',1,'double'); % velocity of drive-axle

bus_defs.SAV_angle = Simulink.Bus;
bus_defs.SAV_angle.Elements(end+1) = utilities.create_bus_elements('theta_0',1,'double'); % yaw angle of tractor
bus_defs.SAV_angle.Elements(end+1) = utilities.create_bus_elements('theta_1',1,'double'); % yaw angle of trailer
bus_defs.SAV_angle.Elements(end+1) = utilities.create_bus_elements('gamma',1,'double'); % articulation angle
bus_defs.SAV_angle.Elements(end+1) = utilities.create_bus_elements('delta',1,'double'); % steering angle of tractor

bus_defs.SAV_rate = Simulink.Bus;
bus_defs.SAV_rate.Elements(end+1) = utilities.create_bus_elements('theta_0_dot',1,'double'); % yaw rate of tractor
bus_defs.SAV_rate.Elements(end+1) = utilities.create_bus_elements('theta_1_dot',1,'double'); % yaw rate of trailer
bus_defs.SAV_rate.Elements(end+1) = utilities.create_bus_elements('x_0_dot',1,'double'); % tractor drive axle velocity in global-x-direction
bus_defs.SAV_rate.Elements(end+1) = utilities.create_bus_elements('y_0_dot',1,'double'); % tractor drive axle velocity in global-y-direction

bus_defs.SAV_pos = Simulink.Bus;
bus_defs.SAV_pos.Elements(end+1) = utilities.create_bus_elements('x_0f',1,'double'); % steer_x
bus_defs.SAV_pos.Elements(end+1) = utilities.create_bus_elements('y_0f',1,'double'); % steer_y
bus_defs.SAV_pos.Elements(end+1) = utilities.create_bus_elements('x_0',1,'double'); % drive_x
bus_defs.SAV_pos.Elements(end+1) = utilities.create_bus_elements('y_0',1,'double'); % drive_y
bus_defs.SAV_pos.Elements(end+1) = utilities.create_bus_elements('x_1f',1,'double'); % kingpin_x
bus_defs.SAV_pos.Elements(end+1) = utilities.create_bus_elements('y_1f',1,'double'); % kingpin_y
bus_defs.SAV_pos.Elements(end+1) = utilities.create_bus_elements('x_1',1,'double'); % semi_x
bus_defs.SAV_pos.Elements(end+1) = utilities.create_bus_elements('y_1',1,'double'); % semi_y

bus_defs.data_bus_KM = Simulink.Bus;
bus_defs.data_bus_KM.Elements(end+1) = utilities.create_bus_elements('velocity',1,'SAV_velocity'); 
bus_defs.data_bus_KM.Elements(end+1) = utilities.create_bus_elements('angle',1,'SAV_angle'); 
bus_defs.data_bus_KM.Elements(end+1) = utilities.create_bus_elements('rate',1,'SAV_rate'); 
bus_defs.data_bus_KM.Elements(end+1) = utilities.create_bus_elements('position',1,'SAV_pos'); 

% bus_defs.data_bus_KM = Simulink.Bus;
% bus_defs.data_bus_KM.Elements(end+1) = utilities.create_bus_elements('data_bus',1,'SAV_comb'); 

