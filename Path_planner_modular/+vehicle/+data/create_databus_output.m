function data_bus_KM = create_databus_output()

% calling in other bus_defs
KM_SAV_velocity = create_velocity_output();
KM_SAV_angle = create_angle_output();
KM_SAV_rate = create_rate_output();
KM_SAV_pos = create_pos_output();

% Initializing the output 
data_bus_KM.velocity = KM_SAV_velocity;
data_bus_KM.angle = KM_SAV_angle;
data_bus_KM.rate = KM_SAV_rate;
data_bus_KM.position = KM_SAV_pos;

