function mp_config = create_motion_primitive_struct()
% Creating motion primitive structure. Each motion primitive can be
% uniquely identified by 4 tags: ti(theta initial), gi(gamma initial), tf (theta final), gf (gamma final)

mp_config.ti = []; 
mp_config.gi = [];
mp_config.tf = [];
mp_config.gf = [];
mp_config.gcost = [];
mp_config.hcost = [];
mp_config.dir = [];