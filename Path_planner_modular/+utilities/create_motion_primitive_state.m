function mp_state = create_motion_primitive_state()
% The state should contain the info of gcost and hcost of moving to that 
% by the motion primitive pred (predecessor).

mp_config = utilities.create_motion_primitive_struct();

mp_state.x = [];
mp_state.y = [];
mp_state.xa = [];
mp_state.ya = [];
mp_state.predxy = [];
mp_state.pred = mp_config;