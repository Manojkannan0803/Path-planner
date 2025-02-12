function out = create_model_mat(outputfilename)

if nargin == 0
    % default output
    outputfilename = 'ModelConstants.mat';
end

% create bus definitions
out = struct;
out = mergefield(out,  '>', utilities.create_model_buses());

% now that all bus definitions are present,
% constants can be instantiated and filled with sensible defaults
out = mergefield(out, '>', utilities.create_basic_constants());

% dirty trick: load bus definitions into BASE workspace, 
% to make the bus definitions available for instantiation in subsequent functions
% (as required by Simulink.Bus.createMATLABStruct, which uses string names ...)
fn = fieldnames(out);
for it = 1:numel(fn)
    assignin('base', fn{it}, getfield(out, fn{it}));
end

if nargout == 0
    save(outputfilename, '-struct', 'out');
end