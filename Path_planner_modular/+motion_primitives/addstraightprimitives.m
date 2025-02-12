function [x_0, y_0, t_0, g_0, d_0, time_0] = addstraightprimitives(x_0, y_0, t_0, g_0, d_0, time_0, input_params)

for length = input_params.straight_lengths
    xgen = linspace(0, length, input_params.num_samples);
    ygen = zeros(1, input_params.num_samples);
    zer = zeros(input_params.num_samples, 1);
    count = size(x_0, 2) + 1;
    x_0(:,count)    =   xgen';
    y_0(:,count)    =   ygen';
    t_0(:,count)    =   zer;
    g_0(:,count)    =   zer;
    d_0(:,count)    =   zer;
    time_0(count) =   length;
end
