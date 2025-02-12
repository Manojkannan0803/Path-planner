function [x_0, y_0, t_0, g_0, d_0, time_0] = generateInitialPrimitives(params)

count = 0;
x_0 = zeros(params.num_samples,1); y_0 = zeros(params.num_samples,1); t_0 = zeros(params.num_samples,1); g_0 = zeros(params.num_samples,1); d_0 = zeros(params.num_samples,1); time_0 = zeros(params.num_samples,1);

for gamma = params.gammad
    folder = strcat(params.motion_primitive_folder, '\thetatrailer_',num2str(0),'_gamma_',num2str(gamma));
    %folder = fullfile(params.motion_primitive_folder, sprintf('thetatrailer_0_gamma_%d', gamma));
    for theta = params.thetad
        for gamma_inner = params.gammad
            file = strcat(folder,'\','mp_',num2str(theta),'_',num2str(gamma_inner),'.mat');
            %file = fullfile(folder, sprintf('mp_%d_%d.mat', theta, gamma_inner));
            if exist(file, 'file')
                load(file, 'out');
                count = count + 1;
                x_0(:,count)    =   out.STATES(:,2);
                y_0(:,count)    =   out.STATES(:,3);
                t_0(:,count)    =   out.STATES(:,4);
                g_0(:,count)    =   out.STATES(:,5);
                d_0(:,count)    =   out.STATES(:,6);  
                time_0(count) =   out.PARAMETERS(1,2);
            end
        end
    end
end

