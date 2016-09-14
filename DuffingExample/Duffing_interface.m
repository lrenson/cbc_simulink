classdef Duffing_interface < simulink_interface
    % SM_INTERFACE: interface to a simple simulink model
    % Written by L. Renson (l.renson@bristol.ac.uk) 2016
    
    properties
        model;
        fourier;
        datafields;
        averaging;
        OriginalOptimizeBlockIOStorage
    end
    
    methods
        function obj = Duffing_interface()
            evalin('base','init_Duffing') ; pause(1) ;
            obj.model = 'cbc_Duffing' ;
            % SM_INTERFACE: interface to a simple simulink model
            
            % Load simulink model and connect to the code:
            load_system(obj.model) ;
            
            % set the simulation mode to normal
            set_param(obj.model,'SimulationMode','normal');
            
            % set the stop time to inf
            set_param(obj.model,'StopTime','Inf');
            obj.add_simulink_var('time_step',obj.model, 'FixedStep');
            
            % Check block outputs and simulink excecution are synchronized:
            obj.OriginalOptimizeBlockIOStorage = get_param(obj.model,'OptimizeBlockIOStorage') ;
            if strcmp(obj.OriginalOptimizeBlockIOStorage,'on'),
                warning('Signal storage reuse optimization option has been turned off for synchronization purposes.') ;
                set_param(obj.model,'OptimizeBlockIOStorage','off') ;
            end
            
            % Start running the simulation
            set_param(obj.model,'SimulationCommand','start') ;
            
            % Add known Simulink variables
            % ----------------------------
            obj.add_simulink_var('run', 'cbc_Duffing/Start', 'Value');
            obj.add_simulink_var('forcing_amp', 'cbc_Duffing/Ampl', 'Value');
            obj.add_simulink_var('forcing_freq', 'cbc_Duffing/Freq', 'Value');
            obj.add_simulink_var('DC_level', 'cbc_Duffing/DClevel', 'Value');
            
            % we still need to get the overall target value?
            obj.add_simulink_obs('x', 'cbc_Duffing/SDOF model/Integrator 1', 41) ;
            obj.add_simulink_obs('x_coeffs', 'cbc_Duffing/Fourier Series Response/Sum') ;
            obj.add_computed_var('x_coeffs_ave', @(x)x.averaging.x_coeffs_ave);
            obj.add_computed_var('x_coeffs_var', @(x)x.averaging.x_coeffs_var);
            
            obj.add_simulink_obs('x_target', 'cbc_Duffing/ForceGen/Sum') ;
            obj.add_simulink_var('x_target_coeffs', 'cbc_Duffing/ForceGen/Target', 'Value');
            
            obj.add_simulink_obs('out', 'cbc_Duffing/Product', 4);
            obj.add_simulink_obs('out_coeffs', 'cbc_Duffing/Fourier Series Force/Sum');
            obj.add_computed_var('out_coeffs_ave', @(x)x.averaging.out_coeffs_ave);
            obj.add_computed_var('out_coeffs_var', @(x)x.averaging.out_coeffs_var);
            
            %obj.add_simulink_obs('x_error', 'CBC_Duffing/Sum2') ;
            %obj.add_simulink_obs('control', 'CBC_Duffing/Controller/Sum2') ;
            
            % Periodic solution controller:
            obj.add_simulink_var('x_control', 'cbc_Duffing/ControlSwitch', 'Value');
            obj.add_simulink_var('x_Kp', 'cbc_Duffing/Controller/Kp', 'Value');
            obj.add_simulink_var('x_Kd', 'cbc_Duffing/Controller/Kd', 'Value');
            
            % Indices into the array of Fourier variables
            n_coeff = length(obj.par.x_coeffs);
            obj.fourier.n_modes = (n_coeff - 1)/2;
            obj.fourier.n_ave = 10; % Number of periods that are averaged to get the result
            obj.fourier.idx_DC = 1;
            obj.fourier.idx_AC = 2:n_coeff;
            obj.fourier.idx_fund = [2, 2 + obj.fourier.n_modes];
            obj.fourier.idx_higher = [(3:1 + obj.fourier.n_modes), (3 + obj.fourier.n_modes:n_coeff)];
            obj.fourier.idx_sin =  2 + obj.fourier.n_modes:n_coeff;
            obj.fourier.idx_cos = 2:1 + obj.fourier.n_modes;
            obj.fourier.idx_steady_conv = 2:15 ;
            %obj.fourier.idx_fixed_pt = [1:15] ;
            
            % Default options for the experiment
            obj.opt.samples = 5000; % Number of samples to record
            obj.opt.downsample = 0; % Number of samples to ignore for every sample recorded
            obj.opt.wait_time = 5; % Time (in secs) to wait for Fourier coefficients to settle
            obj.opt.max_waits = 15; % Maximum number of times to wait
            obj.opt.max_picard_iter = 10; % Maximum number of Picard iterations to do
            obj.opt.x_coeffs_var_tol_rel = 5e-3; % Maximum (normalised) variance of Fourier coefficients for steady-state behaviour
            obj.opt.x_coeffs_var_tol_abs = 5e-3; % Maximum (absolute) variance of Fourier coefficients for steady-state behaviour
            obj.opt.x_coeffs_tol = 1e-3; % Maximum tolerance for difference between two Fourier coefficients (mm)
            
            % Data recording fields
            obj.datafields.stream_id = 1; % The stream to use for data recording (N/A)
            obj.datafields.static_fields = {'x_Kp', 'x_Kd', 'x_control', ...
                                'sample_freq'};
            obj.datafields.dynamic_fields = {'forcing_freq', 'forcing_amp', ...
                                'x_coeffs_ave', 'x_coeffs_var', ...
                                'x_target_coeffs', 'out_coeffs_ave', 'out_coeffs_var', 'DC_level'};
            obj.datafields.stream_fields = {'x', 'Target', ...
                                'ShakerInput', 'ControlSignal', 'Error', 'time', 'ErrorFilt', 'dErrorFilt'}; 
            
            % Create variables for averaging
            obj.opt.n_ave = 10;
            obj.averaging.x_coeffs_ave = zeros(1, n_coeff);
            obj.averaging.x_coeffs_var = zeros(1, n_coeff);
            obj.averaging.x_coeffs_arr = zeros(obj.fourier.n_ave, n_coeff);
            obj.averaging.out_coeffs_ave = zeros(1, n_coeff);
            obj.averaging.out_coeffs_var = zeros(1, n_coeff);
            obj.averaging.out_coeffs_arr = zeros(obj.fourier.n_ave, n_coeff);
            obj.averaging.last_freq = obj.par.forcing_freq ;%obj.get_par('forcing_freq');
            obj.averaging.timer = timer('BusyMode', 'drop', 'ExecutionMode', 'fixedRate', 'TimerFcn', @obj.update_averages);
            obj.averaging.timer.Period = max([round((1/obj.averaging.last_freq)*1000)/1000, 0.05]); % Limit to 10 Hz updates
            start(obj.averaging.timer);
            
            % Default control gains
            obj.par.x_target_coeffs(:) = 0 ;
            obj.par.x_control = 0 ;
            obj.par.x_Kp = -30 ; 
            obj.par.x_Kd = -3 ;
            
            % Set everything moving
            obj.par.forcing_amp = 0 ;
            obj.par.run = 1;
        end
        
        function delete(obj)
            % DELETE  Destroy the interface to Simulink.
            stop(obj.averaging.timer);
            delete(obj.averaging.timer);
            set_param(obj.model,'OptimizeBlockIOStorage',obj.OriginalOptimizeBlockIOStorage) ;
        end
        
        function update_averages(obj, varargin)
            % UPDATE_AVERAGES  Get new data for the x and out coefficients and update
            % the respective averages and variances.
            
            if strcmp(get_param(obj.model,'SimulationStatus'),'running'),
                obj.averaging.x_coeffs_arr(1:end-1, :) = obj.averaging.x_coeffs_arr(2:end, :);
                obj.averaging.x_coeffs_arr(end, :) = obj.par.x_coeffs ; %obj.get_par('x_coeffs');
                obj.averaging.x_coeffs_ave = mean(obj.averaging.x_coeffs_arr);
                obj.averaging.x_coeffs_var = var(obj.averaging.x_coeffs_arr);
                obj.averaging.out_coeffs_arr(1:end-1, :) = obj.averaging.out_coeffs_arr(2:end, :);
                obj.averaging.out_coeffs_arr(end, :) = obj.par.out_coeffs ; %obj.get_par('out_coeffs');
                obj.averaging.out_coeffs_ave = mean(obj.averaging.out_coeffs_arr);
                obj.averaging.out_coeffs_var = var(obj.averaging.out_coeffs_arr);
                forcing_freq = obj.par.forcing_freq ; %obj.get_par('forcing_freq');
                if obj.averaging.last_freq ~= forcing_freq
                    obj.averaging.last_freq = forcing_freq;
                    stop(obj.averaging.timer);
                    obj.averaging.timer.Period = max([round((1/obj.averaging.last_freq)*1000)/1000, 0.1]); % Limit to 10 Hz updates
                    start(obj.averaging.timer);
                end
            else
                stop(obj.averaging.timer) ;
                warning('Simulation is no longer running. Avergaging updates are stopped.') ;
            end
        end
    end
    
end


