classdef (ConstructOnLoad) simulink_interface < handle
    %SIMULINK_INTERFACE  Interface to Simulink.
    
    % Written by L. Renson (l.renson@bristol.ac.uk) 2016
    
    properties (Hidden)
        stream_opts;
    end
    
    properties
        simulink_vars;
        simulink_observable;
        simulink_observable_idx;
        computed_vars;
        par;
        opt;
    end
    
    methods
        
        function obj = simulink_interface()
            %Simulink_INTERFACE  Interface to the Simulink model.
            
            % Create a parameters object
            obj.par = simulink_parameters(obj) ;
            
            % Add the sample frequency to the list of computed variables
            obj.add_computed_var('sample_freq', @(x)(round(1/x.get_par('time_step','FixedStep'))));
        end
        
        function add_simulink_var(obj, var_name, var_address, var_type)
            % ADD_DSPACE_VAR  Add a variable to the list of known variables.
            %
            % OBJ.ADD_DSPACE_VAR(NAME, ADDRESS) adds the variable known as NAME to the
            % list of known variables. ADDRESS is the address of that variable known to
            % dSpace (for example, 'Model Root/X/Value').
            
            % Store the handle to the variable
            obj.simulink_vars.(var_name) = var_address;
            obj.par.add_property(var_name, var_type);
        end
        
        function add_simulink_obs(obj, var_name, var_address, idx)
            % ADD_DSPACE_VAR  Add a variable to the list of known variables.
            %
            % OBJ.ADD_DSPACE_VAR(NAME, ADDRESS) adds the variable known as NAME to the
            % list of known variables. ADDRESS is the address of that variable known to
            % dSpace (for example, 'Model Root/X/Value').
            
            % Store the handle to the variable
            obj.simulink_observable.(var_name) = var_address;
            if nargin < 4 || isempty(idx) || idx == 0,
                obj.simulink_observable_idx.(var_name) = 0 ;
            else
                obj.simulink_observable_idx.(var_name) = idx ;
            end
            obj.par.add_property(var_name, []);
        end
        
        function add_computed_var(obj, var_name, var_func)
            % ADD_COMPUTED_VAR  Add a variable to the list of computed variables.
            %
            % OBJ.ADD_COMPUTED_VAR(NAME, FUNC) adds the variable known as NAME to the
            % list of computed variables. FUNC is called with OBJ as the first
            % parameter to compute the value of the variable. For example,
            %
            % obj.add_computed_var('sample_freq', @(x)round(1/x.get_par('time_step')));
            
            % Store the function to compute the value of the variable
            obj.computed_vars.(var_name) = var_func;
            obj.par.add_property(var_name, []);
        end
        
        function set_par(obj, names, types, values)
            % SET_PAR  Set the values of the specified parameters.
            %
            % OBJ.SET_PAR(NAME, VALUE) sets the value of the parameter NAME to VALUE.
            % Both NAME and VALUE can be cell arrays in the case of setting multiple
            % parameter values simultaneously.
            if ~iscell(names)
                names = {names};
                values = {values};
                types = {types};
            end
            % Iterate over the supplied names
            for i = 1:length(names)
                if isfield(obj.simulink_vars, names{i})
                    % Write to simulink
                    if length(values{i})>1,
                        allOneString = sprintf('%.8f,' , values{i}) ;
                        set_param(obj.simulink_vars.(names{i}), types{i}, ['[',allOneString(1:end-1),']']);
                    else
                        set_param(obj.simulink_vars.(names{i}), types{i}, sprintf('%.8f,' ,values{i}));
                    end
                elseif isfield(obj.simulink_observable, names{i}),
                    error('Read only variable: %s', names{i});
                elseif isfield(obj.computed_vars, names{i})
                    error('Read only variable: %s', names{i});
                else
                    error('Unknown variable: %s', names{i});
                end
            end
        end
        
        function values = get_par(obj, names, types)
            % GET_PAR  Get the values of the specified parameters.
            %
            % OBJ.GET_PAR(NAME) gets the value of the parameter NAME. NAME can be a
            % cell array to get multiple parameter values simultaneously.
            %global simulink_observables
            
            if ~iscell(names)
                names = {names};
                types = {types} ;
                islist = false;
            else
                islist = true;
            end
            % Iterate over the supplied names
            values = cell(size(names));
            
            for i = 1:length(names)
                if isfield(obj.simulink_vars, names{i})
                    % Read from simulink
                    temp = get_param(obj.simulink_vars.(names{i}), types{i});
                    temp_num = str2num(temp) ;
                    if ~isnan(temp_num),
                        values{i} = temp_num ;
                    else
                        if evalin('base',['exist(''',temp,''',''var'')']),
                            values{i} = evalin('base',temp) ;
                        else
                            values{i} = eval(temp) ;
                        end
                    end
                elseif isfield(obj.simulink_observable, names{i}),
                    % Fetch the value:
                    rto = get_param(obj.simulink_observable.(names{i}),'RunTimeObject') ;
                    if isempty(rto),
                        if strcmp(get_param(obj.model,'SimulationStatus'),'running'),
                            error(['Specified observable has no RunTimeObject. ',...
                                'Check if specified observable is a non virtual block...']) ;
                        else
                            warning('Cannot access simulink observable, simulation might not be running.') ;
                        end
                        values{i} = [] ;
                    else
                        if obj.simulink_observable_idx.(names{i}) == 0,
                            values{i} = rto.OutputPort(1).Data' ;
                        else
                            values{i} = rto.OutputPort(1).Data(obj.simulink_observable_idx.(names{i}))' ;
                        end
                    end
                elseif isfield(obj.computed_vars, names{i})
                    % Calculate computed value
                    values{i} = obj.computed_vars.(names{i})(obj);
                else
                    error('Unknown variable: %s', names{i});
                end
            end
            % Return as a list or raw data depending on what was passed originally
            if ~islist
                values = values{1};
            end
        end
        
        function set_stream(obj, ~, parameters, samples, downsample)
            % NOT NECESSARY HERE
        end
        
        function data = get_stream(obj, ~, ~)
            % GET_STREAM  Get the data from a particular stream.
            %
            % OBJ.GET_STREAM(ID) returns an array of data recorded in the stream given
            % by ID. If the stream is not ready, no data is returned.
            %
            % OBJ.GET_STREAM(ID, true) returns a structure with named fields containing
            % the data recorded in the stream given by ID.
            %
            % See also START_STREAM.
            set_param(bdroot,'SimulationCommand','WriteDataLogs');
            while ~evalin('base','exist(''logsout'',''var'')'),
                pause(0.1) ;
            end
            temp = evalin('base','logsout') ;
            for i = 1:length(obj.datafields.stream_fields),
                data.(obj.datafields.stream_fields{i}) = temp.(obj.datafields.stream_fields{i}).Data ;
            end
            data.time = temp.(obj.datafields.stream_fields{i}).Time ;
            data.time_mod_2pi = 2*pi*mod(data.time*obj.par.forcing_freq,1) ;
        end
        
        function result = start_stream(~, ~)
            % START_STREAM  Start a stream recording.
            %
            % OBJ.START_STREAM(ID) starts the stream given by ID recording data with
            % the current parameters from SET_STREAM.
            %
            % See also SET_STREAM.
            result = true;
        end
        
        function data = run_stream(obj, stream, varargin)
            % RUN_STREAM  Start a stream recording and then return the captured data.
            %
            % OBJ.RUN_STREAM(ID) starts the stream given by ID and then returns the
            % captured data.
            %
            % OBJ.RUN_STREAM(ID, Name, Value) overrides the default options for running
            % the stream.
            %
            % Options
            %
            %     start: allowed values are true or false. Default true.
            %         Whether or not to start the stream running before waiting for
            %         available captured data.
            %
            %     wait_period: allowed values are a > 0. Default 0.1.
            %         The period of time the function should pause before checking if
            %         there is captured data available.
            %
            %     struct: allowed values are true or false. Default false.
            %         Whether or not to return the data as a structure.
            %
            % See also START_STREAM, GET_STREAM.
            p = inputParser();
            if ismethod(p, 'addParameter')
                % New versions of Matlab
                add_par = @p.addParameter;
            else
                % Old versions of Matlab
                add_par = @p.addParamValue;
            end
            add_par('start', true, @islogical);
            add_par('wait_period', 0.1, @(x)(x > 0));
            add_par('struct', false, @islogical);
            p.parse(varargin{:});
            if p.Results.start
                if ~obj.start_stream(stream)
                    error('Failed to start stream - perhaps bad parameters');
                end
            end
            StartTime = get_param(obj.model,'SimulationTime') ;
            StopTime = StartTime + obj.opt.samples*obj.par.time_step ;
            while get_param(obj.model,'SimulationTime') < StopTime,
                pause(0.1);
            end
            data = obj.get_stream(stream, p.Results.struct);
        end
        
    end
    
end

