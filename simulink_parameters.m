classdef simulink_parameters < dynamicprops
    % SIMULINK_PARAMETERS A helper class to easily access device parameters.

    properties (Hidden)
        simulink_;
        str_;
    end
    
    methods
        function disp(obj)
            fprintf(obj.str_);
        end
    end

    methods
        function obj = simulink_parameters(simulink)
            obj.simulink_ = simulink ;
            obj.str_ = 'Accessible simulink parameters:\n';
        end
        
        function add_property(obj, names, types)
            if ~iscell(names)
                names = {names};
            end
            if ~iscell(types)
                types = {types};
            end
            for i = 1:length(names)
                name = names{i};
                type = types{i} ;
                obj.str_ = [obj.str_, '\t', name, '\n']; 
                prop = obj.addprop(name);
                prop.GetMethod = @(obj)obj.simulink_.get_par(name, type);
                prop.SetMethod = @(obj, value)obj.simulink_.set_par(name, type, value);
                prop.Dependent = true;
                prop.SetObservable = true;
                prop.Transient = false;
            end
        end
    end        
    
    methods (Static = true)
        function obj = loadobj(~)
            obj = [];
        end
    end
    
end
