classdef AlgorithmFactory < handle
    % ALGORITHMFACTORY - Factory for creating algorithm instances
    % Implements the factory pattern for algorithm instantiation
    
    properties (Access = private)
        registered_algorithms   % Map of available algorithms
    end
    
    methods
        function obj = AlgorithmFactory()
            % Constructor - Register available algorithms
            obj.registered_algorithms = containers.Map();
            obj.register_default_algorithms();
        end
        
        function algorithm = create_algorithm(obj, algorithm_name, config, services)
            % Create algorithm instance by name
            
            if ~obj.registered_algorithms.isKey(algorithm_name)
                error('AlgorithmFactory:UnknownAlgorithm', ...
                    'Algorithm "%s" is not registered. Available: %s', ...
                    algorithm_name, strjoin(obj.get_available_algorithms(), ', '));
            end
            
            algorithm_class = obj.registered_algorithms(algorithm_name);
            
            % Create instance using reflection
            algorithm = feval(algorithm_class, config, services);
        end
        
        function register_algorithm(obj, algorithm_name, algorithm_class)
            % Register new algorithm class
            obj.registered_algorithms(algorithm_name) = algorithm_class;
        end
        
        function algorithm_names = get_available_algorithms(obj)
            % Get list of available algorithm names
            algorithm_names = obj.registered_algorithms.keys();
        end
        
        function remove_algorithm(obj, algorithm_name)
            % Remove algorithm from registry
            if obj.registered_algorithms.isKey(algorithm_name)
                obj.registered_algorithms.remove(algorithm_name);
            end
        end
        
        function info = get_algorithm_info(obj, algorithm_name)
            % Get information about a specific algorithm
            if ~obj.registered_algorithms.isKey(algorithm_name)
                error('AlgorithmFactory:UnknownAlgorithm', ...
                    'Algorithm "%s" is not registered', algorithm_name);
            end
            
            % Create temporary instance to get info
            algorithm_class = obj.registered_algorithms(algorithm_name);
            temp_config = struct();
            temp_services = struct();
            temp_instance = feval(algorithm_class, temp_config, temp_services);
            
            info = temp_instance.get_algorithm_info();
        end
        
        function validate_algorithm_class(obj, algorithm_class)
            % Validate that algorithm class implements required interface
            
            % Check if class exists
            if ~exist(algorithm_class, 'file')
                error('AlgorithmFactory:ClassNotFound', ...
                    'Algorithm class "%s" not found', algorithm_class);
            end
            
            % Additional validation could be added here:
            % - Check if class inherits from AlgorithmInterface
            % - Check if required methods are implemented
        end
    end
    
    methods (Access = private)
        function register_default_algorithms(obj)
            % Register built-in algorithms
            
            % Register A* algorithm
            obj.register_algorithm('astar', 'AStarAlgorithm');
            
            % Register Dijkstra algorithm
            obj.register_algorithm('dijkstra', 'DijkstraAlgorithm');
            
            % Register RRT algorithm
            obj.register_algorithm('rrt', 'RRTAlgorithm');
            
            % Register Hybrid A* algorithm
            obj.register_algorithm('hybrid_astar', 'HybridAStarAlgorithm');
            
            % Register Theta* algorithm
            obj.register_algorithm('theta_star', 'ThetaStarAlgorithm');
            
            % Add more algorithms as they are implemented
        end
    end
    
    methods (Static)
        function factory = get_instance()
            % Singleton pattern implementation
            persistent instance;
            if isempty(instance)
                instance = AlgorithmFactory();
            end
            factory = instance;
        end
        
        function register_custom_algorithm(algorithm_name, algorithm_class)
            % Static method to register custom algorithms
            factory = AlgorithmFactory.get_instance();
            factory.validate_algorithm_class(algorithm_class);
            factory.register_algorithm(algorithm_name, algorithm_class);
        end
        
        function algorithm_names = list_algorithms()
            % Static method to list available algorithms
            factory = AlgorithmFactory.get_instance();
            algorithm_names = factory.get_available_algorithms();
        end
    end
end
