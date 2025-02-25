function entities = generate_ordered_entity_list(casadi_entities, room_configs)
  % this maps interface definitions used in model definitions of casadi with (home assistant) entities defined in room_configs
  % it helps to maintain the abstraction layer between logical / physical names and home assistant names

    % Initialize an empty cell array to hold the mapped entities
    entities = cell(size(casadi_entities));

    % Iterate over the elements in casadi_entities
    for i = 1:length(casadi_entities)
        obs_name = casadi_entities{i};

        % Check if the current control corresponds to a common entity
        if isfield(room_configs{end}, obs_name)
            entities{i} = room_configs{end}.(obs_name);
        else
            % Handle room-specific entities
            % Extract base name (e.g., 'tairmeas' from 'tairmeas_1')
            parts = strsplit(obs_name, '_');
            base_name = parts{1};

            if length(parts) == 1
                % No suffix means the first room
                entities{i} = room_configs{1}.(base_name);
            else
                % Suffix gives the room index
                room_index = str2double(parts{2});
                entities{i} = room_configs{room_index + 1}.(base_name);
            end
        end
    end
end
