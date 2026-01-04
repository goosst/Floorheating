function [expanded_valves_setp, expanded_entities_valves] = process_valve_configs(room_configs, valves_setp, entities_valves)
    % Initialize output cell array for expanded valve entities
    expanded_entities_valves = {};
    current_idx = 1;

    % Process each room that has a valve setpoint
    for i = 1:length(valves_setp)
        % Get the corresponding room config
        room = room_configs{i};

        % Add the primary valve
        expanded_entities_valves{current_idx} = getfield(room, 'valve');
        current_idx = current_idx + 1;

        % Check if there's a second valve defined
        if isfield(room, 'valve2')
            % Add the secondary valve and copy the setpoint
            expanded_entities_valves{current_idx} = getfield(room, 'valve2');
            % Insert the same setpoint value after the current position
            valves_setp = [valves_setp(1:i) valves_setp(i) valves_setp(i+1:end)];
            current_idx = current_idx + 1;
        end
    end

    % Convert cell array to the required format
    expanded_valves_setp = valves_setp;
    final_entities_valves = {};
    for i = 1:length(expanded_entities_valves)
        final_entities_valves{1,i} = expanded_entities_valves{i};
    end
    expanded_entities_valves = final_entities_valves;
end

% Example usage:
% valves_setp = [1 0];
% entities_valves = {
%     'switch.collector',
%     'switch.collector_3'
% };
%
% [new_valves_setp, new_entities_valves] = process_valve_configs(room_configs, valves_setp, entities_valves);
