function [entity_data,hass_time,most_recent_states]=PostHomeAssistant(address_hass,auth_token,entity_id,value)
% I could not get it to work using webwrite in octav, so used curl and some system command


typeiswitch = strfind(entity_id, 'switch.');
typeisshellcommand = strfind(entity_id, 'shell_command.');

if ~isempty(typeiswitch)

          % Determine which API call to make based on the value
        if value >= 0.9
            service = 'turn_on';
        elseif value <= 1e-4
            service = 'turn_off';
        else
            display(value)
            error('Invalid value for switch. Use 1 to turn on or 0 to turn off.');

        end

        % Construct the API endpoint for switch services
        api_endpoint = sprintf('http://%s/api/services/switch/%s', address_hass, service);



        % Construct the curl command to call the API
        curl_cmd = sprintf(['curl -s "%s" ' ...
                            '-H "Authorization: Bearer %s" ' ...
                            '-H "Content-Type: application/json" ' ...
                            '-d ''{"entity_id": "%s"}'' '], ...
                            api_endpoint, auth_token, entity_id);

        % Execute the curl command
        [status, response] = system(curl_cmd);

elseif ~isempty(typeisshellcommand)

  commandedshell=strsplit(entity_id, '.');
  api_endpoint = sprintf('http://%s/api/services/shell_command/%s', address_hass, commandedshell{2});
  curl_cmd = sprintf(['curl -s -X POST --max-time %d "%s" ' ...
                    '-H "Authorization: Bearer %s" ' ...
                    '-H "Content-Type: application/json" ' ...
                    '-d "{}"'], ...
                    58,api_endpoint, auth_token);

  [status, response] = system(curl_cmd)

else

  api_endpoint = sprintf('%s/api/states/%s', ['http://',address_hass], entity_id);

  % Construct curl command
%  curl_cmd = sprintf(['curl "%s" ' ...
%                     '-H "Authorization: Bearer %s" ' ...
%                     '-H "Content-Type: application/json" ' ...
%                     '-d ''{"state":"%s","attributes":{}}'' '], ...
%                     api_endpoint, auth_token,num2str(value,"%5.2f"));

                         % Construct the curl command for numeric value

  unit= '°C';
  % -s silent option
  curl_cmd = sprintf(['curl -s "%s" ' ...
                       '-H "Authorization: Bearer %s" ' ...
                       '-H "Content-Type: application/json" ' ...
                       '-d ''{"state":%f,"attributes":{"unit_of_measurement":"%s"}}'' '], ...
                       api_endpoint, auth_token, value, unit);


  [status, response] = system(curl_cmd);


end

end

