function plotoptimizer(optimized_watersetpoints,optimized_valvesetpoints,optimized_tfloor,optimized_tair,airtemp_setpoints,num_rooms)

%figure
%clf
%
%hold on
%hax1=subplot(4,1,1);
%hold on
%plot(optimized_watersetpoints,'*','DisplayName','optimal water setpoint')
%pause(1)
%legend
%grid minor
%
%hax3=subplot(4,1,2);
%hold on
%for i=1:num_rooms
%  plot(optimized_tfloor(:,i),'*','DisplayName',['floor temp prediction',num2str(i)])
%pause(1)
%end
%legend
%grid minor
%
%hax4=subplot(4,1,3);
%hold on
%for i=1:num_rooms
%  plot(optimized_tair(:,i),'*','DisplayName',['air temp prediction',num2str(i)])
%  pause(1)
%  plot(ones(size(optimized_tair(:,i)))*airtemp_setpoints(i),'DisplayName',['temp setpoint',num2str(i)])
%  pause(0.5)
%end
%
%legend
%grid minor
%
%hax2=subplot(4,1,4);
%hold on
%pause(1)
%for i=1:num_rooms
%plot(optimized_valvesetpoints(:,i),'*','DisplayName',strcat('optimal valve setpoint',num2str(i)))
%pause(1)
%end
%legend
%grid minor
%
%linkaxes ([hax1, hax2,hax3,hax4],"x");


% function plotoptimizer(optimized_watersetpoints, optimized_valvesetpoints, optimized_tfloor, optimized_tair, airtemp_setpoints, num_rooms)
    figure
    clf

    % Water setpoints subplot
    hax1 = subplot(2 + num_rooms, 1, 1);
    hold on
    plot(optimized_watersetpoints, '*', 'DisplayName', 'optimal water setpoint')
    pause(1)
    legend
    grid minor
    title('Water Setpoints')

    % Floor and air temperature subplots for each room
    haxes = zeros(num_rooms + 2, 1);  % Array to store all axis handles
    haxes(1) = hax1;

    for i = 1:num_rooms
        hax_room = subplot(2 + num_rooms, 1, i + 1);
        haxes(i + 1) = hax_room;
        hold on

        % Plot floor temperature
        plot(optimized_tfloor(:,i), '*', 'DisplayName', ['floor temp ' num2str(i)])
        pause(1)

        % Plot air temperature and setpoint
        plot(optimized_tair(:,i), '*', 'DisplayName', ['air temp ' num2str(i)])
        pause(1)
        plot(ones(size(optimized_tair(:,i))) * airtemp_setpoints(i), ...
             'DisplayName', ['temp setpoint ' num2str(i)])
        pause(0.5)

        legend
        grid minor
        title(['Room ' num2str(i) ' Temperatures'])
    end

    % Valve setpoints subplot
    hax_valve = subplot(2 + num_rooms, 1, 2 + num_rooms);
    haxes(end) = hax_valve;
    hold on

    pause(1)
    for i = 1:num_rooms
        plot(optimized_valvesetpoints(:,i), '*', ...
             'DisplayName', ['valve setpoint ' num2str(i)])
        pause(1)
    end
    legend
    grid minor
    title('Valve Setpoints')

    % Link all axes
    linkaxes(haxes, 'x')
end
