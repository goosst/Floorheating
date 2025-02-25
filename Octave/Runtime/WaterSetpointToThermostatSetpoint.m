function ThermostatSetp=WaterSetpointToThermostatSetpoint(water_temp,outdoor_temp,heat_line)

% setting vaillant ecotec plus
% - turn off "binnencompensatie" on thermostat


% heat curve of 0.6 for a target room temperature of 20C
% column 1: out door temperature
% column 2: target flow
%heatcurve=[...
%-20,50
%-3,40
%11,30
%20,20];

% https://energy-stats.uk/vaillant-arotherm-weather-curve-information/
% Table for 20C target room temperature
heat_curve_table.outdoor_temps = [15.0, 10.0, 5.0, 0.0, -3.0];
heat_curve_table.heat_lines = [0.3, 0.35, 0.4, 0.45, 0.5, 0.6, 0.7, 0.8, 1.0, 1.2];
heat_curve_table.water_temps = [
    23.0, 24.0, 24.5, 25.0, 25.0, 26.0, 27.0, 29.0, 29.0, 30.0;
    25.0, 26.0, 27.0, 28.0, 29.0, 31.0, 32.5, 34.0, 36.0, 38.0;
    28.0, 29.0, 30.5, 31.5, 33.0, 35.0, 36.0, 38.0, 41.0, 45.0;
    30.0, 32.0, 33.5, 34.5, 36.0, 38.0, 40.0, 42.0, 46.0, 50.0;
    33.0, 34.0, 35.0, 36.5, 38.0, 40.0, 42.0, 44.0, 49.0, 54.0;
];

watertemp_setp_20C = interp2(heat_curve_table.heat_lines,heat_curve_table.outdoor_temps,  heat_curve_table.water_temps, heat_line,outdoor_temp,'spline');

% I believe to understand the correction is 5 degrees shifted per degree of thermomstat temperature setpoint compared to 20C
ThermostatSetp=(water_temp-watertemp_setp_20C)/5+20;
