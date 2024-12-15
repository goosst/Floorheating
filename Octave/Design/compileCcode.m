% compile code on specific hardware where you want to run the algorithms
% i.e. this script can be used to compile code withour relying on installation of casadi, if c-code was generated on an another device
clc
clear variables
close all

% path of casadi needed for including files
casadipath='/mnt/dietpi_userdata/FloorHeating/casadi-3.6.7-linux64-octave7.3.0';
%addpath('/home/stijn/Projects/Casadi/casadi-3.6.7-linux64-octave7.3.0')
%import casadi.*

cd('../Runtime')

str_include = strcat(casadipath,filesep,'include');
DIR = casadipath;

% compile c-code
%DIR=GlobalOptions.getCasadiPath();
%str_include = GlobalOptions.getCasadiIncludePath();
%str_compile=strcat('gcc -L',DIR,' -Wl,-rpath,',DIR,' -I',str_include,' optimal_control.c -lm -lipopt -o optimal_control')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% compile optimizer code
display('compiling c-code optimizer')
str_compile=strcat('gcc -I',str_include,' optimal_control.c -lm -lipopt -o optimal_control')
[status, output] = system(str_compile)


%test compiled code
state_value = [15; 15];  % Example values for state
outdoortemp_value = 6.0;   % Example value for outdoor temperature
setpoint_value = 21.0;      % Example value for the setpoint

% Prepare the input as a string
input_string = sprintf('%f\n%f\n%f\n%f\n', state_value(1), state_value(2),outdoortemp_value,setpoint_value);

% Use pipes to pass input
command = sprintf('echo "%s" | ./optimal_control optimal_control', input_string);
tic
[status, output] = system(command);
ELAPSED_TIME=toc

display('c-code has been running')
display(['solved in ',num2str(ELAPSED_TIME),' seconds'])
display(output)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% compile integrator / observer code
display('compiling c-code observer / integrator ')

%  str_compile=strcat('gcc -L',DIR,' -Wl,-rpath,',DIR,' -I',str_include,' integrator_c_code.c -lm -lipopt -o integrator_c_code')
  str_compile=strcat('gcc -I',str_include,' integrator_c_code.c -lm -lipopt -o integrator_c_code')
  system(str_compile)

  %Test compiled code
  % Prepare the input as a string
  watertempsetp=35;
  roomairtemp=18;
  valve_living=1;
  controlinputs=[watertempsetp;outdoortemp_value;roomairtemp;valve_living];

  %plant model parameters
  mCfloor = 3.1542e+06;
  mCair = 6.1871e+06;
  hair= 3.1215e+02;
  hisol= 5.1943e+01;
  kwfl= 4.0000e+02;

  %observer correction parameters
  L1 = 5.4348e-03
  L2 = 2.0929e-03
  parameters=[mCfloor;mCair;hair;hisol;kwfl;L1;L2];

  input_string = sprintf('%f\n', [state_value;parameters;controlinputs]);

  % Use pipes to pass input
  command = sprintf('echo "%s" | ./integrator_c_code intg', input_string);
  tic
  [status,output]=system(command)
  toc
  display('integrator/observer has been running')

  display(output)

