# One-sentence-summary
Control of underfloor heating in multiple rooms using a model predictive control approach

# Why

1. I want a floor heating which can heat up quickly and keeps my temperature relatively stable.
Standard control of heating systems seems to me mediocre at best, especially for heating systems with higher thermal inertia like underfloor heating.
In my gas heater I can choose between:
   * heating up the room quickly, followed by completely overshooting its desired temperature, followed by some period of long cooling which doesn't feel enjoyable etc.
   * heating up slowly and waiting forever till a room temperature is reached.

    To me it's clear the approaches from (gas) heater manufacturers using heat curves probably exist from ancient days where controls engineering was a niche and microcontrollers were barely available. But we don't live in the 90's anymore ... .
    More as a rant but people claiming underfloor heating should always be kept on, are just waisting useless energy and/or are lazy to tune their heating systems right.

2. After installation of underfloor heating I wanted to be able to control individual room temperatures out of 1 water collector. This while guaranteeing the valve positions of the individual floor circuits don't create conflicts with the pump / heating requests to the gas boiler. e.g. Always 1 circuit should allow water to flow when requesting heat from the boiler. Also valves should be closed to avoid waisting energy if no water is flowing etc.
3. Unfortunately proportional valves for underfloor heating don't seem to exist today (?), valves are only on/off. This makes things more complicated.
  

# What 
* Model predictive control is very suited to solve the problem: 
  * it works elegantly with constraints (limited valve-boiler combinations), multiple inputs, coupled systems (1 water temperature for all).
  I'd rather avoid creating of a bunch of if-else-logic constructions where always some corner case is missed and is harder to scale to multiple rooms.
  * allows to look ahead and see when you will reach a certain air temperature if the floor was warm for a while.
* Casadi is used as optimization tool
  * Highly flexible tool which allows to generate c-code to run on most single board platforms (without requiring typical  pc-tools)
  * The design of the model predictive control is done in the free tool octave
* Beside octave & casadi, the controller interfaces with:
  * home assistant to get all relevant sensor data and set the setpoints (room temperatures, thermostat setpoints, outdoor temp, valve positions, ... )
  * The ebus project [https://ebusd.eu/] is used to - also through home assistant - interface with the gas heater and its according thermostats. It allows to send "virtual" setpoints to the thermostat - i.e. setting the thermostat temperature every x minutes. This allows to get the desired water temperature out of the gas heater and become quite independent from any manufacturer controls ... . No weird hacking of thermostats or boiler is needed, only an extra device plugged into the communication bus of the heater.
* Model predictive control relies to have a model of your environment. This is however - deliberatly - kept very simple (two node thermal network with an observer to correct model inaccuracies).

# Status
Running now for quite some weeks.
  




