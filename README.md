This is a repo for ce420 group project for creating an rc plane black box.


Project Proposal:

Sensor Array Microcomputers Project
The project which my group would like to tackle will be a sensor array.
The need for this project arises from the SAE Aero team. Currently, the team does not
know where to begin with expected flight parameters. This is because the current flight system
does not track any data. This data would revolutionize the way the team can design parts, as real
world testing is the best way for any parameters to be established. A sensor array could help us
collect all of this data, an example being testing the speed and altitude of the plane along the
designated flight path. With these readings for speed and altitude, we can redesign the wings to
suit the environment, and we can also minimize material wastage as once we establish the
environment along the flight path, we can enter those parameters into simulation software and no
longer have to test with real flights.
This sensor array will be able to measure a variety of data, possibly including
temperature, altitude, air pressure, velocity, acceleration, jerk, stall conditions, RPM, Voltage
readings, and GPS data. The intended microcontroller will vary based on what needs arise, and
there may be multiple different types of MCUs. Our group will be focusing on a set of three
different microcontrollers to choose from. If we need robust wireless communications, an ESP32
or similar device will help ensure that we can reliably communicate between our different
devices. If weight savings or size become bigger concerns, the project may shift to using either
STM32 MCUs or a RP2350. These chips are both relatively easy to produce a custom PCB with,

so these chips would be a better choice in situations where a custom PCB is likely. For
prototyping purposes however, we are sticking with the ESP32 for the course of this project due
to its wireless integration.
This solution is different from currently available options as it is mostly decoupled from
the actual flight system. While the possibility of adding gyroscopic control to the plane is nice, it
can be done through much simpler and easier means. This sensor array should act purely as a
data acquisition device. This means that it may be used elsewhere, outside of SAE Aero. The
goal behind this project is that in any scenario in which data is needed, this sensor array could
simply be dropped in with no complicated re-configuration. We would also like to get this sensor
array to communicate with a base station in order to get more immediate data readouts.
