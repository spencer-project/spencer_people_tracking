Mock perception components of the SPENCER platform
--------------------------------------------------

The scripts in this package publish dummy data for (almost) all perception components of the SPENCER platform.
This data can be used for easy testing as long as not all components have been implemented, or just to save
time compiling complex components that are not really needed for your work.

Prerequisites
-------------
The SciPy package needs to be installed for all scripts to run:

    sudo apt-get install python-scipy
    
Usage
-----
Run on the command line:

    roslaunch spencer_perception_mocks all.launch


Optionally, certain mock components can be disabled by setting them to "false" on the command line, e.g.

    roslaunch spencer_perception_mocks all.launch spokespersons:=false


See all.launch for details.
