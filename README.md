Initial setup
=============

    1) mkdir /path/to/workspace/
    2) cd /path/to/workspace/; qibuild init
    3) cp -R /path/to/sdk /path/to/workspace/sdk
    4) qitoolchain create mytoolchain /path/to/workspace/sdk/toolchain.xml
    5) qibuild add-config mytoolchain -t mytoolchain --default
    6) cd /path/to/workspace/; git clone <controller-pepper>


Compilation
===========

If you followed instructions above and your cross-compilation toolchain is
named "mytoolchain" then:

    - Type '`make controller-pepper`' inside the root directory of the project.

If your toolchain has a different name:

    - Type '`make controller-pepper TC=name-of-your-toolchain`'.

Inside cmake input script pepper_controller.cmake located inside cmake
directory, user can enable or disable options for using hot-starting and
logging as well as specify the desired solver to be used.

Installation and usage
======================
From '`build-<toolchain-name>/sdk/lib/naoqi`' folder copy '`.so`' using scp
onto robot. Declare your module in the 'autoload.ini' as follows:
    [user]
        /path/to/libmymodule.so

Directory '`<controller-pepper>/config/`' contains configuration files needed by the 
controller as well as sample configuration files with motion parameters.
All files within this directory shall be copied onto the robot inside the directory
'`$HOME/config-pepper/`'. Path of the directory with configuration files can
be changed, however this change needs to be reflected in the PepperController 
class contructor.

Use '`move.py`' script to send commands to the module. By
default the script connects to the ip adress ('`10.42.0.61`'). The IP address of
the robot can be specified using '`-b 11.22.33.44`', port using '`-p 9559`'. The
default port is 9559. Last parameter is the yaml configuration file with
the desired motion specified as '`-c name-of-the-file`'. Examples of 
configuration files with motion parameters are in the '`<controller-pepper>/config/`'
directory. Before using, inside the script one should modify parameters such as desired joints stiffness, 
time to reach initial and rest position to their liking.

Script '`go_and_log.py`' can be used as en example when one wants to log 
some values from the robot. It has the same command-line arguments as the 
aforementioned script for setting the ip and port. Additionally, one can 
use the option '`-n`' in which case robot's arms will be disabled.

It is also convenient to work with the controller in a interactive manner (ex.
using '`ipython`').


Known issues in Aldebaran's software
====================================

1) cross-config.cmake
    '`CMAKE_CXX_FLAGS`' is overriden by '`CMAKE_C_FLAGS`'
    Workaround:
    copy all C++ flags to '`CMAKE_C_FLAGS`' in '`CMakeLists.txt`'

2) boost/share/cmake/boost/boost-config.cmake
    '`BOOST_INCLUDE_DIRS`' is defined instead of '`Boost_INCLUDE_DIRS`'
    Workaround: processing of '`boost-config.cmake`' by '`find_package(Boost)`'
    can be disabed with `'Boost_NO_BOOST_CMAKE`'.

3) When using function '`setActuators()`' from inside '`python`' code (ex.
    '`call_peppercontroller`') to set joints to a given configuration make sure
    all numbers have a floating point (ex. 0.0 instead of 0). Otherwise the
    commands are not executed properly.

4) Although '`synchronisedDCMCallback()`' is supposed to be invoked every 10ms,
    it is actually invoked with a variable time interval (typically 11ms, 12ms,
    13ms). This poses certain difficulties for control of the wheels using
    velocities. Therefore, we perform correction of the state of the model
    taking into account the measured time difference between subsequent
    function calls.
