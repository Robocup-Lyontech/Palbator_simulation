Overview
+++++++++++++++++++++

- Manages the HSR-B gazebo configuration files and launch files.

Management files
++++++++++++

launch
^^^^^^

Manages the simulator launch files.

* Start the simulator using controllers, sensors, and an empty environment.

.. code-block:: bash

                $ roslaunch hsrb_gazebo_bringup hsrb_empty_world.launch

* Start the simulator using gain adjustment controllers, sensors, and an empty environment.

.. code-block:: bash

                $ roslaunch hsrb_gazebo_bringup hsrb_gain_tuning.launch

params
^^^^^^

Manages the controller and sensor configuration files as well as the PID gain configuration files.

LICENSE
+++++++++

This software is released under the BSD 3-Clause Clear License, see LICENSE.txt.
