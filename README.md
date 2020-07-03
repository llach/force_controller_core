# Force Controller Template

This repoistory contains platform-independent code for a Force Trajecotry Controller. It switches between trajectory execution and force control based on platform specific conditions. Controller state transitions and force control are implemented in a generic way.

### Algorithm

Currently, `init()` and `update()` of a ROS controller are implemented. During `update()`, several calls are made to platform-specific methods (pink).

**`update_sensors()`**

Fetch new force data and store it in `forces_`. Most sensors are platform-specific or might require some additional calculations.

**`check_controller_transition()`**

Returns `true` if all conditions to enter force control mode are met. For parallel grippers this can be as simple as measuring force in both sensors, while for hands things like opposing fingers might be interesting to consider as well.

**`publish_debug_info()`**

Allows for publishing debug info after desired positions are updated.

**`force_finished()`**

Callback for when force control mode finished successfully.

![pseudocode](doc/pseudo.jpg "")