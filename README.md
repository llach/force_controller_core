# Force Controller Template

[toc]

This repoistory contains platform- and Middleware-independent code for a Joint Force Controller. It maintaines an internal joint state and trajectory sample time. As this class is controlling one joint only, a  higher-level controller needs to manage the controller ensemble consisting of one controller instance per joint.

### Methods


**`update_joint_states()`**

Determines new joint state based on current force data

**`on_transition()`**

Is called once the higher-level controller decides to transition from trajectory to force control. Stores reference data from the time of transition.

**`calculate()`**

Performs force control calculations. Resulting new desired position and velocity can be accessed via `get_p_des()` and `get_v_des()`.

**`finish_iteration()`**

Stores data for next iteration.

**`reset_parameters()`**

Resets all controller parameters to their default values. A joint time can be given which will be stored as a trajectory sampling time offset.

### Usage

The high-level needs to call the methods in correct order. **`force_` is a pointer to the force value and the high-level needs to make sure that it points to the newest force data before performing force control calculations.**

These are the steps for the high-level:

1. Update forces
2. Call `update_joint_states()`
3. Decide whether to transition into force control. If so, call `on_transition()`
4. Calculate new desired state using `calculate()`. Retreive position and velocity via getters
5. At the end of each update loop, call `finish_iteration()`

The high-level itself can decide at which point the force control is terminated (e.g. some target force is reached) or if force control continues indefinitely. Before new goals, `reset_parameters()` should always be called to start with clean controller states.

Example usage can be found in the [ta11_controller](https://github.com/llach/ta11_sensor_tools/tree/master/controller) repository, where [ta11_controller_impl.h](https://github.com/llach/ta11_sensor_tools/blob/master/controller/include/ta11_controller_impl.h) is most interesting.