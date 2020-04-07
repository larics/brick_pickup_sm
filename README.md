# Brick Pickup State Machine

A collection of state machines for a UAV brick pickup scenario.

## State Machine Structure

Overall structure consits of four hierarchicaly connected state machines.

* **Master State Machine**
  * OFF
  * SEARCH - e.g. search a global area with a predefined pattern
  * ACTION - e.g. pickup and deliver brick
* **Global State Machine**
  * OFF
  * APPROACH
  * SEARCH - search local area around target global position
  * ATTEMPT_PICKUP
  * DROPOFF
* **Visual Servo State Machine**
  * OFF
  * BRICK_ALIGNMENT
  * DESCENT
  * TOUCHDOWN_ALIGNMENT
  * TOUCHDOWN
