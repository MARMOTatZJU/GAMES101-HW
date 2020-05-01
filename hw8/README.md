Legenda:
- blue: explicit euler (may be overlapped with red rope)
- red: semi implicit euler
- green: verlet

Completed tasks:

- euler method (images/euler_comparison.jpg)
  - Explicit Euler simulation (blue)
    - diverge after several minutes
  - Semi Implicit Euler simulation (red)
    - do not diverge, but will not stop trembling
  - global damping
  - internal damping
- verlet method
  - gloal damping

Observation:
- internal damping works excelently to stabilize euler simulation
  - even explicit euler will be stabilized after introducing internal damping.
  - however, internal damping will no stop rope from oscillating (thus global damping is needed)
- under settings with both internal damping and global damping, explicit euler will get very close simulation result with semi implicit euler
  - tow ropes are totally overlapped

P.S.
- The formula for explicit Euler method seems to have a wrong sign. The substraction sign may be needed to removed? 
  - After removing it, my simulation is able to run normaly.
- A red rope has been added to simulate semi implicit euler and compare it with explicit euler.
  - void simulateSemiImplicitEuler(...);
