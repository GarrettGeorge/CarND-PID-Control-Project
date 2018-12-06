## Reflection

### Effect of P, I, and D terms in PID algorithm

- P | Proportional : This term changes the output based on how large the `cte` is. In the case of a car being a distance `d` from the center line of lane, the `p` term will pull the car towards the line *with* overshooting. This overshooting will lead to a stable oscillation pattern.
- D | Derivative : This term changes the output based on how large the *change* to `cte` is. This term aims to dampen the oscillation from the `p` term.
- I | Integral : Without an `i` term there could exist an implicit bias in the steering angle due to an alignment issue with the wheels. The `i` term then sums up the `cte` over time such that an native bias can be reduce and ideally eliminated. 

These theoretical statments are verified in my experience with self-tuning the hyperparameters. When only setting `p` and `i`, a oscillation would develop and eventually push the car out of the driving region.

### Choosing P, I, and D values

Using the primary motivation behind **Ziegler-Nichols** method, I started with `p` in order to get a stable oscillation. After keeping the oscillation within the drivable region, I guess and checked with the `d` term in order to reduce the amplitude of the oscillation. From there I utilized a small `i` term to help with the intial placement of the vehicle on simulation load. 