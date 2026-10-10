# DC gearmotor model

The first motor model estimates an operating point at a specified output speed:

```sh
ixion motor-point examples/esp32_diff_drive/robot.yaml --component left_motor --voltage 6 --rpm 150 --load-torque 0.1
```

For the illustrative reference values this gives about 0.95 A, 0.125 N m output
torque and 0.025 N m after subtracting the specified load. These are model
estimates, not measurements of a particular N20 motor. JSON output includes the
input evidence/references and derived parameters. RPM means gearbox output RPM.

## Equations and conventions

The model uses the standard brushed DC motor relations described in the
[University of Michigan motor tutorial](https://ctms.engin.umich.edu/CTMS/?example=MotorSpeed&section=SystemModeling).
Electrical inductance is neglected here, so current settles instantaneously:

```text
rotor speed = output speed * gear ratio
back EMF = K * rotor speed
current = (terminal voltage - back EMF) / resistance
electromagnetic torque = K * current
rotor shaft torque = electromagnetic torque - b * rotor speed
```

K is both torque constant (N m/A) and back-EMF constant (V s/rad), enforcing their
SI equality. The direct `DCMotor` API accepts resistance, K, viscous friction b,
gear ratio and efficiency. All public numerical inputs must be finite.

In motoring, output torque is rotor shaft torque times ratio times efficiency.
When backdriven, it is rotor shaft torque times ratio divided by efficiency.
This represents the extra input power needed to overcome gearbox losses in reverse
power flow. It assumes a backdrivable gearbox, not a self-locking worm drive.
Efficiency is constant; using it at stall is an approximate torque-transfer fit.

Positive electrical power enters the motor. Positive mechanical power leaves it.
Reported copper, viscous-friction and gearbox losses are nonnegative and obey:
electrical power = output mechanical power + losses. Negative electrical power
indicates idealized generation; it does not establish battery/driver acceptance.

Zero voltage means shorted terminals. A moving shorted motor develops braking
current. Open-circuit coasting needs a driver connection mode and is not modeled
by commanding zero volts. Load torque is signed in the output-axis coordinate and
subtracted from available torque; it is not automatically reversed with speed.

## Fitting common datasheet values

`MotorDatasheet` takes nominal voltage V, output no-load RPM, no-load current I0,
stall current Is, output stall torque Ts and gear ratio G. The fit is:

```text
R = V / Is
w0 = output_no_load_rpm * 2*pi/60 * G
K = (V - I0*R) / w0
b = K*I0 / w0
effective gear efficiency = Ts / (K*Is*G)
```

No-load current must be less than stall current. A fit requiring efficiency above
one is rejected. Check rotor-versus-output units and consistent test conditions
instead of clipping the efficiency. The fitted friction lumps losses into a
viscous term; it does not model brush friction, stiction or gearbox backlash.

There is no rotor inertia integration, inductive transient, thermal estimate,
current limiting, PWM ripple, battery sag, driver loss or wheel/contact coupling
yet. Current can exceed the nominal stall current during reverse-voltage braking;
the model does not silently clamp it. The CLI reports an estimate and preserves
the input assumptions. It does not decide whether the hardware is safe to use.
