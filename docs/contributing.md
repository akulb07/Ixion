# Contributing

1. Read the architecture and relevant mathematical model before modifying it.
2. Explain equations, units, inputs, outputs, assumptions and failure cases.
3. Keep pure geometry/kinematics independent of simulation, IO and visualization.
4. Use type hints, small functions, immutable values and explicit dependencies.
5. Add independent numerical tests and a reproducible example for a new model.
6. Run `python -m pytest` and the appropriate demo; inspect any generated plots.
7. Update assumptions, interfaces and limitations in documentation.
8. Commit coherent changes; keep generated runs, environments and build caches
   out of Git. Never commit private data or large generated datasets by default.

`pytest` is the standard runner. Use `python -m pip install -e ".[dev,plot]"`
inside a virtual environment. Code targets Python 3.12+. A future frontend must
consume the Python core's results rather than implement a second robotics model.

Work stops after the current milestone's validation report. Starting the next
milestone requires explicit instruction from the project owner.
