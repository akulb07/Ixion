# Release 0.16.0 validation

Validated on Windows with Python 3.13, FastAPI 0.141.1, Uvicorn 0.53.0,
Starlette 1.7.0, HTTPX 0.28.1, and the existing numerical dependencies.

- Source suite: 257 tests and 157 subtests passed.
- Installed-wheel suite, with source import path disabled: same results.
- Ruff lint and formatting checks passed; Git whitespace check passed.
- One non-failing dependency warning: Starlette deprecates its HTTPX test-client
  integration in favor of HTTPX2. Production serving does not use TestClient.
- Real loopback HTTP demonstration used the installed package: submitted the
  wheel-slip preset, polled completion, retrieved the frame at three seconds,
  confirmed the active fault and measurement delivery times, and downloaded
  the manifest. The server shut down after validation.
- Demo: 10 simulated seconds, 2.65 m traveled, zero collisions.

New checks cover cooperative cancellation, queue capacity, retained failures,
exclusive storage ownership, restart recovery, corrupted artifacts, workload
bounds, request-size bounds including streamed bodies, local origin/host checks,
API validation, frame delivery timing, and trajectory endpoint preservation.

Machine-readable test reports are in the sibling `validation-milestone-16`
output directory. HTTP evidence and saved run files are in `demo-milestone-16`.
The interactive workspace and navigation scenario API are the next milestone.
