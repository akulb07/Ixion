# Release 0.17.0 validation

- Python source and installed wheel: 258 tests and 157 subtests passed.
- Frontend: strict TypeScript check, production bundle, and three numerical tests passed.
- Ruff checks and formatting passed. The existing non-failing Starlette/HTTPX
  test-client deprecation warning remains.
- Browser checks: submitted the slip preset, inspected completed metrics,
  reopened history, selected the same run twice, played replay, scrubbed into an
  active fault, returned to the start, cloned a setup, and rejected invalid JSON
  without creating a run. Browser console inspection found no errors/warnings.
- Desktop layout inspected. Mobile breakpoint checked at 390 x 844; document
  width matched its available viewport width with no horizontal overflow.
- Installed package served all assets and completed the 10-second slip run:
  2.65 m traveled, zero collisions, 501 recorded states. Final pose rounded to
  (1.790, 1.701) m and -35.3 degrees.

Test XML is in the sibling `validation-milestone-17` directory; saved browser
demo runs and a workspace image are in `demo-milestone-17`. The screenshot is
actual rendered software, not the earlier concept image.

The shipped workspace is local and supports the existing simulation job type.
It does not claim navigation execution, SLAM display or world editing yet.
