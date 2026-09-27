# 0.18 checks

- Full Python source suite: 273 tests and 157 subtests passed.
- Built and installed the 0.18.0 wheel in a fresh folder. All 18 API/planning
  checks passed against that installed package.
- TypeScript check, production frontend build and three numerical tests passed.
- Ruff lint and formatting checks passed for the planning/API code.
- Browser: A* found a 7.686 m path with 24 waypoints, 103 expansions and 305
  collision checks. Changing a goal cleared the old result; a goal crossing the
  boundary returned `invalid_goal`.
- The wheel-slip run finished with 501 saved states, 2.65 m traveled and zero
  collisions. Playback reached 10.00 s; final pose was (1.790, 1.701) m and
  -35.3 degrees. The fault interval was visible in the inspector.
- Setup tabs worked with mouse and keyboard. Activity navigation focused the
  requested panel. No browser console errors or warnings were recorded.
- At 390 px viewport width the document was 380 px wide, with no horizontal
  overflow. The panels stack on narrow screens.

The source suite still emits the Starlette/HTTPX test-client deprecation warning.
The XML report is in `../validation-milestone-18/source.xml` relative to the repo
root. The desktop screenshot and saved demo are in `../demo-milestone-18/`.

The planners use a known, static map. This release doesn't execute a planned
path or add navigation scenarios to the simulation queue yet.
