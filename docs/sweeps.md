# Small experiment sweeps

The chart icon in the activity bar opens the sweep workspace. It uses the current
draft as the base setup. To sweep a saved navigation run, open it, click **Use
setup**, then open sweeps.

For a first try, leave `robot.wheel_radius` as the parameter, enter
`0.04, 0.05, 0.06` as values, and use seeds `42, 43`. Preview shows six trials.
Nothing runs until **Run sweep** is clicked. Remove the parameter for seed-only
trials, or add a second parameter for a Cartesian product of values.

This changes the actual robot configuration. Sweeping physical wheel radius is
not the same as giving an estimator the wrong calibration. Parameter paths refer
to the full resolved config; values use that field's units. The form accepts
numeric values. The API also accepts other finite JSON values.

Invalid individual values stay in the design with their validation errors and
aren't simulated. A misspelled parameter path rejects the design. Each valid
trial gets its own normal run ID, config hash, replay and exports. Trials are
submitted one at a time through the existing run queue. An unrelated single run
can still use the queue. Only one batch can be active at once.

Cancelling stops the current trial cooperatively and marks unstarted trials as
cancelled. Completed trials remain available. A server restart marks unfinished
batches interrupted; it never silently reruns them. A submitted child run that
finished before the restart keeps its result.

## Reading the result

The progress count includes completed, failed, invalid, cancelled and interrupted
trials. A batch marked completed means all its trials were processed, not that
every robot reached its goal.

Group summaries report finished counts, failure fraction and its Wilson 95%
interval. Failure means status other than completed, including collision and
budget exhaustion. Metric mean, sample standard deviation, median and 5th/95th
percentiles use every available measurement, including measured failure runs.
Measured counts are included; a single measurement has no sample standard
deviation. An empty cell is unavailable, not zero. These statistics do not correct
for model error, correlated seeds or selection bias. Use enough independent trials
for the conclusion being tested.

**Report** downloads the current snapshot, with trial statuses, run IDs, config
hashes and group statistics. **Design** downloads the batch specification.
**Resolved inputs** includes each exact valid config and the attempted inputs
for invalid trials, in trial order. All three are saved locally; each submitted
run also keeps its usual integrity manifest and artifacts.

## API and limits

- `POST /api/experiments/preview`: resolve and validate without creating jobs.
- `POST /api/experiments`: submit the same specification, returning 202 and an ID.
- `GET /api/experiments?offset=0&limit=20`: saved batch history.
- `GET /api/experiments/{id}`: progress, trials and group statistics.
- `POST /api/experiments/{id}/cancel`: cooperative cancellation.
- `GET /api/experiments/{id}/report`: download a report snapshot.
- `GET /api/experiments/{id}/artifacts/{filename}`: design, inputs or batch record.

The specification uses the existing CLI experiment format: `name`, `base`,
`seeds`, `axes`, `max_runs`, `max_total_steps`. An axis has a `path` and `values`.
The API permits 32 trials, 8 distinct JavaScript-safe nonnegative seeds, 2 axes,
100,000 total steps, 1,800 total simulated seconds, 500,000 estimated readings
and 2 million LiDAR rays. Every valid trial also passes the ordinary run limits.
These bound work, not elapsed runtime. Unknown paths and excessive work return
422; an already active batch returns 429. The 1 MB body limit and local-origin
checks apply. Storage sits under the run service's `batches` directory and shares
its process lock.
