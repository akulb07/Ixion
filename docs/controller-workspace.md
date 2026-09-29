# Tuning wheel feedback

Choose the PID feedback laboratory in the workspace. It requests 5 rad/s for
three seconds, then 2 rad/s for three seconds. Both motors have a 0.15 s time
constant and a 10 rad/s speed limit, so the controller has a lagged response to
work with. The preset starts with Kp 0.8, Ki 1, Kd 0 and feedforward gain 1.

Open Wheel feedback to edit each wheel's PID gains, output bounds, integral
limit and derivative filter. These are draft settings for the next run. Saved
results keep their original configuration. Turning feedback off returns to
open-loop commands; turning it back on starts with default PID settings.

After a run, Analyze controller shows the target, encoder measurement and PID
output for either wheel. PID contributions are in the expandable section.
The measurement is the encoder's average shaft speed over its capture interval;
the output is a requested wheel speed before motor delay, limits and slip. The
time axis is controller update time, including sensor delivery latency.

The table reports tracking RMSE, mean/peak absolute error, signed final error,
peak requested output and saturation percentage. These are weighted equally
per controller update. Saturation percentage is not a fraction of elapsed time,
and feedback error is not ground-truth robot motion error. Dropouts can leave
output held without producing another update. The reported interval makes that
coverage visible. No settling-time or stability guarantee is inferred.

`GET /api/runs/{id}/control?max_points=1000` exports the saved analysis. Plot
samples are bounded at 2,000; metrics always use all updates, up to 100,000.
Open-loop runs and runs without enough complete encoder samples return a clear
unavailable response. The analysis never reruns a controller with new gains.
Change gains, run again and keep both saved experiments for comparison.
