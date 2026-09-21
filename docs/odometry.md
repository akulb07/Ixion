# Encoder odometry

Given cumulative encoder differences `dl_ticks, dr_ticks` and resolution N,
wheel angle increments are `2*pi*delta_ticks/N`. Wheel distances are radius times
these increments. Body displacement is `(dl+dr)/2`; heading change is
`(dr-dl)/wheel_separation`. The SE(2) exponential integrates the corresponding
constant twist using the stable midpoint/sinc implementation in DifferentialDrive.

EncoderOdometry consumes only EncoderReading contracts, geometry calibration, and
an explicit initial pose prior. It cannot inspect ground truth. The first complete
pair establishes a count baseline and produces the supplied pose. Later complete
pairs produce capture-time pose and interval-average twist estimates. Incomplete
pairs produce None; the next complete pair bridges the whole gap. Different motion
segments within a gap cannot be reconstructed from cumulative counts alone.

Streams require strictly increasing capture times and sequences and fixed sensor,
frame and resolution. Counters are unbounded integers; hardware adapters must unwrap
rollover and handle resets. Delivery latency does not alter the historical estimate
timestamp. Online consumers must wait for delivery themselves. No covariance or
global correction is claimed. Slip, calibration errors and noise accumulate drift.
The distance field accumulates absolute center travel under the interval model;
it cannot recover reversals hidden between measurements.
