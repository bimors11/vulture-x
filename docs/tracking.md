# Tracking

The synthetic source produces deterministic BGR frames using a configurable
seed, resolution, frame rate, target shape/size/velocity, background, Gaussian
noise, disappearance interval, sudden movement, and extra delay.

`OpenCvTracker` wraps OpenCV CSRT or KCF. It supports explicit initialization,
update, loss reporting, bounding-box validation, and reset. Synthetic tests use
the generator's known initial bounding box; a future interactive application
will provide manual ROI selection.

OpenCV's classic tracker API returns success and a bounding box but no calibrated
confidence score. The first milestone therefore maps a valid successful update
to confidence `1.0` and a failed/invalid update to `0.0`. This is an explicit
initial heuristic, not a probabilistic confidence estimate.

Normalized errors use `-1` at the left/top edge, `0` at image center, and `+1`
at the right/bottom edge.

