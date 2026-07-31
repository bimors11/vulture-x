# Tracking

The synthetic source produces deterministic BGR frames using a configurable
seed, resolution, frame rate, target shape/size/velocity, background, Gaussian
noise, disappearance interval, sudden movement, and extra delay.

`OpenCvTracker` wraps OpenCV CSRT, KCF, or the `TEMPLATE` matcher adapted from
the fixed-wing visual-nav prototype. It supports explicit initialization,
update, loss reporting, bounding-box validation, and reset. Synthetic tests use
the generator's known initial bounding box; the SITL UI can provide manual ROI
selection for local simulation helpers.

The `TEMPLATE` matcher searches near the last bounding box with OpenCV template
matching, rejects matches below a configured score, and slowly updates its
template after successful locks. It uses the same local-search behavior as the
prototype, with an added low-variance path for solid-color synthetic markers.

OpenCV's classic tracker API and the local template matcher do not provide a
calibrated confidence score. The first milestone therefore maps a valid
successful update to confidence `1.0` and a failed/invalid update to `0.0`.
This is an explicit initial heuristic, not a probabilistic confidence estimate.

Normalized errors use `-1` at the left/top edge, `0` at image center, and `+1`
at the right/bottom edge.

Machine-readable tracking failure reasons used by the SITL/UI helper path
include `missing_custom_selection_file`, `custom selection is missing; select a
target in the UI first`, `OpenCV rejected the custom target selection`, and
`target_not_detected`. The unresolved safety constraint remains unchanged:
image size and template lock do not prove physical range or separation.
