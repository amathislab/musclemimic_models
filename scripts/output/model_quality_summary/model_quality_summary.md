# Three-model curve-quality summary

| Model | Maximum normalized FD jump | Maximum normalized MA jump | FD jumps | MA jumps | Mean bilateral NRMSE among asymmetric pairs | Bilateral asymmetry |
|---|---:|---:|---:|---:|---:|---:|
| MyoFullBody | 0 | 0 | 0.00% (0/476) | 0.00% (0/476) | NA (0 pairs) | 0.00% (0/238) |
| myo_sim | 1.31119 | 0.909956 | 4.62% (22/476) | 4.62% (22/476) | 0.295318 (38 pairs) | 15.97% (38/238) |
| MS-Human-700 | 1.40315 | 0.997128 | 2.95% (16/542) | 3.32% (18/542) | 0.50907 (4 pairs) | 1.48% (4/271) |

## Definitions

Curves were sampled at 201 evenly spaced points over each joint range, giving a normalized joint-position interval of 1/200 (0.5% of the range). FD is the two-dimensional force–length curve; length and force are each min-max normalized before adjacent Euclidean distances are calculated. MA is independently min-max normalized before adjacent differences are calculated.

The jump detector follows msk_metrics.tendon_jump_max: its EMA starts at the first step and is updated with α = 0.01; a step is flagged when it exceeds max(10 × EMA, 0.01). Maximum jump columns contain the largest flagged event, or zero when none is detected. Percentages use the model's mapped, spanning muscle–joint curves as the denominator.

Each preliminary event is resampled at twice the resolution. If the refined maximum step is less than 0.75 times the original, it is classified as a steep continuous curve rather than a discontinuity. Smooth changes approach a ratio of 0.5; true jumps approach 1.0.

Before every MuJoCo forward pass, active joint equality constraints are evaluated explicitly: each dependent joint is set from its master joint using the compiled equality polynomial.

Bilateral NRMSE is the root mean square of the normalized left/right errors across moment arm, MTU length, and force, with each channel normalized by its pooled bilateral range. A bilateral pair is asymmetric when NRMSE > 0.1. The table averages NRMSE only across those asymmetric pairs.
