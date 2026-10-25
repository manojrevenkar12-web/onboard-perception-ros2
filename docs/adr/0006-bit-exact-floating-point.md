# 0006. Bit-exact results on x86 and aarch64

Status: accepted

## Context

Equivalence is checked on developer machines and CI (x86-64) but the code flies on a Jetson
(aarch64). GCC fuses `a*b+c` into one fused multiply-add by default on aarch64, which changes the
last bit of results and, through the tracker's gating, occasionally a decision.

## Decision

- All perception code and the simulator build with `-ffp-contract=off`.
- The simulator uses only correctly rounded operations (+, -, *, /, sqrt, floor), no `sin`/`exp`
  from libm, whose results differ between C libraries.
- `test_scene` pins a fingerprint of one rendered frame; CI rebuilds the core under arm64
  emulation and runs the equivalence tests there.

## Consequences

Small loss of speed from unfused arithmetic. In exchange, "same output on the vehicle as on the
desk" is a tested property, not an assumption.
