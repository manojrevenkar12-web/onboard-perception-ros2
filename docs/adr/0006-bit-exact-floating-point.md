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

## What the check found

The first arm64 run in CI failed the fingerprint test. The floating point was fine. The
simulator drew two random numbers inside one call's argument list
(`add(0.2 * w + jitter(20), 0.45 * h + jitter(20), ...)`). C++ does not specify the order in which
a call's arguments are evaluated, and GCC evaluates them in different orders on x86 and aarch64.
Every flight therefore started its targets in different places on the two machines. Each draw is
now its own statement, and x86 and arm64 render identical frames
(checked locally with an aarch64 cross compiler under qemu, and in CI). The goldens and the
tracking report were regenerated from visionbox in the same commit.

The lesson for vehicle code: "same compiler flags" is not "same program". Order of evaluation,
`char` signedness and FMA contraction all differ between the desk and the Jetson, and only running
the tests on the target architecture shows which of them a codebase depends on.
