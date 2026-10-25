# 0001. Characterise the legacy program before changing it; byte equality is the acceptance test

Status: accepted

## Context

visionbox 2.3 has flown for years. Its behaviour is the specification nobody wrote down: which
blobs become tracks, when a track coasts, what the ground station receives. A migration that
"works" but changes those decisions is a new tracker, and needs its own field validation.

## Decision

- Before touching any code, replay recorded flights through visionbox and keep its telemetry
  (`src/opr_core/test/golden/`, 7 scenarios x 3 seeds, 3,150 datagrams).
- The migration is accepted only if the new code produces the same bytes on every one of them,
  at three levels: the extracted library (`test_equivalence`), all nodes in one container, and
  the deployment split into three processes with shared-memory transport (launch tests).
- visionbox.cpp is frozen (`FROZEN` pins its hash). CI rebuilds it, regenerates the goldens and
  fails if they differ from the committed ones, so the reference cannot drift quietly.
- Behaviour changes (fixes to findings in `docs/MIGRATION.md`) come after equivalence, one at a
  time, each with its own regenerated goldens and a reason.

## Consequences

- Refactoring is constrained: arithmetic keeps visionbox's operation order. That is the price of
  being able to say "nothing changed" with evidence instead of confidence.
- Known weaknesses of the old tracker (ghost tracks while two targets overlap) are deliberately
  kept until the migration is done, so they are fixed as a visible change, not a side effect.
