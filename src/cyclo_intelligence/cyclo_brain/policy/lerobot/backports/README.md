# FastWAM build-time backport

LeRobot stays pinned to `c8ce413d738da15a2eed2d0832315779ea28cbf9` (0.5.2).
The source checkout and other policies are not replaced.

`fastwam/` comes from LeRobot commit
`240b4a0314ae0879cdd928c7f4bdc1eee9a01b3b`, the commit referenced by the
baseline Cyclo `docs/fastwam-inference.md`.
`upstream_sha256.json` records the original files; the upstream README symlink
is materialized from `docs/source/policy_fastwam_README.md`. The only package adaptation
is the optional `FastWAMConfig.pretrained_revision` field, which 0.5.2's base
configuration does not have but newer checkpoints may serialize.

During a LeRobot image build, the Dockerfile copies this package and applies
`fastwam-c8ce413.patch` to register its configuration, lazy model loader,
processors and dependency extra. Existing dependency version ranges remain
unchanged. Patch application is checked before installation; a conflicting
source version fails the build rather than being silently rewritten.

This restores the missing source integration, not a validated trained model.
Checkpoint loading, GPU memory/offload behavior, training and robot execution
still require model-specific validation. Native T-Rex and the 1044 hand driver
are separate requirements and are not provided by this backport.
