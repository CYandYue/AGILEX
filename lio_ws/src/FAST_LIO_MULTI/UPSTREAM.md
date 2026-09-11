# Upstream provenance

This directory is vendored source tracked directly by the AGILEX repository,
including `include/ikd-Tree`. It is not a Git submodule. Local adaptations are
already applied; do not apply the saved patch a second time.

| Component | Repository | Imported commit |
|---|---|---|
| FAST-LIO-MULTI | https://github.com/engcang/FAST_LIO_MULTI | `6c21072e2c12876d7f13697e9f3492fb460c107c` |
| ikd-Tree | https://github.com/engcang/ikd-Tree | `0438b0daa6ccfaaad9f65f45a3addc318b19ae7a` |

The original upstream submodule declaration is preserved as
`.gitmodules.upstream` for reference. Upstream licenses remain in `LICENSE` and
`include/ikd-Tree/LICENSE.md`, with other bundled notices retained in place.

The patch in `../../patches/fast_lio_multi_driver2_bundle.patch` records the
algorithm/build adaptations relative to the imported FAST-LIO-MULTI commit.
It does not include this provenance document or the submodule-to-vendored
packaging change. The ikd-Tree source is unchanged from its imported commit.

See `../../README.md` for the robot integration, configuration, and validation.
Future upstream updates should be reviewed against these pinned revisions and
rerun the integration tests; do not replace the local adaptations blindly.
