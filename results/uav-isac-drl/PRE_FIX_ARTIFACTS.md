# Invalidated smoke artifacts

Do not use these directories in analysis:

- `ns3-smoke-training/`
- `ns3-smoke-validation/`
- `ns3-smoke-comparison/`
- top-level `episode-000000-seed-1001/` and `episode-000000-seed-1234/`

They predate the correction of the inherited radar-range error that omitted
vertical UAV-to-ground distance. They remain only as debugging provenance.
Use the corresponding `*-fixed/` directories for the post-fix pipeline smoke
test. Even the fixed results are not paper results: the DQN was trained for
only three episodes (six transitions) and validation/test each used one seed.
