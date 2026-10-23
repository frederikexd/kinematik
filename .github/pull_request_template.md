## What this changes

<!-- One or two sentences: what changes and why. Link the issue if there is one (Fixes #123). -->

## Type

- [ ] Bug fix
- [ ] Physics or solver change
- [ ] New feature
- [ ] Documentation, benchmark or paper text
- [ ] Other

## Checks

- [ ] `pytest` passes locally
- [ ] Physics changes come with a test that pins the correct result (see `fsae_suspension/tests/`)
- [ ] If a printed number can move: `python scripts/paper_check.py` still passes
- [ ] If a benchmark is affected: `BENCHMARK.md` and `VALIDATION.md` are updated
- [ ] `CHANGELOG.md` has an entry under the next version
- [ ] No secrets, credentials or personal data are committed

## Evidence

<!-- For physics or solver changes: the case you ran, the result before and after, and why the new one is right. -->
