# Contributing to KinematiK

## Challenge the Physics

If you believe there is an error in the mathematics or physics, open an issue.

Be specific: name the module, the equation, and what you believe the correct result should be. General skepticism without a technical objection is not a contribution.

This is a standing challenge: confirmed breaks are credited by name in [CHALLENGE.md](CHALLENGE.md), along with
independent reproductions, successful or not.

## Bug Reports

Open an issue with:
- The values you used
- The output you got
- The output you expected and why

## Feature Requests

Open an issue describing the engineering problem you are trying to solve. KinematiK prioritises features that active teams actually need.

## Pull Requests

PRs are welcome. Any change that touches physics must include a test that pins the correct result. See `fsae_suspension/tests/` for examples.

### Set up

```bash
git clone https://github.com/frederikexd/kinematik.git
cd kinematik
pip install -r requirements.txt
pip install -e './fsae_suspension[dev]'
pip install pre-commit && pre-commit install      # runs the lint before each commit
```

Python 3.12 or newer.

### What every pull request has to pass

These run on GitHub for every push and pull request. Run them locally first; none needs an account or a token.

| Gate | Command (from `fsae_suspension/`) | Rule |
|---|---|---|
| Tests | `pytest -n auto` | All pass. A deprecation warning is an error. |
| Coverage | `pytest --cov=suspension --cov=powertrain` | The floors in `.github/workflows/tests.yml` only go up. The core solvers stay at 100 % of statements and branches. |
| Lint | `ruff check .` | Zero findings on the selected rules. |
| Mutation | `python scripts/mutation_gate.py --changed <files>` names the groups a change touches; `--group <name>` runs one | Each listed physics module keeps 99 % of mutants killed. |
| Security | `pip-audit -r ../requirements.txt` and `bandit -r . -x tests -ll` | No known-vulnerable dependency, no medium or high finding. |
| Units | `pytest tests/test_repo_accuracy_audit.py` | Every public physics function states its units. |
| Untrusted files | `pytest tests/test_pcb_upload_fuzz.py` | A damaged upload is refused with a message or read; it never crashes. |

### Benchmarks

A benchmark result is only worth something if the test was fixed before the answer was known. The rule for every
benchmark in this repository:

1. Write the cases, the reference and the pass limit into the plan document (for example
   `docs/ELECTRICAL_BENCHMARK_PLAN.md`) and commit that **before** running anything.
2. Name the commit the model is frozen at. Keep design data (what you tuned on) apart from held-out data (scored once).
3. Publish the result whichever way it goes. A miss stays in the document, with its cause when it is found.

A pull request that improves a benchmark number without a registered plan will be asked for one.

### Dependencies and CI

GitHub Actions are pinned to a commit, with the version in a comment; Dependabot proposes the updates. The numbers
the paper quotes are recomputed against `requirements-reproduce.lock`, so change that lock only in a commit of its own.

## Code of Conduct

Engage with the engineering. If you have not used the interface or read the physics, you do not have a technical opinion yet.

The full code of conduct, and how to report a problem, is in [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md). Accessibility barriers can be reported as described in [ACCESSIBILITY.md](ACCESSIBILITY.md).
