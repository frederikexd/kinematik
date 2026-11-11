# Break it

KinematiK makes specific, checkable claims: benchmark results, sealed predictions of real cars, and reproducible
runs. This is a standing invitation to prove any of them wrong. A confirmed break is credited by name, fixed in the
open, and stays listed here.

## What counts

- **A wrong result.** An equation, model or output that is wrong. Show it with a derivation, a published
  reference, a measurement, or another tool's result (Adams, Lotus, OptimumKinematics, GT-SUITE, FASTSim, test
  data).
- **A claim that doesn't reproduce.** A number in README.md, VALIDATION.md, BENCHMARK.md or the docs that you can't
  reproduce with the commands given. Attach your output and your environment.
- **A sealed prediction that was changed.** A ledger record (`benchmarks/ledger/`) or a team blind-test record
  whose SHA-256 doesn't match, or a prediction that today's code doesn't reproduce.
- **A test that passes on broken code.** A change to the physics that the test suite doesn't catch.

General doubt without a technical objection doesn't count. "It's junk" is not a break; a case that shows a wrong
number is.

## How to submit

Open an issue with the **Challenge the physics** template, or the **I reproduced it / I couldn't** template for a
reproduction. Include the inputs, KinematiK's result, the result you believe is correct, and why.

## What happens next

- Every challenge gets a public reply. If it's right, the fix comes with a test that pins the correct result, and
  the changelog and this page credit you.
- If it's not a break, the reply shows why, with numbers. Both outcomes stay public.
- A confirmed break that changes a published number is also noted where the number was published.

## Confirmed breaks

None yet. The challenge has been open since November 2026.

| Date | Who | What was wrong | Fix |
|---|---|---|---|

## Independent reproductions

People outside the project who ran the checks themselves and reported the outcome, success or failure.

| Date | Who | What they reproduced | Result |
|---|---|---|---|
