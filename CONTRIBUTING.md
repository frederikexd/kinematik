# Contributing to KinematiK

Thanks for wanting to help. Two things to read before you write any code — the
second one is unusual and I'd rather you know about it up front than discover
it in a pull request.

## Report things

The most useful contribution is telling me something is wrong. If a parser
misreads your board, a number looks implausible, or the tool tells you a
perfectly good net is open — open an issue with the file if you can share it,
or a description if you can't. Every significant bug fixed so far was found by
running real boards from real teams, not by reasoning about the code.

You do not need to sign anything to report a bug.

## The Contributor Licence Agreement

**KinematiK is offered under the AGPL, and also under separate commercial terms.
Because of that, code contributions are only accepted with copyright
assignment.**

By submitting a pull request you confirm that:

1. You wrote the contribution yourself, or otherwise have the right to submit
   it, and it is not encumbered by anyone else's rights — including an
   employer's or a university's, which is worth checking if you are a student
   with a research contract.
2. You assign copyright in your contribution to Frederik Thio.
3. You understand the contribution will be distributed under the AGPL and may
   also be distributed under other terms, including commercial ones.
4. You are granted a licence back to use your own contribution however you
   wish, for any purpose, forever.

Add this line to your commit message:

    Signed-off-by: Your Name <your@email>   # I agree to the CLA in CONTRIBUTING.md

### Why

Being honest about this rather than burying it: it is so the project can be
dual-licensed later. If ten people each hold copyright in one function, no
relicensing is possible without tracking down all ten — and in a student
project people graduate and stop answering email. Sole ownership keeps that
option open.

What it costs you: you give up the right to control how your contribution is
licensed. What it doesn't cost you: anything else. Point 4 means you keep full
use of your own work, and everything you contribute stays available under the
AGPL to everyone, permanently. Nothing you write here can be taken private in a
way that stops you or anyone else using it.

If that trade isn't one you want to make, please open an issue instead of a
pull request. A well-described problem is worth more to me than a patch, and it
costs you nothing.

## Free for student teams

Formula SAE and Formula Student teams have free use of KinematiK and that will
not change, whatever the licensing arrangements around it. If you are a student
team, none of the above affects you.

## Practical notes

- Run the test suite before opening a pull request: `pytest tests/`
- New behaviour needs a test. The parsers especially — every silent
  wrong-answer bug so far was one that produced confident, plausible, wrong
  output rather than an error.
- Read `SAFETY_CONTRACT` in `suspension/pcb_doctor.py` before touching any
  connectivity tolerance. False alarms are acceptable; false all-clears are
  not. Several tests exist purely to enforce that asymmetry, and if one fails,
  the assertion is not the thing that's wrong.
