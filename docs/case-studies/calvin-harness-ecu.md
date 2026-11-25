# Checking the harness against the ECU before power-up: Calvin Racing

**Team data:** the team's tune, wiring tables and harness drawings are not published, not stored in the KinematiK
repository and not available on request. They were used only for this check and deleted after each check.

Calvin Racing (Calvin University Formula SAE, [calvinfsae.org](https://calvinfsae.org)) runs a Performance Electronics
PE3 standalone ECU with in-house harnesses. A common risk before first power-up is harness drawings that don't match
the ECU configuration, like a relay wired to a different output than the one set up in software.

KinematiK read the PE3 tune and three harness wiring tables and checked them against each other and PE's pinout. Every
wired pin matched, including the fuel pump and fan. It flagged one dependency: the fan relay's coil was fed through
another relay's output, so losing that relay or its fuse would also take out the fan. The team plans to move the feed
to the main relay. After the check, and on PE's advice, the team will also name every output in the tune and click
through Output Diagnostics before power-up.

**What the check can't do:** confirm a relay is physically wired to the right output. That still needs the bench test.

**Credits.** Nicholas Stroobosscher (President) and Jonathan Hipple (Electrical Co-Lead), Calvin Racing, shared their
tune and wiring tables, and Nicholas's feedback shaped this check. Performance Electronics support confirmed how output
names are read from the tune and recommended the Output Diagnostics step. Credit records their contribution and does
not imply endorsement.

The check itself: `fsae_suspension/suspension/ecu_pins.py`, tests in `fsae_suspension/tests/test_ecu_pins.py`; in the
app under Electronics (PCB) → ECU pin check.
