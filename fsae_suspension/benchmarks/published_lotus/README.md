# Published Lotus SHARK studies: someone else's model and definitions

Two Formula SAE front corners whose hardpoints and Lotus Suspension Analysis (SHARK) results were published by other
teams, transcribed unadjusted:

- **A**: Biswal, Prasanth, Sakhamuri, Selhi, WCE 2016, pp. 1062-1066: 16 hardpoints and a bump table from −25 to
  +25 mm (camber, toe, castor, kingpin, damper ratio).
- **B**: Singh, Patil, Arwari, Waingankar, IJERT 9(3), 2021: front hardpoints; camber and KPI at −30 and +30 mm.

Limits set before KinematiK was run on them: 0.01° for angles, 0.005 for the damper ratio. Scoring rules set at the
same time are in the script's docstring.

## Result

| corner | measure | worst difference | limit | result |
|---|---|---|---|---|
| A | camber, 49 rows | 0.003° | 0.01° | **pass** |
| A | toe, −25 to 0 mm, 26 rows | 0.007° | 0.01° | **pass** |
| A | caster | 0.0102° | 0.01° | **fail** |
| A | kingpin inclination | 0.043° | 0.01° | **fail** |
| A | damper ratio | 0.013 | 0.005 | **fail** |
| B | camber, KPI at −30 mm | 0.004°, 0.003° | 0.01° | pass |
| B | camber, KPI at +30 mm | 0.10°, 0.26° | 0.01° | **fail** |

## Why the fails are kept as fails, and what they point to

Project Chrono, solving the same published hardpoints with no KinematiK code, agrees with KinematiK within 0.000002° on
both corners at every checked travel (`chrono_check.json`). So the published tables do not follow from their own
hardpoints at those rows: A's kingpin column matches KinematiK to about 0.001° one row up from −14 mm (a row appears to
be missing), its caster column is out of sequence around 0 mm, and B's +30 mm values disagree while its −30 mm values
match. A's damper ratio may be defined differently from wheel travel per damper travel; the paper does not say. None of
these explanations changes the score.

A's toe from +1 mm is printed without decimals ("836" at +25 mm). It is not scored; KinematiK gives 0.0836° there.

## What it does show

Where a published table is consistent with its own hardpoints, KinematiK's camber, toe and kingpin inclination agree
with Lotus's definitions to a few thousandths of a degree. Published student data is a weak reference: a measured car,
or a team's own model, is still the test that counts.
