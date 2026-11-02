# Line mode

In free mode, the xbots may travel anywhere on the table, and the planner picks each way afresh.
With `MODE=line` they travel only on **lanes** you lay out, so the way between two places is
always the same, and they turn only on the **turn spots** you choose. Operators can see where
the shuttles will go, stations can stand beside the lanes, and parts of the table stay free.

## A layout

The layout is JSON, in mm in the table's coordinates (the same as the skill's X and Y; slot
centres are at 60, 180, 300, ...). `config/line-layout.example.json`:

```json
{
  "lanes": [
    {"name": "loop", "points": [[180, 180], [780, 180], [780, 540], [180, 540], [180, 180]], "oneWay": true},
    {"name": "shortcut", "points": [[420, 180], [420, 540]]}
  ],
  "turnSpots": [[840, 360]],
  "idleArea": [[60, 180], [60, 540]]
}
```

At the table's Reset the controller logs the layout on the table like this, the top row first.
This is Planar Motor's simulator's table, with the flyway (2, 1) missing:

```text
########     > < ^ v  a one-way lane: the way on from that slot
Iv<+<<<#     +        a two-way lane, or where lanes meet or cross
Iv#+..^T     I        the idle area
Iv#+..^T     T        what a turn on a turn spot sweeps (beyond the lanes)
I>>+>>^#     #        a slot of no lane
########     .        a missing flyway's slot
```

The loop runs counterclockwise one slot in from the edge, the shortcut crosses it both ways, the
free xbots wait on the left edge, and they turn at the right edge's middle flyway.

## The parts of a layout

### lanes

Each lane is a line through slot centres, every stretch of it along a row or a column.

- `points`: at least two `[x, y]` points in mm, each a slot centre. Between two points the lane
  covers every slot on the way.
- `oneWay`: `true` drives the lane from its first point to its last only; `false`, the default,
  drives it both ways. A closed loop repeats its first point at the end.
- `name`: for the messages; `lane 1`, `lane 2`, ... by default.

Where lanes meet or cross, an xbot may change from one to the other. Between two lane slots side
by side there is no way but along a lane, so two parallel lanes are separate. The lanes together
must lead from every lane slot to every other, also with the one-way lanes' directions.

### Targets off the lanes

A station need not be on a lane. A target off the lanes is reached from the nearest lane slot,
the shortest way across slots of no lane, and left the same way. That way is its **spur**. Only
the xbot going there or coming from there may drive on a spur. Keep stations off the lanes: a
shuttle held at a station on a lane blocks the lane until it is released.

### turnSpots

The points the xbots turn on. Each is a flyway's centre, or a point where four flyways meet;
another point inside a flyway stands for that flyway's centre. The PMC turns an xbot nowhere
else.

- A turn sweeps the square one flyway wide around its spot, so a turn spot on a lane blocks the
  lane while an xbot turns there. The square must have a lane slot in it.
- An xbot turns on the turn spot nearest to it along the lanes. While another turns there, it
  waits behind it rather than driving on to another spot.
- Without turn spots, xbots turn on any flyway centre that is free.
- A target turned off the axes (say 45 degrees) is still turned on its own spot: the target must
  then be on a spot.

### idleArea

A line like a lane's: the loading queue in order, its first point the loading position. The
free xbots wait there, reached and left like a target off the lanes. `LOADING_STATION` is not
used in line mode. Without an idle area, a free xbot goes back to where it was.

Give the idle area a slot for every xbot on the table. A free xbot that does not fit waits where
it is, and on a one-way lane that soon blocks the others. The controller warns at the Reset when
it is too small.

## What a layout must keep to

A layout that breaks a rule is refused with the reason. At start-up the controller stops; at a
Reset of the table, the Reset fails with LineLayoutInvalid (31) and the reason in
`PlanarTable/Equipment/LineLayoutProblem`. For instance, on the simulator's table:

| Rule | The message |
| --- | --- |
| Every point is a slot centre on the table | `lane 1: (150, 180) mm is not a slot centre on the table` |
| Every stretch is along a row or a column | `lane 1: from (180, 180) mm to (300, 300) mm is not along a row or a column` |
| No lane crosses a missing flyway | `lane 1: crosses a hole at (540, 300) mm` |
| Every lane slot leads to every other | `the lanes lead nowhere from (300, 180) mm to (180, 180) mm: every lane slot must lead to every other (mind the one-way lanes)` |
| A turn spot is a flyway centre or where four flyways meet | `turnSpots: (480, 360) mm is between flyways: a turn spot is a flyway's centre or where four flyways meet` |
| A turn spot has a lane slot under it | `turnSpots: (840, 120) mm has no lane slot under it` |
| The idle area passes no slot twice | `idleArea: passes a slot twice` |
| Only the keys above | `the line layout: unknown lane (known: idleArea, lanes, turnSpots)` |

### Checking a layout

From the repository's root, against a table of 4 × 3 flyways with the flyway (2, 1) missing:

```bash
python -c "from planar.lines import load, Lines; from planar.layout import Layout; print(Lines(load('config/line-layout.example.json'), Layout(4, 3, frozenset({(2, 1)}))).describe())"
```

It prints the map above, or raises a `LineError` with the reason.

## How line mode plans

Line mode changes what the planner may do, not how it plans
([The planner](planner.md)):

- **The graph.** The planner plans on a directed graph of slots. Between two lane slots it may
  only step along a lane, and on a one-way lane only forward. Any other step between neighbouring
  slots goes both ways.
- **Keeping off.** Each xbot may use the lane slots, plus the spurs of where it is and of where
  it goes. A free xbot may also use the idle area and the spurs to it. Every other slot is kept
  out of its plan.
- **No straight moves.** An xbot never takes a straight line off the lanes, even when it is
  free.
- **Fixed turn spots.** An xbot waits for its turn spot, behind it on the lane, while another
  turns there.
- **Claims.** An xbot waiting for its claim (a turn spot, a station) to clear stays where it is.
  Pushed off along a one-way lane, it would have to go all the way round, its claim blocking
  the lane. An xbot standing in another's claim is first planned to the nearest slot out of it
  that it can reach past the others. A claim that has not cleared in 5 s is let go for 5 s, so
  the xbots in its way can get out.
- **Waiting.** An xbot whose place is taken waits on the slot fewest steps short of it along the
  lanes: behind the place, not beside or past it.
- **Free xbots** are pushed only along the lanes, and queue in the idle area.

The table's Reset still sends the shuttles into the queue with the PMC's auto routing, which
knows nothing of the lanes. The lanes apply from then on.

## Laying out lanes

- **Prefer one-way loops.** Xbots on a lane one slot wide cannot pass each other. Two-way lanes
  jam as soon as several xbots go opposite ways on them. In the simulated tests, 8 xbots on a
  single two-way lane mostly do. Use two-way lanes only for short branches.
- **Put stations beside the lanes**, on slots of no lane, so a shuttle being worked on does not
  block the lane.
- **Put turn spots where a waiting xbot is not in the others' way.** A turn spot whose flyway
  square covers a lane blocks the lane while an xbot turns there.
- **Give the idle area a slot for every xbot.**
- **Leave the edge free for stations**: a loop one slot in from the edge lets stations stand on
  the edge, each beside the loop.

## Reading the layout again

`LINE_LAYOUT` is read at start-up and again at every Reset of the table. A layout changed in the
file, or in the AAS, is taken by pressing Reset, without restarting the controller.

| When | It cannot be read | It is no layout, or does not fit the table |
| --- | --- | --- |
| At start-up | a file: the controller stops; a URL: it waits for the server, trying every 10 s | the controller stops with the reason |
| At a Reset | the layout read before is used; `LineLayoutProblem` says so | the Reset fails with LineLayoutInvalid (31); `LineLayoutProblem` says why |

`PlanarTable/Equipment/Mode` reads `line`, and `LineLayout` the layout's JSON as it was last
read.

## From an AAS

In the lab, the line's AAS holds the layout as a `PlanarRouting` collection in the line's
`Parameters` submodel. `LINE_LAYOUT` is then its `$value`:

```text
LINE_LAYOUT=http://<aas-env>/submodels/<base64url submodel id>/submodel-elements/PlanarRouting/$value
```

```text
PlanarRouting       SubmodelElementCollection
  Lanes             SubmodelElementList of collections, one per lane:
    Name            Property xs:string
    OneWay          Property xs:boolean
    Points          SubmodelElementList of points
  TurnSpots         SubmodelElementList of points
  IdleArea          SubmodelElementList of points
a point             SubmodelElementCollection of X and Y, Properties xs:double in mm
```

Its value is read as the repository gives it:

```json
{"Lanes": [{"Name": "loop", "OneWay": "true", "Points": [{"X": "180", "Y": "180"}, {"X": "780", "Y": "180"}]}],
 "TurnSpots": [{"X": "840", "Y": "360"}],
 "IdleArea": [{"X": "60", "Y": "180"}, {"X": "60", "Y": "540"}]}
```

BaSyx gives the values as text; numbers and `true`/`false` are read too. A JSON string holding
the file's form is read as well: the `$value` of a `Property`, or an AAS `File`'s
`/attachment`.

The lab's AAS Web UI draws the layout. Its Configurator (Production lines, Shuttle routing) lays
out the lanes, turn spots and idle area on the table as the table's AAS reports it. It checks
them as the controller does, keeps them in the line's AAS, and shows the `LINE_LAYOUT` URL to
set.

## Layouts to test with

`config/layouts/` holds line layouts with what it takes to test them. Each folder has the
layout (`layout.json`), stations for the test agents (`stations.json`), the controller's
settings (`controller.env`) and a `README.md`. `clockwise-outwards` runs clockwise round the
table, one slot in from the edge, with a turn spot on every edge flyway, so each shuttle faces
the module on its side:

```text
TTTTTTTT     the one-way loop, clockwise
I>>>>>vT     I  the idle area, the loading position at its top
I^##..vT     T  a turn spot on every edge flyway
I^##..vT
T^<<<<<T
TTTTTTTT
```

`pytest` runs the test agents on each of these layouts on the simulated table. `tools/sim_check.py`
runs them on Planar Motor's simulator. See [Testing](testing.md).
