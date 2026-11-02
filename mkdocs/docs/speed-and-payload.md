# Speed and payload

## How fast an xbot moves

Every motion an xbot is sent is made with:

- **speed**: the table's speed, or the shuttle's own limit if that is lower;
- **acceleration**: the table's acceleration, or less, as the shuttle's own limit and the
  acceleration the PMC allows with its payload (`MaxAcceleration`) have it.

Then:

- **in a zone** with a `speed` factor, the PMC scales both by that factor (see [Zones](zones.md));
- **the jerk** of every xbot is limited by `XBOT_JERK`, if set;
- **turns** run at 3 rad/s and 10 rad/s², whatever the settings above.

A shuttle's `Equipment/Xbot/Speed` and `Acceleration` read what its moves are made with now,
zones apart.

## The table's speed

`PlanarTable/Equipment/SetSpeed(Session, Speed, Acceleration)` sets how fast every xbot moves at
most, in m/s and m/s². `PlanarTable/Equipment/Speed` and `Acceleration` read it back. They start
as `XBOT_SPEED` (0.5 m/s) and `XBOT_ACCEL` (2 m/s²).

- 0 or NaN sets that value back to `XBOT_SPEED` or `XBOT_ACCEL`. The two are set together, so
  to change only the speed, pass the acceleration as it is.
- A negative value is refused with OutOfRange (8).
- Whoever may command the table may set it: its occupant, or anyone while it is not occupied.
  Otherwise NotPermitted (5).

## A shuttle's own limit

`PlanarShuttle<N>/Equipment/Xbot/SetSpeed(Session, Speed, Acceleration)` keeps one xbot slower
than the table, e.g. while it carries an open vial. It moves at the lower of its own limit and
the table's; its own never makes it faster. 0 or NaN: no limit of its own.

- Whoever may command the shuttle may set it: its occupant, or anyone while it is not occupied.
- Before the table's first Reset it answers XbotNotDetected (28): the xbot is not known yet.
- The limit stays until it is set again, or the controller restarts. It lasts through releases
  and Resets, so set it back to 0 when the product leaves the shuttle.

## When a new speed counts

From each xbot's next motion. A motion under way keeps the speed it was sent with, so a long
straight run goes on as it was. The planner estimates its plans with each xbot's speed as it is
when it plans. The table's Reset sends the shuttles into the loading queue together, at the
slowest one's speed.

## What the PMC does with it

The PMC takes any speed, but moves no xbot faster than the xbot can go. On Planar Motor's
simulator an xbot topped out at about 3 m/s, whether 4, 6 or 10 m/s was set, so the controller
does not cap the speed itself. On the simulator, the same 840 mm move took:

| Set over OPC UA | The move took |
| --- | --- |
| the table's 0.5 m/s, 2 m/s² (the defaults) | 1.95 s |
| the table's 1 m/s, 4 m/s² | 1.14 s |
| a shuttle's own 0.2 m/s, the table's 1 m/s, 4 m/s² | 4.23 s |
| the table's 2 m/s, 10 m/s² | 0.67 s |

Each arrived within about 0.03 s of the time the PMC said it would take, and within about 0.05 s
of the trapezoidal profile the planner estimates with. That profile has the xbot accelerate at
*a* up to speed *v*, cruise, and brake at *a*:

- a distance *d* of at least *v*²/*a* takes *d*/*v* + *v*/*a*;
- a shorter one takes 2√(*d*/*a*).

For example, 840 mm at 0.5 m/s and 2 m/s² takes 1.68 + 0.25 = 1.93 s.

## Payload

A station that puts something on a shuttle (filling, a part) tells the PMC what it now carries:

`PlanarShuttle<N>/Equipment/Xbot/SetPayload(Session, PayloadKg, CgHeight)`: the payload in kg,
and the height of its centre of gravity above the xbot in mm.

- Only the shuttle's occupant may set it, and not while its skill runs or its xbot moves or is
  held: NotPermitted (5), Busy (6).
- The PMC tunes its control to the payload and says how fast the xbot may accelerate with it:
  `MaxAcceleration`, in m/s². The xbot's moves keep to it when it is below their acceleration,
  so a full vial is driven more gently.
- The PMC refuses a payload over what the xbot takes: OutOfRange (8). Planar Motor's simulator's
  M3-06 xbots take at most 0.5 kg.
- The PMC keeps the payload until it is told another; the controller reads it back at every
  Reset. Set it to 0 when the product leaves the shuttle.

`PayloadKg`, `PayloadCgHeight` and `MaxAcceleration` beside the method read what the PMC was
told or has; NaN while unknown.

Planar Motor's simulator takes the payload but does not keep it. It reads back 0 kg, and
`MaxAcceleration` 20 m/s² whatever it was told, so only a real table slows a loaded xbot down.

## Jerk

`XBOT_JERK`, in m/s³, limits the jerk of every xbot, which smooths their starts and stops. The
controller sets it on every xbot at each Reset of the table. Empty: the PMC's own limit.
