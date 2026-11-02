# The planner

This page explains how the controller turns `MoveToPosition` requests into motions of the
xbots. It covers what the controller plans on, how it finds one plan for every xbot, how it
executes the plan safely, and the special cases: turns, stations between slots, the loading
queue, held xbots and zones. Line mode has [its own page](line-mode.md).

The code is in `planar/coordinator.py` (the coordinator: requests, execution, the special
cases) and `planar/mapf.py` (the multi-xbot path finder). The geometry is in `planar/layout.py`.

## The table as the planner sees it

The table is a grid of flyways, 240 mm square, some of which may be missing. The controller reads
the grid from the PMC at every Reset of the table.

The planner does not plan in millimetres. It plans on a lattice of **slots**, one xbot wide:
120 mm, 2 × 2 slots per flyway. Slot (i, j) has its centre at ((i + 0.5) · 120, (j + 0.5) · 120)
mm: slot (0, 0) at (60, 60), slot (2, 5) at (300, 660). An xbot parked on a slot centre with
its sides along the axes covers exactly that slot. Two xbots on neighbouring slots touch
without overlapping, as in the PMC's own home layout.

Planar Motor's simulator has 4 × 3 flyways, the flyway (2, 1) missing. As slots, top row first
(`#` a slot, `.` a missing one):

```text
########      row 5
########
####..##      the missing flyway (2, 1): slots (4..5, 2..3)
####..##
########
########      row 0; column 0 on the left
```

Two rules come from the PMC:

- **An xbot turns only on a spot**: a flyway centre, or a point where four flyways meet.
- **An xbot travels square to the axes.** Another angle is possible only as its final pose,
  on a spot.

## Where an xbot can be

Every xbot on the table, offered as a shuttle or not, is in one of three situations:

- **On the lattice**: within 0.2 mm of a slot centre, square to the axes. It covers that slot,
  and the planner may move it.
- **Docked**: anywhere else. That may be a station between slots, a flyway centre, an angle
  off the axes, or wherever an Abort stopped it. It covers every slot its square overlaps, and
  the planner treats those slots as walls.
- **Busy with a macro**: a straight move, a turn, or the last moves onto or off a pose between
  slots. It has claimed the boxes it sweeps (see [Claims](#claims)) and covers them until it is
  done.

## The life of a request

1. **Start checks the target** at once (see [the gate](opcua-interface.md#the-gate)). It must
   be on flyways, outside closed zones, and square to the axes unless it is on a spot.
2. **The request becomes a task**: the target pose, and a number that says in which order the
   requests were made.
3. **Every 20 ms the coordinator looks at each task** and decides what the xbot does next, in
   this order:
    1. **Wait its turn.** An older request still on its way wants the same place (their
       squares overlap): this one waits, on a slot. See
       [Requests for the same place](#requests-for-the-same-place).
    2. **Arrived?** It is at the target, turned to the target's angle: Succeeded.
    3. **Straight there.** In free mode, a move whose straight line is free goes straight,
       slots or not. See [Straight moves](#straight-moves).
    4. **Docked?** On a spot, it turns there when it is at the target and only the angle is
       left, or when the target's angle is a multiple of 90 degrees. Otherwise it steps onto
       the slot it covers that is nearest to where it goes. See
       [Leaving a station](#leaving-a-station).
    5. **A turn is needed.** It goes to a spot, claims it, and turns there. See
       [Turns](#turns).
    6. **The target is a slot centre.** It wants that slot, and the planner takes it there.
    7. **The target is between slots.** It wants a slot from which its last moves lead in, and
       claims the boxes those moves sweep. See [Stations between slots](#stations-between-slots).
4. **The planner moves every xbot** toward the slot each wants ([below](#one-plan-for-every-xbot)).
   The coordinator executes the plan by its dependencies
   ([Executing a plan](#executing-a-plan)).
5. **The task ends.** It Succeeds when the xbot is at the target. It Fails with an ErrorID
   when it cannot get there, and with Interrupted (7) when it is stopped. `Results` hold where
   the xbot ended.

## One plan for every xbot

Whenever something changes, the coordinator makes one plan for all the xbots it may move. It
works from where each is (or where its motion in flight ends) to where each should be.

### Who takes part

The planner moves an xbot when it is on the lattice, fit (no fault), not busy with a macro, and
not fenced in a closed zone, and either:

- it has a task, or
- it is free: nobody holds it, so the planner may push it out of the way and bring it back
  (see [Pinned xbots](#pinned-xbots)).

Every other xbot is a **wall**: the slots it covers, the slots it has claimed, and the slots of
a zone that is switched off.

### Where each one should go

Each xbot gets a goal slot, in this order:

1. The xbots with a task, the oldest request first: the slot their task wants now (the target,
   a slot by a spot, a way in to a station).
2. The free xbots: their places in the [loading queue](#the-loading-queue), or, without one, back
   to where they were.

A goal must be a slot the xbot can reach and may use. It must not be another's goal, nor claimed
by another xbot. When the slot it wants is not such a slot, the xbot is planned to the usable
slot nearest to it, and planning is tried again a second later. That is how an xbot waits next
to a place that is taken. In line mode, "nearest" means fewest steps short of it along the
lanes, so it queues behind a taken place rather than beside or past it. When an xbot is boxed
in and can only stay, it keeps its own slot, and the xbot that was given that slot is placed
elsewhere.

### Finding the plan: LaCAM with PIBT

A plan is a sequence of **configurations**, each one slot per xbot. From one configuration to
the next, every xbot stays or moves to a neighbouring slot, and:

- no two xbots are in the same slot;
- no two xbots swap slots;
- an xbot may move into a slot another leaves in the same step (it follows), but no ring of
  xbots may follow each other: each would wait for the next.

The configurations come from **LaCAM** (Okumura, *LaCAM: Search-Based Algorithm for Quick
Multi-Agent Pathfinding*, AAAI 2023), with **PIBT** (Okumura et al., *Priority Inheritance with
Backtracking for Iterative Multi-agent Path Finding*, AIJ 2022) proposing each next step:

- **PIBT** moves every xbot one step at once. The xbots farthest from their goals choose first.
  Each takes the free neighbouring slot nearest to its goal. Between equally near slots it goes
  on in the direction it came from, so of the shortest paths it takes the straight ones: one
  corner instead of a staircase. An xbot that wants a slot another is on pushes that one, which
  moves on in turn with the pusher's priority (priority inheritance). A push that leads nowhere
  is taken back (backtracking), and the pusher tries its next slot. This is what gets two xbots
  past each other in a one-lane passage by way of a pocket.
- **LaCAM** searches over configurations and has PIBT propose the next. When PIBT's proposals
  lead nowhere, it constrains them, one xbot at a time, to try the alternatives systematically.
  That makes the search complete: if every xbot can reach its goal, it finds a way.

The search has 0.5 s. When it finds no complete plan in that time, or there is none, the plan
leads to the configuration nearest the goals that it has found. The planner tries again a
second later.

### Choosing the fastest plan

LaCAM's first plan depends on the seed that breaks PIBT's ties. Once a plan is found, the planner
tries more seeds for up to 50 ms (at most 16 in all). It keeps the plan **estimated to be done
soonest**, and of those, the one with the least time summed over the xbots.

The estimate (`mapf.execution_time`) executes the plan the way the coordinator does. Each xbot
moves at its own speed and acceleration, with a trapezoidal speed profile. It starts a move once
the move it depends on is made, and goes on in one straight motion through the slots ahead whose
dependencies are met by then. So every corner and every wait for another xbot costs a stop and
a start. A straight route beats a staircase, and a detour that saves waiting can beat the
shortest way.

### Example: two xbots in a one-lane passage

A passage one slot wide and seven long, with a pocket above its middle slot. A starts at the
left end for the right end, B at the right end for the left. This is the planner's output, best
of 16 seeds:

```text
t=0       t=1       t=2       t=3       t=4       t=5       t=6       t=7       t=8
   .         .         .         .         A         .         .         .         .
A.....B   .A...B.   ..A.B..   ...AB..   ...B...   ..BA...   .B..A..   B....A.   B.....A
```

A steps into the pocket at t=4, B passes below it, and A comes out behind B. At 0.5 m/s and
2 m/s², the estimate is 3.5 s until both are done.

### When it plans again

The coordinator plans again at once whenever any of these changes:

- the walls;
- the claims;
- who has a task, and which slot each wants.

That covers a new request, an xbot occupied or released, a turn or station claimed, and a zone
switched off. When the last plan left an xbot short of where it wants to be, the coordinator
also plans again a second later.

A new plan does not stop the xbots that are moving. Each xbot in flight is planned from the
slot its motion ends on. No other xbot enters a slot it still has to leave until it has left
it. Nobody needs to stop for the new plan.

## Executing a plan

### Moves and their dependencies

A plan becomes one list of moves per xbot, each one slot. A move may start once the xbot
that was planned to be on its target slot before has left that slot (`mapf.actions`).

Here is the 4 × 1 flyway strip (8 × 2 slots). A goes from slot (0, 0) to (5, 0); B is free
and stands on (2, 0), where it was:

```text
t=0        t=1        t=2        t=3        t=4        t=5
........   ........   ..B.....   ........   ........   ........
A.B.....   .AB.....   ..A.....   ..BA....   ..B.A...   ..B..A..
```

| Move | Xbot | From | To | May start once |
| --- | --- | --- | --- | --- |
| 1 | A | (0, 0) | (1, 0) | at once |
| 2 | A | (1, 0) | (2, 0) | move 3 is made: B has left (2, 0) |
| 4 | A | (2, 0) | (3, 0) | at once |
| 6 | A | (3, 0) | (4, 0) | at once |
| 7 | A | (4, 0) | (5, 0) | at once |
| 3 | B | (2, 0) | (2, 1) | at once |
| 5 | B | (2, 1) | (2, 0) | move 4 is made: A has left (2, 0) |

The plan has steps (t=0, 1, ...), but the coordinator does not keep to them. Plans are
executed **by dependencies, not by time**: an xbot starts into a slot only once every xbot
planned there before it has fully left. A slow xbot, or a late one, delays the others but
cannot collide with them.

### One motion per straight run

The PMC gets few, long motions rather than a step per slot. When an xbot's next move may start,
the coordinator takes it together with the moves after it that go on in the same direction and
whose dependencies are met already. It sends them as one straight motion to the last one's slot
centre.

In the example, A and B start at once. A's first motion goes only to (1, 0): its move into
(2, 0) waits for B to leave. B steps up. When B is clear of (2, 0), A goes on from (1, 0) to
(5, 0) in one motion, and B steps back down behind it. In the passage example above, A drives
from the left end to the pocket's slot in one motion, and B drives from (4, 0) to the left end
in one motion once A is in the pocket.

### Following closely

While a motion is in flight, the coordinator watches the xbot pass each slot centre on its way.
When the xbot's centre reaches a slot's centre, it has fully left the slot behind it, and the
move into that slot is made. An xbot that waits for that slot may start at once, while the one
ahead is still moving.

### Keeping out of another xbot's trail

The PMC has an obstacle check of its own, and it stops more than collisions. On Planar Motor's
simulator, it stopped an xbot sent into the way another xbot was still travelling, even with
that xbot 50 to 100 mm ahead. The stopped xbot reports `XBOT_OBSTACLE_DETECTED`. A **trail**
(`Layout.trail`) is that way: the box a straight motion sweeps, from one slot behind its start
to its target, and anything touching it. The simulator stopped a move into a trail while the
motion in it had more than about 0.65 s to go.

So the coordinator holds back a motion into another xbot's trail while that motion has more than
`TRAIL_WAIT_S` (0.4 s) to go, or is paused. The motion leaves as soon as the one ahead is nearly
done. A follower therefore keeps a little more distance than the dependencies alone would give
it.

This rule is measured, not documented by Planar Motor, and it does not cover every case: in one
run, the simulator stopped an xbot whose target only touched the corner of another's start slot,
with that motion nearly done. A stopped move fails with XbotFault (21). See
[After a failure](opcua-interface.md#after-a-failure).

### Timeouts and failures

The PMC says how long each motion should take. A motion that has not arrived within half as long
again, and 2 s more, fails with MotionTimeout (23). One that ends somewhere other than where it
was sent fails with MotionDeviated (30). An xbot the PMC reports in a fault fails its task with
XbotFault (21), and one it stops reporting, or switches off, with XbotNotDetected (28). A
command the PMC refuses fails with PmcError (22).

A failed xbot is stopped where it is and becomes a wall: the others are planned around it. Its
shuttle refuses new moves until it is Stopped, Reset and Started again.

## Straight moves

In free mode, a move first tries the obvious route: one straight motion from where the xbot
stands to its target. It takes that route when all of these hold:

- the xbot is square to the axes, and needs no turn;
- the box the motion sweeps (from its square now to its square at the target) is on the table
  and in no closed zone;
- the box overlaps at most 16 slots (`DIRECT_MAX_SLOTS`), so long routes go through the
  planner, which can share the table better;
- no other xbot is in the box, on its way into it, or planned through it, and no other claim
  overlaps it.

The motion may be diagonal. It claims the box until it is done, and the planner keeps the others
off its slots meanwhile.

## Turns

The PMC turns an xbot only on a spot. To turn, an xbot:

1. **Chooses a spot**:
    - A target turned off the axes, e.g. 45 degrees, is turned on the target's own spot, the
      only place it may have that angle.
    - In line mode with turn spots, it takes the turn spot nearest to it along the lanes. It
      waits for that spot while another xbot turns there, rather than driving on to another.
    - Otherwise, it takes the flyway centre with the least detour (to the spot, then on to the
      target), out of those whose whole flyway square is clear of xbots that stay where they
      are and of others' claims.
2. **Goes to a slot under the spot's square**, by the plan.
3. **Claims the square one flyway wide around the spot**, which the turn sweeps. It waits there
   until the others have left the square.
4. **Drives to the spot, turns, and drives out**: back onto a slot of the square, the one
   nearest to its target, around the others. When every way is blocked for now, it takes the
   one toward its target anyway, not the one back toward the xbot behind it. When the target
   is the spot itself, it stays.

It turns the shortest way, by at most half a turn. The PMC does not wrap angles: told to turn
to an absolute 180 degrees from -90, it turns 270 degrees, the long way. So the controller tells
it the difference from the xbot's angle instead. Turns run at 3 rad/s and 10 rad/s² whatever
the speed settings.

## Stations between slots

A station's pose need not be a slot centre. The xbot's last moves lead it there from a slot:

- **Straight in**, from a slot the target covers, when the box that move sweeps is clear of
  the xbots that stay where they are and of every claim.
- **Through a stage**, a pose one xbot width beside the target, from a slot under the stage:
  first to the stage, then straight in. This is the way into a gap between two xbots it touches
  on both sides: from beside the gap, straight in.

The xbot goes to a slot from which a way leads in, claims the boxes its last moves sweep, waits
until they are clear, and drives in. Two docked xbots may stand side by side and touch, even
sharing slots. Final moves are checked against the other xbots' real squares, not their slots.

### Leaving a station

From a docked pose, an xbot steps onto the slot it covers that is nearest to where it goes next.
When every slot it covers is touched by another xbot, it first backs out sideways, one xbot
width, onto a pose with a free slot under it. A free xbot left between slots, e.g. after an
Abort, steps back onto the nearest slot by itself once nobody holds it.

An xbot turned off the axes can only travel once it is square again, which it can only become
on a spot. On a spot, it turns to the target's angle, or the nearest multiple of 90 degrees.
Off a spot it cannot move at all: the move fails with XbotCrooked (29), and a Start is refused
with it.

## Claims

A turn, a straight move and the last moves into or out of a station sweep more than single
slots. Before such a **macro**, the xbot **claims** the boxes it will sweep, and with them the
slots they overlap. The planner then keeps the other xbots off those slots. An xbot that stands
in another's claim may leave it, but nobody enters it. The macro starts when its boxes are clear.

Several claims may wait at once, but only while every xbot the planner has to clear out of a
claim can still get out of all of them, with room enough outside. That keeps two claims from
leaving each other's xbots nowhere to go.

Claims can still need each other's room. So a claim that has waited 5 s (`CLAIM_YIELD_S`) for
its boxes to clear gives way to an older claim that is still waiting, and claims again after
another 5 s. In line mode, it lets go for 5 s instead, so the xbots in its way can get out.

A straight move or a step off a station claims only when nothing is in its boxes or on its way
into them, so it never waits.

## Requests for the same place

Requests for the same place are served in the order they were made. A request waits while an
older one, still on its way, wants a place its own overlaps. It waits on a slot, where the
planner may move it aside: a docked xbot would be a wall, perhaps in the way of the very request
it waits for. Once the older xbot has arrived, the younger one heads there and waits nearby
until the place is free.

A turn of a free xbot to the idle orientation makes no request wait: any request replaces it.

## Idle xbots

An xbot is free when nobody holds it: its shuttle is not occupied, or it is not offered as a
shuttle and the planner may move it. Free xbots keep out of the way:

- Pushed aside by a plan, a free xbot comes back to its place when the plan is done.
- Without a loading station, its place is where it was: the slot its last move ended on, or the
  slot it stepped onto from a station.
- A free xbot left between slots steps back onto the nearest slot.

### The loading queue

With `LOADING_STATION` (or, in line mode, an idle area) the free xbots line up in a queue
instead: the first on the slot under the station, the others on the slots nearest to it. A slot
inside the table counts two steps farther than one on the table's edge, or on a hole's edge. So
the queue grows along the edges, rather than into the middle of the table where the others pass.

Places are handed out in the queue's order, each to the nearest free xbot not placed yet. When
the first leaves the queue, the others close up with as few moves as possible.

`PlanarTable/Equipment/LoadingQueue` names the shuttles waiting there and ready, the one on the
station first: the one to occupy for the next product.

### The idle orientation

With `IdleRz` set, a free xbot turned otherwise turns to it on the nearest clear flyway centre.
That happens when a shuttle is released at a station facing another way, or when `IdleRz` is
changed. Such a turn is the coordinator's own task, and any request for the xbot replaces it at
once. A turn that fails is not tried again for 10 s.

### At the table's Reset

The table's Resetting sends every shuttle into the loading queue with the PMC's own auto routing,
in one command for up to 10 xbots (`AUTO_DRIVE_MAX`). The auto routing does not turn the xbots,
so with `IdleRz` set, each is then turned to it on a flyway centre. With more xbots, or if the
PMC refuses, the planner lines them up instead. Meanwhile the planner may move every shuttle,
held or not. The table turns Idle when they are all in place and turned. That has to happen
within 120 s, or the Reset fails with MotionTimeout (23). In line mode the auto routing knows
nothing of the lanes; the lanes apply from then on.

## Pinned xbots

The planner never pushes a **pinned** xbot. The modules decide which xbots are pinned, at the
moment the planner plans:

- A shuttle that is occupied and not moving is pinned: a station may be working on it. With
  `DISPLACE_HELD=1`, such shuttles may be pushed too.
- A shuttle whose module is not in Idle or Execute is pinned: Stopped, Held, Aborted, ...
- An xbot that is not offered as a shuttle is pinned: nobody here may command it.

Pinned xbots are walls. If they wall a target off, the move waits until they leave, or fails
after `BLOCKED_TIMEOUT_S` (below).

## Hold

A Hold pauses xbots where they are. The PMC pauses the motions in flight, and nothing more is
sent to a paused xbot. A paused xbot's tasks, motions and claims do not time out; the time it is
held is added on when it carries on. Others keep out of a paused xbot's trail. When only one
shuttle is held, the others go on, waiting for it wherever their plans need it to move.

## Blocked moves

By default a move waits as long as it takes for its way to clear, until it is stopped. With
`BLOCKED_TIMEOUT_S` set, a move fails with Blocked (20) when none of its motions has arrived for
that long. The usual causes are pinned shuttles, docked xbots, or a closed zone walling off its
target.

## Zones, speed and timing

- **Zones** that are switched off or fenced are walls for every xbot. No plan enters them, no
  turn is made in them, no claim reaches into them, and a target in one is refused. See
  [Zones](zones.md).
- **Speed**: each xbot moves at the table's speed, or slower as its own limit and its payload
  have it. The planner's estimates use each xbot's own. See
  [Speed and payload](speed-and-payload.md).
- **Slow zones**: the PMC slows an xbot by a zone's speed factor, but leaves the factor out of
  the time it says a motion takes. A motion through a zone is therefore given that much longer
  before it times out.

## Constants

| Constant | Value | Where | |
| --- | --- | --- | --- |
| loop period | 20 ms | `Coordinator.start` | how often the coordinator reads the PMC, plans and dispatches |
| `plan_timeout` | 0.5 s | `Coordinator` | LaCAM's time for one plan |
| `plan_budget` | 50 ms | `Coordinator` | time to look for a faster plan once one is found |
| `max_seeds` | 16 | `mapf.plan` | seeds tried at most |
| `DIRECT_MAX_SLOTS` | 16 | `coordinator.py` | the longest straight move taken without the planner |
| `TRAIL_WAIT_S` | 0.4 s | `coordinator.py` | a motion into another's trail waits while that one has longer to go |
| `CLAIM_YIELD_S` | 5 s | `coordinator.py` | a waiting claim gives way after this |
| `AUTO_DRIVE_MAX` | 10 | `coordinator.py` | xbots the PMC's auto routing takes in one command |
| `GATHER_TIMEOUT_S` | 120 s | `coordinator.py` | lining up and turning the xbots at a Reset |
| `ZONE_READ_S` | 0.5 s | `coordinator.py` | how often the zones' states are read |
| `FLYWAY_READ_S` | 5 s | `coordinator.py` | how often the flyways' power and temperatures are read |
| `POSITION_TOLERANCE_M` | 0.5 mm | `layout.py` | arrived, and on a spot to turn |
| `ANGLE_TOLERANCE_RAD` | 0.005 rad | `layout.py` | turned right (about 0.3 degrees) |
| `SLOT_TOLERANCE_M` | 0.2 mm | `layout.py` | on a slot centre |
| `TRAIL_MARGIN_M` | 1 mm | `layout.py` | how far a trail reaches beyond the box it sweeps |
| turn speed | 3 rad/s, 10 rad/s² | `PmcDriver` | how fast xbots turn |

## Try it yourself

The planner works without a PMC. The strip example above:

```python
from planar import mapf
from planar.layout import Layout

strip = Layout(cols=4, rows=1)  # 4 x 1 flyways: 8 x 2 slots
instance = mapf.Instance(strip.cells, strip.adjacency, starts=[(0, 0), (2, 0)], goals=[(5, 0), (2, 0)])
result = mapf.plan(instance, timeout=2.0, budget=2.0)  # every one of the 16 seeds
print(result.complete, result.configs)
print(mapf.actions(result.configs))        # per xbot: (move, from, to, the move it waits for)
print(mapf.execution_time(result.configs))  # (s until the last is done, s summed): (1.70, 3.28)
```

To see the coordinator at work, run a test with logging, e.g.
`python -m pytest tests/test_coordinator.py -k passage -o log_cli=true --log-cli-level=INFO`:
each plan, claim, turn and arrival is logged.
