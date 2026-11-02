# OPC UA interface

The server listens on `OPCUA_ENDPOINT` (`opc.tcp://<host>:4840/` by default), without security
(policy *None*), so it belongs on a trusted network. Every node is in the namespace
`urn:aau:smartproductionlab:planar`; look its index up by that URI rather than assuming one.

The objects are laid out like the lab's generated IEC 61499 modules: one object per module
under `Objects`, each with the same four parts.

## The address space

```text
Objects
+-- PlanarTable
|   +-- Occupation      Occupy(Session), Release(Session); Occupied
|   +-- Module          Reset, Start, Hold, Unhold, Stop, Abort, Clear (Session); State, ErrorID
|   +-- Equipment
|       +-- PMC         Connected, Columns, Rows, FlywaySize, Holes
|       |   +-- Flyways
|       |       +-- Flyway_<c>_<r>   PowerConsumption, CpuTemperature,
|       |                            AmplifierTemperature, MotorTemperature
|       +-- LoadingQueue, IdleRz (writable)
|       +-- SetSpeed(Session, Speed, Acceleration); Speed, Acceleration
|       +-- Mode, LineLayout, LineLayoutProblem
|       +-- ZoneProblem
|       +-- Zones
|           +-- <name>  Deactivate(Session), Activate(Session); State, Xbots
+-- PlanarShuttle<N>    (one for each xbot offered, N its id)
    +-- Occupation      Occupy(Session), Release(Session); Occupied
    +-- Module          Reset, Start, Hold, Unhold, Stop, Abort, Clear (Session); State, ErrorID
    +-- Skills
    |   +-- MoveToPosition   Start(Session, X, Y, Rz), Stop, Abort, Reset (Session);
    |                        State, ErrorID, Parameters/X, Y, Rz, Results/X, Y, Rz
    +-- Equipment
        +-- Xbot        X, Y, Rz, XbotState
                        SetPayload(Session, PayloadKg, CgHeight); PayloadKg, PayloadCgHeight,
                        MaxAcceleration
                        SetSpeed(Session, Speed, Acceleration); Speed, Acceleration
```

Every node's browse name is the name shown, in the controller's namespace. A path such as
`PlanarShuttle1/Skills/MoveToPosition/State` is reached from `Objects` with `get_child` (or the
client's equivalent), one browse name per step.

## Methods and their answers

Every method answers at once with two output arguments:

| Output | Type | |
| --- | --- | --- |
| `Accepted` | Boolean | whether the command was taken |
| `ErrorID` | Int32 | why not (see [ErrorID](#errorid)); 0 when accepted |

What an accepted command starts finishes later and shows in a `State` variable. Before a method
answers, the controller refreshes every variable, so a client that reads `State` right after an
accepted `Start` sees Running, never what the run before ended in.

Input arguments are `Session` (String) and numbers (Double).

Variables are refreshed from the controller every 50 ms and written when they change, so a
subscription is notified of the changes. Numbers are Double, states and ErrorIDs Int32,
everything else String or Boolean.

## Occupation

A client picks a session, a string that best is a UUID, and sends it with every command. Whether
a module is occupied is published; the session never is.

| | |
| --- | --- |
| `Occupy(Session)` | Takes the module for this session. Answers NotPermitted (5) while another session holds it, OutOfRange (8) for an empty session. Occupying again with the same session is accepted. |
| `Release(Session)` | Lets it go. NotPermitted (5) unless this session holds it. |
| `Occupied` | Boolean. |

Only the occupant may command an occupied module: its `Module` methods, its skill, and its
equipment's methods. A module nobody occupies takes commands from any session, even an empty
one. `Abort` is taken from anyone, always. A skill's methods and `SetPayload` need the
occupation, always.

The table can be occupied too, e.g. by an orchestrator that alone may Reset, Hold or Stop the
line. Occupying the table does not occupy its shuttles.

## The module state machine

`Module/State` is a PackML state number, `Module/ErrorID` why the module last failed or aborted.

```text
                 Reset              (done)              Start
   Stopped 2 ---------> Resetting 15 -----> Idle 4 -----------> Execute 6
       ^                     |                                   |      ^
       |      (failed: ErrorID set)                          Hold|      |(done)
       +---------------------+                                   v      |
       |                                                    Holding 10  Unholding 12
       |   Stop: from Idle, Execute, Resetting,                  |      ^
       +-- Stopping 7 <-- or any Hold state                      v      |Unhold
       |                                                       Held 11 -+
       |
       +-- Clear -- Aborted 9 <-- Abort (from any state but Aborting, Aborted)
```

| Command | From | |
| --- | --- | --- |
| `Reset` | Stopped | Runs the Resetting procedure; Idle when it succeeds, back to Stopped with the ErrorID when it fails. A shuttle only while the table is Idle or Execute. |
| `Start` | Idle | Execute. A shuttle only while the table is in Execute. |
| `Hold` | Execute | Holding, then Held: the xbots pause where they are. |
| `Unhold` | Held | Unholding, then Execute: they carry on. A shuttle only while the table is in Execute. |
| `Stop` | Idle, Execute, Resetting, Holding, Held, Unholding | Stopping: the skills stop in order (held xbots carry on first), then Stopped. |
| `Abort` | any but Aborting, Aborted | Aborted at once: skills aborted, xbots stopped where they are. |
| `Clear` | Aborted | Stopped. |

A command from another state answers NotReady (4).

### The table and its shuttles

The table is its shuttles' parent:

- The table's **Stop** and **Abort** reach every shuttle.
- Its **Reset, Start, Hold, Unhold** and **Clear** reach the shuttles in the matching state: one
  Reset and one Start of the table bring everything up.
- A shuttle's skill runs only while both the shuttle and the table are in Execute.

### What the procedures do

- **The table's Resetting** connects to the PMC and takes mastership. It recovers the xbots the
  PMC switched off after an accident, and levitates the xbots (waiting up to 30 s for the PMC).
  It reads the table's layout from the PMC, reads `LINE_LAYOUT` again, defines the zones, and
  reads each xbot's payload. With a loading station, it then sends the shuttles into the loading
  queue with the PMC's own auto routing, and with `IdleRz` set it turns them to it. A Stop
  meanwhile stops them. It fails with the ErrorID of what went wrong: PmcNotConnected (27),
  MastershipLost (32), PmcError (22, e.g. the PMC not ready in 30 s), LineLayoutInvalid (31),
  ZonesInvalid (34), or XbotFault (21) and MotionTimeout (23) when lining up the xbots goes
  wrong.
- **A shuttle's Resetting** recovers its xbot if the PMC switched it off after an accident,
  waiting up to 15 s for it to levitate. It then checks that the xbot is there and fit to
  move: XbotNotDetected (28) or XbotFault (21) otherwise. It also forgets an earlier failure
  of the xbot (see [After a failure](#after-a-failure)).
- **Holding** pauses the xbots that are moving: the PMC pauses their motions in mid-move. The
  table's Holding pauses every xbot, a shuttle's its own. No xbot that is held is sent
  anything more, and nothing times out while it is held. **Unholding** carries on with the same
  motions. A Holding or Unholding that the PMC refuses aborts the module with its ErrorID.
- **Stopping** waits until the skills started over OPC UA have stopped, and the shuttles have
  stopped. A Stop while held lets the paused motions finish first; an Abort stops them at once.

### Losing the PMC

A lost connection to the PMC aborts the table with PmcNotConnected (27). Another program taking
mastership of the PMC (PMT, another controller) aborts it with MastershipLost (32). Both are
noticed within a second. Clear, Reset and Start bring it back; the Reset takes mastership again.

## The MoveToPosition skill

A shuttle's skill, laid out as the lab's SKILL_Core:

```text
   Idle 0 --Start--> Running 1 --(arrived)--> Succeeded 3 --(1.5 s)--> Idle 0
     ^                 |    \
     |                 |     +--(cannot get there)--> Failed 4
     |                Stop
     |                 v
     |             Stopping 2 --(the motion it is on has ended)--> Failed 4, ErrorID 7
     |
     +--Reset-- Aborted 5 <--Abort-- (any state but Aborted)
```

| Method | |
| --- | --- |
| `Start(Session, X, Y, Rz)` | From Idle, Succeeded or Failed. X and Y in mm, 0 to 10000; Rz in degrees, -180 to 180. Answers at once; the skill turns Running. |
| `Stop(Session)` | From Running: the xbot ends the motion it is on (a straight run, which may cross several slots, or a turn it has begun) and stays there; then Failed with Interrupted (7). While held, it takes effect at the Unhold. |
| `Abort(Session)` | Aborted at once: the xbot stops where it is, even between slots. |
| `Reset(Session)` | From Aborted only: Idle. |

| Variable | |
| --- | --- |
| `State` | as above |
| `ErrorID` | why it failed; 0 otherwise |
| `Parameters/X`, `Y`, `Rz` | the target of the last Start |
| `Results/X`, `Y`, `Rz` | where the xbot ended (mm, degrees), also when it failed |

**Succeeded turns back to Idle after 1.5 s**; Failed stays until the next Start. A client that
polls `State` slower than that may see Running, then Idle: that was a success. Poll faster, or
subscribe.

### The gate

`Start` checks, in this order, and answers the first that fails:

1. The session holds the shuttle's occupation: NotPermitted (5).
2. The shuttle and the table are in Execute: NotReady (4). A held shuttle refuses too.
3. The skill is not Running or Stopping: Busy (6). Not Aborted: NotReady (4).
4. The parameters are in range: OutOfRange (8).
5. The PMC is connected (27); the xbot is there (28) and not in a fault (21), and has not failed
   before (see [After a failure](#after-a-failure)); it is not on its way already (6).
6. The target:
    - An Rz that is not a multiple of 90 degrees is only possible on a spot, a flyway centre
      or a point where four flyways meet: AngleNotAllowed (26).
    - The xbot's square at the target is entirely on flyways: NotOnTable (25).
    - It is in no zone that is switched off: ZoneClosed (33).
    - The xbot is not turned off the axes away from a spot, where it cannot travel:
      XbotCrooked (29).

### After a failure

When a move fails because of the xbot (XbotFault 21, PmcError 22, MotionTimeout 23,
XbotNotDetected 28, XbotCrooked 29, MotionDeviated 30), the PMC stops the xbot and the
controller remembers the failure. The shuttle refuses every Start with that ErrorID until it is
Stopped, Reset and Started again. Its Resetting checks the xbot afresh. Meanwhile the xbot stays
where it stopped, and the others go around it. The occupant keeps the shuttle throughout. See
[Recover a shuttle after a fault](examples.md#recover-a-shuttle-after-a-fault).

Blocked (20) and Interrupted (7) leave the xbot fit: the next Start is taken.

## Equipment

### The table: `PlanarTable/Equipment`

| Node | Type | |
| --- | --- | --- |
| `PMC/Connected` | Boolean | whether the controller is connected to the PMC |
| `PMC/Columns`, `Rows` | Int32 | the table in flyways, as the PMC reports it; 0 until the first Reset |
| `PMC/FlywaySize` | Double | mm (240) |
| `PMC/Holes` | String | the missing flyways as `"<column>,<row>"`, apart by spaces, e.g. `"2,1"`; empty when none |
| `PMC/Flyways/Flyway_<c>_<r>/PowerConsumption` | Double | W; added at the first Reset, read every 5 s, NaN until read |
| `PMC/Flyways/Flyway_<c>_<r>/CpuTemperature`, `AmplifierTemperature`, `MotorTemperature` | Double | °C, likewise. Planar Motor's simulator reports 0. |
| `LoadingQueue` | String | the free shuttles waiting in the loading queue, ready, the one on the loading station first, apart by spaces, e.g. `"PlanarShuttle3 PlanarShuttle1"`: the one to occupy for the next product. Empty without a loading station. |
| `IdleRz` | Double, writable | degrees, a multiple of 90: the orientation the free shuttles are turned to; NaN: as they are. Any other value is not taken and reads back as it was. Starts as `IDLE_RZ`. |
| `SetSpeed(Session, Speed, Acceleration)` | method | every xbot's speed (m/s) and acceleration (m/s²) at most. 0 or NaN sets that one back to `XBOT_SPEED` or `XBOT_ACCEL`. Whoever may command the table. See [Speed and payload](speed-and-payload.md). |
| `Speed`, `Acceleration` | Double | the table's, as they are now |
| `Mode` | String | `free` or `line` |
| `LineLayout` | String | line mode: the layout's JSON as it was last read; empty in free mode |
| `LineLayoutProblem` | String | line mode: why the last Reset did not take the line layout as it is now; empty when it did |
| `ZoneProblem` | String | why the last Reset did not define the zones; empty when it did |
| `Zones/<name>/State` | String | the PMC's state of the zone: `ACTIVATED_UNFENCED` (part of the table), `DEACTIVATING`, `LOADING_FENCED` (switched off), `ACTIVATING`, `ACTIVATED_FENCED` (on, its fence not removed yet), ...; `UNKNOWN` until the first Reset |
| `Zones/<name>/Xbots` | String | the xbot ids inside, as the PMC says, apart by spaces |
| `Zones/<name>/Deactivate(Session)`, `Activate(Session)` | method | switch the zone off and on; see [Zones](zones.md) |

### A shuttle: `PlanarShuttle<N>/Equipment/Xbot`

| Node | Type | |
| --- | --- | --- |
| `X`, `Y` | Double | mm, the xbot's live position; NaN until the PMC has been read, at the first Reset |
| `Rz` | Double | degrees in (-180, 180]; NaN likewise |
| `XbotState` | String | the PMC's state of the xbot: `XBOT_IDLE`, `XBOT_MOTION`, `XBOT_WAIT`, `XBOT_OBSTACLE_DETECTED`, `XBOT_DISABLED`, `XBOT_LANDED`, ...; `ACCIDENT (...)` when the PMC switched it off after an accident; `not detected` when the PMC does not report it; `UNKNOWN` until the first Reset |
| `SetPayload(Session, PayloadKg, CgHeight)` | method | the occupant's, while the skill is not running: what the xbot carries (kg) and its centre of gravity's height above the xbot (mm). See [Speed and payload](speed-and-payload.md#payload). |
| `PayloadKg`, `PayloadCgHeight` | Double | the payload as the PMC was told it, or has it (read at every Reset); NaN while unknown |
| `MaxAcceleration` | Double | m/s²: what the PMC allows with that payload, which its moves keep to; NaN while unknown |
| `SetSpeed(Session, Speed, Acceleration)` | method | keeps this xbot slower than the table (m/s, m/s²); 0 or NaN: no limit of its own. Whoever may command the shuttle. Answers XbotNotDetected (28) before the table's first Reset. |
| `Speed`, `Acceleration` | Double | what the xbot's moves are made with now, zones apart; NaN until the first Reset |

## ErrorID

A refused method answers `[false, ErrorID]`. A skill or procedure that fails shows its ErrorID
in `ErrorID`, beside its `State`.

| ErrorID | Name | When |
| --- | --- | --- |
| 0 | None | |
| 4 | NotReady | The module or skill is not in a state that takes the command: a skill Start while the shuttle or the table is not in Execute (or is held), a skill Reset that is not Aborted, Activate of a zone that is on, Deactivate of one that is not. |
| 5 | NotPermitted | The caller does not hold the occupation: occupying what another session holds, releasing or commanding what it does not hold. |
| 6 | Busy | The skill is running, or the xbot is on its way already. SetPayload while the xbot moves or is held. Deactivate while an xbot is on its way into, out of or across the zone, or stands across its edge. |
| 7 | Interrupted | Stopped before it arrived: a skill Stop, or a Stop of the shuttle or the table. |
| 8 | OutOfRange | A parameter out of range (X, Y 0 to 10000 mm; Rz -180 to 180 degrees), an empty session, a negative speed, a payload the xbot does not take. |
| 20 | Blocked | No motion of the move arrived within `BLOCKED_TIMEOUT_S`, when that is set. |
| 21 | XbotFault | The PMC reports the xbot in error, stopped, or detecting an obstacle. |
| 22 | PmcError | The PMC refused a command. |
| 23 | MotionTimeout | A motion did not arrive in time (half as long again as the PMC said it would take, and 2 s more). |
| 24 | ProcedureFailed | Resetting or Stopping did not complete. |
| 25 | NotOnTable | The target is not entirely on flyways: off the edge, or over a hole. |
| 26 | AngleNotAllowed | An Rz that is not a multiple of 90 degrees, away from a spot. |
| 27 | PmcNotConnected | No connection to the PMC: never made, or lost. |
| 28 | XbotNotDetected | The PMC does not report this xbot, or reports it switched off: missing, landed, in a zone switched off, after an accident. |
| 29 | XbotCrooked | The xbot is turned off the axes away from a spot and cannot travel. |
| 30 | MotionDeviated | A motion ended somewhere other than where it was sent. |
| 31 | LineLayoutInvalid | Line mode: at the table's Reset, the line layout as read again is no layout, or does not fit the table. `LineLayoutProblem` says why. |
| 32 | MastershipLost | Another program (PMT, another controller) has taken mastership of the PMC, or kept it at the Reset. |
| 33 | ZoneClosed | The target is in a zone that is switched off or fenced. |
| 34 | ZonesInvalid | At the table's Reset, a zone of `ZONES` is not on the table, or the PMC would not define it. `ZoneProblem` says why. |
