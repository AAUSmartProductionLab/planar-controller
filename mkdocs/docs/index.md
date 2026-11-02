# Planar controller

The planar controller drives a Planar Motor table, the PMC and its xbots, and offers it to a
production line over OPC UA. It was built for the AAU Smart Production Lab and is laid out like
the lab's generated IEC 61499 modules: the table and each xbot are a module with an occupation,
a PackML state machine and skills.

A client (an orchestrator, a product agent) works with it in four steps:

1. Reset and Start the table, once.
2. Occupy a shuttle with a session of its own.
3. Run the shuttle's `MoveToPosition` skill to each station, as often as it likes.
4. Release the shuttle.

The controller does the rest. Every xbot on the table moves as part of one plan, so shuttles get
past each other in narrow passages, and idle ones step aside and come back. Requests for the
same place are served in the order they were made.

## What it does

- **Plans for every xbot at once** with LaCAM and PIBT, a complete multi-agent path finder: if
  every xbot can reach its target, it finds a way. Of several plans it takes the one estimated
  to be done soonest. See [The planner](planner.md).
- **Executes plans by dependencies, not by time**: an xbot enters a slot only once the xbot
  planned there before it has left. A slow or late xbot cannot cause a collision.
- **Reaches any pose on the table**: stations between slots, on a flyway centre, or turned to
  any angle on a flyway centre. Xbots turn only where the PMC can turn them.
- **Free mode or line mode**: anywhere on the table, or only on lanes you lay out, so the way
  between two places is always the same. See [Line mode](line-mode.md).
- **Keeps the free shuttles in order**: queued at a loading station (the head of the queue
  published, so the next product takes it) and turned to one orientation.
- **PackML behaviour**: Hold pauses the xbots in mid-move and Unhold carries on, Stop lets them
  finish, Abort stops them at once. The table aborts within a second when the PMC connection is
  lost, or when another program takes mastership of the PMC.
- **The PMC's other features over OPC UA**: speed per table and per shuttle, payload, zones to
  take xbots off and put them on, and each flyway's power and temperatures. See
  [Speed and payload](speed-and-payload.md) and [Zones](zones.md).
- **Runs in Docker** from one settings file, and is installed with one command. See
  [Getting started](getting-started.md).

## How it fits together

```text
   orchestrator, product agents, UaExpert, ...
                     |
                     |  OPC UA: opc.tcp://<host>:4840/, namespace urn:aau:smartproductionlab:planar
                     v
 +------------------------------------------------------------------------------+
 | OPC UA server                                                opcua_server.py |
 |   PlanarTable and PlanarShuttle1..N: Occupation, Module, Skills, Equipment   |
 +------------------------------------------------------------------------------+
 | Modules                                         modules.py, statemachines.py |
 |   the occupation, the PackML state machine, the MoveToPosition skill         |
 +------------------------------------------------------------------------------+
 | Coordinator                                                   coordinator.py |
 |   requests -> tasks -> one plan for every xbot -> motions, every 20 ms       |
 |   the planner (mapf.py), the table (layout.py), lanes (lines.py), zones.py   |
 +------------------------------------------------------------------------------+
 | Driver                                                             driver.py |
 |   PmcDriver: the PMC through pmclib         SimDriver: a stand-in for tests  |
 +------------------------------------------------------------------------------+
                     |
                     v
          the PMC, or Planar Motor's simulator
```

The code is in `planar/`; [Architecture](architecture.md) goes through it.

## Words used in these pages

| Word | Meaning |
| --- | --- |
| PMC | The Planar Motor Controller, the table's controller. The planar controller talks to it through Planar Motor's Python library, pmclib. |
| xbot | A mover that levitates on the table, 120 × 120 mm. |
| flyway | One 240 × 240 mm tile of the table. The table is a grid of them; some may be missing (holes). |
| slot | A 120 × 120 mm square, a quarter of a flyway. The planner plans on the grid of slots. |
| table | The module `PlanarTable`: the PMC and everything on it. |
| shuttle | An xbot as a module: `PlanarShuttle<N>` for xbot N. |
| session | A string, best a UUID, that a client picks and sends with every command. |
| occupation | Which session holds a module. Only that session may command the module. |
| skill | An operation of a module with its own state machine: a shuttle's `MoveToPosition`. |
| on the lattice | An xbot on a slot centre with its sides along the axes. The planner may move it. |
| docked | An xbot anywhere else: at a station between slots, on a flyway centre, turned. |
| spot | Where the PMC can turn an xbot: a flyway centre, or a point where four flyways meet. |
| claim | The slots an xbot reserves for a turn, or for its last moves onto a station between slots. |
| pinned | An xbot the planner may not push: its shuttle is occupied and not moving, for instance. |
| loading queue | Where the free shuttles wait, the head of it on the loading station. |
| lane, turn spot, idle area | The parts of a line layout in [line mode](line-mode.md). |
| zone | A rectangle of the table that the PMC can switch off, or where it slows the xbots down. |

Positions over OPC UA are in millimetres in the table's coordinates: the origin is the table's
corner, X runs along the columns, Y along the rows. Slot centres are at 60, 180, 300, ... mm.
Angles are in degrees, counterclockwise, in (-180, 180]. Inside the code, as in pmclib,
positions are in metres and angles in radians.

## Where to go next

- To run it: [Getting started](getting-started.md), then [Configuration](configuration.md).
- To write a client: [OPC UA interface](opcua-interface.md) and [Examples](examples.md).
- To understand how the xbots move: [The planner](planner.md).
- When something goes wrong: [Troubleshooting](troubleshooting.md).
