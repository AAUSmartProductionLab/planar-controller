# Zones

A zone is a rectangle of the table that the PMC knows as a zone. Zones have two uses:

- **Switch a zone off** to take xbots off the table there, or put new ones on, while the rest of
  the table goes on.
- **Slow the xbots down** inside a zone, e.g. near a station that handles open vials.

## Defining zones

`ZONES` names a JSON file (in Docker, a file in `config/`) or an `http(s)://` URL.
`config/zones.example.json`:

```json
{"zones": [
  {"name": "Service", "from": [720, 480], "to": [960, 720]},
  {"name": "Filling", "from": [240, 480], "to": [480, 720], "speed": 0.5}
]}
```

| Key | |
| --- | --- |
| `name` | The zone's name over OPC UA: a letter, then letters, digits or `_`, up to 64 characters. Unique. |
| `from`, `to` | Opposite corners in mm, in the table's coordinates. Best on flyway edges (multiples of 240). |
| `speed` | More than 0 and at most 1, default 1: scales the speed and acceleration of every xbot inside. |

A file that is not such a list stops the controller at start-up with the reason. The zones are
defined on the PMC at every Reset of the table, numbered 1, 2, ... in the file's order. A zone
that is not on the table, or that the PMC will not define, fails the Reset with ZonesInvalid
(34), and `PlanarTable/Equipment/ZoneProblem` says why.

## Over OPC UA

Each zone is `PlanarTable/Equipment/Zones/<name>`:

| Node | |
| --- | --- |
| `State` | The PMC's state of the zone, read every 0.5 s; `UNKNOWN` until the first Reset. |
| `Xbots` | The ids of the xbots inside, as the PMC says, apart by spaces. |
| `Deactivate(Session)` | Switch it off. |
| `Activate(Session)` | Switch it on again. |

Only the table's occupant may switch a zone, or anyone while the table is not occupied.

The states that matter:

| State | |
| --- | --- |
| `ACTIVATED_UNFENCED` | On, and part of the table: the normal state. |
| `DEACTIVATING` | Being switched off. |
| `LOADING_FENCED` | Off and fenced: the xbots inside are switched off, and may be taken off or put on. |
| `ACTIVATING` | Being switched on: the PMC levitates the xbots inside. |
| `ACTIVATED_FENCED` | On, but still fenced: the controller removes the fence next. |

The PMC has a few more (`UNDEFINED`, `ERROR`, `LANDED`, `LEVITATED`, `DISCOVER`).

## Switching a zone off

`Deactivate` is refused while the zone is in use:

- NotReady (4) when the zone is not on (`ACTIVATED_UNFENCED`);
- Busy (6) when an xbot stands across the zone's edge;
- Busy (6) when an xbot is on its way into, out of or across it: moving, planned to, turning or
  docking there, or with its target inside.

Then the PMC fences the zone and switches off the xbots inside. Their shuttles' `XbotState`
shows the PMC's state for that, e.g. `XBOT_DISABLED`. A move of one fails, and a Start is
refused, with XbotNotDetected (28). An operator may now take them off the table, or put others
on.

## Switching it on again

`Activate` has the PMC levitate the xbots inside. Once the PMC reports the zone
`ACTIVATED_FENCED`, the controller removes the fence, and the zone is `ACTIVATED_UNFENCED` again.
`Activate` answers NotReady (4) when the zone is on already.

An xbot that was taken off reads as missing: its shuttle refuses moves with XbotNotDetected (28)
until it is back. An xbot put on that the controller did not know takes part as a wall: it stays
where it is, since it is not offered as a shuttle and nobody may move it. It is offered once the
controller starts again with it (in `SHUTTLES`, or found on the PMC when `SHUTTLES` is empty).

## While a zone is closed

A zone is closed whenever it is not `ACTIVATED_UNFENCED`. The PMC would stop an xbot at its
fence with an obstacle, so:

- the planner keeps every xbot out of it, and plans no straight move through it;
- no xbot turns on a spot whose flyway square reaches into it, and no claim reaches into it;
- a target in it is refused with ZoneClosed (33);
- the xbots inside are not moved.

## Slow zones

A zone's `speed` is the PMC's override of speed and acceleration inside it: at 0.5, the xbots
move at half speed and half acceleration there. The PMC leaves the factor out of the time it
says a motion takes, so the controller stretches a motion's timeout by the slowest zone it
passes through. A motion that would otherwise time out arrives in its own time.

## Limits

- pmclib has no command to delete a zone. A zone stays defined on the PMC until the PMC (or the
  simulation) restarts, so a zone removed from `ZONES` keeps its last definition there,
  including its speed.
- The zones are numbered by their place in the file: reordering it redefines them at the next
  Reset.

For a client that switches a zone off and on, see [Examples](examples.md#take-xbots-off-through-a-zone).
