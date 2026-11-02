# Configuration

The controller reads its settings from environment variables. With Docker Compose they come from
`.env` in the repository's folder (`install.sh` makes it from `.env.example`). Without Docker,
each setting is also a command-line option of `python -m planar`; an option given wins over the
variable.

## Settings

| Variable | Option | Default | |
| --- | --- | --- | --- |
| `PMC_IP` | `--pmc-ip` | empty: search | The PMC's IP address. Empty: search the local network for it (needs host networking in Docker). The simulator: `127.0.0.1`. |
| `OPCUA_PORT` | | `4840` | Docker Compose only: the OPC UA port. The server is `opc.tcp://<this machine>:<port>/`. |
| `OPCUA_ENDPOINT` | `--endpoint` | `opc.tcp://0.0.0.0:4840/` | Without Docker: where the OPC UA server listens. |
| `SHUTTLES` | `--shuttles` | the ids the PMC knows | The xbot ids to offer as shuttles, e.g. `1,2,3`. Xbots on the table that are not offered still take part in planning, but nobody here moves them (see [pinned xbots](planner.md#pinned-xbots)). |
| `AUTOSTART` | `--autostart` | `0` | `1`: Reset and Start the table when the controller starts, and keep trying every 10 s while the PMC is not there yet. `0`: the table waits Stopped for Reset and Start over OPC UA. |
| `DISPLACE_HELD` | `--displace-held` | `0` | `1`: the planner may push occupied shuttles that are not moving out of the way. Leave it at `0` when a station may be working on a shuttle that stands still. |
| `BLOCKED_TIMEOUT_S` | `--blocked-timeout` | `0` | Seconds a move may go without any motion arriving before it fails with Blocked (20). `0`: it waits until the way is free, or until it is stopped. |
| `LOADING_STATION` | `--loading-station` | empty | `X,Y` in mm, e.g. `60,180`. The free xbots queue up on the slots nearest to it, the first one on it, and the table's Reset sends them there. Empty: a free xbot stays where it was. See [the loading queue](planner.md#the-loading-queue). |
| `IDLE_RZ` | `--idle-rz` | empty | Degrees, a multiple of 90, e.g. `0`. The free xbots are turned to it, and the table's Reset turns every shuttle to it. Empty: as they are. Also `PlanarTable/Equipment/IdleRz` over OPC UA. |
| `MODE` | `--mode` | `free` | `free`: the xbots may travel anywhere on the table. `line`: only on the lanes of `LINE_LAYOUT`. See [Line mode](line-mode.md). |
| `LINE_LAYOUT` | `--line-layout` | empty | Line mode: the layout, a JSON file (in Docker, a file in `config/`) or an `http(s)://` URL, e.g. the `$value` of a `PlanarRouting` collection in an AAS. Read again at every Reset of the table. |
| `ZONES` | `--zones` | empty | Zones of the table: a JSON file or URL. See [Zones](zones.md). |
| `XBOT_SPEED` | `--speed` | `0.5` | m/s: every xbot's speed, at most. `PlanarTable/Equipment/SetSpeed` changes it while the controller runs. See [Speed and payload](speed-and-payload.md). |
| `XBOT_ACCEL` | `--accel` | `2.0` | m/s²: every xbot's acceleration, at most. |
| `XBOT_JERK` | `--jerk` | empty: the PMC's | m/s³: every xbot's jerk limit, set on the PMC at every Reset. It smooths starts and stops. |
| `LOG_LEVEL` | `--log-level` | `INFO` | `DEBUG`, `INFO`, `WARNING` or `ERROR`, in any case. |

Flags (`AUTOSTART`, `DISPLACE_HELD`) take `1`, `true`, `yes` or `on`; anything else is off.

## What is checked when it starts

The controller stops at once, with one line that says why, when a setting is wrong:

```text
python -m planar: error: argument --speed: not a number: 'abc'
python -m planar: error: argument --speed: not a number above 0: '0'
python -m planar: error: argument --shuttles: not xbot ids like 1,2,3: '1,a'
python -m planar: error: argument --idle-rz: not a multiple of 90 degrees: '45'
MODE=line needs LINE_LAYOUT (a JSON file or URL)
LINE_LAYOUT config/my-line.json: lanes: a list of at least one lane
ZONES config/zones.json: zones[0] (Service): speed must be more than 0 and at most 1
OPC UA endpoint opc.tcp://0.0.0.0:4840/: its port is in use by another program (...)
```

It waits rather than stops in two cases, since the other side may still be starting up:

- **`SHUTTLES` empty, the PMC not reachable**: it asks the PMC for its xbots every 5 s.
- **`LINE_LAYOUT` a URL that cannot be reached**: it tries again every 10 s.

With `LINE_LAYOUT` set but `MODE=free`, it warns that the lanes are not used.

## What can change while it runs

| | How |
| --- | --- |
| The table's speed and acceleration | `PlanarTable/Equipment/SetSpeed`; `0` or NaN goes back to `XBOT_SPEED` and `XBOT_ACCEL` |
| A shuttle's own speed limit | `PlanarShuttle<N>/Equipment/Xbot/SetSpeed` |
| A shuttle's payload | `PlanarShuttle<N>/Equipment/Xbot/SetPayload` |
| The idle orientation | write `PlanarTable/Equipment/IdleRz` |
| A zone, on or off | `PlanarTable/Equipment/Zones/<name>/Deactivate`, `Activate` |
| The line layout | change the file, or the AAS, and Reset the table: `LINE_LAYOUT` is read again at every Reset |

A Reset of the table also reads the table's layout from the PMC again and defines the zones on
the PMC again. Everything else needs a restart of the controller (`docker compose up -d` after
changing `.env`).

The speeds set over OPC UA last until they are set again or the controller restarts; a restart
starts from `XBOT_SPEED` and `XBOT_ACCEL` again. The PMC keeps a payload until it is told
another.

## Examples

**Free mode, a loading station at the infeed.** The free shuttles queue up along the table's
edge from (60, 180) mm, facing east, and the controller brings the table up by itself:

```bash
PMC_IP=192.168.10.100
AUTOSTART=1
LOADING_STATION=60,180
IDLE_RZ=0
```

**Line mode, from a file in `config/`:**

```bash
PMC_IP=192.168.10.100
MODE=line
LINE_LAYOUT=config/my-line.json
IDLE_RZ=180
```

**Line mode, from the line's AAS.** The layout is the `$value` of the `PlanarRouting`
collection in the line's `Parameters` submodel; the AAS Web UI's Configurator shows the URL:

```bash
MODE=line
LINE_LAYOUT=http://aas-env:8081/submodels/<base64url submodel id>/submodel-elements/PlanarRouting/$value
```

**Zones**, to take xbots off at a service corner and slow down at the filling station:

```bash
ZONES=config/zones.json
```

```json
{"zones": [
  {"name": "Service", "from": [720, 480], "to": [960, 720]},
  {"name": "Filling", "from": [240, 480], "to": [480, 720], "speed": 0.5}
]}
```

**Planar Motor's simulator on a Windows PC, without Docker** (port 4840 is taken by the OPC UA
Local Discovery Server there):

```powershell
$env:PMC_IP = "127.0.0.1"
$env:OPCUA_ENDPOINT = "opc.tcp://127.0.0.1:4850/"
$env:AUTOSTART = "1"
python -m planar
```

**Several settings files.** Docker Compose reads more than one, the later ones winning. The
layouts in `config/layouts/` each have their settings in a `controller.env`:

```bash
docker compose --env-file .env --env-file config/layouts/clockwise-outwards/controller.env up -d
```

## Docker

`docker-compose.yml` passes the settings into the container, mounts `config/` read-only at
`/app/config`, and uses host networking. The container restarts unless it was stopped, keeps
five log files of 10 MB, and is healthy while the OPC UA port takes connections. The image is
Python 3.11 with mono and pmclib; its entry point is `python -m planar`, so options given to
`docker compose run` are passed on.
