# Line layouts to test with

Each folder is a line-mode layout with stations for the product agents of
`tests/line_scenario.py` (Infeed, Filling, Stoppering, Capping, Outfeed, in that order):

| File | |
| --- | --- |
| `layout.json` | the controller's `LINE_LAYOUT` (see [Line mode](../../README.md#line-mode)) |
| `stations.json` | `{"<station>": [x mm, y mm, rz deg]}`, where the agents send the shuttles |
| `controller.env` | the controller's settings for it: `MODE=line`, `LINE_LAYOUT`, and e.g. `IDLE_RZ` |
| `README.md` | what it is for, the map, what a run showed |

| Layout | |
| --- | --- |
| [clockwise-outwards](clockwise-outwards/) | one-way clockwise round the edge, each station facing its module, a turn spot on every edge flyway |

## Testing one

`pytest tests/test_line.py` runs the agents on every layout here on the simulated table (4 x 3
flyways with the hole at (2, 1), 3 xbots): the products must go through, the shuttles never
collide, step along a lane only its way and turn only on turn spots, by at most half a turn.

On Planar Motor's simulator, every layout in one go: `python tools/sim_check.py` (with the Python
the controller runs with; `--start-sim` starts the simulator first, `clockwise-outwards` only that
one). For each it runs a controller with the layout's settings on a free port, then the scenario
below, and prints a line, PASS or FAIL:

```
PASS clockwise-outwards (83 s): 9 products: 8 finished, 1 discarded; ...; 39 lane steps; turns 1x -180 deg, 34x -90 deg; 0 problems
```

`--hold-every 8` holds the table every 8 s during the scenario (nothing may move while it is
held); `--mastership` then takes mastership from another session (the table must abort with
MastershipLost and take it back at a Reset).

Against a controller you run yourself, from the repository's root:

```sh
# the controller, with the layout's settings (PMC_IP=127.0.0.1 for the simulator)
docker compose --env-file .env --env-file config/layouts/clockwise-outwards/controller.env up -d
# or without Docker: export the settings of controller.env, then python -m planar

python tests/line_scenario.py --endpoint opc.tcp://127.0.0.1:4840/ --layout config/layouts/clockwise-outwards --agents 3 --products 4
```

The scenario resets and starts the table, runs the agents and watches the shuttles over OPC UA
with the same checks; it prints each product and a summary (lane steps, turns by how much) and
exits 1 on any problem.

## Adding one

Make a folder with the four files. Draw the layout in the AAS Web UI's Configurator (Shuttle
routing) or by hand; `python -c "from planar.lines import load, Lines; from planar.layout import
Layout; print(Lines(load('config/layouts/<name>/layout.json'), Layout(4, 3, frozenset({(2, 1)}))).describe())"`
checks it fits the table and draws it. Put each station on a slot of no lane, beside one (a
station on a lane blocks it while the shuttle there is held).
