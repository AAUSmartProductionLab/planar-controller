# Clockwise, facing outwards

The 4 x 3 flyway table (the hole at flyway (2, 1) or not). The shuttles go round clockwise only,
on the ring one slot in from the edge, and turn on the edge flyways' centres, half a flyway in
from the edge. At each station a shuttle faces the module on that side of the table, so it turns
a quarter turn clockwise for every new side:

```
TTTTTTTT     > < ^ v  the one-way loop, the way on
I>>>>>vT     I        the idle area: the loading queue, the loading position on top
I^##..vT     T        what a turn on a turn spot sweeps (a spot on each edge flyway's centre)
I^##..vT     .        the missing flyway (2, 1), on the lab's table
T^<<<<<T
TTTTTTTT     north at the top
```

| Station | Where (mm) | Side | Rz | |
| --- | --- | --- | --- | --- |
| Infeed | 60, 540 | west | 180 | loading: the head of the loading queue |
| Filling | 300, 660 | north | 90 | |
| Stoppering | 900, 300 | east | 0 | |
| Capping | 540, 60 | south | -90 | |
| Outfeed | 60, 180 | west | 180 | unloading, below the queue |

The free shuttles wait in the queue facing the loading module (`IDLE_RZ=180`). A shuttle turns
on the turn spot nearest to it, which is at the station it leaves: it goes on facing the next
side's module. A product discarded after a step goes straight to the Outfeed, turning +90 deg
(after Filling) or half a turn (after Stoppering).

On the Planar Motor simulator (3 xbots, 3 agents x 4 products) all products went through, every
lane step clockwise, every turn on a turn spot by a quarter turn (a half turn for a product
discarded after Stoppering).
