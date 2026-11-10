"""Extend the classic coordinator with optional buffered, rounded XY corners."""

import logging

from .coordinator import Coordinator as ClassicCoordinator, Motion
from .driver import DriverError
from .layout import hull, is_square_aligned
from .rounded_path import RoundedPath
from .statemachines import ErrorID

log = logging.getLogger(__name__)


class Coordinator(ClassicCoordinator):
    """Reuse classic coordination while blending clear corners in free mode."""
    def _candidate(self, agent):
        """Return an eligible corner and swept boxes; dispatch checks traffic clearance."""
        if (self.lines or not self.driver.supports_blended_moves or agent.motion is not None
                or agent.ops or agent.claim or agent.fault or agent.paused_at is not None
                or agent.task is None or agent.task.halting or agent.cell is None
                or not is_square_aligned(agent.pose.rz)):
            return None
        # Only commit actions whose traffic dependencies are already complete. At most
        # two straight runs: stop at the endpoint before choosing another continuation.
        runs, directions = [], []
        previous = agent.cell
        for action in agent.actions:
            if action.frm != previous or not action.deps <= self._done:
                break
            step = (action.to[0]-action.frm[0], action.to[1]-action.frm[1])
            if abs(step[0])+abs(step[1]) != 1:
                break
            if not directions or step != directions[-1]:
                if len(runs) == 2:
                    break
                if directions and sum(a*b for a, b in zip(step, directions[-1])) != 0:
                    break  # no reversals
                directions.append(step)
                runs.append([])
            runs[-1].append(action)
            previous = action.to
        if len(runs) != 2:
            return None
        start = self.layout.center(agent.cell)
        corner = self.layout.center(runs[0][-1].to)
        end = self.layout.center(runs[1][-1].to)
        path = RoundedPath(start, corner, end, self.layout.pitch/4)
        boxes = path.boxes(self.layout, agent.pose.rz)
        if any(not self.layout.footprint_of(b) <= self.layout.cells-self._closed_cells() for b in boxes):
            return None
        return path, boxes

    def _dispatch(self, now):
        """Claim eligible corners as macros, then delegate remaining dispatch to the classic coordinator."""
        for agent in self.agents.values():
            candidate = self._candidate(agent)
            if candidate is None:
                continue
            path, boxes = candidate
            if any(self._in_trail(agent, b, now) for b in boxes):
                continue
            if not self._claim(agent, boxes, immediate=True):
                continue
            # Treat the entire path as a claimed macro. Replanning keeps its swept area
            # unavailable to everybody else until arrival, Hold, failure or Abort resolve.
            agent.actions.clear()
            self._run_ops(agent, [('blend', path)], boxes)
            self._signature = None
        super()._dispatch(now)

    def _send_op(self, agent, now):
        """Send a buffered corner with a reserved trail, or delegate ordinary macro operations."""
        if agent.ops[0][0] != 'blend':
            return super()._send_op(agent, now)
        path = agent.ops[0][1]
        if any(self._in_trail(agent, b, now) for b in agent.claim_boxes):
            return
        agent.ops.popleft()
        try:
            expected = self.driver.move_blended(agent.xid, path, *self.speed_of(agent))
            trail = hull(self.layout.trail(path.start, path.corner),
                         self.layout.trail(path.corner, path.end))
            agent.motion = Motion('blend', path.end, now,
                                  expected/self._zone_speed(agent.claim), trail=trail)
            log.info('xbot %d: rounded XY corner, radius %.0f mm', agent.xid, path.radius*1000)
        except DriverError as exc:
            log.error('xbot %d: blended move refused: %s', agent.xid, exc)
            self._fail(agent, ErrorID.PMC_ERROR)
