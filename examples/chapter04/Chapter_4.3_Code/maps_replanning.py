"""Small deterministic mechanisms for Sections 4.3.5--4.3.6.

Analytic circle distances stand in for a map query in a horizontal slice. This
does not implement a grid ESDF, ROS, a controller, or a flight safety system.
"""
from dataclasses import dataclass
from pathlib import Path
import json
import math
import numpy as np
from numpy.polynomial import polynomial as poly
from minco import generate,basis


def occupancy_update(log_odds, hit):
    """Prior p=0.5; the supplied inverse-sensor increments are illustrative."""
    return float(np.clip(log_odds + (.85 if hit else -.4), -2., 3.5))


def occupancy_class(log_odds, observed):
    if not observed:
        return 'unknown'
    p = 1. / (1. + math.exp(-log_odds))
    return 'occupied' if p >= .65 else 'free' if p <= .35 else 'unknown'


@dataclass(frozen=True)
class Circle:
    center: tuple = (2., 0.)
    radius: float = .4

    def query(self, point):
        delta = np.asarray(point, float) - self.center
        length = float(np.linalg.norm(delta))
        if length < 1e-14:
            raise ValueError('Distance gradient is not unique at the center')
        return length-self.radius, delta/length


def obstacle_penalty(point, obstacle, required=.3, weight=1.):
    distance, gradient = obstacle.query(point)
    violation = max(0., required-distance)
    return weight*violation**2, -2.*weight*violation*gradient


def interval_clearance(distances, speed_bound, max_step, required, error=0.):
    """Sufficient test only: true static distance is 1-Lipschitz; endpoints included.

    Caller must supply a valid speed bound, bounded distance error, and known
    space over the complete intervals. Failure means inconclusive, not collision.
    """
    if min(speed_bound, max_step, required, error) < 0:
        raise ValueError('nonnegative bounds required')
    values = np.asarray(distances, float)
    return bool(values.size and np.all(np.isfinite(values)) and
                np.min(values) >= required+error+speed_bound*max_step/2.)


def fixed_sample_penalty(tr, samples, obstacle, required=.3):
    """Cost and coefficient partials at supplied fixed segment-local times.

    Each sample is (segment_index, physical_local_time, positive_weight).
    This function does not differentiate a duration-dependent sampling rule.
    """
    value=0.;gc=np.zeros_like(tr.coefficients)
    for i,theta,weight in samples:
        if not 0<=theta<=tr.durations[i] or weight<=0:
            raise ValueError('sample must be in segment with positive weight')
        cost,gradient=obstacle_penalty(tr.value(i,theta),obstacle,required,weight)
        value+=cost;gc[i]+=np.outer(basis(0,theta),gradient)
    return value,gc


def at(tr, elapsed, derivative=0):
    if elapsed < -1e-10 or elapsed > sum(tr.durations)+1e-10:
        raise ValueError('outside trajectory time interval')
    elapsed = float(np.clip(elapsed, 0., sum(tr.durations)))
    i = min(int(np.searchsorted(np.cumsum(tr.durations), elapsed, side='right')),
            len(tr.durations)-1)
    return tr.value(i, elapsed-sum(tr.durations[:i]), derivative)


def circle_minimum(tr, obstacle):
    """Numerical whole-polynomial distance minimum via stationary roots.

    A teaching check, with floating-point root tolerance, not a formal proof.
    """
    best = (math.inf, None)
    for i, c in enumerate(tr.normalized_coefficients()):
        delta = c.copy();delta[0] -= obstacle.center
        squared = sum((poly.polymul(delta[:, j], delta[:, j]) for j in range(2)),
                      start=np.zeros(15))
        roots = poly.polyroots(poly.polyder(squared))
        points = [0., 1.] + [float(z.real) for z in roots
                             if abs(z.imag) < 1e-7 and 0 < z.real < 1]
        for s in points:
            distance = math.sqrt(max(0., float(poly.polyval(s, squared))))-obstacle.radius
            if distance < best[0]:best=(distance, sum(tr.durations[:i])+s*tr.durations[i])
    return best


@dataclass(frozen=True)
class Ticket:
    parent_id: int
    checked_map: int
    switch_time: float
    frame: str = 'world'


def admission(ticket, now, active_id, current_map, boundary_error,
              prefix_ok=True, candidate_ok=True, tracking_ok=True,
              frame='world', reserve=.05):
    """Illustrative admission gate; real validation supplies the three booleans.

    A changed map requires fresh validation and a refreshed checked_map ticket.
    Calling this function does not perform collision or dynamics validation.
    """
    if not math.isfinite(now) or now > ticket.switch_time-reserve:return 'late'
    if active_id != ticket.parent_id:return 'stale-parent'
    if frame != ticket.frame:return 'wrong-frame'
    if current_map != ticket.checked_map:return 'revalidate-map'
    if not math.isfinite(boundary_error) or boundary_error > 1e-7:return 'discontinuous'
    if not prefix_ok or not tracking_ok:return 'recover-prefix'
    if not candidate_ok:return 'reject-candidate'
    return 'scheduled'


def handoff_case():
    switch = 1.6
    derivatives = np.array([[1., 0.], [0., 0.], [0., 0.]])
    tr = generate([[switch, 0.], [2.1, 1.3], [5., 0.]], [1.4, 3.],
                  start=derivatives)
    expected = np.vstack(([switch, 0.], derivatives))
    error = max(float(np.max(np.abs(at(tr, 0., r)-expected[r]))) for r in range(4))
    return tr, error


def run():
    obstacle=Circle()
    cost,grad=obstacle_penalty([1.4, 0.], obstacle)
    tr,error=handoff_case();ticket=Ticket(7,12,1.6)
    new_obstacle=Circle((2.7, 0.), .35)
    minimum,where=circle_minimum(tr,new_obstacle)
    result={
        'scope':'analytic static 2D slice; no grid-ESDF/ROS/controller reproduction',
        'occupancy':{'two_hits':occupancy_class(occupancy_update(occupancy_update(0., True),True),True),
                     'three_misses':occupancy_class(-1.2,True),'unobserved':occupancy_class(0.,False)},
        'penalty':{'point':[1.4,0.],'distance':.2,'required':.3,'cost':cost,'gradient':grad.tolist()},
        'sampling':{'endpoints':[[0.,0.],[4.,0.]],'sample_distances':[1.6,1.6],
                    'required':.3,'sample_only_passes':True,'actual_minimum_distance':-.4,
                    'interval_test_passes':interval_clearance([1.6,1.6],1.,4.,.3)},
        'handoff':{'request_time':1.,'switch_time':1.6,'ready_time':1.3,
                   'durations':tr.durations.tolist(),'waypoints':[[1.6,0.],[2.1,1.3],[5.,0.]],
                   'derivative_mismatch_0_to_3':error,
                   'minimum_new_obstacle_distance':minimum,'minimum_at_local_time':where,
                   'required':.2,'on_time':admission(ticket,1.3,7,12,error,candidate_ok=minimum>=.2),
                   'late':admission(ticket,1.58,7,12,error),
                   'map_changed':admission(ticket,1.3,7,13,error),
                   'parent_replaced':admission(ticket,1.3,8,12,error),
                   'wrong_clock_reset_position_error':.3},
        'stopping':{'speed':2.,'latency':.2,'deceleration':2.,'margin':.3,
                    'ideal_required_distance':2.*.2+2.**2/(2.*2.)+.3}}
    out=Path(__file__).resolve().parent/'results/maps_replanning.json'
    out.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result,indent=2))
    return result


if __name__=='__main__':run()
