"""Checks whole-segment feasibility, event execution and paired 3D inputs."""
import math
from simulations import Scene3D,rrt3d,moving_replanning,validate3d

def main():
    events=moving_replanning()['snapshots']
    assert [e['step'] for e in events]==[0,14,20,23]
    assert [tuple(e['current']) for e in events]==[(4,8),(18,8),(18,14),(21,11)]
    assert events[-1]['status']=='hold' and events[-1]['remaining_cost'] is None
    scene=Scene3D()
    pairs=[((5,5,1),(8,5,1),False),((5,5,3.4),(8,5,3.4),False),
           ((5,5,3.41),(8,5,3.41),True),((8,5,2.2),(11,5,2.2),False),
           ((8,5,2.19),(11,5,2.19),True)]
    for a,b,expected in pairs:
        assert scene.segment_free(a,b)==expected
        assert scene.segment_free(b,a)==expected
    # Collision between free endpoints in a thin 3D box.
    thin=Scene3D(boxes=((6.,6.01,4.,6.,.5,2.),))
    assert thin.free((5,5,1)) and thin.free((8,5,1))
    assert not thin.segment_free((5,5,1),(8,5,1))
    a=rrt3d(Scene3D(),star=False);b=rrt3d(Scene3D(),star=True)
    assert a.nodes==b.nodes and b.rewires>0 and b.cost<a.cost
    low=rrt3d(Scene3D(high=(14,10,3.4)),budget=300)
    assert not low.path and low.cost is None
    print('Simulation checks passed: moving hold, whole 3D segments, paired inputs, finite-budget failure.')

if __name__=='__main__':main()
