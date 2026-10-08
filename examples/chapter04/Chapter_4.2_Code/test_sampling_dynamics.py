"""Meaningful geometry, rewiring, query independence and lattice-cost checks."""
import math
from sampling_dynamics import *

def main():
    scene=Scene();start=(1.0,3.0);goal=(13.0,3.0)
    a=prm(scene,start,goal);b=prm(scene,(1.0,2.0),(13.0,2.0))
    assert a.nodes[:250]==b.nodes[:250]
    assert [e for e in a.edges if max(e)<250]==[e for e in b.edges if max(e)<250]
    assert not scene.segment_free((1,3),(13,3))
    assert not scene.segment_free((6,6),(7,6)), 'Obstacle contact must be rejected'
    for seed in [3,17,41]:
        result=rrt(scene,start,goal,star=True,seed=seed)
        assert result.path and result.rewires>0
        validate_geometric(scene,result,start,goal)
    initial=(*start,2.0,0.0);target=(*goal,0.0,0.0)
    informed=kinodynamic_search(scene,initial,target)
    truth=kinodynamic_search(scene,initial,target,use_h=False)
    assert abs(informed.cost-truth.cost)<1e-7
    assert informed.states[-1][2:]==(0.0,0.0)
    thin=Scene(4,3,((.9,1.1,.4,.6),))
    assert thin.free((0,.5)) and thin.free((2,.5))
    assert primitive_status(thin,(0,.5,2,0),(0,0))=='collision'
    # The quadratic reaches its maximum between endpoints and crosses the top boundary.
    cap=Scene(5,2.1,())
    assert primitive_status(cap,(1,2,0,1),(0,-1),tau=2)=='bounds'
    blocked=Scene(14,10,((6,7,0,10),))
    fail=rrt(blocked,start,goal,budget=100)
    assert not fail.path and math.isinf(fail.cost)
    print('Sampling/dynamics checks passed; lattice optimum',truth.cost,
          'A* expansions',informed.expanded,'zero-heuristic expansions',truth.expanded)

if __name__=='__main__':main()
