"""Run with: python test_planning.py. No third-party dependencies."""
import math
import random
from planning import *

def bellman_ford(grid,start):
    nodes=[(x,y) for y in range(grid.height) for x in range(grid.width) if grid.free((x,y))]
    distance={v:INF for v in nodes}; distance[start]=0.
    edges=[(u,v,c) for u in nodes for v,c in grid.neighbors(u)]
    for _ in range(len(nodes)-1):
        changed=False
        for u,v,c in edges:
            if distance[u]+c+EPS<distance[v]:
                distance[v]=distance[u]+c; changed=True
        if not changed: break
    return distance

def main():
    rng=random.Random(20260916); checks=0
    for case in range(60):
        start=(0,0); goal=(7,7)
        blocked={(x,y) for x in range(8) for y in range(8) if rng.random()<.20}-{start,goal}
        grid=Grid(8,8,blocked)
        expected=bellman_ford(grid,start)[goal]
        for weight in (0.,1.,1.5,2.):
            r=search(grid,start,goal,weight)
            if weight<=1: assert equal(r.cost,expected),(case,weight,r.cost,expected)
            elif math.isfinite(expected): assert r.cost<=weight*expected+EPS
            if r.path: validate_path(grid,r.path,start,goal)
            checks+=1
        t=search(grid,start,goal,theta=True)
        assert bool(t.path)==math.isfinite(expected)
        if t.path: validate_path(grid,t.path,start,goal)
        dstar=DStarLite(grid,start,goal)
        for event in range(12):
            dr=dstar.compute(); truth=bellman_ford(grid,dstar.start)[goal]
            assert equal(dr.cost,truth),(case,event,dr.cost,truth)
            if dr.path:
                validate_path(grid,dr.path,dstar.start,goal)
                if len(dr.path)>1 and event%3==0: dstar.move_start(dr.path[1])
            changed=[]
            for _ in range(2):
                p=(rng.randrange(8),rng.randrange(8))
                if p not in (dstar.start,goal): changed.append((p,p not in grid.blocked))
            dstar.update_cells(changed); checks+=1
    # Closed-obstacle corner policy and blocked/unreachable endpoints.
    g=Grid(3,3,{(1,0)})
    assert not g.visible((0,0),(1,1))
    assert not math.isfinite(g.cost((0,0),(1,1)))
    assert not search(g,(1,0),(2,2)).path
    assert search(Grid(2,2),(0,0),(0,0)).path==[(0,0)]
    print(f'PASS: {checks} search/replanning comparisons, plus Theta* feasibility and boundary cases.')

if __name__=='__main__': main()
