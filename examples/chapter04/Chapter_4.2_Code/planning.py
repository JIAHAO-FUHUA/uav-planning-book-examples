"""Original teaching implementations for Sections 4.2.1--4.2.4.

Python >= 3.10, standard library only. Coordinates are (x, y) cell centers.
Occupied squares are closed; diagonal corner cutting is forbidden.
"""
from __future__ import annotations
import heapq
import itertools
import math
from dataclasses import dataclass
from time import perf_counter

INF = float('inf')
EPS = 1e-9
DIRECTIONS = [(1,0),(0,1),(-1,0),(0,-1),(1,1),(-1,1),(-1,-1),(1,-1)]

def octile(a, b):
    dx, dy = abs(a[0]-b[0]), abs(a[1]-b[1])
    return max(dx,dy)+(math.sqrt(2)-1)*min(dx,dy)

def euclidean(a,b):
    return math.hypot(a[0]-b[0],a[1]-b[1])

def equal(a,b):
    return a == b or abs(a-b) <= EPS

def key_less(a,b):
    """Lexicographic comparison with tolerance for accumulated sqrt(2)."""
    if not equal(a[0],b[0]): return a[0]<b[0]
    return not equal(a[1],b[1]) and a[1]<b[1]

def segment_box(a,b,x0,x1,y0,y1):
    """Closed line segment versus closed AABB; tangency is collision."""
    lo, hi = 0.,1.
    for start,delta,mn,mx in ((a[0],b[0]-a[0],x0,x1),(a[1],b[1]-a[1],y0,y1)):
        if delta == 0:
            if start < mn or start > mx: return False
        else:
            p,q=(mn-start)/delta,(mx-start)/delta
            if p>q: p,q=q,p
            lo,hi=max(lo,p),min(hi,q)
            if lo>hi+1e-12: return False
    return True

class Grid:
    def __init__(self,width,height,blocked=()):
        self.width,self.height=width,height
        self.blocked={tuple(p) for p in blocked}
        self.los_calls=0

    def inside(self,p):
        return 0<=p[0]<self.width and 0<=p[1]<self.height

    def free(self,p):
        return self.inside(p) and p not in self.blocked

    def adjacent(self,p):
        """Geometric neighbors, including blocked ones, for repair propagation."""
        for dx,dy in DIRECTIONS:
            q=(p[0]+dx,p[1]+dy)
            if self.inside(q): yield q

    def cost(self,a,b):
        if not self.free(a) or not self.free(b): return INF
        dx,dy=b[0]-a[0],b[1]-a[1]
        if max(abs(dx),abs(dy))!=1: return INF
        if dx and dy and (not self.free((a[0]+dx,a[1])) or not self.free((a[0],a[1]+dy))):
            return INF
        return math.hypot(dx,dy)

    def neighbors(self,p):
        for q in self.adjacent(p):
            c=self.cost(p,q)
            if math.isfinite(c): yield q,c

    def visible(self,a,b):
        self.los_calls += 1
        if not self.free(a) or not self.free(b): return False
        xmin,xmax=sorted((a[0],b[0])); ymin,ymax=sorted((a[1],b[1]))
        for x,y in self.blocked:
            if xmin-.5<=x<=xmax+.5 and ymin-.5<=y<=ymax+.5:
                if segment_box(a,b,x-.5,x+.5,y-.5,y+.5): return False
        return True

@dataclass
class Result:
    path: list
    cost: float
    expanded: list
    milliseconds: float
    los_calls: int = 0

def search(grid,start,goal,weight=1.,theta=False):
    """Dijkstra: weight=0; A*: weight=1; weighted A*: weight>1.
    Theta mode uses Euclidean h and checks a parent's direct connection.
    Improved nodes are reopened. Heap records carry their inserted g value.
    """
    begin=perf_counter(); los0=grid.los_calls
    if not grid.free(start) or not grid.free(goal):
        return Result([],INF,[],0.)
    heuristic=euclidean if theta else octile
    h=lambda p: weight*heuristic(p,goal)
    seq=itertools.count(); queue=[(h(start),h(start),next(seq),0.,start)]
    g={start:0.}; parent={start:start}; expanded=[]
    while queue:
        _,_,_,old_g,u=heapq.heappop(queue)
        if old_g > g[u]+EPS: continue
        expanded.append(u)
        if u==goal:
            path=[goal]
            while path[-1]!=start: path.append(parent[path[-1]])
            path.reverse()
            return Result(path,g[goal],expanded,(perf_counter()-begin)*1000,grid.los_calls-los0)
        for v,c in grid.neighbors(u):
            candidate=g[u]+c; predecessor=u
            pu=parent[u]
            if theta and grid.visible(pu,v):
                shortcut=g[pu]+euclidean(pu,v)
                if shortcut < candidate: candidate,predecessor=shortcut,pu
            if candidate+EPS < g.get(v,INF):
                g[v]=candidate; parent[v]=predecessor
                hv=h(v)
                heapq.heappush(queue,(candidate+hv,hv,next(seq),candidate,v))
    return Result([],INF,expanded,(perf_counter()-begin)*1000,grid.los_calls-los0)

def shortcut_path(grid,path):
    if not path: return []
    result=[path[0]]; i=0
    while i<len(path)-1:
        j=len(path)-1
        while j>i+1 and not grid.visible(path[i],path[j]): j-=1
        result.append(path[j]); i=j
    return result

def length(path):
    return sum(euclidean(a,b) for a,b in zip(path,path[1:]))

class DStarLite:
    """Basic D* Lite (Koenig & Likhachev, 2002, Figure 4).
    g and rhs estimate distance TO the goal, unlike forward A*.
    The basic stop condition requires the current start to be consistent.
    """
    def __init__(self,grid,start,goal):
        self.grid,self.start,self.goal=grid,start,goal
        self.last=start; self.km=0.
        self.g={}; self.rhs={goal:0.}
        self.queue=[]; self.active={}; self.sequence=itertools.count()
        self.expanded=[]
        self._insert(goal)

    def gv(self,u): return self.g.get(u,INF)
    def rv(self,u): return self.rhs.get(u,INF)
    def key(self,u):
        v=min(self.gv(u),self.rv(u))
        return (v+octile(self.start,u)+self.km,v)

    def _insert(self,u):
        k=self.key(u); serial=next(self.sequence)
        self.active[u]=(k,serial)
        heapq.heappush(self.queue,(k,serial,u))

    def _clean(self):
        while self.queue:
            k,n,u=self.queue[0]
            if self.active.get(u)==(k,n): break
            heapq.heappop(self.queue)

    def topkey(self):
        self._clean()
        return self.queue[0][0] if self.queue else (INF,INF)

    def update_vertex(self,u):
        if u!=self.goal:
            self.rhs[u]=min((c+self.gv(v) for v,c in self.grid.neighbors(u)),default=INF)
        self.active.pop(u,None)
        if not equal(self.gv(u),self.rv(u)): self._insert(u)

    def move_start(self,new_start):
        self.km+=octile(self.last,new_start)
        self.start=self.last=new_start

    def update_cells(self,changes):
        """changes: [(cell, is_blocked), ...].
        All tails of changed edges are updated; the 3x3 neighborhood also
        includes diagonal edges whose corner-clearance changes.
        """
        affected=set()
        for p,blocked in changes:
            if not self.grid.inside(p): raise ValueError('out-of-map update')
            if blocked: self.grid.blocked.add(p)
            else: self.grid.blocked.discard(p)
            affected.add(p); affected.update(self.grid.adjacent(p))
        for u in sorted(affected): self.update_vertex(u)

    def compute(self):
        begin=perf_counter(); self.expanded=[]
        while key_less(self.topkey(),self.key(self.start)) or not equal(self.rv(self.start),self.gv(self.start)):
            self._clean()
            if not self.queue: raise RuntimeError('inconsistent state with empty queue')
            oldkey,n,u=heapq.heappop(self.queue); self.active.pop(u,None)
            if key_less(oldkey,self.key(u)): self._insert(u); continue
            self.expanded.append(u)
            if self.gv(u)>self.rv(u):
                self.g[u]=self.rv(u)
                for p in self.grid.adjacent(u): self.update_vertex(p)
            else:
                self.g[u]=INF; self.update_vertex(u)
                for p in self.grid.adjacent(u): self.update_vertex(p)
        path=self.extract_path()
        return Result(path,length(path) if path else INF,list(self.expanded),(perf_counter()-begin)*1000)

    def extract_path(self):
        if not self.grid.free(self.start) or not self.grid.free(self.goal) or not math.isfinite(self.gv(self.start)):
            return []
        path=[self.start]; visited={self.start}
        while path[-1]!=self.goal:
            u=path[-1]
            candidates=[(c+self.gv(v),v) for v,c in self.grid.neighbors(u)]
            if not candidates: return []
            score,v=min(candidates)
            if not math.isfinite(score) or v in visited: raise RuntimeError('invalid D* Lite path')
            path.append(v); visited.add(v)
        return path

def static_scene():
    b=set()
    for y in range(0,25):
        if not 7<=y<=10: b.add((18,y))
    for y in range(7,32):
        if not 23<=y<=26: b.add((32,y))
    b.update((x,y) for x in range(8,12) for y in range(13,19))
    b.update((x,y) for x in range(38,42) for y in range(12,18))
    return Grid(48,32,b),(3,5),(44,26)

def dynamic_scene():
    b={(24,y) for y in range(32) if not (6<=y<=9 or 24<=y<=27)}
    return Grid(48,32,b),(4,8),(44,8)

def validate_path(grid,path,start,goal):
    assert path and path[0]==start and path[-1]==goal
    assert all(grid.visible(a,b) for a,b in zip(path,path[1:]))
