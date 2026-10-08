"""Normalized polynomial interpolation and fixed-time minimum snap, Chapter 4.3.

Coefficients are ascending powers of local s in [0,1]. Durations are seconds.
This is an equality-constrained educational QP, not an obstacle-constrained planner.
"""
from dataclasses import dataclass
from pathlib import Path
import math,json
import numpy as np
from numpy.polynomial import polynomial as pol

ROOT=Path(__file__).resolve().parent
TOL=1e-8

def derivative_row(order,s,duration,degree=7):
    return np.array([0.0 if k<order else math.factorial(k)/math.factorial(k-order)*s**(k-order)/duration**order for k in range(degree+1)])

def interpolate(start,end,duration,order=3):
    """Fix derivatives 0..order at both endpoints; degree = 2*order+1."""
    if duration<=0 or not np.isfinite(duration):raise ValueError('duration must be finite and positive')
    start=np.asarray(start,float);end=np.asarray(end,float)
    if start.shape!=end.shape or len(start)!=order+1:raise ValueError('endpoint derivative arrays must agree')
    degree=2*order+1
    a=np.vstack([derivative_row(r,s,duration,degree) for s in [0.,1.] for r in range(order+1)])
    return np.linalg.solve(a,np.concatenate([start,end],axis=0))

def snap_matrix(duration):
    h=np.zeros((8,8))
    for k in range(4,8):
        for l in range(4,8):
            h[k,l]=math.factorial(k)/math.factorial(k-4)*math.factorial(l)/math.factorial(l-4)/(k+l-7)/duration**7
    return h

@dataclass
class Trajectory:
    coefficients:np.ndarray  # segments x 8 x dimensions
    durations:np.ndarray
    residual:float=0.
    stationarity:float=0.
    condition:float=0.

    @property
    def total_time(self):return float(sum(self.durations))

    def value(self,i,s,order=0):
        return derivative_row(order,s,self.durations[i])@self.coefficients[i]

    def sample(self,count=201,order=0):
        ts=[];values=[];offset=0.
        for i,duration in enumerate(self.durations):
            for s in np.linspace(0,1,count):
                ts.append(offset+s*duration);values.append(self.value(i,s,order))
            offset+=duration
        return np.array(ts),np.array(values)

    def snap_cost(self):
        return float(sum(np.trace(c.T@snap_matrix(d)@c) for c,d in zip(self.coefficients,self.durations)))

    def dilated(self,rho):
        if rho<=0:raise ValueError('time scale must be positive')
        return Trajectory(self.coefficients.copy(),self.durations*rho)

    def maximum_norm(self,order):
        """Endpoints and every real stationary root of squared derivative norm."""
        maximum=0.;where=None
        for i,c in enumerate(self.coefficients):
            coeff=np.array([pol.polyder(c[:,j],order)/self.durations[i]**order for j in range(c.shape[1])])
            norm2=np.zeros(2*(8-order)-1)
            for v in coeff:norm2[:len(pol.polymul(v,v))]+=pol.polymul(v,v)
            critical=unit_roots(pol.polyder(norm2))
            for s in [0.,1.,*critical]:
                value=float(np.linalg.norm(self.value(i,s,order)))
                if value>maximum:maximum=value;where=(i,s)
        return maximum,where

def minimum_snap(waypoints,durations,start_derivatives=None,end_derivatives=None):
    """Septic segments, waypoint interpolation, C3 knots, fixed positive times.

    By default start/end velocity, acceleration and jerk are zero. Internal values
    are free and coupled across segments. Assemble 1/2 c^T Q c, A c = b.
    """
    w=np.asarray(waypoints,float);durations=np.asarray(durations,float)
    m=len(durations)
    if w.ndim!=2 or len(w)!=m+1 or not m or np.any(durations<=0) or not np.all(np.isfinite(w)) or not np.all(np.isfinite(durations)):raise ValueError('finite waypoints and positive segment durations required')
    dim=w.shape[1];n=8*m;rows=[];rhs=[]
    q=np.zeros((n,n))
    for i,d in enumerate(durations):
        q[i*8:(i+1)*8,i*8:(i+1)*8]=2*snap_matrix(d)
        for s,target in [(0.,w[i]),(1.,w[i+1])]:
            row=np.zeros(n);row[i*8:(i+1)*8]=derivative_row(0,s,d);rows.append(row);rhs.append(target)
    start=np.zeros((3,dim)) if start_derivatives is None else np.asarray(start_derivatives,float)
    end=np.zeros((3,dim)) if end_derivatives is None else np.asarray(end_derivatives,float)
    if start.shape!=(3,dim) or end.shape!=(3,dim):raise ValueError('velocity/acceleration/jerk arrays must have shape(3,dimensions)')
    for i,s,data in [(0,0.,start),(m-1,1.,end)]:
        for r in range(1,4):
            row=np.zeros(n);row[i*8:(i+1)*8]=derivative_row(r,s,durations[i]);rows.append(row);rhs.append(data[r-1])
    for i in range(m-1):
        for r in range(1,4):
            row=np.zeros(n);row[i*8:(i+1)*8]=derivative_row(r,1.,durations[i]);row[(i+1)*8:(i+2)*8]=-derivative_row(r,0.,durations[i+1]);rows.append(row);rhs.append(np.zeros(dim))
    a=np.array(rows);b=np.array(rhs)
    # Common objective scale and per-row constraint scale preserve the minimizer.
    qs=q/np.max(np.abs(q));scale=np.linalg.norm(a,axis=1);as_=a/scale[:,None];bs=b/scale[:,None]
    kkt=np.block([[qs,as_.T],[as_,np.zeros((len(a),len(a)))]])
    sol=np.linalg.solve(kkt,np.vstack([np.zeros((n,dim)),bs]));c=sol[:n]
    residual=float(np.max(np.abs(a@c-b)))
    stationarity=float(np.max(np.abs(qs@c+as_.T@sol[n:])))
    if residual>1e-7 or stationarity>1e-8:raise ArithmeticError('QP residual too large; revise durations/conditioning')
    return Trajectory(c.reshape(m,8,dim),durations,residual,stationarity,float(np.linalg.cond(kkt)))

def stopped(waypoints,durations):
    w=np.asarray(waypoints,float);dim=w.shape[1];cs=[]
    for a,b,d in zip(w[:-1],w[1:],durations):
        cs.append(interpolate(np.vstack([a,np.zeros((3,dim))]),np.vstack([b,np.zeros((3,dim))]),d))
    return Trajectory(np.array(cs),np.array(durations,float))

def unit_roots(coeff):
    coeff=pol.polytrim(np.asarray(coeff,float),tol=1e-12)
    if len(coeff)<2:return []
    return sorted(float(r.real) for r in pol.polyroots(coeff) if abs(r.imag)<1e-7 and -TOL<=r.real<=1+TOL)

def collision_intervals(traj,low,high):
    """Whole polynomial vs closed AABB using all slab-boundary roots.

    Floating arithmetic with conservative 1e-8 boundary tolerance; reported
    intervals are numerical validation, not a formal exact-arithmetic certificate.
    """
    low=np.asarray(low,float);high=np.asarray(high,float);hits=[]
    for i,c in enumerate(traj.coefficients):
        cuts=[0.,1.]
        for j in range(len(low)):
            for boundary in [low[j],high[j]]:
                a=c[:,j].copy();a[0]-=boundary;cuts.extend(unit_roots(a))
        cuts=sorted(set(max(0.,min(1.,x)) for x in cuts))
        def inside(s):
            p=traj.value(i,s);return bool(np.all(p>=low-TOL) and np.all(p<=high+TOL))
        for a,b in zip(cuts[:-1],cuts[1:]):
            if inside((a+b)/2):hits.append((i,a,b))
        for s in cuts:
            if inside(s):hits.append((i,s,s))
    return hits

def build_cases():
    w=np.array([[1.,3.,1.],[4.,7.,1.],[9.,7.,1.],[13.,3.,1.]])
    ds=np.array([2.,2.,2.]);through=minimum_snap(w,ds);stop=stopped(w,ds)
    speed=through.maximum_norm(1)[0];accel=through.maximum_norm(2)[0]
    rho=max(1.,speed/2.,math.sqrt(accel/2.5))*1.01
    scaled=through.dilated(rho)
    stop_rho=max(1.,stop.maximum_norm(1)[0]/2.,math.sqrt(stop.maximum_norm(2)[0]/2.5))*1.01
    safe_stop=stop.dilated(stop_rho)
    box=(np.array([6.,0.,.5]),np.array([7.,6.,1.5]))
    upper_box=(np.array([5.8,9.1,.5]),np.array([6.6,9.7,1.5]))
    corner_w=np.array([[0.,0.],[2.,0.],[2.,2.]])
    corner=minimum_snap(corner_w,[2.,2.]);corner_stop=stopped(corner_w,[2.,2.])
    return dict(waypoints=w,durations=ds,through=through,stop=stop,scaled=scaled,rho=rho,safe_stop=safe_stop,stop_rho=stop_rho,box=box,upper_box=upper_box,corner=corner,corner_stop=corner_stop)

def record(traj):
    out={'coefficients':traj.coefficients.tolist(),'durations':traj.durations.tolist(),'total_time':traj.total_time,'snap_cost':traj.snap_cost(),'constraint_residual':traj.residual,'stationarity_residual':traj.stationarity,'kkt_condition':traj.condition}
    out['peak_speed']=traj.maximum_norm(1)[0];out['peak_acceleration']=traj.maximum_norm(2)[0];out['peak_snap']=traj.maximum_norm(4)[0]
    return out

def main():
    cases=build_cases();data={'waypoints':cases['waypoints'].tolist(),'limits':{'speed':2.,'acceleration':2.5},'rho':cases['rho'],'boxes':[[x.tolist() for x in cases[n]] for n in ['box','upper_box']]}
    for name in ['through','stop','scaled','safe_stop','corner','corner_stop']:
        tr=cases[name];data[name]=record(tr)
        print(name,'time',f'{tr.total_time:.6f}','snap',f'{tr.snap_cost():.6f}','speed',f'{tr.maximum_norm(1)[0]:.6f}','accel',f'{tr.maximum_norm(2)[0]:.6f}')
    data['obstacle_intersections']={n:collision_intervals(cases['through'],*cases[n]) for n in ['box','upper_box']}
    data['single_coefficients']={str(r):interpolate(np.array([0.]+[0.]*r),np.array([1.]+[0.]*r),1.,r).tolist() for r in [2,3]}
    data['stopped_corner_jerk']=cases['corner_stop'].value(0,1.,3).tolist()
    (ROOT/'results').mkdir(exist_ok=True)
    (ROOT/'results/trajectory_results.json').write_text(json.dumps(data,indent=2,allow_nan=False),encoding='utf-8')
    print('Obstacle intersections:',data['obstacle_intersections']);print('rho',cases['rho'])

if __name__=='__main__':main()
