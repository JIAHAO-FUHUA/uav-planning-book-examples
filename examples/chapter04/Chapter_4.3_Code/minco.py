"""MINCO s=4 teaching solver: physical local time, banded solve and adjoint.

Original implementation of the equations in Section 4.3.4. Fixed endpoint p/v/a/j,
intermediate positions, positive durations; no collision or vehicle constraints.
The fixed-band LU deliberately has no pivoting: use moderately scaled teaching
data, check residuals, and use a robust pivoted band solver for difficult scales.
"""
from dataclasses import dataclass
import math
import numpy as np


def basis(r,t):
    return np.array([0. if k<r else math.factorial(k)/math.factorial(k-r)*t**(k-r) for k in range(8)])


class Banded:
    width=8
    def __init__(self,n):self.n=n;self.data=np.zeros((n,17))
    def get(self,i,j):return self.data[i,j-i+8] if abs(j-i)<=8 else 0.
    def add(self,i,j,v):
        if abs(i-j)>8:
            if v:raise ValueError('outside fixed band')
        else:self.data[i,j-i+8]+=v
    def dense(self):return np.array([[self.get(i,j) for j in range(self.n)] for i in range(self.n)])
    def multiply(self,x):
        return np.array([sum((self.get(i,j)*x[j] for j in range(max(0,i-8),min(self.n,i+9))),start=np.zeros(x.shape[1])) for i in range(self.n)])
    def factor(self):
        out=Banded(self.n);out.data=self.data.copy()
        for k in range(self.n):
            pivot=out.get(k,k)
            if abs(pivot)<1e-13:raise ArithmeticError('small band pivot; rescale or use pivoted solver')
            for i in range(k+1,min(self.n,k+9)):
                if out.get(i,k)==0:continue
                factor=out.get(i,k)/pivot;out.data[i,k-i+8]=factor
                for j in range(k+1,min(self.n,k+9)):out.add(i,j,-factor*out.get(k,j))
        return out
    def solve(self,rhs,transpose=False):
        x=np.array(rhs,float,copy=True)
        if not transpose:
            for i in range(self.n):
                for j in range(max(0,i-8),i):x[i]-=self.get(i,j)*x[j]
            for i in reversed(range(self.n)):
                for j in range(i+1,min(self.n,i+9)):x[i]-=self.get(i,j)*x[j]
                x[i]/=self.get(i,i)
        else:
            for i in range(self.n):
                for j in range(max(0,i-8),i):x[i]-=self.get(j,i)*x[j]
                x[i]/=self.get(i,i)
            for i in reversed(range(self.n)):
                for j in range(i+1,min(self.n,i+9)):x[i]-=self.get(j,i)*x[j]
        return x


def system(waypoints,durations,start=None,end=None):
    w=np.asarray(waypoints,float);ts=np.asarray(durations,float);m=len(ts)
    if w.ndim!=2 or len(w)!=m+1 or not m or np.any(ts<=0) or not np.all(np.isfinite(ts)) or not np.all(np.isfinite(w)):raise ValueError('finite waypoints and positive times required')
    d=w.shape[1];head=np.zeros((4,d));tail=head.copy();head[0]=w[0];tail[0]=w[-1]
    if start is not None:head[1:]=np.asarray(start,float)
    if end is not None:tail[1:]=np.asarray(end,float)
    if not np.all(np.isfinite(head)) or not np.all(np.isfinite(tail)):raise ValueError('finite boundary derivatives required')
    a=Banded(8*m);b=np.zeros((8*m,d));time_rows=[[] for _ in ts]
    def row(k,i,r,at_end,continuity=False):
        vals=basis(r,ts[i] if at_end else 0.)
        for j,v in enumerate(vals):a.add(k,8*i+j,v)
        if at_end:time_rows[i].append((k,r))
        if continuity:
            for j,v in enumerate(basis(r,0.)):a.add(k,8*(i+1)+j,-v)
    for r in range(4):row(r,0,r,False);b[r]=head[r]
    for i in range(m-1):
        base=8*i+4
        for j,r in enumerate((4,5,6)):row(base+j,i,r,True,True)
        row(base+3,i,0,True);b[base+3]=w[i+1]
        for r in range(4):row(base+4+r,i,r,True,True)
    for r in range(4):row(8*m-4+r,m-1,r,True);b[8*m-4+r]=tail[r]
    return a,b,time_rows


@dataclass
class Minco:
    coefficients:np.ndarray
    durations:np.ndarray
    matrix:Banded
    lu:Banded
    time_rows:list
    residual:float
    def value(self,i,theta,r=0):return basis(r,theta)@self.coefficients[i]
    def normalized_coefficients(self):return self.coefficients*self.durations[:,None,None]**np.arange(8)[None,:,None]
    def sample(self,count=150,r=0):
        time=[];values=[];offset=0.
        for i,tau in enumerate(self.durations):
            for theta in np.linspace(0,tau,count):time.append(offset+theta);values.append(self.value(i,theta,r))
            offset+=tau
        return np.array(time),np.array(values)
    def energy(self):
        value=0.;gc=np.zeros_like(self.coefficients);gt=[]
        for i,(c,tau) in enumerate(zip(self.coefficients,self.durations)):
            h=np.zeros((8,8))
            for k in range(4,8):
                for l in range(4,8):h[k,l]=math.prod(range(k-3,k+1))*math.prod(range(l-3,l+1))*tau**(k+l-7)/(k+l-7)
            value+=float(np.sum(c*(h@c)));gc[i]=2*h@c;snap=self.value(i,tau,4);gt.append(float(snap@snap))
        return value,gc,np.array(gt)
    def propagate(self,partial_coefficients,partial_times):
        adj=self.lu.solve(np.asarray(partial_coefficients).reshape(-1,self.coefficients.shape[2]),transpose=True)
        gq=np.array([adj[8*i+7] for i in range(len(self.durations)-1)]).reshape(-1,self.coefficients.shape[2])
        gt=np.array(partial_times,float,copy=True)
        for i,rows in enumerate(self.time_rows):
            for row,r in rows:gt[i]-=adj[row]@self.value(i,self.durations[i],r+1)
        return gq,gt


def generate(waypoints,durations,start=None,end=None):
    a,b,rows=system(waypoints,durations,start,end);lu=a.factor();c=lu.solve(b)
    residual=float(np.max(np.abs(a.multiply(c)-b)))
    if residual>1e-7:raise ArithmeticError('trajectory solve residual too large; check scaling')
    return Minco(c.reshape(-1,8,b.shape[1]),np.array(durations,float),a,lu,rows,residual)


def objective(waypoints,durations,time_weight=1.):
    tr=generate(waypoints,durations);energy,gc,gt=tr.energy();gq,gt=tr.propagate(gc,gt)
    return energy+time_weight*sum(durations),gq,gt+time_weight


def optimize_times(waypoints,initial,time_weight=1.,iterations=120):
    """Small damped BFGS example in log-duration variables; positions stay fixed.

    Bounds on log duration keep this teaching problem numerically scaled.
    Production corridor and dynamics optimization requires additional constraints.
    """
    x=np.log(initial);n=len(x);h=np.eye(n);history=[]
    def evaluate(z):
        val,_,gt=objective(waypoints,np.exp(z),time_weight);return val,gt*np.exp(z)
    value,g=evaluate(x)
    for _ in range(iterations):
        history.append(value)
        if np.linalg.norm(g,np.inf)<1e-6:break
        direction=-h@g
        if direction@g>=0:h=np.eye(n);direction=-g
        step=1.
        for _ in range(50):
            trial=x+step*direction
            if np.max(np.abs(trial))<2.5:
                nv,ng=evaluate(trial)
                if nv<=value+1e-4*step*(g@direction):break
            step*=.5
        else:break
        delta=trial-x;y=ng-g;ys=float(y@delta)
        if ys>1e-12:
            rho=1/ys;v=np.eye(n)-rho*np.outer(delta,y);h=v@h@v.T+rho*np.outer(delta,delta)
        else:h=np.eye(n)
        x,value,g=trial,nv,ng
    return np.exp(x),history,float(np.linalg.norm(g,np.inf))
