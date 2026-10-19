"""Independent KKT equivalence, optimality continuity and adjoint checks."""
import numpy as np
from minco import generate,system,objective,basis
from trajectory_generation import minimum_snap

def run():
    rng=np.random.default_rng(19);worst_curve=0.;worst_gradient=0.
    for m in (1,2,3,6):
        w=rng.normal(size=(m+1,3));ts=rng.uniform(.9,2.1,m)
        start=rng.normal(size=(3,3))*.15;end=rng.normal(size=(3,3))*.15
        tr=generate(w,ts,start,end);qp=minimum_snap(w,ts,start,end)
        error=float(np.max(np.abs(tr.normalized_coefficients()-qp.coefficients)))
        assert error<2e-7,(m,error);worst_curve=max(worst_curve,error)
        for i in range(m-1):
            for r in range(7):assert np.allclose(tr.value(i,ts[i],r),tr.value(i+1,0.,r),atol=1e-7)
        a,b,_=system(w,ts,start,end);dense=a.dense()
        assert np.allclose(tr.lu.solve(b),np.linalg.solve(dense,b),rtol=1e-9,atol=1e-9)
        random_rhs=rng.normal(size=b.shape)
        assert np.allclose(tr.lu.solve(random_rhs,True),np.linalg.solve(dense.T,random_rhs),rtol=1e-8,atol=1e-8)
    w=np.array([[0.,0.,0.],[1.,.8,.2],[2.,0.,0.]])
    ts=np.array([1.2,1.8]);value,gq,gt=objective(w,ts,2.)
    for kind,grad in (('q',gq),('t',gt)):
        for idx in np.ndindex(grad.shape):
            wp=w.copy();wm=w.copy();tp=ts.copy();tm=ts.copy();eps=2e-5
            if kind=='q':wp[1+idx[0],idx[1]]+=eps;wm[1+idx[0],idx[1]]-=eps
            else:tp[idx]+=eps;tm[idx]-=eps
            fd=(objective(wp,tp,2.)[0]-objective(wm,tm,2.)[0])/(2*eps)
            rel=abs(fd-grad[idx])/max(1.,abs(fd),abs(grad[idx]));worst_gradient=max(worst_gradient,rel)
            assert rel<2e-6,(kind,idx,fd,grad[idx],rel)
    # A generic coefficient-dependent cost checks propagation beyond pure energy.
    tr=generate(w,ts);weights=rng.uniform(.5,1.5,tr.coefficients.shape)
    gc=2*weights*tr.coefficients;direct=np.array([.4,.7]);qg,tg=tr.propagate(gc,direct)
    def generic(wp,tp):return np.sum(weights*generate(wp,tp).coefficients**2)+direct@tp
    for i in range(2):
        p=ts.copy();n=ts.copy();p[i]+=2e-5;n[i]-=2e-5
        fd=(generic(w,p)-generic(w,n))/4e-5
        assert abs(fd-tg[i])/max(1.,abs(fd))<2e-6
    for bad in ([0.,1.],[-1.,1.],[float('nan'),1.]):
        try:generate(w,bad)
        except ValueError:pass
        else:raise AssertionError('invalid durations accepted')
    print(f'MINCO checks passed: KKT coefficient error {worst_curve:.3g}; gradient relative error {worst_gradient:.3g}')
    return {'kkt_coefficient_max_error':worst_curve,'gradient_max_relative_error':worst_gradient}

if __name__=='__main__':run()
