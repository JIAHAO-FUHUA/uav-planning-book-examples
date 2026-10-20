"""Checks address missed collisions, gradient direction and unsafe handoffs."""
import numpy as np
from maps_replanning import (Circle,Ticket,occupancy_update,occupancy_class,
    obstacle_penalty,interval_clearance,handoff_case,admission,circle_minimum,at,
    fixed_sample_penalty)
from minco import generate

def run():
    assert occupancy_class(0.,False)=='unknown'
    assert occupancy_class(occupancy_update(0.,True),True)=='occupied'
    assert occupancy_class(-1.2,True)=='free'
    assert occupancy_update(3.5,True)==3.5
    c=Circle();p=np.array([1.4,.1]);value,grad=obstacle_penalty(p,c)
    fd=np.array([(obstacle_penalty(p+np.eye(2)[j]*1e-6,c)[0]-
                  obstacle_penalty(p-np.eye(2)[j]*1e-6,c)[0])/2e-6 for j in range(2)])
    assert np.max(np.abs(fd-grad))<1e-8
    assert obstacle_penalty(p-.05*grad,c)[0]<value
    assert not interval_clearance([1.6,1.6],1.,4.,.3)
    assert c.query([0.,0.])[0]>.3 and c.query([4.,0.])[0]>.3
    # A separated straight line has clearance at least 0.6 m throughout.
    xs=np.linspace(0.,4.,41);ds=[c.query([x,1.])[0] for x in xs]
    assert interval_clearance(ds,1.,.1,.3)
    assert not interval_clearance([float('nan')],1.,.1,.3)
    # Polynomial libraries trim constant axes: add coefficient arrays by degree.
    separated=generate([[0.,1.],[4.,1.]],[4.])
    assert abs(circle_minimum(separated,c)[0]-.6)<1e-7
    tr,error=handoff_case();assert error<1e-8
    # Chain a new obstacle cost through the existing MINCO adjoint.
    samples=[(0,.5,.7),(0,.9,1.1),(1,.3,.8)]
    obs=Circle((2.7,0.),.35);cost,gc=fixed_sample_penalty(tr,samples,obs,.8)
    gq,_=tr.propagate(gc,np.zeros(2))
    for j in range(2):
        points=np.array([[1.6,0.],[2.1,1.3],[5.,0.]])
        plus=points.copy();minus=points.copy();plus[1,j]+=1e-6;minus[1,j]-=1e-6
        start=[[1.,0.],[0.,0.],[0.,0.]]
        fd=(fixed_sample_penalty(generate(plus,[1.4,3.],start=start),samples,obs,.8)[0]-
            fixed_sample_penalty(generate(minus,[1.4,3.],start=start),samples,obs,.8)[0])/2e-6
        assert abs(fd-gq[0,j])<1e-6
    obstacle=Circle((2.7,0.),.35);minimum,t=circle_minimum(tr,obstacle)
    _,points=tr.sample(1500)
    sampled=min(obstacle.query(p)[0] for p in points)
    assert .2<minimum<=sampled+1e-7 and sampled-minimum<1e-4
    assert abs(obstacle.query(at(tr,t))[0]-minimum)<1e-7
    ticket=Ticket(7,12,1.6)
    assert admission(ticket,1.3,7,12,error)=='scheduled'
    assert admission(ticket,1.58,7,12,error)=='late'
    assert admission(ticket,1.3,8,12,error)=='stale-parent'
    assert admission(ticket,1.3,7,13,error)=='revalidate-map'
    assert admission(ticket,1.3,7,12,error,frame='odom')=='wrong-frame'
    assert admission(ticket,1.3,7,12,.1)=='discontinuous'
    assert admission(ticket,1.3,7,12,error,prefix_ok=False)=='recover-prefix'
    assert admission(ticket,1.3,7,12,error,candidate_ok=False)=='reject-candidate'
    # At absolute t=1.9, local candidate time is 0.3, not 0.
    assert np.linalg.norm(at(tr,.3)-at(tr,0.))>.25
    print('Map, distance-gradient, missed-collision, C3-handoff and stale-result checks passed.')

if __name__=='__main__':run()
