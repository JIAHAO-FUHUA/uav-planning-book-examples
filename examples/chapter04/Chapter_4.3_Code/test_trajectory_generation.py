"""Independent mathematical checks for the Chapter 4.3 teaching examples."""
import math
import numpy as np
from numpy.polynomial import polynomial as pol
from trajectory_generation import interpolate,minimum_snap,build_cases,collision_intervals,Trajectory,derivative_row,snap_matrix

q5=interpolate([0,0,0],[1,0,0],1,2)
q7=interpolate([0,0,0,0],[1,0,0,0],1,3)
np.testing.assert_allclose(q5,[0,0,0,10,-15,6],atol=1e-10)
np.testing.assert_allclose(q7,[0,0,0,0,35,-84,70,-20],atol=1e-10)
# Nonzero derivatives and unequal time ensure physical-time scaling is applied.
c=interpolate([2,.7,-.1,.2],[5,-.3,.4,-.2],2.3)
for s,values in [(0.,[2,.7,-.1,.2]),(1.,[5,-.3,.4,-.2])]:
 for r,value in enumerate(values):assert abs(derivative_row(r,s,2.3)@c-value)<1e-10
single=minimum_snap([[0],[1]],[1.])
np.testing.assert_allclose(single.coefficients[0,:,0],q7,atol=1e-9)
# Exact integral of the single-segment squared fourth derivative equals 100800.
assert abs(single.snap_cost()-100800)<1e-5
cases=build_cases();tr=cases['through'];slow=cases['scaled']
assert tr.snap_cost()<cases['stop'].snap_cost()
for i in range(len(tr.durations)-1):
 for r in range(4):np.testing.assert_allclose(tr.value(i,1,r),tr.value(i+1,0,r),atol=1e-7)
assert np.linalg.norm(tr.value(0,1,1))>.1
rho=cases['rho']
for r in range(5):
 for i in range(3):np.testing.assert_allclose(slow.value(i,.37,r),tr.value(i,.37,r)/rho**r,atol=1e-8)
assert abs(slow.snap_cost()/tr.snap_cost()-rho**-7)<1e-10
assert slow.maximum_norm(1)[0]<2 and slow.maximum_norm(2)[0]<2.5
assert tr.residual<1e-7 and tr.stationarity<1e-8
# Root-based maxima cross-check a dense independent grid (grid is a check, not certificate).
for r in [1,2,4]:
 _,vv=tr.sample(5001,r);grid_peak=np.max(np.linalg.norm(vv,axis=1));root_peak=tr.maximum_norm(r)[0]
 assert root_peak+1e-7>=grid_peak and abs(root_peak-grid_peak)<2e-4
assert not collision_intervals(tr,*cases['box'])
assert collision_intervals(tr,*cases['upper_box'])
assert collision_intervals(slow,*cases['upper_box'])==collision_intervals(tr,*cases['upper_box'])
assert not collision_intervals(cases['stop'],*cases['upper_box'])
safe=cases['safe_stop']
assert safe.maximum_norm(1)[0]<2 and safe.maximum_norm(2)[0]<2.5
assert all(not collision_intervals(safe,*cases[key]) for key in ['box','upper_box'])
# A thin box between two safe polynomial endpoints, plus tangency to a closed box.
line=np.zeros((1,8,2));line[0,1,0]=1
thin=Trajectory(line,np.array([1.]))
assert collision_intervals(thin,[.4999,-.1],[.5001,.1])
tangent=line.copy();tangent[0,0,1]=1
assert collision_intervals(Trajectory(tangent,np.array([1.])),[.4,0],[.6,1])
# Perturb within the exact equality nullspace: no lower objective than the QP optimum.
m=3;n=24;rows=[]
for i,d in enumerate(tr.durations):
 for s in [0,1]:
  row=np.zeros(n);row[8*i:8*(i+1)]=derivative_row(0,s,d);rows.append(row)
for i,s in [(0,0),(2,1)]:
 for r in [1,2,3]:
  row=np.zeros(n);row[8*i:8*(i+1)]=derivative_row(r,s,tr.durations[i]);rows.append(row)
for i in [0,1]:
 for r in [1,2,3]:
  row=np.zeros(n);row[8*i:8*(i+1)]=derivative_row(r,1,tr.durations[i]);row[8*(i+1):8*(i+2)]=-derivative_row(r,0,tr.durations[i+1]);rows.append(row)
a=np.array(rows);_,_,vh=np.linalg.svd(a,full_matrices=True);z=vh[len(rows):].T
h=np.zeros((n,n))
for i,d in enumerate(tr.durations):h[i*8:(i+1)*8,i*8:(i+1)*8]=snap_matrix(d)
assert np.linalg.eigvalsh(z.T@h@z).min()>0
base=tr.coefficients[:,:,0].ravel();rng=np.random.default_rng(123)
for _ in range(10):
 candidate=base+z@rng.normal(size=z.shape[1]);assert candidate@h@candidate>=base@h@base-1e-6
try:minimum_snap([[0],[1]],[0]);raise AssertionError('invalid duration accepted')
except ValueError:pass
print('PASS: endpoint interpolation, physical time, known integral, KKT/nullspace optimum, C3 continuity, extrema, dilation, thin-box/tangent collision and invalid time.')
