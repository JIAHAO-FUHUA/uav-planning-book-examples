"""Offline assessment of saved ROS traces; no ROS import is required.

Distances are exact for each line segment of a replay polyline in floating point.
This is not a continuous certificate for the upstream spline/polynomial curves.
"""
from pathlib import Path
import json
import numpy as np
ROOT=Path(__file__).resolve().parents[1]

def segment_box_distance(start,end,low,high):
    a=np.array(start,float);b=np.array(end,float);lo=np.array(low,float);hi=np.array(high,float);d=b-a
    breaks=[0.,1.]
    for j in range(3):
        if abs(d[j])>1e-14:
            breaks.extend((x-a[j])/d[j] for x in (lo[j],hi[j]) if 0<(x-a[j])/d[j]<1)
    breaks=sorted(set(breaks));best=float('inf')
    for left,right in zip(breaks[:-1],breaks[1:]):
        mid=a+(left+right)*.5*d;constant=np.zeros(3);linear=np.zeros(3)
        for j in range(3):
            if mid[j]<lo[j]:constant[j]=a[j]-lo[j];linear[j]=d[j]
            elif mid[j]>hi[j]:constant[j]=a[j]-hi[j];linear[j]=d[j]
        candidates=[left,right]
        if linear@linear>0:candidates.append(float(np.clip(-(constant@linear)/(linear@linear),left,right)))
        best=min(best,*(float(np.linalg.norm(constant+t*linear)) for t in candidates))
    return best

def assess(result):
    a=np.array(result['trace'],float);boxes=result['scene']['boxes'];assert np.isfinite(a).all()
    distances=[min(segment_box_distance(p,q,box['low'],box['high']) for box in boxes) for p,q in zip(a[:-1,1:4],a[1:,1:4])]
    point_distances=[min(segment_box_distance(p,p,box['low'],box['high']) for box in boxes) for p in a[:,1:4]]
    steps=np.linalg.norm(np.diff(a[:,1:4],axis=0),axis=1);gaps=np.diff(a[:,0])
    # An external assessment rule, not an upstream planner guarantee.
    consistent=bool(np.all(steps<=result['scene']['nominal_speed']*gaps+.05))
    return {'arrived':result['arrived'],'polyline_length_m':float(steps.sum()),
            'polyline_minimum_boundary_distance_m':min(distances),
            'recorded_maximum_speed_mps':float(np.linalg.norm(a[:,4:7],axis=1).max()),
            'recorded_maximum_acceleration_mps2':float(np.linalg.norm(a[:,7:10],axis=1).max()),
            'goal_error_m':float(np.linalg.norm(a[-1,1:4]-result['scene']['goal'])),
            'trajectory_messages':result['trajectory_messages'],'stored_points':len(a),
            'largest_recorded_gap_s':float(np.max(np.diff(a[:,0]))),
            'minimum_recorded_altitude_m':float(a[:,3].min()),
            'largest_recorded_position_step_m':float(steps.max()),
            'position_step_consistency_passes':consistent,
            'altitude_0_30_m_passes_replay':bool(a[:,3].min()>=.3),
            'point_distances_m':point_distances,
            'margin_0_30_m_passes_replay':min(distances)>=.3,
            'margin_0_40_m_passes_replay':min(distances)>=.4}

def main():
    metrics={'scope':'one recorded run per planner, ideal tracking, spherical static sensing; polyline replay only; not a speed ranking'}
    for key in ('fast','ego'):
        result=json.loads((ROOT/f'results/ros_{key}/result.json').read_text());metrics[key]=assess(result)
        small={k:v for k,v in metrics[key].items() if k!='point_distances_m'};print(key,json.dumps(small,indent=2))
    (ROOT/'results/ros_case_comparison.json').write_text(json.dumps(metrics,indent=2),encoding='utf-8')
    return metrics

if __name__=='__main__':main()
