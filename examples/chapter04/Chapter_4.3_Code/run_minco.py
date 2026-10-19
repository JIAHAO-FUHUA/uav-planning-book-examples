"""Small MINCO example and numerical results used by the teaching figures."""
from pathlib import Path
import json
import numpy as np
from minco import generate,optimize_times
from trajectory_generation import minimum_snap

def main():
    w=np.array([[0.,0.,1.],[1.,.8,1.],[2.,0.,1.]])
    times=np.array([1.2,1.8]);tr=generate(w,times);qp=minimum_snap(w,times)
    _,a=tr.sample();_,b=qp.sample(150)
    error=float(np.max(np.linalg.norm(a-b,axis=1)))
    spatial=w.copy();spatial[1,1]=1.2
    changed_times=np.array([1.8,1.2])
    optimized,history,grad=optimize_times(w,times,time_weight=20.)
    result={'waypoints':w.tolist(),'durations':times.tolist(),'energy':tr.energy()[0],
      'kkt_curve_max_error_m':error,'solve_residual':tr.residual,
      'changed_waypoints':spatial.tolist(),'changed_durations':changed_times.tolist(),
      'time_only_optimization':{'time_weight':20.,'durations':optimized.tolist(),'objective_history':history,'log_time_gradient_inf':grad},
      'scope':'Fixed p/v/a/j at overall endpoints; s=4, no obstacle or vehicle constraints. Timing example keeps positions fixed.'}
    out=Path(__file__).resolve().parent/'results';out.mkdir(exist_ok=True)
    (out/'minco_results.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(f'KKT/MINCO curve difference: {error:.3g} m')
    print(f'Snap cost: {tr.energy()[0]:.9f}; band residual: {tr.residual:.3g}')
    print('Optimized durations:', ' '.join(f'{x:.6f}' for x in optimized))
    print(f'Time objective: {history[0]:.6f} -> {history[-1]:.6f}')
    return result

if __name__=='__main__':main()
