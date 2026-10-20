"""Validate recorded case evidence, topic wiring, and replay geometry."""
from pathlib import Path
import json,sys,xml.etree.ElementTree as E
import numpy as np
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT/'ros_cases'))
from assess_cases import segment_box_distance,assess

def main():
    box=([1.,-.1,-.1],[1.01,.1,.1])
    assert segment_box_distance([0,0,0],[2,0,0],*box)==0
    assert abs(segment_box_distance([0,1,0],[2,1,0],*box)-.9)<1e-12
    assert abs(segment_box_distance([0,1,1],[2,1,1],*box)-np.sqrt(1.62))<1e-12
    for key in ('fast','ego'):
        result=json.loads((ROOT/f'results/ros_{key}/result.json').read_text())
        metric=assess(result)
        assert metric['arrived'] and metric['goal_error_m']<.25
        assert metric['polyline_minimum_boundary_distance_m']>.3
        assert result['commands']>100 and result['trajectory_messages']>1
        assert np.array(result['trace']).shape[1]==11
        launch=E.parse(ROOT/f'ros_cases/generated/{key}_book.launch').getroot()
        assert len(launch.findall('node'))==2
        planner=launch.find('node');maps=[r.get('to') for r in planner.findall('remap') if r.get('from').endswith('/cloud')]
        assert maps==['/book/cloud']
        if key=='fast':
            assert not metric['altitude_0_30_m_passes_replay']
            assert not metric['position_step_consistency_passes']
            assert metric['largest_recorded_position_step_m']>2
        else:
            assert metric['altitude_0_30_m_passes_replay'] and metric['position_step_consistency_passes']
            assert not metric['margin_0_40_m_passes_replay']
    print('System-case evidence, replay geometry and launch wiring checks passed.')

if __name__=='__main__':main()
