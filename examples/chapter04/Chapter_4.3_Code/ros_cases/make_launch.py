"""Create case launch files from pinned upstream parameter XML.

Algorithms are unchanged. Both use the same ideal reference-to-odometry bridge
and a static local spherical point-cloud model, without RViz/GPU/controller.
"""
from pathlib import Path
import argparse,json,xml.etree.ElementTree as E

def create(source_root,output):
    source_root=Path(source_root);output=Path(output);output.mkdir(parents=True,exist_ok=True)
    scene=json.loads((Path(__file__).parent/'scene.json').read_text())
    common={'map_size_x_':'20.0','map_size_y_':'12.0','map_size_z_':'3.0',
            'odometry_topic':'/book/odom','camera_pose_topic':'/book/unused_pose',
            'depth_topic':'/book/unused_depth','cloud_topic':'/book/cloud',
            'cx':'321.046','cy':'243.45','fx':'387.23','fy':'387.23',
            'max_vel':str(scene['nominal_speed']),'max_acc':str(scene['nominal_acceleration']),
            'max_jer':'20.0','planning_horizon':'7.5','use_multitopology_trajs':'false',
            'flight_type':'1','point_num':'1','drone_id':'0'}
    for i in range(5):
        for dim,value in zip('xyz',scene['goal']):common[f'point{i}_{dim}']=str(value)
    paths={'fast':source_root/'fast_ws/src/fast_planner/plan_manage/launch/kino_algorithm.xml',
           'ego':source_root/'ego_ws/src/planner/plan_manage/launch/include/advanced_param.xml'}
    for name,p in paths.items():
        root=E.parse(p).getroot()
        for arg in list(root.findall('arg')):
            key=arg.get('name');arg.attrib={'name':key,'value':common[key]}
        node=root.find('node');node.set('name','book_planner')
        # Absolute names eliminate drone-prefix ambiguity in the common bridge.
        for remap in node.findall('remap'):
            source=remap.get('from')
            if 'odom' in source:remap.set('to','/book/odom')
            elif source.endswith('/cloud'):remap.set('to','/book/cloud')
            elif source=='/goal':remap.set('to','/goal_with_id')
            elif source.endswith('planning/trajectory'):remap.set('to','/book/trajectory')
            elif source.endswith('planning/heartbeat'):remap.set('to','/book_traj_server/heartbeat')
        def param(key,val):
            old=next((e for e in node.findall('param') if e.get('name')==key),None)
            if old is None:old=E.SubElement(node,'param',name=key)
            old.set('value',str(val))
        if name=='fast':
            for key,val in {'sdf_map/virtual_ceil_height':3.0,'sdf_map/obstacles_inflation':.1,
                            'optimization/dist0':.3,'search/lambda_heu':1.0,
                            'optimization/max_iteration_time2':.03,
                            'optimization/max_iteration_time3':.03}.items():param(key,val)
        else:
            param('grid_map/obstacles_inflation',.1);param('optimization/obstacle_clearance',.2)
        server=E.SubElement(root,'node',pkg='plan_manage' if name=='fast' else 'ego_planner',name='book_traj_server',type='traj_server',output='screen')
        E.SubElement(server,'remap',{'from':'/position_cmd' if name=='fast' else 'position_cmd','to':'/book/pos_cmd'})
        if name=='fast':E.SubElement(server,'remap',{'from':'/odom_world','to':'/book/odom'})
        else:E.SubElement(server,'remap',{'from':'~planning/trajectory','to':'/book/trajectory'})
        E.SubElement(server,'param',name='traj_server/time_forward',value='1.0')
        dest=output/f'{name}_book.launch';E.ElementTree(root).write(dest,encoding='utf-8',xml_declaration=True)
        print(dest)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',default=str(Path.home()/'uav_book_cases_20261020'));p.add_argument('--output',default=str(Path(__file__).parent/'generated'))
    args=p.parse_args();create(args.root,args.output)
