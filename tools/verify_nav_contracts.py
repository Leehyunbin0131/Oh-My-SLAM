#!/usr/bin/env python3
"""P4 installed assets, geometry, and actual planner/critic boundary probe.

No simulator or physical output is connected. Unit-test evidence is produced by
colcon, and plugin/sensor readiness is exercised once by the integrated drive.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import traceback
import xml.etree.ElementTree as ET
import yaml
import numpy as np
from scipy.spatial.transform import Rotation
from build_navigation import DEFAULT_BUILD, DEFAULT_VALIDATION, verified_build, write_manifest
from generate_p4_course import generate
from p4_geometry import polygon, rectangle, intersects

WORKSPACE=Path(__file__).resolve().parents[1]


def digest(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def collision_envelope(urdf):
    root=ET.fromstring(urdf)
    joints={j.find('child').get('link'):j for j in root.findall('joint')}
    shapes=[]
    def origin(node):
        mat=np.eye(4)
        if node is not None:
            mat[:3,3]=list(map(float,node.get('xyz','0 0 0').split()))
            mat[:3,:3]=Rotation.from_euler('xyz',list(map(float,node.get('rpy','0 0 0').split()))).as_matrix()
        return mat
    def transform(name,theta):
        if name not in joints: return np.eye(4)
        j=joints[name]; mat=origin(j.find('origin'))
        if 'caster_swivel' in j.get('name',''):
            axis=np.array(list(map(float,j.find('axis').get('xyz').split())))
            rot=np.eye(4);rot[:3,:3]=Rotation.from_rotvec(axis*theta).as_matrix();mat=mat@rot
        return transform(j.find('parent').get('link'),theta)@mat
    for link in root.findall('link'):
        for collision in link.findall('collision'):
            shape=collision.find('geometry')[0]
            if shape.tag not in ('box','cylinder','sphere'):
                raise ValueError('Unsupported physical collision primitive '+shape.tag)
            shapes.append((link.get('name'),collision,shape))
    lower=np.full(3,np.inf); upper=np.full(3,-np.inf)
    for theta in np.linspace(-np.pi,np.pi,1441):
        for name,collision,shape in shapes:
            mat=transform(name,theta)@origin(collision.find('origin')); rot=mat[:3,:3]
            if shape.tag=='box': ext=np.abs(rot)@(.5*np.array(list(map(float,shape.get('size').split()))))
            elif shape.tag=='sphere': ext=np.full(3,float(shape.get('radius')))
            else:
                ext=float(shape.get('radius'))*np.sqrt(rot[:,0]**2+rot[:,1]**2)+float(shape.get('length'))/2*np.abs(rot[:,2])
            lower=np.minimum(lower,mat[:3,3]-ext);upper=np.maximum(upper,mat[:3,3]+ext)
    return {'min':lower.tolist(),'max':upper.tolist(),'swivel_step_rad':float(np.pi/720),
            'sampling_margin_m':0.00015,'collision_shapes':len(shapes)}


def run(output, build=DEFAULT_BUILD, integrated=False, domain=None):
    from ament_index_python.packages import get_package_prefix,get_package_share_directory
    from omslam_description.model import simulation_urdf
    from omslam_navigation.launch_support import load_configuration
    directory=Path(output).resolve()
    if (directory/'results.json').exists(): raise FileExistsError('Refusing to overwrite prior results')
    directory.mkdir(parents=True,exist_ok=True)
    result={'status':'RUNNING','checks':{}}
    def check(name,condition,**evidence):
        result['checks'][name]={'status':'PASS' if condition else 'FAIL',**evidence}
        (directory/'results.json').write_text(json.dumps(result,indent=2)+'\n')
        if not condition: raise AssertionError(name)
    try:
        packages=['omslam_navigation','omslam_navigation_contracts','nav2_smac_planner','omm_perception','omm_perception_msgs','omm_costmap_layers','omslam_sim']
        prefixes={p:str(Path(get_package_prefix(p)).resolve()) for p in packages}
        check('P0.1.prefixes',all(Path(v)==WORKSPACE/'install'/k for k,v in prefixes.items()),prefixes=prefixes)
        manifest = verified_build(build)
        result.update(source_sha256=manifest['source_sha256'], build_manifest_sha256=digest(build))
        check('P0.1.current_build', True, build_manifest=str(build), source_files=len(manifest['source_sha256']))
        nav=Path(get_package_share_directory('omslam_navigation')); sim=Path(get_package_share_directory('omslam_sim'))
        lattice=nav/'config/lattice/diff_5cm_16h_r0p5.json'
        installed=json.loads(lattice.read_text()); metadata=installed['lattice_metadata']
        expected='105b37351d15e340cc2921fa8878c8ea727a65cd7e89c39c977be0654be3b2a9'
        check('P0.2.lattice',digest(lattice)==expected and metadata['motion_model']=='diff' and metadata['grid_resolution']==.05
              and metadata['num_of_headings']==16 and len(installed['primitives'])==112,metadata=metadata,sha256=digest(lattice))
        with tempfile.TemporaryDirectory(prefix='p4-fixture-') as tmp:
            generated=generate(WORKSPACE/'tools/p4_course.json',tmp)
            check('P0.2.fixture',all(digest(sim/p)==h for p,h in generated['files'].items()),generated=generated)
        course=json.loads((WORKSPACE/'tools/p4_course.json').read_text())
        env=collision_envelope(simulation_urdf())
        footprint=np.array(course['footprint']);lo=footprint.min(axis=0);hi=footprint.max(axis=0)
        check('P0.2.footprint_envelope',np.all(np.array(env['min'])[:2]-.00015>=lo) and np.all(np.array(env['max'])[:2]+.00015<=hi),
              physical_collision_envelope=env,footprint=course['footprint'])
        legal=[]
        for event in course['events']:
            for goal in event.get('goals',[event['goal']] if 'goal' in event else []):
                if event.get('expected')=='ABORTED': continue
                shape=polygon(goal,footprint)
                legal.append(not any(intersects(shape,rectangle(o['bounds'])) for o in course['static_boxes']))
        check('P0.2.course_goals',bool(legal) and all(legal),goal_count=len(legal))
        configuration=load_configuration(nav)
        planner_parameters={'planner_server':configuration['planner_server']}
        parameter_file=directory/'planner_parameters.yaml'
        parameter_file.write_text(yaml.safe_dump(planner_parameters,sort_keys=False))
        check('P0.7.production_planner_configuration',
              planner_parameters['planner_server']['ros__parameters']['GridBased']['smooth_path'] is True,
              parameters=planner_parameters,sha256=digest(parameter_file))
        command=['ros2','run','omslam_navigation_contracts','plugin_boundary_probe',str(lattice),
                 '--ros-args','--params-file',str(parameter_file)]
        envvars=dict(os.environ,ROS_DOMAIN_ID=str(domain if domain is not None else 177),ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
        completed=subprocess.run(command,env=envvars,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=45)
        (directory/'plugin_boundary.log').write_text(completed.stdout)
        lines=[line.split('P4_BOUNDARY_RESULT ',1)[1] for line in completed.stdout.splitlines() if 'P4_BOUNDARY_RESULT ' in line]
        observed=json.loads(lines[-1]) if lines else {}
        check('P0.7.actual_plugins',completed.returncode==0 and len(observed)==10 and all(observed.values()),
              command=command,exit_code=completed.returncode,observed=observed)
        if integrated:
            destination = directory / 'command-chain'
            chain = subprocess.run([sys.executable, str(WORKSPACE/'tools/probe_p4_command_chain.py'),
                                    '--output', str(destination), '--domain', str(domain if domain is not None else 190)], cwd=WORKSPACE)
            probe = json.loads((destination/'results.json').read_text())
            check('command_chain', chain.returncode == 0 and probe['status'] == 'PASS',
                  result_file=str(destination/'results.json'))
            result['command_chain'] = result['checks']['command_chain']
            junit = directory/'map-correction.xml'
            gate = subprocess.run([sys.executable, '-m', 'pytest', '-q', '--junitxml', str(junit),
                    str(WORKSPACE/'src/omslam_navigation/test/test_navigation_guard.py')+'::test_map_correction_gate'],
                    cwd=WORKSPACE)
            cases = ET.parse(junit).findall('.//testcase')
            executed = len(cases) == 1 and cases[0].get('name') == 'test_map_correction_gate' and not any(
                cases[0].find(kind) is not None for kind in ('skipped', 'failure', 'error'))
            check('map_correction_gate', gate.returncode == 0 and executed, junit=str(junit))
            result['tests'] = {'status': 'PASS', 'map_correction_gate': True}
        result['status']='PASS'
        result['deferred_to_integrated']=['actual lifecycle/plugin/parameter readiness','live command chain and cancellation','sensor freshness and fault stop']
    except Exception as exc:
        result['status']='FAIL';result['error']={'message':str(exc),'traceback':traceback.format_exc()}
    (directory/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    if result['status']=='PASS':
        write_manifest(DEFAULT_VALIDATION, result)
    print(str(directory/'results.json'), result['status'], result.get('error', {}).get('message', ''));return 0 if result['status']=='PASS' else 1

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--build-manifest', type=Path, default=DEFAULT_BUILD)
    parser.add_argument('--integrated', action='store_true', help='Also verify the command chain and map correction gate')
    parser.add_argument('--domain', type=int, help='Use this isolated DDS domain for the sequential ROS probes')
    args=parser.parse_args()
    if args.domain is not None and not 1 <= args.domain <= 232:
        parser.error('--domain must be in 1..232')
    raise SystemExit(run(args.output, args.build_manifest.resolve(), args.integrated, args.domain))
