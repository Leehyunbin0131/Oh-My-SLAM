#!/usr/bin/env python3
"""Generate the deterministic P4 world/map fixture from p4_course.json.

The raster is a geometric fixture. It is never substituted for SLAM output.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def element(parent, tag, text=None, **attrs):
    node = ET.SubElement(parent, tag, attrs)
    if text is not None:
        node.text = str(text)
    return node


def box_model(name, center, size, color='0.65 0.72 0.78 1'):
    model = ET.Element('model', name=name)
    element(model, 'static', 'true')
    element(model, 'pose', f'{center[0]} {center[1]} {size[2]/2} 0 0 0')
    link = element(model, 'link', name='body')
    for kind in ('collision', 'visual'):
        item = element(link, kind, name=name+'_'+kind)
        geometry = element(item, 'geometry')
        element(element(geometry, 'box'), 'size', ' '.join(map(str, size)))
        if kind == 'visual':
            material = element(item, 'material')
            element(material, 'ambient', color); element(material, 'diffuse', color)
    return model


def object_sdf(name, definition):
    sdf = ET.Element('sdf', version='1.9')
    sdf.append(box_model(name, definition['center'], definition['size'], '0.82 0.31 0.15 1'))
    return ET.tostring(sdf, encoding='unicode')


def generate(definition, destination):
    spec = json.loads(Path(definition).read_text())
    out = Path(destination); (out/'worlds').mkdir(parents=True, exist_ok=True); (out/'maps').mkdir(exist_ok=True)
    sdf = ET.Element('sdf', version='1.9'); world = element(sdf, 'world', name=spec['world'])
    physics = element(world, 'physics', name='p4_physics', type='ignored')
    element(physics, 'max_step_size', '.004'); element(physics, 'real_time_factor', '1')
    for filename, classname in [('physics','Physics'),('user-commands','UserCommands'),
                                ('scene-broadcaster','SceneBroadcaster'),('sensors','Sensors'),('imu','Imu')]:
        plugin = element(world, 'plugin', filename='gz-sim-'+filename+'-system', name='gz::sim::systems::'+classname)
        if classname == 'Sensors': element(plugin, 'render_engine', 'ogre2')
    element(world, 'gravity', '0 0 -9.81')
    scene = element(world, 'scene'); element(scene, 'ambient', '.8 .8 .8 1'); element(scene, 'background', '.6 .7 .8 1')
    light = element(world, 'light', name='sun', type='directional')
    element(light, 'pose', '0 0 20 0 0 0'); element(light, 'diffuse', '.9 .9 .9 1'); element(light, 'direction', '-.3 -.2 -1')
    floor = box_model('floor', [5,4], [13,11,.1], '.75 .75 .75 1')
    floor.find('pose').text = '5 4 -.05 0 0 0'; world.append(floor)
    for obj in spec['static_boxes']:
        x0,x1,y0,y1=obj['bounds']
        world.append(box_model(obj['name'],[(x0+x1)/2,(y0+y1)/2],[x1-x0,y1-y0,obj['height']]))
    for name, obj in spec['objects'].items():
        if obj['initial']: world.append(box_model(name,obj['center'],obj['size'],'0.82 0.31 0.15 1'))
    ET.indent(sdf)
    world_path = out/'worlds'/f"{spec['world']}.sdf"
    ET.ElementTree(sdf).write(world_path, encoding='utf-8', xml_declaration=True)
    # A border outside the internal course bounds ensures all walls have raster support.
    res=spec['resolution']; b=spec['bounds']; origin=[b[0]-.2,b[2]-.2]
    width=round((b[1]-b[0]+.4)/res); height=round((b[3]-b[2]+.4)/res)
    pixels=bytearray([254])*(width*height)
    for row in range(height):
        y=origin[1]+(height-row-.5)*res
        for col in range(width):
            x=origin[0]+(col+.5)*res
            if any(a<=x<=c and d<=y<=e for a,c,d,e in (o['bounds'] for o in spec['static_boxes'])):
                pixels[row*width+col]=0
    pgm=out/'maps'/'p4_course.pgm'; pgm.write_bytes(f'P5\n{width} {height}\n255\n'.encode()+pixels)
    yaml=out/'maps'/'p4_course.yaml'
    yaml.write_text(f'image: p4_course.pgm\nmode: trinary\nresolution: {res}\norigin: [{origin[0]}, {origin[1]}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    manifest={'definition_sha256':hashlib.sha256(Path(definition).read_bytes()).hexdigest(),
              'world':spec['world'],'resolution':res,'map_origin':origin,'width':width,'height':height,
              'purpose':'static geometry fixture, not a SLAM map',
              'objects_excluded_from_map':list(spec['objects']),
              'files':{str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (world_path,pgm,yaml)}}
    (out/'maps'/'p4_course.manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (out/'maps'/'p4_course.json').write_text(json.dumps(spec,indent=2)+'\n')
    return manifest


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--definition',type=Path,default=Path(__file__).with_name('p4_course.json'))
    parser.add_argument('--output',type=Path,default=Path(__file__).resolve().parents[1]/'src/omslam_sim')
    args=parser.parse_args(); print(json.dumps(generate(args.definition,args.output),indent=2))

if __name__=='__main__': main()
