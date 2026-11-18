#!/usr/bin/env python3
"""Generate the self-contained Hanul airport environment (metres, Z up).

The two-terminal layout is fictional and inspired by large Korean airports.
All geometry, furniture, signage, maps and textures are procedurally authored
here. No external model download is necessary. The occupancy image is a
geometry-derived reference, never a SLAM result. Run from any directory.
"""
from __future__ import annotations

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import random
import xml.etree.ElementTree as ET

from PIL import Image, ImageDraw, ImageFont


PALETTE = {
    'stone': ((0.76, .79, .79, 1), .82, .0),
    'tile_joint': ((.58, .63, .65, 1), .8, .0),
    'ivory': ((.88, .88, .83, 1), .67, .0),
    'frame': ((.18, .25, .29, 1), .32, .65),
    'steel': ((.61, .67, .68, 1), .25, .75),
    'glass': ((.25, .55, .64, .30), .16, .05),
    'wood': ((.46, .27, .13, 1), .75, .0),
    'wood_light': ((.70, .50, .28, 1), .72, .0),
    'seat_blue': ((.075, .19, .31, 1), .72, .0),
    'seat_teal': ((.06, .30, .29, 1), .72, .0),
    'accent_blue': ((.08, .28, .44, 1), .65, .0),
    'accent_teal': ((.10, .40, .35, 1), .65, .0),
    'foliage': ((.19, .36, .15, 1), .92, .0),
    'foliage_light': ((.35, .49, .20, 1), .92, .0),
    'soil': ((.12, .09, .055, 1), 1., .0),
    'black': ((.035, .055, .065, 1), .38, .0),
    'screen': ((.035, .24, .32, 1), .23, .0),
    'light': ((.95, .95, .83, 1), .6, .0),
    'safety': ((.92, .62, .16, 1), .6, .0),
}

TERMINAL_CENTERS=(-84.,0.)


def rotate(x, y, yaw):
    return x*math.cos(yaw)-y*math.sin(yaw), x*math.sin(yaw)+y*math.cos(yaw)


class Mesh:
    def __init__(self):
        self.vertices = []
        self.faces = []

    def polygon(self, vertices):
        start = len(self.vertices)
        self.vertices.extend(vertices)
        for i in range(1, len(vertices)-1):
            self.faces.append((start, start+i, start+i+1))

    def box(self, x, y, z, sx, sy, sz, yaw=0):
        v = []
        for dz in (-sz/2, sz/2):
            for dx, dy in ((-sx/2,-sy/2),(sx/2,-sy/2),(sx/2,sy/2),(-sx/2,sy/2)):
                rx, ry = rotate(dx, dy, yaw)
                v.append((x+rx, y+ry, z+dz))
        for indices in ((0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)):
            self.polygon([v[i] for i in indices])

    def cylinder(self, x, y, z, radius, length, segments=12):
        rings = [[(x+radius*math.cos(2*math.pi*i/segments),
                   y+radius*math.sin(2*math.pi*i/segments), z+d)
                  for i in range(segments)] for d in (-length/2,length/2)]
        self.polygon(list(reversed(rings[0])))
        self.polygon(rings[1])
        for i in range(segments):
            j=(i+1)%segments
            self.polygon([rings[0][i],rings[0][j],rings[1][j],rings[1][i]])

    def ellipsoid(self, x,y,z,sx,sy,sz, slices=10, stacks=6):
        for j in range(stacks):
            a0=-math.pi/2+math.pi*j/stacks
            a1=-math.pi/2+math.pi*(j+1)/stacks
            for i in range(slices):
                b0=2*math.pi*i/slices
                b1=2*math.pi*(i+1)/slices
                def pt(a,b):
                    return (x+sx*math.cos(a)*math.cos(b),y+sy*math.cos(a)*math.sin(b),z+sz*math.sin(a))
                self.polygon([pt(a0,b0),pt(a0,b1),pt(a1,b1),pt(a1,b0)])

    def save_dae(self, path, rgba, emissive=False):
        """Export normals and a bound material, rather than a bare OBJ.

        Gazebo's native OBJ loader does not assign a material to material-less
        shapes; relying only on an SDF/PBR override yielded white unlit geometry
        in the Ogre2 sensor renderer. Keep a complete common-profile material
        inside each mesh, as supported by both Ogre and Ogre2.
        """
        ns='http://www.collada.org/2005/11/COLLADASchema'
        ET.register_namespace('',ns)
        def el(parent,tag,attrs=None,text=None):
            e=ET.SubElement(parent,'{'+ns+'}'+tag,attrs or {})
            if text is not None:e.text=text
            return e
        root=ET.Element('{'+ns+'}COLLADA',{'version':'1.4.1'})
        asset=el(root,'asset')
        el(asset,'created',text='2026-11-12T00:00:00Z')
        el(asset,'modified',text='2026-11-12T00:00:00Z')
        el(asset,'unit',{'name':'meter','meter':'1'})
        el(asset,'up_axis',text='Z_UP')
        effect=el(el(root,'library_effects'),'effect',{'id':'surface-effect'})
        phong=el(el(el(effect,'profile_COMMON'),'technique',{'sid':'common'}),'phong')
        color=' '.join(str(c) for c in rgba)
        el(el(phong,'emission'),'color',text='.35 .35 .28 1' if emissive else '0 0 0 1')
        el(el(phong,'ambient'),'color',text=color)
        el(el(phong,'diffuse'),'color',text=color)
        el(el(phong,'specular'),'color',text='.08 .08 .08 1')
        el(el(phong,'shininess'),'float',text='12')
        el(el(phong,'index_of_refraction'),'float',text='1')
        material=el(el(root,'library_materials'),'material',{'id':'surface-material','name':path.stem})
        el(material,'instance_effect',{'url':'#surface-effect'})
        mesh=el(el(el(root,'library_geometries'),'geometry',{'id':'geometry','name':path.stem}),'mesh')
        normals=[]
        for ia,ib,ic in self.faces:
            a,b,c=(self.vertices[i] for i in (ia,ib,ic))
            u=[b[i]-a[i] for i in range(3)];v=[c[i]-a[i] for i in range(3)]
            n=(u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0])
            length=math.sqrt(sum(x*x for x in n))
            normals.append(tuple(x/length for x in n) if length>1e-12 else (0.,0.,1.))
        for name,values in (('positions',self.vertices),('normals',normals)):
            source=el(mesh,'source',{'id':name})
            arr=el(source,'float_array',{'id':name+'-array','count':str(len(values)*3)},' '.join(f'{c:.6f}' for v in values for c in v))
            access=el(el(source,'technique_common'),'accessor',{'source':'#'+arr.attrib['id'],'count':str(len(values)),'stride':'3'})
            for param in ('X','Y','Z'):el(access,'param',{'name':param,'type':'float'})
        vertex=el(mesh,'vertices',{'id':'vertices'})
        el(vertex,'input',{'semantic':'POSITION','source':'#positions'})
        triangles=el(mesh,'triangles',{'count':str(len(self.faces)),'material':'surface-material'})
        el(triangles,'input',{'semantic':'VERTEX','source':'#vertices','offset':'0'})
        el(triangles,'input',{'semantic':'NORMAL','source':'#normals','offset':'1'})
        el(triangles,'p',text=' '.join(f'{vertex} {n}' for n,face in enumerate(self.faces) for vertex in face))
        scene=el(el(root,'library_visual_scenes'),'visual_scene',{'id':'Scene'})
        inst=el(el(scene,'node',{'id':'mesh-node','name':path.stem}),'instance_geometry',{'url':'#geometry'})
        el(el(el(inst,'bind_material'),'technique_common'),'instance_material',{'symbol':'surface-material','target':'#surface-material'})
        el(el(root,'scene'),'instance_visual_scene',{'url':'#Scene'})
        ET.indent(root)
        ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)


class Airport:
    def __init__(self, package):
        self.package=package
        self.assets=package/'models/omslam_airport_assets'
        self.meshes=defaultdict(Mesh)
        self.collisions=[]
        self.obstacles=[]
        self.signs=[]
        self.object_index=0
        self.floor_area=0.
        self.boarding_seats=0
        self.semantic_counts=defaultdict(int)
        self.rng=random.Random(1142)
        self.layout={
            'schema_version':1, 'world':'airport', 'units':'m',
            'description':'Fictional two-terminal airport inspired by Incheon; not an as-built replica.',
            'bounds':[-122,-62,38,62],
            'spawn':{'x':-108.,'y':0.,'yaw':0.},
            'floors':[], 'terminals':[], 'corridors':[], 'obstacles':self.obstacles,
            'landmarks':[], 'pedestrian_routes':[], 'viewpoints':[],
            'reference_map':{'resolution':.1,'origin':[-122.,-62.,0.],
                             'image':'airport_reference.pgm','kind':'geometry_reference_not_slam',
                             'obstacle_projection_z':[.08,2.]},
        }

    def name(self, kind):
        self.object_index+=1
        return f'{kind}_{self.object_index:04d}'

    def box(self, material,x,y,z,sx,sy,sz,yaw=0,collision=False,name=None):
        self.meshes[material].box(x,y,z,sx,sy,sz,yaw)
        if collision:self.collision_box(name or self.name(material),x,y,z,sx,sy,sz,yaw)

    def cylinder(self, material,x,y,z,radius,length,collision=False,name=None):
        self.meshes[material].cylinder(x,y,z,radius,length)
        if collision:
            nm=name or self.name(material)
            self.collisions.append({'name':nm,'type':'cylinder','pose':[x,y,z,0,0,0],
                                    'radius':radius,'length':length})
            polygon=[[x+radius*math.cos(i*math.pi/8),y+radius*math.sin(i*math.pi/8)] for i in range(16)]
            self.obstacles.append({'name':nm,'footprint':polygon,'z_min':z-length/2,'z_max':z+length/2})

    def collision_box(self,name,x,y,z,sx,sy,sz,yaw=0):
        self.collisions.append({'name':name,'type':'box','pose':[x,y,z,0,0,yaw],'size':[sx,sy,sz]})
        points=[]
        for dx,dy in ((-sx/2,-sy/2),(sx/2,-sy/2),(sx/2,sy/2),(-sx/2,sy/2)):
            rx,ry=rotate(dx,dy,yaw);points.append([round(x+rx,5),round(y+ry,5)])
        self.obstacles.append({'name':name,'footprint':points,'z_min':round(z-sz/2,5),'z_max':round(z+sz/2,5)})

    def floor(self,name,bounds):
        self.layout['floors'].append({'name':name,'bounds':bounds})

    def corridor(self,name,width,bounds):
        self.layout['corridors'].append({'name':name,'width':width,'bounds':bounds})

    def goal(self,name,x,y,yaw=0):
        self.layout['landmarks'].append({'name':name,'pose':[x,y,yaw]})

    def sign(self,label,x,y,z,width,height,yaw=0,horizontal=False):
        self.signs.append({'label':label,'center':[x,y,z],'width':width,'height':height,
                           'yaw':yaw,'horizontal':horizontal})

    def wall_segment(self,name,x1,y1,x2,y2,height=5.6,glass=True):
        length=math.hypot(x2-x1,y2-y1)
        if length<.01:return
        x,y=(x1+x2)/2,(y1+y2)/2
        yaw=math.atan2(y2-y1,x2-x1)
        self.collision_box(name,x,y,height/2,length,.24,height,yaw)
        if glass:
            self.box('ivory',x,y,.45,length,.24,.9,yaw)
            self.box('glass',x,y,2.9,length,.09,4.,yaw)
            self.box('frame',x,y,5.1,length,.20,.22,yaw)
            count=max(1,int(length/4))
            for i in range(count+1):
                t=i/count
                self.box('steel',x1+t*(x2-x1),y1+t*(y2-y1),2.8,.10,.10,5.6)
        else:
            self.box('ivory',x,y,height/2,length,.24,height,yaw)
            self.box('wood_light',x,y,.7,length,.26,.10,yaw)

    def inside_wall(self,name,x,y,sx,sy,height=2.6):
        self.box('ivory',x,y,height/2,sx,sy,height,collision=True,name=name)
        if sx>sy:self.box('wood_light',x,y,.85,sx,sy+.015,.11)
        else:self.box('wood_light',x,y,.85,sx+.015,sy,.11)

    def room(self,name,x0,y0,x1,y1,top_door=None,bottom_door=None):
        self.inside_wall(name+'_west',x0,(y0+y1)/2,.20,y1-y0)
        self.inside_wall(name+'_east',x1,(y0+y1)/2,.20,y1-y0)
        for yy,door,suffix in ((y0,bottom_door,'south'),(y1,top_door,'north')):
            if door:
                center,width=door
                for a,b in ((x0,center-width/2),(center+width/2,x1)):
                    self.inside_wall(name+'_'+suffix+f'_{a:.1f}',(a+b)/2,yy,b-a,.20)
            else:self.inside_wall(name+'_'+suffix,(x0+x1)/2,yy,x1-x0,.20)

    def bench(self,x,y,seat_mat='seat_blue',yaw=0,back_to_back=True):
        self.semantic_counts['boarding_bench_groups']+=1
        self.boarding_seats+=8 if back_to_back else 4
        nm=self.name('airport_bench')
        def part(mat,dx,dy,z,sx,sy,sz):
            rx,ry=rotate(dx,dy,yaw);self.box(mat,x+rx,y+ry,z,sx,sy,sz,yaw)
        for side in (-1,1) if back_to_back else (1,):
            for i in range(4):
                dx=(i-1.5)*.68
                part(seat_mat,dx,side*.32,.48,.60,.54,.12)
                part(seat_mat,dx,side*.05,.80,.60,.10,.58)
                part('steel',dx-.31,side*.30,.68,.045,.54,.04)
                part('steel',dx+.31,side*.30,.68,.045,.54,.04)
            for dx in (-.95,.95):part('steel',dx,side*.27,.23,.10,.12,.46)
        part('steel',0,0,.33,2.72,.16,.12)
        self.collision_box(nm,x,y,.54,2.76,1.28 if back_to_back else .75,1.08,yaw)

    def armchair(self,x,y,material='seat_teal',yaw=0):
        self.semantic_counts['lounge_armchairs']+=1
        def part(mat,dx,dy,z,sx,sy,sz):
            rx,ry=rotate(dx,dy,yaw);self.box(mat,x+rx,y+ry,z,sx,sy,sz,yaw)
        part(material,0,0,.38,.84,.80,.22)
        part(material,0,.34,.71,.84,.17,.76)
        for dx in (-.40,.40):part(material,dx,0,.58,.15,.86,.40)
        for dx in (-.31,.31):
            for dy in (-.27,.27):part('wood',dx,dy,.12,.07,.07,.24)
        self.collision_box(self.name('lounge_chair'),x,y,.52,1.,.92,1.04,yaw)

    def table(self,x,y,r=.68,height=.76,stools=0):
        self.semantic_counts['cafe_tables' if stools else 'coffee_tables']+=1
        self.semantic_counts['cafe_stools']+=stools
        self.cylinder('wood_light',x,y,height,r,.08,True,self.name('table_top'))
        self.cylinder('steel',x,y,height/2,.08,height)
        self.cylinder('black',x,y,.04,r*.46,.08,True,self.name('table_base'))
        for i in range(stools):
            a=2*math.pi*i/stools
            xx=x+(r+.45)*math.cos(a);yy=y+(r+.45)*math.sin(a)
            self.cylinder('seat_teal',xx,yy,.47,.25,.10)
            self.cylinder('steel',xx,yy,.23,.07,.46)
            self.cylinder('black',xx,yy,.03,.23,.06,True,self.name('stool'))

    def plant(self,x,y,r=.7,tree=False):
        self.semantic_counts['tree_planters' if tree else 'low_planters']+=1
        self.cylinder('ivory',x,y,.30,r,.60,True,self.name('planter'))
        self.cylinder('soil',x,y,.605,r*.90,.02)
        if tree:
            self.cylinder('wood',x,y,1.6,.12,2.2)
            for dx,dy,dz,rr in ((0,0,0,1.6),(.7,.1,.4,1.1),(-.5,.2,.3,1.1)):
                self.meshes['foliage'].ellipsoid(x+dx,y+dy,3.2+dz,rr,rr,.85)
        else:
            for i in range(5):
                a=2*math.pi*i/5
                self.meshes['foliage' if i%2 else 'foliage_light'].ellipsoid(
                    x+.3*math.cos(a),y+.3*math.sin(a),.95,r*.60,r*.36,.5)

    def kiosk(self,x,y,yaw=0):
        self.semantic_counts['kiosks']+=1
        self.box('ivory',x,y,.75,.64,.55,1.5,yaw,True,self.name('kiosk'))
        self.box('frame',x,y,1.50,.72,.18,.58,yaw)
        self.sign('KIOSK',x,y-.105,1.50,.62,.43,yaw)
        self.box('black',x,y,.08,.80,.70,.16,yaw)

    def counter(self,x,y,width=4.,label='CHECK IN'):
        self.semantic_counts['service_counters']+=1
        self.box('wood_light',x,y,.52,width,1.1,1.04,collision=True,name=self.name('counter'))
        self.box('stone',x,y,1.08,width+.12,1.25,.12)
        for dx in (-width*.3,width*.3):
            self.box('black',x+dx,y,1.28,.08,.08,.3)
            self.box('screen',x+dx,y,1.52,.60,.12,.38)
        self.sign(label,x,y-.57,.66,width*.7,.45)

    def luggage(self,x,y,color='accent_blue',yaw=0):
        self.semantic_counts['luggage']+=1
        self.box(color,x,y,.38,.48,.31,.68,yaw,True,self.name('luggage'))
        self.box('black',x,y,.81,.22,.035,.20,yaw)
        self.box('black',x,y,.91,.25,.05,.04,yaw)
        for dx in (-.15,.15):self.cylinder('black',x+dx,y,.045,.045,.09)

    def build_structure(self):
        for i,cx in enumerate(TERMINAL_CENTERS,1):
            self.floor(f'terminal_{i}',[cx-34,-46,cx+34,46])
            self.floor(f'terminal_{i}_service_annex',[cx-34,-60,cx+34,-46])
            self.layout['terminals'].append({'id':f'T{i}','name':f'Terminal {i}',
                                             'bounds':[cx-34,-46,cx+34,46]})
        self.floor('north_transfer_gallery',[-118,46,34,60])
        for a,b in ((-50,-34),):
            for y,width in ((-28.,8.),(0.,12.),(28.,8.)):
                self.floor(f'link_{a}_{y}',[a,y-width/2,b,y+width/2])
                effective=6.4 if width==12 else 3.8
                self.corridor(f'interterminal_{a}_{y}',effective,[a,y-effective/2,b,y+effective/2])
                self.layout['corridors'][-1]['structural_width']=width
        # Build a disjoint rectangle cover and the boundary of its union.
        rects=[r['bounds'] for r in self.layout['floors']]
        xs=sorted({b[i] for b in rects for i in (0,2)})
        ys=sorted({b[i] for b in rects for i in (1,3)})
        cells=set()
        for ix in range(len(xs)-1):
            for iy in range(len(ys)-1):
                mx=(xs[ix]+xs[ix+1])/2;my=(ys[iy]+ys[iy+1])/2
                if any(a<=mx<=c and b<=my<=d for a,b,c,d in rects):
                    cells.add((ix,iy))
                    self.floor_area+=(xs[ix+1]-xs[ix])*(ys[iy+1]-ys[iy])
                    self.box('stone',mx,my,-.055,xs[ix+1]-xs[ix],ys[iy+1]-ys[iy],.11)
        self.collision_box('continuous_floor',-42,0,-.13,160,124,.24)
        # merge collinear boundary edges to avoid duplicated window frames.
        edges=defaultdict(list)
        for ix,iy in cells:
            if (ix-1,iy) not in cells:edges[('v',xs[ix])].append((ys[iy],ys[iy+1]))
            if (ix+1,iy) not in cells:edges[('v',xs[ix+1])].append((ys[iy],ys[iy+1]))
            if (ix,iy-1) not in cells:edges[('h',ys[iy])].append((xs[ix],xs[ix+1]))
            if (ix,iy+1) not in cells:edges[('h',ys[iy+1])].append((xs[ix],xs[ix+1]))
        for (axis,value),segments in edges.items():
            merged=[]
            for a,b in sorted(segments):
                if merged and abs(merged[-1][1]-a)<1e-5:merged[-1]=(merged[-1][0],b)
                else:merged.append((a,b))
            for a,b in merged:
                if axis=='h':self.wall_segment(self.name('facade'),a,value,b,value)
                else:self.wall_segment(self.name('facade'),value,a,value,b)
        self.corridor('north_transfer_gallery',3.6,[-116,52,32,55.6])
        self.layout['corridors'][-1]['structural_width']=14.
        # Ceramic floor grout: decorative visual only, clipped to floor union.
        for x in range(-116,35,4):
            for y in range(-58,60,4):
                if any(a+.05<x<c-.05 and b+.05<y<d-.05 for a,b,c,d in rects):
                    self.box('tile_joint',x,y,.003,.025,3.98,.004)
                    self.box('tile_joint',x,y,.003,3.98,.025,.004)

    def build_terminal(self,i,cx):
        seat=['seat_blue','seat_teal'][i-1]
        accent=['accent_blue','accent_teal'][i-1]
        self.corridor(f'T{i}_main_east_west',6.4,[cx-33,-3.2,cx+33,3.2])
        self.corridor(f'T{i}_main_north_south',6.4,[cx-3.2,-45,cx+3.2,59])
        for yy in (-28.,28.):self.corridor(f'T{i}_cross_{yy}',3.8,[cx-33,yy-1.9,cx+33,yy+1.9])
        # Terminal identity floor inlays and elevated directional signs.
        for yy in (-3.2,3.2):self.box(accent,cx,yy,.008,67,.18,.015)
        self.sign(f'T{i}',cx,0,.018,8.0,3.0,horizontal=True)
        for yy in (-8.,8.):
            self.box('frame',cx,yy,4.6,11,.18,1.5)
            self.sign(f'T{i}',cx,yy-.10,4.6,10.7,1.32)
            self.sign(f'T{i}',cx,yy+.10,4.6,10.7,1.32,math.pi)
            for dx in (-4.6,4.6):self.cylinder('steel',cx+dx,yy,5.8,.035,1.4)
        self.goal(f'T{i}_arrival',cx-25,0)
        self.goal(f'T{i}_atrium',cx,0)
        self.goal(f'T{i}_north_lounge',cx,37,math.pi/2)
        self.goal(f'T{i}_gates',cx,-34,-math.pi/2)
        self.goal(f'T{i}_service',cx,-58,0)
        # Slender steel columns and open roof ribs keep the halls readable.
        for xx in (-29.,29.):
            for yy in (-40.,-12.,12.,40.):
                self.cylinder('ivory',cx+xx,yy,3.4,.30,6.8,True,self.name('column'))
                self.cylinder('steel',cx+xx,yy,.20,.42,.40)
        for yy in (-40.,-12.,12.,40.):
            self.box('ivory',cx,yy,7.0,60,.34,.34)
            self.box('light',cx,yy,6.81,54,.11,.045)
        for xx in (-27.,27.):self.box('ivory',cx+xx,0,7.1,.30,83,.30)
        # Back-to-back boarding benches, with clear cross corridors at +/-28 m.
        for xx in (-23.,-13.,13.,23.):
            for yy in (-38.,-18.):
                self.bench(cx+xx,yy,seat)
                self.bench(cx+xx,yy+3.4,seat)
        for xx in (-29.,29.):
            for yy in (-33.,-21.,21.,33.):self.plant(cx+xx,yy,.62)
        # Check-in / gate desks and glazed gate portals.
        for k,xx in enumerate((-15.,15.)):
            self.counter(cx+xx,-42.2,5.6,'GATES')
            self.box('frame',cx+xx,-44.5,3.5,7.,.18,.65)
            self.sign('GATES',cx+xx,-44.61,3.5,6.7,.52)
            for q in (-2.5,2.5):self.cylinder('steel',cx+xx+q,-44.5,1.6,.07,3.2)
        for xx in (-10.,10.):self.kiosk(cx+xx,-9)
        # Cafe court: timber flooring, counter, island tables and bar stools.
        self.box('wood_light',cx-21,16.2,.014,21.,15.,.025)
        for xx in (-29.,-21.,-13.):
            for yy in (12.,20.):self.table(cx+xx,yy,.78,.77,4)
        self.counter(cx-20.5,41.0,15.0,'CAFE')
        for xx in (-25.,-20.,-15.):
            self.box('steel',cx+xx,41.1,1.35,.75,.55,.50)
            self.box('black',cx+xx,40.79,1.40,.60,.06,.25)
        self.sign('CAFE',cx-20.5,42.0,3.35,14.0,1.2)
        self.goal(f'T{i}_cafe',cx-21,34,math.pi/2)
        # Garden lounge / quiet seating with non-identical terminal geometry.
        for xx,yy in ((15.,15.),(25.,16.),(15.,39.),(25.,39.)):
            self.armchair(cx+xx-1.25,yy,seat,-math.pi/2)
            self.armchair(cx+xx+1.25,yy,seat,math.pi/2)
            self.table(cx+xx,yy,.48,.45)
        self.plant(cx+20,21,1.35,True)
        self.plant(cx+21,34,1.25,True)
        if i==2:
            # A larger garden gives T2 a distinct geometry and visual identity.
            self.plant(cx+10.5,19.0,1.05,True)
            self.plant(cx+26.0,20.2,.90,True)
            self.plant(cx+10.5,35.0,1.0,True)
            self.plant(cx+28.0,35.0,.85)
            for xx in (10.,14.):
                self.armchair(cx+xx,11.,seat,math.pi)
        self.sign('LOUNGE',cx+20,43.,3.5,13.,1.2)
        # Flight displays with structural frames.
        for xx in (-9.,9.):
            self.box('frame',cx+xx,11.0,1.9,.18,.18,3.8)
            self.box('black',cx+xx,11.,3.45,4.0,.20,1.50)
            self.sign('FLIGHTS',cx+xx,10.88,3.45,3.8,1.30)
        # Trolley corral and loose low obstacles away from guaranteed lanes.
        self.box('steel',cx-30,-11,.45,4.6,.07,.08)
        for j in range(4):
            x=cx-31.5+j*.8
            self.box('steel',x,-12.,.25,.65,1.1,.05)
            self.box('steel',x,-11.5,.60,.64,.05,.70)
            self.box('wood_light',x,-12.0,.48,.48,.60,.42)
            self.collision_box(self.name('luggage_trolley'),x,-12.,.5,.70,1.2,1.)
        for xx,yy in ((27,-9),(9,-39),(-8,36)):
            self.luggage(cx+xx,yy,accent,.15*i)
        self.box('wood_light',cx+29.0,-15.,.13,1.7,1.3,.26,collision=True,name=self.name('low_parcel'))
        # Service annex: enclosed rooms, 2.4 m doors, narrow through passage.
        for a,b in ((-34,-21.2),(-18.8,-1.5),(1.5,18.8),(21.2,34)):
            self.inside_wall(f'T{i}_annex_partition_{a}',cx+(a+b)/2,-46,b-a,.20,3.)
        self.room(f'T{i}_service_west',cx-30,-56,cx-6,-49,top_door=(cx-18,2.4),bottom_door=(cx-18,2.4))
        self.room(f'T{i}_service_east',cx+6,-56,cx+30,-49,top_door=(cx+18,1.4),bottom_door=(cx+18,2.4))
        # Parallel walls form a genuine 1.4 m wide, 4 m long corridor.
        for xx in (17.2,18.8):self.inside_wall(f'T{i}_narrow_wall_{xx}',cx+xx,-52.5,.20,4.0,2.6)
        self.corridor(f'T{i}_narrow_service_passage',1.4,[cx+17.3,-54.5,cx+18.7,-50.5])
        self.corridor(f'T{i}_service_north',2.8,[cx-33,-48.9,cx+33,-46.1])
        self.corridor(f'T{i}_service_south',3.6,[cx-33,-59.8,cx+33,-56.2])
        self.counter(cx-24,-52.5,5.0,'SERVICE')
        self.box('wood_light',cx-10,-52.5,.75,3.,1.4,1.5,collision=True,name=self.name('storage_cabinet'))
        self.counter(cx+10,-52.7,4.,'SERVICE')
        self.box('wood_light',cx+27,-52.8,.75,3.,1.4,1.5,collision=True,name=self.name('storage_cabinet'))
        self.sign('SERVICE',cx,-45.8,2.55,3.0,.70)
        self.goal(f'T{i}_narrow_passage',cx+18,-52.5,-math.pi/2)
        # Lower columns support a gallery along the terminal back edge.
        for xx in (-26.,-13.,13.,26.):
            self.cylinder('ivory',cx+xx,48.0,3.4,.25,6.8,True,self.name('gallery_column'))
        self.sign('TRANSFER',cx,46.0,4.5,10.,1.3)
        self.sign('TRANSFER',cx,46.0,4.5,10.,1.3,math.pi)
        # Four route families per terminal: two long transfer/crossing loops
        # and two short cafe/boarding loops. Four walkers share each family at
        # equal speed, spaced by one quarter of the cycle to avoid bunching.
        routes=[
            [[cx-28,-1.2],[cx+28,-1.2],[cx+28,1.2],[cx-28,1.2]],
            [[cx-1.3,-31.],[cx-1.3,55.6],[cx+1.3,55.6],[cx+1.3,-31.]],
            [[cx-26,14.2],[cx-16,14.2],[cx-16,17.8],[cx-26,17.8]],
            [[cx+9.5,-22.],[cx+19.5,-22.],[cx+19.5,-11.5],[cx+9.5,-11.5]],
        ]
        for j,points in enumerate(routes):
            speed=round(.80+.07*(i-1)+.06*j,2)
            length=sum(math.dist(a,b) for a,b in zip(points,points[1:]+points[:1]))
            for person in range(4):
                k=(i-1)*16+j*4+person
                self.layout['pedestrian_routes'].append({'name':f'pedestrian_{k:02d}',
                    'speed':speed,'phase':round(length/speed*person/4+i*1.7,5),
                    'waypoints':points,'route_family':f'T{i}_'+('transfer','crossing','cafe','boarding')[j]})

    @staticmethod
    def segment_hits_rect(a,b,rect):
        """Closed-segment / axis-aligned rectangle intersection."""
        low,high=0.,1.
        for axis in range(2):
            delta=b[axis]-a[axis]
            if abs(delta)<1e-10:
                if not rect[axis]<=a[axis]<=rect[axis+2]:return False
            else:
                u=(rect[axis]-a[axis])/delta;v=(rect[axis+2]-a[axis])/delta
                low=max(low,min(u,v));high=min(high,max(u,v))
                if high<low:return False
        return True

    def can_place(self,x,y,sx,sy,clearance=1.05,protect_corridors=True):
        rect=(x-sx/2,y-sy/2,x+sx/2,y+sy/2)
        if not any(a<=rect[0] and b<=rect[1] and c>=rect[2] and d>=rect[3]
                   for a,b,c,d in (f['bounds'] for f in self.layout['floors'])):
            return False
        # Keep the clear spine widths physical, not just metadata labels.
        if protect_corridors:
            for c in self.layout['corridors']:
                a,b,cc,d=c['bounds']
                if rect[0]<cc and rect[2]>a and rect[1]<d and rect[3]>b:return False
        for goal in self.layout['landmarks']:
            gx,gy,_=goal['pose']
            if rect[0]-1.1<gx<rect[2]+1.1 and rect[1]-1.1<gy<rect[3]+1.1:return False
        # 0.30 m physical pedestrian radius plus 0.25 m raster/tolerance margin.
        padded=(rect[0]-.55,rect[1]-.55,rect[2]+.55,rect[3]+.55)
        seen=set()
        for route in self.layout['pedestrian_routes']:
            family=route['route_family']
            if family in seen:continue
            seen.add(family)
            pts=route['waypoints']
            if any(self.segment_hits_rect(a,b,padded) for a,b in zip(pts,pts[1:]+pts[:1])):return False
        for o in self.obstacles:
            if o['z_max']<=.08 or o['z_min']>=2.:continue
            a=min(p[0] for p in o['footprint'])-clearance
            b=min(p[1] for p in o['footprint'])-clearance
            c=max(p[0] for p in o['footprint'])+clearance
            d=max(p[1] for p in o['footprint'])+clearance
            if rect[0]<c and rect[2]>a and rect[1]<d and rect[3]>b:return False
        return True

    def retail_stand(self,x,y,accent):
        """A compact display island with shelves, products and a glazed back."""
        self.semantic_counts['retail_stands']+=1
        self.box('wood_light',x,y,.48,2.5,1.2,.96)
        self.box('stone',x,y,1.0,2.6,1.3,.08)
        self.box('glass',x,y+.50,1.42,2.4,.06,.80)
        for xx in (-.85,-.42,0.,.42,.85):
            self.cylinder(accent,x+xx,y-.15,1.19,.12,.30)
            self.box('ivory',x+xx,y+.20,1.18,.25,.20,.28)
        self.sign('RETAIL',x,y-.61,.60,2.25,.50)
        self.collision_box(self.name('retail_stand'),x,y,.9,2.6,1.3,1.8)

    def low_partition(self,x,y,accent,yaw=0):
        self.semantic_counts['low_partitions']+=1
        self.box('wood_light',x,y,.48,2.6,.24,.96,yaw,True,self.name('low_partition'))
        self.box(accent,x,y,.98,2.7,.30,.10,yaw)
        for dx in (-1.15,1.15):
            rx,ry=rotate(dx,0,yaw)
            self.box('steel',x+rx,y+ry,.07,.36,.55,.14,yaw)

    def densify(self):
        """Fill former empty lounge fields with furniture, preserving routes."""
        for i,cx in enumerate(TERMINAL_CENTERS,1):
            seat=['seat_blue','seat_teal'][i-1]
            accent=['accent_blue','accent_teal'][i-1]
            xs=[sign*(4.7+4.4*k) for k in range(7) for sign in (-1,1)]
            ys=[-4.0,-7.2,-10.4,-13.6,-16.8,-20.,-25.2,-22.,-30.8,-34.,-37.2,-40.4]
            # Dense gate seating: 1.6–1.9 m aisles between full bench groups.
            for iy,y in enumerate(ys):
                for ix,x in enumerate(xs):
                    if not self.can_place(cx+x,y,2.80,1.32):continue
                    if (ix+2*iy+i)%9==0:self.retail_stand(cx+x,y,accent)
                    else:self.bench(cx+x,y,seat)
            # Place the northern cross-aisle edge islands before other pods,
            # so the former 8 m field becomes a physically bounded ~4.3 m aisle.
            for y in (25.2,30.8):
                for ix,x in enumerate(xs):
                    if self.can_place(cx+x,y,2.80,1.32):
                        if (ix+i)%4==0:self.retail_stand(cx+x,y,accent)
                        else:self.bench(cx+x,y,seat)
            # Left: cafes; right: lounge seating pods, retail, and partitions.
            for iy,y in enumerate((5.0,9.7,14.4,19.1,23.8,32.8,37.5,42.2)):
                for ix,x in enumerate((-6.,-10.7,-15.4,-20.1,-24.8,-29.5)):
                    if self.can_place(cx+x,y,2.75,2.75,clearance=1.0):
                        if (ix+iy+i)%5==0:self.retail_stand(cx+x,y,accent)
                        else:self.table(cx+x,y,.72,.77,4)
                for ix,x in enumerate((6.,11.3,16.6,21.9,27.2,31.0)):
                    if not self.can_place(cx+x,y,3.5,1.5,clearance=1.05):continue
                    kind=(ix+iy+i)%5
                    if kind==0:self.low_partition(cx+x,y,accent)
                    elif kind==1:self.retail_stand(cx+x,y,accent)
                    elif kind==2:self.bench(cx+x,y,seat)
                    else:
                        self.armchair(cx+x-1.20,y,seat,-math.pi/2)
                        self.armchair(cx+x+1.20,y,seat,math.pi/2)
                        self.table(cx+x,y,.44,.45)
            # Human-scale wayfinding islands and low planters break residual
            # large diagonal sightlines without closing the connected aisles.
            for iy,y in enumerate(range(-41,45,4)):
                for ix,x in enumerate(range(-30,32,4)):
                    if not self.can_place(cx+x,y,1.15,1.15,clearance=1.25):continue
                    if (ix+iy+i)%3==0:self.kiosk(cx+x,y)
                    else:self.plant(cx+x,y,.55,tree=False)
            # Furnish the previously empty north gallery in three parallel rows.
            for iy,y in enumerate((47.0,51.0,57.1)):
                for ix,x in enumerate((-30.,-25.6,-21.2,-16.8,-12.4,-8.,8.,12.4,16.8,21.2,25.6,30.)):
                    if self.can_place(cx+x,y,2.8,1.32,clearance=.8):
                        if (ix+iy)%4==0:self.retail_stand(cx+x,y,accent)
                        else:self.bench(cx+x,y,seat)
        # Side furniture in the inter-terminal links reduces real free width.
        for cx in (-42.,):
            for yy,width in ((-28.,8.),(0.,12.),(28.,8.)):
                offset=4.0 if width==12 else 2.6
                for dx in (-5.,0.,5.):
                    for side in (-1,1):
                        y=yy+side*offset
                        if self.can_place(cx+dx,y,2.8,1.32,clearance=.50):
                            self.bench(cx+dx,y,'seat_teal')

    def finish_layout(self):
        self.goal('transfer_west',-42,0)
        self.goal('north_gallery_west',-42,54)
        for x in (-42,):
            for y in (-28,0,28):
                self.box('frame',x,y-3.,4.0,10,.18,.95)
                self.sign('TRANSFER',x,y-3.11,4.,9.6,.8)
                self.sign('TRANSFER',x,y-2.89,4.,9.6,.8,math.pi)
        self.layout['viewpoints']=[
            {'name':'overview','pose':[-42,-168,166,0,.72,math.pi/2],'hfov':1.20},
            {'name':'terminal_1','pose':[-114,-40,20,0,.40,.72],'hfov':1.05},
            {'name':'terminal_2','pose':[-27,-39,18,0,.37,.75],'hfov':1.05},
            {'name':'concourse_eye','pose':[-110,-2.5,1.65,0,0,.02],'hfov':1.15},
        ]
        self.layout['statistics']={'terminal_count':len(TERMINAL_CENTERS),'walking_routes':len(self.layout['pedestrian_routes']),
            'collision_shapes':len(self.collisions),'boarding_seating_capacity':self.boarding_seats,
            'semantic_counts':dict(self.semantic_counts),
            'floor_area_m2':round(self.floor_area,2),
            'area_note':'Connected indoor floor area computed from the non-overlapping floor union.'}

    def atlas(self):
        labels=['T1','T2','GATES','CAFE','TRANSFER','LOUNGE','KIOSK','FLIGHTS','SERVICE','RETAIL']
        text={
            'T1':('TERMINAL 1','제1터미널  ·  GATES 101–116'),
            'T2':('TERMINAL 2','제2터미널  ·  GATES 201–216'),
            'GATES':('DEPARTURES','탑승구  ·  BOARDING GATES'),
            'CAFE':('CAFE & DINING','카페  ·  REST & RECHARGE'),
            'TRANSFER':('T1  ↔  T2','환승 연결 통로  ·  TRANSFERS'),
            'LOUNGE':('GARDEN LOUNGE','휴게 라운지  ·  QUIET ZONE'),
            'KIOSK':('CHECK IN','SELF SERVICE'),
            'FLIGHTS':('DEPARTURES','10:30  SEOUL     A12\n11:15  TOKYO     B04\n12:40  PARIS      C09'),
            'SERVICE':('SERVICE ACCESS','시설 관리  ·  STAFF ACCESS'),
            'RETAIL':('DUTY FREE','TRAVEL ESSENTIALS'),
        }
        cellw,cellh,cols=768,256,3
        rows=math.ceil(len(labels)/cols)
        im=Image.new('RGB',(cellw*cols,cellh*rows),(8,20,28));d=ImageDraw.Draw(im)
        fontpath='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
        if not Path(fontpath).exists():fontpath='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
        font=ImageFont.truetype(fontpath,58)
        small=ImageFont.truetype(fontpath,27)
        tile={}
        for idx,label in enumerate(labels):
            x=(idx%cols)*cellw;y=(idx//cols)*cellh
            bg=(12,40,60)
            if label=='T2':bg=(12,66,59)
            elif label=='CAFE':bg=(68,43,28)
            d.rectangle((x+2,y+2,x+cellw-2,y+cellh-2),fill=bg)
            d.rectangle((x+24,y+22,x+cellw-24,y+26),fill=(205,176,113))
            title,sub=text[label]
            f=font if label!='TRANSFER' else ImageFont.truetype(fontpath,48)
            d.text((x+34,y+45),title,font=f,fill=(247,248,240))
            d.multiline_text((x+36,y+140),sub,font=small,spacing=5,fill=(210,224,223))
            tile[label]=(x/im.width,y/im.height,(x+cellw)/im.width,(y+cellh)/im.height)
        path=self.assets/'textures/signage_atlas.png';im.save(path,optimize=True)
        return tile

    def save_sign_mesh(self,tiles):
        vertices=[];uv=[];normals=[];indices=[]
        for sign in self.signs:
            x,y,z=sign['center'];w,h=sign['width'],sign['height'];yaw=sign['yaw']
            quad=[]
            for dx,dy in ((-w/2,-h/2),(w/2,-h/2),(w/2,h/2),(-w/2,h/2)):
                if sign['horizontal']:
                    rx,ry=rotate(dx,dy,yaw);quad.append((x+rx,y+ry,z))
                else:
                    rx,ry=rotate(dx,0,yaw);quad.append((x+rx,y+ry,z+dy))
            a,b,c,d=tiles[sign['label']]
            # DAE v points upward; atlas origin is at its top-left.
            tex=[(a,1-d),(c,1-d),(c,1-b),(a,1-b)]
            normal=(0.,0.,1.) if sign['horizontal'] else (math.sin(yaw),-math.cos(yaw),0.)
            start=len(vertices);vertices.extend(quad);uv.extend(tex);normals.extend([normal]*4)
            indices.extend((start,start+1,start+2,start,start+2,start+3))
        ns='http://www.collada.org/2005/11/COLLADASchema'
        ET.register_namespace('',ns)
        def el(parent,tag,attrs=None,text=None):
            e=ET.SubElement(parent,'{'+ns+'}'+tag,attrs or {})
            if text is not None:e.text=text
            return e
        root=ET.Element('{'+ns+'}COLLADA',{'version':'1.4.1'})
        asset=el(root,'asset');el(asset,'created',text='2026-11-12T00:00:00Z');el(asset,'modified',text='2026-11-12T00:00:00Z');el(asset,'unit',{'name':'meter','meter':'1'});el(asset,'up_axis',text='Z_UP')
        images=el(root,'library_images');image=el(images,'image',{'id':'atlas'});el(image,'init_from',text='../textures/signage_atlas.png')
        effects=el(root,'library_effects');effect=el(effects,'effect',{'id':'sign-effect'});profile=el(effect,'profile_COMMON')
        surface=el(el(profile,'newparam',{'sid':'atlas-surface'}),'surface',{'type':'2D'});el(surface,'init_from',text='atlas')
        sampler=el(el(profile,'newparam',{'sid':'atlas-sampler'}),'sampler2D');el(sampler,'source',text='atlas-surface')
        phong=el(el(profile,'technique',{'sid':'common'}),'phong')
        el(el(phong,'emission'),'color',text='.08 .08 .08 1')
        el(el(phong,'ambient'),'color',text='1 1 1 1')
        el(el(phong,'diffuse'),'texture',{'texture':'atlas-sampler','texcoord':'UVSET0'})
        el(el(phong,'specular'),'color',text='0 0 0 1')
        el(el(phong,'shininess'),'float',text='1')
        extra=el(effect,'extra');tech=el(extra,'technique',{'profile':'GOOGLEEARTH'});el(tech,'double_sided',text='1')
        materials=el(root,'library_materials');mat=el(materials,'material',{'id':'sign-material','name':'sign-material'});el(mat,'instance_effect',{'url':'#sign-effect'})
        geo=el(el(root,'library_geometries'),'geometry',{'id':'sign-geometry','name':'sign-geometry'});mesh=el(geo,'mesh')
        for name,values,stride,params in (('positions',vertices,3,['X','Y','Z']),('normals',normals,3,['X','Y','Z']),('uv',uv,2,['S','T'])):
            source=el(mesh,'source',{'id':'sign-'+name});arr=el(source,'float_array',{'id':'sign-'+name+'-array','count':str(len(values)*stride)},' '.join(f'{c:.6f}' for v in values for c in v))
            acc=el(el(source,'technique_common'),'accessor',{'source':'#'+arr.attrib['id'],'count':str(len(values)),'stride':str(stride)})
            for param in params:el(acc,'param',{'name':param,'type':'float'})
        verts=el(mesh,'vertices',{'id':'sign-vertices'});el(verts,'input',{'semantic':'POSITION','source':'#sign-positions'})
        tris=el(mesh,'triangles',{'count':str(len(indices)//3),'material':'sign-material'});el(tris,'input',{'semantic':'VERTEX','source':'#sign-vertices','offset':'0'});el(tris,'input',{'semantic':'NORMAL','source':'#sign-normals','offset':'1'});el(tris,'input',{'semantic':'TEXCOORD','source':'#sign-uv','offset':'2','set':'0'});el(tris,'p',text=' '.join(f'{i} {i} {i}' for i in indices))
        vs=el(el(root,'library_visual_scenes'),'visual_scene',{'id':'Scene'});node=el(vs,'node',{'id':'signs','name':'signs'});inst=el(node,'instance_geometry',{'url':'#sign-geometry'});bind=el(el(inst,'bind_material'),'technique_common');im=el(bind,'instance_material',{'symbol':'sign-material','target':'#sign-material'});el(im,'bind_vertex_input',{'semantic':'UVSET0','input_semantic':'TEXCOORD','input_set':'0'})
        el(el(root,'scene'),'instance_visual_scene',{'url':'#Scene'})
        ET.indent(root)
        ET.ElementTree(root).write(self.assets/'meshes/signage.dae',encoding='utf-8',xml_declaration=True)

    def save_world(self):
        root=ET.Element('sdf',{'version':'1.9'});world=ET.SubElement(root,'world',{'name':'airport'})
        def add(parent,tag,text=None,**attrs):
            e=ET.SubElement(parent,tag,attrs)
            if text is not None:e.text=str(text)
            return e
        physics=add(world,'physics',name='airport_physics',type='ignored')
        add(physics,'max_step_size','.004');add(physics,'real_time_factor','1')
        for filename,name in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster'),('sensors','Sensors'),('imu','Imu')]:
            p=add(world,'plugin',filename='gz-sim-'+filename+'-system',name='gz::sim::systems::'+name)
            if name=='Sensors':add(p,'render_engine','ogre2')
        add(world,'gravity','0 0 -9.81')
        scene=add(world,'scene');add(scene,'ambient','.72 .76 .78 1');add(scene,'background','.64 .75 .83 1');add(scene,'shadows','true')
        light=add(world,'light',name='daylight',type='directional');add(light,'pose','0 0 30 0 0 0');add(light,'diffuse','.90 .88 .82 1');add(light,'specular','.1 .1 .1 1');add(light,'direction','-.35 .15 -1');add(light,'cast_shadows','true')
        light=add(world,'light',name='sky_fill',type='directional');add(light,'pose','0 0 30 0 0 0');add(light,'diffuse','.28 .34 .4 1');add(light,'direction','.2 -.5 -1');add(light,'cast_shadows','false')
        gui=add(world,'gui',fullscreen='false')
        camera=add(gui,'camera',name='user_camera');add(camera,'pose','-42 -168 166 0 .72 1.57079632679');add(camera,'view_controller','orbit')
        model=add(world,'model',name='hanul_airport_architecture');add(model,'static','true');link=add(model,'link',name='structure')
        for c in self.collisions:
            col=add(link,'collision',name=c['name']);add(col,'pose',' '.join(str(v) for v in c['pose']));geo=add(col,'geometry')
            if c['type']=='box':add(add(geo,'box'),'size',' '.join(str(v) for v in c['size']))
            else:
                cylinder=add(geo,'cylinder');add(cylinder,'radius',c['radius']);add(cylinder,'length',c['length'])
        for name,mesh in self.meshes.items():
            if not mesh.vertices:continue
            rgba,roughness,metalness=PALETTE[name]
            mesh.save_dae(self.assets/'meshes'/f'{name}.dae',rgba,emissive=(name=='light'))
            (self.assets/'meshes'/f'{name}.obj').unlink(missing_ok=True)
            vis=add(link,'visual',name=name);geo=add(vis,'geometry');add(add(geo,'mesh'),'uri',f'model://omslam_airport_assets/meshes/{name}.dae')
            if name=='glass':add(vis,'transparency','.70');add(vis,'cast_shadows','false')
        vis=add(link,'visual',name='airport_signage');add(add(add(vis,'geometry'),'mesh'),'uri','model://omslam_airport_assets/meshes/signage.dae')
        ET.indent(root)
        ET.ElementTree(root).write(self.package/'worlds/airport_static.sdf',encoding='utf-8',xml_declaration=True)
        (self.assets/'model.config').write_text('<?xml version="1.0"?>\n<model><name>Hanul Airport Assets</name><version>1.0</version><sdf version="1.9">model.sdf</sdf><description>Procedural airport geometry, furniture and signage. Apache-2.0.</description></model>\n')
        (self.assets/'model.sdf').write_text('<?xml version="1.0"?>\n<sdf version="1.9"><model name="omslam_airport_assets"><static>true</static><link name="asset_library"/></model></sdf>\n')

    def save_maps(self):
        resolution=.1;xmin,ymin,xmax,ymax=self.layout['bounds']
        width=round((xmax-xmin)/resolution);height=round((ymax-ymin)/resolution)
        im=Image.new('L',(width,height),205);draw=ImageDraw.Draw(im)
        def xy(x,y):return ((x-xmin)/resolution,(ymax-y)/resolution)
        for floor in self.layout['floors']:
            a,b,c,d=floor['bounds'];draw.rectangle([xy(a,d),xy(c,b)],fill=254)
        for obstacle in self.obstacles:
            if obstacle['z_max']<=.08 or obstacle['z_min']>=2.:continue
            draw.polygon([xy(x,y) for x,y in obstacle['footprint']],fill=0)
        maps=self.package/'maps';im.save(maps/'airport_reference.pgm')
        (maps/'airport_reference.yaml').write_text('''# Geometry-derived reference; this file is not a SLAM result.
image: airport_reference.pgm
mode: trinary
resolution: 0.1
origin: [-122.0, -62.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
''')
        # Human-readable plan from the same floors and collision projections.
        scale=11.;margin=95;top=170;bottom=180
        plan=Image.new('RGB',(round((xmax-xmin)*scale)+2*margin,round((ymax-ymin)*scale)+top+bottom),(13,28,38));d=ImageDraw.Draw(plan)
        fontpath='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
        if not Path(fontpath).exists():fontpath='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
        fonts={size:ImageFont.truetype(fontpath,size) for size in (18,22,26,36,58)}
        def pt(x,y):return (margin+(x-xmin)*scale,top+(ymax-y)*scale)
        d.text((margin,38),'HANUL AIRPORT  /  TERMINALS 1 + 2',font=fonts[58],fill=(239,242,233))
        d.text((margin,112),'Two terminals · dense lounges · 32 pedestrians · 1.4–6.4 m clear passages',font=fonts[26],fill=(167,194,198))
        for floor in self.layout['floors']:
            a,b,c,e=floor['bounds'];d.rectangle([pt(a,e),pt(c,b)],fill=(216,223,217))
        colors=[(52,113,152),(47,137,121)]
        for terminal,color in zip(self.layout['terminals'],colors):
            a,b,c,e=terminal['bounds'];d.rectangle([pt(a,e),pt(c,b)],outline=color,width=6)
        for o in self.obstacles:
            if o['z_max']<=.08 or o['z_min']>=2.:continue
            d.polygon([pt(x,y) for x,y in o['footprint']],fill=(73,91,96))
        for route in self.layout['pedestrian_routes']:
            points=[pt(x,y) for x,y in route['waypoints']+[route['waypoints'][0]]]
            d.line(points,fill=(203,143,71),width=2)
        for terminal,color in zip(self.layout['terminals'],colors):
            a,b,c,e=terminal['bounds'];cx=(a+c)/2
            x,y=pt(cx,3.5)
            d.rounded_rectangle((x-130,y-27,x+130,y+68),radius=8,fill=color)
            d.text((x-110,y-20),terminal['id'],font=fonts[58],fill='white')
            d.text((x-20,y+5),'LOUNGE',font=fonts[26],fill='white')
            x,y=pt(cx-29,43);d.text((x,y),'CAFE / DINING',font=fonts[18],fill=(44,64,71))
            x,y=pt(cx+9,43);d.text((x,y),'GARDEN LOUNGE',font=fonts[18],fill=(44,64,71))
            x,y=pt(cx-28,-32);d.text((x,y),'GATES / BOARDING SEATS',font=fonts[18],fill=(44,64,71))
            x,y=pt(cx-26,-57);d.text((x,y),'SERVICE ROOMS · 1.4 m CHOKE',font=fonts[18],fill=(44,64,71))
        sx,sy=pt(self.layout['spawn']['x'],self.layout['spawn']['y'])
        d.ellipse((sx-9,sy-9,sx+9,sy+9),fill=(207,65,65),outline='white',width=3)
        d.text((sx-35,sy+14),'START',font=fonts[18],fill=(133,30,38))
        y=plan.height-135
        d.text((margin,y),'160 m × 124 m overall  |  amber: pedestrian loops  |  dark: obstacle projection',font=fonts[26],fill=(189,209,210))
        d.text((margin,y+50),'Fictional Incheon-inspired setting. Geometry reference only — not a surveyed floor plan or SLAM output.',font=fonts[22],fill=(146,172,180))
        # 20 m scale bar.
        x=plan.width-margin-240;yy=plan.height-55
        d.line((x,yy,x+20*scale,yy),fill=(239,242,233),width=5)
        d.text((x+50,yy-35),'20 metres',font=fonts[22],fill=(239,242,233))
        plan.save(maps/'airport_floorplan.png',optimize=True)

    def generate(self):
        for path in (self.package/'worlds',self.package/'maps',self.package/'config',self.assets/'meshes',self.assets/'textures'):path.mkdir(parents=True,exist_ok=True)
        self.build_structure()
        for i,cx in enumerate(TERMINAL_CENTERS,1):self.build_terminal(i,cx)
        self.densify()
        self.finish_layout()
        self.save_sign_mesh(self.atlas())
        self.save_world();self.save_maps()
        # Generated library ownership is local to this directory: remove
        # obsolete terminal material meshes after a terminal is deleted.
        expected=set(self.meshes)|{'signage'}
        for old in (self.assets/'meshes').glob('*.dae'):
            if old.stem not in expected:old.unlink()
        self.layout['statistics']['visual_meshes']=len(self.meshes)+1
        self.layout['statistics']['triangles']=sum(len(m.faces) for m in self.meshes.values())+2*len(self.signs)
        (self.package/'config/airport_layout.json').write_text(json.dumps(self.layout,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(self.layout['statistics'],ensure_ascii=False,indent=2))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package',type=Path,default=Path(__file__).resolve().parents[1]/'src/omslam_sim')
    args=parser.parse_args()
    Airport(args.package.resolve()).generate()


if __name__=='__main__':main()
