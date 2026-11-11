"""Optional schematic Blender scene, Python only.

Run from repository root with Blender 4.x:
blender --background --python scripts/blender_scene.py -- results/reference
Output: results/reference/twin_scene.blend. No external assets required.
The Blender runtime is optional and is not used by the simulator or its tests.
"""
import csv
import json
import math
import sys
from pathlib import Path
import bpy

root=Path(__file__).resolve().parents[1]
directory=root/(sys.argv[sys.argv.index('--')+1] if '--' in sys.argv else 'results/reference')
with (directory/'trajectory.csv').open() as f:rows=list(csv.DictReader(f))
report=json.loads((directory/'results.json').read_text())
# Build in a new scene, leaving existing user scenes intact.
scene=bpy.data.scenes.new('VT440 educational twin');bpy.context.window.scene=scene
scene.render.fps=24;scene.unit_settings.system='METRIC'


def material(name,color):
    mat=bpy.data.materials.new(name);mat.diffuse_color=(*color,1);return mat


def cube(name,location,scale,mat,parent=None):
    bpy.ops.mesh.primitive_cube_add(size=1,location=location)
    obj=bpy.context.object;obj.name=name;obj.scale=scale;obj.data.materials.append(mat)
    if parent:obj.parent=parent
    return obj


def terrain(x,y):return 120+65*math.exp(-((x-10500)/2200)**2-((y+300)/2500)**2)+14*math.sin(x/1600)*math.cos(y/2100)

land=material('Taiga',(0.17,.28,.20));body=material('Airframe',(.12,.24,.32));orange=material('Rotor',(.95,.40,.10));red=material('Obstacle',(.65,.15,.13))
vertices=[];faces=[];nx=100;ny=40
for j in range(ny):
    for i in range(nx):
        x=i*20000/(nx-1);y=-1700+j*5100/(ny-1);vertices.append((x,y,terrain(x,y)))
for j in range(ny-1):
    for i in range(nx-1):
        a=j*nx+i;faces.append((a,a+1,a+nx+1,a+nx))
mesh=bpy.data.meshes.new('Terrain');mesh.from_pydata(vertices,[],faces);mesh.materials.append(land)
obj=bpy.data.objects.new('Synthetic terrain',mesh);scene.collection.objects.link(obj)
bpy.ops.object.empty_add();vehicle=bpy.context.object;vehicle.name='VT440 schematic metres'
cube('Fuselage',(0,0,0),(3.6,1.5,1.6),body,vehicle)
cube('Tail boom',(-2.7,0,.2),(3.,.18,.18),body,vehicle)
cube('Rotor blade A',(0,0,1.2),(7.,.13,.06),orange,vehicle)
cube('Rotor blade B',(0,0,1.2),(.13,7.,.06),orange,vehicle)
for y in (-.65,.65):cube('Skid',(0,y,-1.),(3.,.08,.08),body,vehicle)
for row in rows[::30]+rows[-1:]:
    frame=1+round(float(row['t_s'])/30*24)
    vehicle.location=tuple(float(row[k]) for k in ('x_m','y_m','z_amsl_m'))
    vehicle.rotation_euler=tuple(float(row[k]) for k in ('roll_rad','pitch_rad','yaw_rad'))
    vehicle.keyframe_insert('location',frame=frame);vehicle.keyframe_insert('rotation_euler',frame=frame)
scene.frame_end=frame
if vehicle.animation_data and vehicle.animation_data.action:
    # Blender supports interpolated keyframes; visual playback is 30x real time.
    pass
for ob in report['route']['obstacles']:
    x,y=ob['x_m'],ob['y_m'];z=terrain(x,y)
    if ob['type']=='bridge':
        cube('Bridge deck',(x,y,z+87),(16,130,4),body)
        for sign in (-1,1):cube('Bridge pier',(x,y+sign*55,z+42.5),(12,10,85),body)
    else:
        h=250 if ob['type']=='no_fly' else ob['height_m']
        bpy.ops.mesh.primitive_cylinder_add(vertices=32,radius=ob['radius_m'],depth=h,location=(x,y,z+h/2))
        obj=bpy.context.object;obj.name=f"Obstacle {ob['id']} {ob['type']}";obj.data.materials.append(red)
        if ob['type']=='no_fly':obj.display_type='WIRE';obj.hide_render=True
curve=bpy.data.curves.new('Route','CURVE');curve.dimensions='3D';curve.bevel_depth=3
poly=curve.splines.new('POLY');selected=rows[::20]+rows[-1:];poly.points.add(len(selected)-1)
for p,row in zip(poly.points,selected):p.co=tuple(float(row[k]) for k in ('x_m','y_m','z_amsl_m'))+(1.,)
obj=bpy.data.objects.new('Trajectory',curve);scene.collection.objects.link(obj);curve.materials.append(orange)
bpy.ops.object.light_add(type='SUN',location=(0,0,2000));bpy.context.object.data.energy=3
scene.world=bpy.data.worlds.new('Daylight');scene.world.color=(.35,.40,.50)
scene.frame_set(1);bpy.ops.wm.save_as_mainfile(filepath=str(directory/'twin_scene.blend'))
print('Saved schematic scene:',directory/'twin_scene.blend')
