# coding=utf-8

import bpy, os
from mathutils import *
from math import *

LongCells = 16
LatCells = 8
LightDistance = 10.0

#-------------#
bpy.ops.object.add(type='EMPTY')
target = bpy.context.selected_objects[0]
target.name = "IbrTarget"

bpy.ops.object.lamp_add(type='SUN')
ibrLight = bpy.context.selected_objects[0]
ibrLight.name = "ibrLight"
ibrLight.scale = (0.03, 0.03, 0.03)
ibrLight.location = (0.0, 0.0, 1.0)

bpy.context.scene.objects.active = target
bpy.ops.object.track_set(type='TRACKTO')

scene = bpy.data.scenes[0]

positions = []
longStep = 2*pi / float(LongCells)
latStep = pi / float(LatCells)
for v in range(LatCells):
    for u in range(LongCells):
        longitude = (u + 0.5) * longStep
        latitude = pi - (v + 0.5) * latStep
        p = Vector((cos(longitude) * abs(sin(latitude)), sin(longitude) * abs(sin(latitude)), cos(latitude)))
        positions.append(p)

for i in range(len(positions)):
    scene.frame_set(i)
    ibrLight.location = positions[i] * LightDistance
    bpy.ops.object.select_pattern(pattern="ibrLight")
    bpy.ops.anim.keyframe_insert_menu(type="Location")
