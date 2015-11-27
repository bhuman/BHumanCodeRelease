#!BPY
"""
Name: 'ODE Convex...'
Blender: 244
Group: 'Export'
Tooltip: 'Export to Open Dynamics.'
"""
__author__ = "Rodrigo Hernandez"
__url__ = ("http://www.ode.org")
__version__ = "0.1"
__bpydoc__ = """\
ODE Convex Exporter
This script Exports a Blender scene as a series of ODE Convex Geom data stored in a C header file.
"""
import Blender
import bpy
#import struct

def WriteMesh(file,ob):
	mesh = ob.getData(mesh=1)
	# Write Point Count
	file.write("unsigned int %s_pointcount = %d;\n" % (ob.getName(),len(mesh.verts)))
	# Write Plane Count
	file.write("unsigned int %s_planecount = %d;\n" % (ob.getName(),len(mesh.faces)))
	# Write Points
	file.write("dReal %s_points[%d]={\n" % (ob.getName(),(len(mesh.verts)*3)))
	for vert in mesh.verts:
		if vert.index==(len(mesh.verts)-1):
			file.write("%f,%f,%f\n};\n" % (vert.co[0],vert.co[1],vert.co[2]))
		else:
			file.write("%f,%f,%f,\n" % (vert.co[0],vert.co[1],vert.co[2]))
	# Write Polygons
	file.write("unsigned int %s_polygons[]={\n" % ob.getName())
	for face in mesh.faces:
		file.write("%d," % len(face.verts))
		for vert in face.verts:
			file.write("%d," % vert.index)
		if face.index==(len(mesh.faces)-1):
			file.write("\n};\n");
		else:
			file.write("\n");
	# Write Planes
	file.write("dReal %s_planes[]={\n" % ob.getName())
	for face in mesh.faces:
		# d calculated separatelly for code readability
		d = face.no[0]*face.verts[0].co[0]+face.no[1]*face.verts[0].co[1]+face.no[2]*face.verts[0].co[2]
		file.write("%f,%f,%f,%f,\n" % (face.no[0],face.no[1],face.no[2],d))
	file.write("};\n");
	
# Entry Point
sce = bpy.data.scenes.active
in_editmode = Blender.Window.EditMode()
if in_editmode: Blender.Window.EditMode(0)
file = open("convex.h",mode='wt')
for ob in sce.objects:
	if ob.getType()=='Mesh':
		WriteMesh(file,ob)
file.close()
if in_editmode:
	Blender.Window.EditMode(1)
print "ODE Export Done"