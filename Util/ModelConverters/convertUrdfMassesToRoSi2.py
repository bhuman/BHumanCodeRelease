#!/usr/bin/env python3.6

import numpy as np
import sys
import xml.etree.ElementTree as ET

assert len(sys.argv) == 2
assert sys.argv[1][-5:] == '.urdf'
tree = ET.parse(sys.argv[1])
root = tree.getroot()
assert root.tag == 'robot'
for element in root:
    if element.tag != 'link' or len(element.attrib) != 1 or element.attrib['name'][-5:] != '_link' or len(element) == 0 or element[0].tag != 'inertial':
        continue
    inertial = element[0]
    if len(inertial) != 3:
        continue
    mass = inertial[0]
    assert mass.tag == 'mass'
    origin = inertial[1]
    assert origin.tag == 'origin'
    inertia = inertial[2]
    assert inertia.tag == 'inertia'
    m = float(mass.attrib['value']) * 1000
    I_S = np.matrix([[float(inertia.attrib['ixx']), float(inertia.attrib['ixy']), float(inertia.attrib['ixz'])],
                     [float(inertia.attrib['ixy']), float(inertia.attrib['iyy']), float(inertia.attrib['iyz'])],
                     [float(inertia.attrib['ixz']), float(inertia.attrib['iyz']), float(inertia.attrib['izz'])]]) * 1000000000
    a1, a2, a3 = [float(_) * 1000 for _ in origin.attrib['xyz'].split(' ')]
    r, p, y = [float(_) for _ in origin.attrib['rpy'].split(' ')]
    around_x = np.matrix([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    around_y = np.matrix([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    around_z = np.matrix([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    rotation = (around_z * around_y * around_x).transpose()
    a1, a2, a3 = rotation * np.matrix([a1, a2, a3]).transpose()
    a = np.matrix([[0, -a3, a2], [a3, 0, -a1], [-a2, a1, 0]])
    I = I_S + m * a * a.transpose()
    print(element.attrib['name'])
    print(f'<InertiaMatrixMass value="{m:.2f}g" x="{a1[0, 0]:.2f}mm" y="{a2[0, 0]:.2f}mm" z="{a3[0, 0]:.2f}mm"')
    print(f'  ixx="{I[0, 0]:.2f}g*mm^2" ixy="{I[0, 1]:.2f}g*mm^2" ixz="{I[0, 2]:.2f}g*mm^2"')
    print(f'  iyy="{I[1, 1]:.2f}g*mm^2" iyz="{I[1, 2]:.2f}g*mm^2" izz="{I[2, 2]:.2f}g*mm^2"/>')
    print('')
