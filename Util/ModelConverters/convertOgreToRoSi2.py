#!/usr/bin/env python3.6

import sys
import xml.etree.ElementTree as ET

def parse_geometry_element(e, is_shared, with_tex_coords, with_normals):
    assert len(e) == 1
    assert len(e.attrib) == 1
    vertex_count = e.attrib['vertexcount']
    vertex_buffer = e[0]
    assert vertex_buffer.tag == 'vertexbuffer'
    assert len(vertex_buffer) == int(vertex_count)
    positions = vertex_buffer.attrib['positions']
    assert positions == 'true'
    print(f'  <Vertices name="{name}">' if is_shared else f'    <Vertices name="{name}">')
    for vertex in vertex_buffer:
        assert vertex.tag == 'vertex'
        had_position = False
        for vertex_attribute in vertex:
            if not vertex_attribute.tag == 'position':
                continue
            assert not had_position
            had_position = True
            position = vertex_attribute
            assert len(position) == 0
            assert len(position.attrib) == 3
            x = position.attrib['x']
            y = position.attrib['y']
            z = position.attrib['z']
            print(f'{x} {y} {z}')
        assert had_position
    print('  </Vertices>' if is_shared else '    </Vertices>')
    if is_shared:
        print('')
    if with_tex_coords:
        texture_coords = vertex_buffer.attrib['texture_coords']
        assert texture_coords == '1'
        texture_coord_dimensions_0 = vertex_buffer.attrib['texture_coord_dimensions_0']
        assert texture_coord_dimensions_0 == 'float2'
        print(f'  <TexCoords name="{name}">' if is_shared else f'    <TexCoords name="{name}">')
        for vertex in vertex_buffer:
            assert vertex.tag == 'vertex'
            had_texcoord = False
            for vertex_attribute in vertex:
                if not vertex_attribute.tag == 'texcoord':
                    continue
                assert not had_texcoord
                had_texcoord = True
                texcoord = vertex_attribute
                assert len(texcoord) == 0
                assert len(texcoord.attrib) == 2
                u = texcoord.attrib['u']
                v = 1 - float(texcoord.attrib['v'])
                print(f'{u} {v}')
            assert had_texcoord
        print('  </TexCoords>' if is_shared else '    </TexCoords>')
        if is_shared:
            print('')
    if with_normals:
        normals = vertex_buffer.attrib['normals']
        assert normals == 'true'
        print(f'  <Normals name="{name}">' if is_shared else f'    <Normals name="{name}">')
        for vertex in vertex_buffer:
            assert vertex.tag == 'vertex'
            had_normal = False
            for vertex_attribute in vertex:
                if not vertex_attribute.tag == 'normal':
                    continue
                assert not had_normal
                had_normal = True
                normal = vertex_attribute
                assert len(normal) == 0
                assert len(normal.attrib) == 3
                x = normal.attrib['x']
                y = normal.attrib['y']
                z = normal.attrib['z']
                print(f'{x} {y} {z}')
            assert had_normal
        print('  </Normals>' if is_shared else '    </Normals>')
        if is_shared:
            print('')

assert len(sys.argv) > 1
assert sys.argv[1][-4:] == '.xml'
name = sys.argv[1][:-4]
tree = ET.parse(sys.argv[1])
with_tex_coords = len(sys.argv) > 2 and '--texture' in sys.argv[2:]
with_normals = len(sys.argv) > 2 and '--normals' in sys.argv[2:]
root = tree.getroot()
assert root.tag == 'mesh'
assert len(root.attrib) == 0
i = 0
had_shared_geometry = False
for child in root:
    if child.tag == 'submeshes':
        assert len(child.attrib) == 0
        for submesh in child:
            assert submesh.tag == 'submesh'
            assert len(submesh.attrib) == 4
            material = submesh.attrib['material']
            usesharedvertices = submesh.attrib['usesharedvertices']
            use32bitindices = submesh.attrib['use32bitindexes']
            assert use32bitindices == 'false'
            operationtype = submesh.attrib['operationtype']
            assert operationtype == 'triangle_list'
            print(f'  <ComplexAppearance name="{name}_{i}">')
            print(f'    <Surface ref="{material}"/>')
            had_faces = False
            for faces_or_geometry in submesh:
                if faces_or_geometry.tag == 'faces':
                    assert not had_faces
                    had_faces = True
                    assert len(faces_or_geometry.attrib) == 1
                    face_count = faces_or_geometry.attrib['count']
                    assert len(faces_or_geometry) == int(face_count)
                    print('    <Triangles>')
                    for face in faces_or_geometry:
                        assert face.tag == 'face'
                        assert len(face.attrib) == 3
                        v1 = face.attrib['v1']
                        v2 = face.attrib['v2']
                        v3 = face.attrib['v3']
                        if with_normals:
                            print(f'{v1} {v1} {v2} {v2} {v3} {v3}')
                        else:
                            print(f'{v1} {v2} {v3}')
                    print('    </Triangles>')
                elif faces_or_geometry.tag == 'geometry':
                    assert usesharedvertices == 'false'
                    parse_geometry_element(faces_or_geometry, False, with_tex_coords, with_normals)
                else:
                    assert False
            assert had_faces
            if usesharedvertices == 'true':
                print(f'    <Vertices ref="{name}"/>')
                if with_tex_coords:
                    print(f'    <TexCoords ref="{name}"/>')
                if with_normals:
                    print(f'    <Normals ref="{name}"/>')
            else:
                assert usesharedvertices == 'false'
            print('  </ComplexAppearance>')
            print('')
            i += 1
    elif child.tag == 'sharedgeometry':
        assert not had_shared_geometry
        had_shared_geometry = True
        parse_geometry_element(child, True, with_tex_coords, with_normals)
    elif child.tag == 'submeshnames':
        pass
    else:
        assert False

if i > 1:
    print(f'  <Appearance name="{name}">')
    for j in range(i):
        print(f'    <ComplexAppearance ref="{name}_{j}"/>')
    print('  </Appearance>')
    print('')
