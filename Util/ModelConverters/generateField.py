#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from string import Template

# number of segments for one half of the center circle
CENTER_CIRCLE_SEGMENTS = 32

# number of segments for a field mark
FIELD_MARK_SEGMENTS = 32

# number of segments for a corner arc (i.e. a quarter circle)
CORNER_ARC_SEGMENTS = 16

# edge length of a single mesh of the net
NET_MESH_SIZE = 0.05

# thickness of the net support structure
NET_SUPPORT_THICKNESS = 0.02

# vertical offset of the visual carpet to prevent z-fighting with the field lines
CARPET_Z = -0.002

# thickness of the physical floor
FLOOR_THICKNESS = 0.2

# thickness and height of the physical barrier around the field
FENCE_THICKNESS = 0.1
FENCE_HEIGHT = 0.3

# thickness and height of the physical barrier around the goal
GOAL_FENCE_THICKNESS = 0.01
GOAL_FENCE_HEIGHT = 0.2

# size of the grass texture in pixels (used to keep its aspect ratio)
GRASS_TEXTURE_WIDTH = 788
GRASS_TEXTURE_HEIGHT = 900


@dataclass
class Dimensions:
    """The measurements of a field in meters (relative to line centers) and what is derived from them."""

    field_length: float  # length of the touchlines
    field_width: float  # length of the goal lines
    line_width: float  # width/thickness of the field lines
    penalty_mark_size: float  # edge length of an AABB around the penalty mark (same as for the center mark)
    goal_area_length: float  # length of the goal area's two parallel lines
    goal_area_width: float  # length of the goal area's connecting line parallel to the goal line
    penalty_area_length: float  # length of the penalty area's two parallel lines
    penalty_area_width: float  # length of the penalty area's connecting line parallel to the goal line
    penalty_mark_distance: float  # distance between goal line and center of the penalty mark
    center_circle_diameter: float  # diameter of the center circle
    border_strip_width: float  # (minimum) distance from a touchline / goal line to the edge of the carpet
    corner_arc_radius: float | None  # radius of the corner arcs, if there are any
    goal_depth: float  # depth of the goal, measured from the front edge of the goalposts to the net at ground level
    goal_inner_width: float  # distance between the goalposts, measured from inner edge to inner edge
    goalpost_diameter: float  # diameter of the goalposts
    goal_height: float  # height of the goal, measured from the ground to the lower edge of the crossbar

    @classmethod
    def from_parameters(cls, parameters):
        field, goal = parameters["field"], parameters["goal"]
        line_width = field["lineWidth"]
        d = dict(
            field_length=field["length"],
            field_width=field["width"],
            line_width=line_width,
            penalty_mark_size=field["penaltyMarkSize"],
            goal_area_length=field["goalAreaLength"],
            goal_area_width=field["goalAreaWidth"],
            penalty_area_length=field["penaltyAreaLength"],
            penalty_area_width=field["penaltyAreaWidth"],
            penalty_mark_distance=field["penaltyMarkDistance"],
            center_circle_diameter=field["centerCircleDiameter"],
            border_strip_width=field["borderStripWidth"],
            corner_arc_radius=field.get("cornerArcRadius"),
            goal_depth=goal["depth"],
            goal_inner_width=goal["innerWidth"],
            goalpost_diameter=goal["postDiameter"],
            goal_height=goal["height"],
        )

        # convert dimensions relative to line centers, if necessary
        if field["edge_to_edge"]:
            for key in ("field_length", "field_width", "goal_area_length", "goal_area_width",
                        "penalty_area_length", "penalty_area_width", "center_circle_diameter"):
                d[key] -= line_width
            # (Note: even with edge to edge measurements, the penalty mark distance is from the outer edge of the
            # goal line to the *center* of the penalty mark.)
            d["penalty_mark_distance"] -= 0.5 * line_width
            d["border_strip_width"] += 0.5 * line_width
            if d["corner_arc_radius"] is not None:
                d["corner_arc_radius"] -= 0.5 * line_width
        return cls(**d)

    def __post_init__(self):
        # Derived quantities (the order of the operands is chosen such that the results are identical to earlier versions)
        self.lw_2 = 0.5 * self.line_width  # half a line width
        self.goalpost_radius = 0.5 * self.goalpost_diameter
        self.center_circle_radius = 0.5 * self.center_circle_diameter
        self.penalty_mark_radius = 0.5 * self.penalty_mark_size

        # size of the physical floor
        self.floor_length = self.field_length + 2 * self.border_strip_width + 1
        self.floor_width = self.field_width + 2 * self.border_strip_width + 1

        # absolute value of the x-coordinate of the goal line and of the y-coordinate of the touchlines
        self.goal_line_x = 0.5 * self.field_length
        self.touchline_y = 0.5 * self.field_width

        # absolute value of the x-coordinate of the goal (aligned with the center of the goalposts)
        self.goal_x = 0.5 * self.field_length - 0.5 * self.line_width + self.goalpost_radius
        # absolute value of the x-offset of the "back" part of the net, relative to the center of the goal posts
        self.net_offset_x = self.goal_depth - self.goalpost_radius
        # distance between the centers of the goalposts
        self.goal_width = self.goal_inner_width + 2 * self.goalpost_radius
        # absolute value of the y-coordinate of the goalpost centers
        self.goalpost_y = 0.5 * self.goal_width
        # height of the goalposts, same as z-coordinate of the crossbar (center)
        self.goalpost_height = self.goal_height + self.goalpost_radius
        # length of the crossbar (including the capsule's half-spheres)
        self.crossbar_length = self.goal_width + 2 * self.goalpost_radius

        # maximum UV coordinates for the net texture
        self.net_texture_depth = (self.goal_depth - self.goalpost_diameter) / NET_MESH_SIZE
        self.net_texture_height = self.goal_height / NET_MESH_SIZE
        self.net_texture_width = self.goal_width / NET_MESH_SIZE

        # absolute value of the x-coordinate of the "long" and the y-coordinate of the "short" goal area lines
        self.goal_area_x = self.goal_line_x - self.goal_area_length
        self.goal_area_y = 0.5 * self.goal_area_width

        # absolute value of the x-coordinate of the "long" and the y-coordinate of the "short" penalty area lines
        self.penalty_area_x = self.goal_line_x - self.penalty_area_length
        self.penalty_area_y = 0.5 * self.penalty_area_width

        # absolute value of the x-coordinate of the penalty mark center
        self.penalty_mark_x = 0.5 * self.field_length - self.penalty_mark_distance

        # maximum UV coordinates for the grass texture
        self.texture_x = self.goal_line_x + self.border_strip_width
        self.texture_y = (self.touchline_y + self.border_strip_width) * GRASS_TEXTURE_WIDTH / GRASS_TEXTURE_HEIGHT


def rows(*lines):
    """Joins lines to a block of text that ends with a line break."""
    return "".join(f"{line}\n" for line in lines)


def vertex_ring(r_inner, r_outer, angles):
    """Vertices of a strip along an arc: for each pair of angles, one inner and one outer vertex (all in the xy plane)."""
    vertices = ""
    for angle_inner, angle_outer in angles:
        vertices += rows(f"{math.cos(angle_inner) * r_inner} {math.sin(angle_inner) * r_inner} 0",
                         f"{math.cos(angle_outer) * r_outer} {math.sin(angle_outer) * r_outer} 0")
    return vertices


def strip_quads(count):
    """The indices of the quads of a strip of vertices (as generated by vertex_ring)."""
    return rows(*(f"{i * 2} {i * 2 + 1} {i * 2 + 3} {i * 2 + 2}" for i in range(count)))


def translation(**axes):
    return "<Translation " + " ".join(f'{axis}="{value}"' for axis, value in axes.items()) + "/>"


def box_geometry(width, height, depth, *, color=None, **position):
    color = f' color="{color}"' if color else ""
    return f"""    <BoxGeometry width="{width}m" height="{height}m" depth="{depth}m"{color}>
      {translation(**position)}
    </BoxGeometry>
"""


def net_support(name, depth, width, height, **position):
    return f"""    <BoxAppearance depth="{depth}m" width="{width}m" height="{height}m" name="{name}">
      <Surface ref="goalWhite"/>
      {translation(**position)}
    </BoxAppearance>
"""


def mm(value):
    return f"{1000 * value:.0f}"


def assignments(*items):
    """Renders configuration lines (name, value, comment) with aligned comments (without a final line break)."""
    statements = [f"{name} = {value};" for name, value, _ in items]
    width = max(map(len, statements)) + 1
    return "\n".join(statement.ljust(width) + f"// {comment}" for statement, (_, _, comment) in zip(statements, items))


def field_line_vertices(d):
    """The rectangles of the field lines in one half of the field (the other half is the same, but rotated)."""
    lw, gx, ty = d.lw_2, d.goal_line_x, d.touchline_y

    def area(name, x, y):
        return [
            (f"{name} line left", [(x + lw, y + lw), (x + lw, y - lw), (gx - lw, y - lw), (gx - lw, y + lw)]),
            (f"{name} line right", [(gx - lw, -(y + lw)), (gx - lw, -(y - lw)), (x + lw, -(y - lw)), (x + lw, -(y + lw))]),
            (f"{name} line front", [(x - lw, y + lw), (x - lw, -(y + lw)), (x + lw, -(y + lw)), (x + lw, y + lw)]),
        ]

    return [
        ("halfway line", [(-lw, ty - lw), (-lw, 0), (lw, 0), (lw, ty - lw)]),
        ("goal line", [(gx - lw, ty + lw), (gx - lw, -(ty + lw)), (gx + lw, -(ty + lw)), (gx + lw, ty + lw)]),
        ("touchline left", [(-(gx - lw), ty + lw), (-(gx - lw), ty - lw), (gx - lw, ty - lw), (gx - lw, ty + lw)]),
        *area("penalty area", d.penalty_area_x, d.penalty_area_y),
        *area("goal area", d.goal_area_x, d.goal_area_y),
    ]


def generate_scene(d):
    lw, gx, ty = d.lw_2, d.goal_line_x, d.touchline_y

    center_circle_angles = [(angle, angle) for angle in (i / CENTER_CIRCLE_SEGMENTS * math.pi for i in range(CENTER_CIRCLE_SEGMENTS + 1))]
    center_circle_vertices = vertex_ring(d.center_circle_radius - lw, d.center_circle_radius + lw, center_circle_angles)

    field_mark_vertices = "0 0 0\n"
    for i in range(FIELD_MARK_SEGMENTS):
        angle = i / FIELD_MARK_SEGMENTS * 2 * math.pi
        field_mark_vertices += f"{math.cos(angle) * d.penalty_mark_radius} {math.sin(angle) * d.penalty_mark_radius} 0\n"
    field_mark_triangles = rows(*(f"0 {i + 1} {(i + 1) % FIELD_MARK_SEGMENTS + 1}" for i in range(FIELD_MARK_SEGMENTS)))

    corner_arc_instantiation = corner_arc_appearance = ""
    if d.corner_arc_radius is not None:
        r_inner, r_outer = d.corner_arc_radius - lw, d.corner_arc_radius + lw
        start_inner, start_outer = math.asin(d.line_width / r_inner), math.asin(d.line_width / r_outer)
        end_inner, end_outer = math.pi / 2 - start_inner, math.pi / 2 - start_outer
        angles = [(start_inner, start_outer)]
        angles += [(angle, angle) for angle in (start_inner + i / CORNER_ARC_SEGMENTS * (end_inner - start_inner)
                                                 for i in range(1, CORNER_ARC_SEGMENTS))]
        angles += [(end_inner, end_outer)]

        corner_arc_instantiation = f"""
    <ComplexAppearance ref="fieldCornerArc">
      <Translation x="{gx + lw}" y="{ty + lw}" z="0"/>
      <Rotation z="180degree"/>
    </ComplexAppearance>
    <ComplexAppearance ref="fieldCornerArc">
      <Translation x="{gx + lw}" y="{-ty - lw}" z="0"/>
      <Rotation z="90degree"/>
    </ComplexAppearance>"""

        corner_arc_appearance = f"""
  <ComplexAppearance name="fieldCornerArc">
    <Surface ref="fieldWhite"/>
    <Vertices>
{vertex_ring(r_inner, r_outer, angles)}    </Vertices>
    <Quads>
{strip_quads(CORNER_ARC_SEGMENTS)}    </Quads>
  </ComplexAppearance>
"""

    # physical floor and fence around the field
    floor = box_geometry(d.floor_width, FLOOR_THICKNESS, d.floor_length, color="rgb(80, 80, 80)", z=f"{-0.5 * FLOOR_THICKNESS}m")
    for side in (-1, 1):
        floor += box_geometry(FENCE_THICKNESS, FENCE_HEIGHT, d.floor_length, color="rgb(80, 80, 80)",
                              y=f"{side * 0.5 * (d.floor_width + FENCE_THICKNESS)}m", z=f"{0.5 * FENCE_HEIGHT}m")
    for side in (-1, 1):
        floor += box_geometry(d.floor_width, FENCE_HEIGHT, FENCE_THICKNESS, color="rgb(80, 80, 80)",
                              x=f"{side * 0.5 * (d.floor_length + FENCE_THICKNESS)}m", z=f"{0.5 * FENCE_HEIGHT}m")

    # goalposts and net supports (post1/netSupport1 are on the side of positive y)
    goal = ""
    for name, y in (("post1", -d.goalpost_y), ("post2", d.goalpost_y)):
        goal += f"""    <CylinderGeometry height="{d.goal_height}m" radius="{d.goalpost_radius}m" name="{name}">
      <Translation y="{y}" z="{0.5 * d.goal_height}m"/>
    </CylinderGeometry>
"""
    goal += "\n"
    for name, y in (("post1", -d.goalpost_y), ("post2", d.goalpost_y)):
        goal += f"""    <CylinderAppearance height="{d.goalpost_height}m" radius="{d.goalpost_radius}m" name="{name}">
      <Translation y="{y}" z="{0.5 * d.goalpost_height}m"/>
      <Surface ref="$color"/>
    </CylinderAppearance>
"""
    goal += f"""    <CapsuleAppearance height="{d.crossbar_length}m" radius="{d.goalpost_radius}m" name="crossbar">
      <Rotation x="90degree"/>
      <Translation z="{d.goalpost_height}m"/>
      <Surface ref="$color"/>
    </CapsuleAppearance>

"""
    t = NET_SUPPORT_THICKNESS
    back_x = f"{-(d.net_offset_x - 0.5 * t)}m"
    for number, y in (("1", d.goalpost_y), ("2", -d.goalpost_y)):
        goal += net_support(f"netSupport{number}", d.goal_depth - d.goalpost_diameter, t, t,
                            x=f"{-0.5 * d.goal_depth}m", y=f"{y}m", z=f"{d.goal_height - 0.5 * t}m")
        goal += net_support(f"netSupport{number}b", d.goal_depth - d.goalpost_diameter, t, t,
                            x=f"{-0.5 * d.goal_depth}m", y=f"{y}m", z=f"{0.5 * t}m")
        goal += net_support(f"netSupport{number}c", t, t, d.goal_height - 2 * t,
                            x=back_x, y=f"{y}m", z=f"{0.5 * d.goal_height}m")
    goal += net_support("netSupportQ", t, d.goal_width - t, t, x=back_x, y="0m", z=f"{d.goal_height - 0.5 * t}m")
    goal += net_support("netSupportQb", t, d.goal_width - t, t, x=back_x, y="0m", z=f"{0.5 * t}m")
    goal += '    <ComplexAppearance ref="fieldNet"/>\n\n'
    goal += box_geometry(2 * d.goalpost_y, GOAL_FENCE_HEIGHT, GOAL_FENCE_THICKNESS,
                         x=f"{-d.net_offset_x}m", z=f"{0.5 * GOAL_FENCE_HEIGHT}m")
    for y in (-d.goalpost_y, d.goalpost_y):
        goal += box_geometry(GOAL_FENCE_THICKNESS, GOAL_FENCE_HEIGHT, d.goal_depth - d.goalpost_diameter,
                             x=f"{-0.5 * d.goal_depth}m", y=f"{y}m", z=f"{0.5 * GOAL_FENCE_HEIGHT}m")

    lines = field_line_vertices(d)
    line_vertices = "\n\n".join(rows(f"# {name}", *(f"{x} {y} 0" for x, y in points)).rstrip("\n") for name, points in lines)
    line_quads = rows(*(f"{4 * i} {4 * i + 1} {4 * i + 2} {4 * i + 3}" for i in range(len(lines))))

    green = d.goal_line_x + d.border_strip_width, ty + d.border_strip_width

    return f"""<Simulation>

  <Compound name="field">

{floor}
    <ComplexAppearance ref="fieldLines" name="fieldLines1"/>
    <ComplexAppearance ref="fieldLines" name="fieldLines2">
      <Rotation z="180degree"/>
    </ComplexAppearance>
    <ComplexAppearance ref="fieldMark"/>
    <ComplexAppearance ref="fieldGreen"/>

    <Compound ref="fieldGoal" name="Goal1">
      <Translation x="{-d.goal_x}m"/>
      <Set name="color" value="goalWhite"/>
    </Compound>
    <Compound ref="fieldGoal" name="Goal2">
      <Translation x="{d.goal_x}m"/>
      <Rotation z="180degree"/>
      <Set name="color" value="goalWhite"/>
    </Compound>

  </Compound>

  <Compound name="fieldGoal">

{goal}
  </Compound>

  <Surface name="fieldGreen" albedo="rgb(39, 129, 39)" roughness="1.0" texture="../Textures/grass.jpg"/>
  <Surface name="fieldNet" albedo="rgb(100%, 100%, 100%)" texture="../Textures/net.png"/>
  <Surface name="fieldWhite" albedo="rgb(100%, 100%, 100%)" roughness="0.4" texture="../Textures/grass.jpg"/>
  <Surface name="goalWhite" albedo="rgb(70%, 70%, 70%)" roughness="0.2"/>

  <Vertices name="fieldLines">
{line_vertices}
  </Vertices>

  <Vertices name="fieldMark">
{field_mark_vertices}  </Vertices>

  <Vertices name="fieldCenterCircle">
{center_circle_vertices}  </Vertices>

  <Vertices name="fieldGreen">
{-green[0]} {-green[1]} {CARPET_Z}
{green[0]} {-green[1]} {CARPET_Z}
{green[0]} {green[1]} {CARPET_Z}
{-green[0]} {green[1]} {CARPET_Z}
  </Vertices>
  <TexCoords name="fieldGreen">
0 0
{d.texture_x} 0
{d.texture_x} {d.texture_y}
0 {d.texture_y}
  </TexCoords>

  <Vertices name="fieldNet">
{-d.goalpost_radius} {-d.goalpost_y} 0
{-d.goalpost_radius} {-d.goalpost_y} {d.goal_height}
{-d.net_offset_x} {-d.goalpost_y} {d.goal_height}
{-d.net_offset_x} {-d.goalpost_y} 0

{-d.goalpost_radius} {d.goalpost_y} 0
{-d.goalpost_radius} {d.goalpost_y} {d.goal_height}
{-d.net_offset_x} {d.goalpost_y} {d.goal_height}
{-d.net_offset_x} {d.goalpost_y} 0

{-d.net_offset_x} {-d.goalpost_y} 0
{-d.net_offset_x} {d.goalpost_y} 0
{-d.net_offset_x} {d.goalpost_y} {d.goal_height}
{-d.net_offset_x} {-d.goalpost_y} {d.goal_height}
  </Vertices>
  <Normals name="fieldNet">
0 -1 0
0 1 0
1 0 0
-1 0 0
  </Normals>
  <TexCoords name="fieldNet">
{d.net_texture_depth} {d.net_texture_height}
{d.net_texture_depth} 0
0 0
0 {d.net_texture_height}

{d.net_texture_depth} {d.net_texture_height}
{d.net_texture_depth} 0
0 0
0 {d.net_texture_height}

0 0
{d.net_texture_width} 0
{d.net_texture_width} {d.net_texture_height}
0 {d.net_texture_height}
  </TexCoords>

  <ComplexAppearance name="fieldLines">
    <Surface ref="fieldWhite"/>
    <Vertices ref="fieldLines"/>
    <Quads>
{line_quads}    </Quads>
    <ComplexAppearance ref="fieldMark">
      <Translation x="{d.penalty_mark_x}" y="0" z="0"/>
    </ComplexAppearance>{corner_arc_instantiation}
    <ComplexAppearance ref="fieldCenterCircle"/>
  </ComplexAppearance>

  <ComplexAppearance name="fieldMark">
    <Surface ref="fieldWhite"/>
    <Vertices ref="fieldMark"/>
    <Triangles>
{field_mark_triangles}    </Triangles>
  </ComplexAppearance>

  <ComplexAppearance name="fieldCenterCircle">
    <Surface ref="fieldWhite"/>
    <Vertices ref="fieldCenterCircle"/>
    <Quads>
{strip_quads(CENTER_CIRCLE_SEGMENTS)}    </Quads>
  </ComplexAppearance>
{corner_arc_appearance}
  <ComplexAppearance name="fieldGreen">
    <Surface ref="fieldGreen"/>
    <Vertices ref="fieldGreen"/>
    <TexCoords ref="fieldGreen"/>
    <Quads>
0 1 2 3
    </Quads>
  </ComplexAppearance>

  <ComplexAppearance name="fieldNet">
    <Surface ref="fieldNet"/>
    <Vertices ref="fieldNet"/>
    <Normals ref="fieldNet"/>
    <TexCoords ref="fieldNet"/>
    <Quads>
0 0 1 0 2 0 3 0
3 1 2 1 1 1 0 1
4 0 5 0 6 0 7 0
7 1 6 1 5 1 4 1
8 2 9 2 10 2 11 2
11 3 10 3 9 3 8 3
    </Quads>
  </ComplexAppearance>

</Simulation>
"""


FIELD_DIMENSIONS_TEMPLATE = Template("""// This is the field configuration for ...

$x_positions
xPosPenaltyStrikerStartPosition = xPosOpponentPenaltyArea;
xPosHalfwayLine = 0;
xPosOwnPenaltyArea = -xPosOpponentPenaltyArea;
xPosOwnPenaltyMark = -xPosOpponentPenaltyMark;
xPosOwnGoalArea = -xPosOpponentGoalArea;
xPosOwnGoalLine = -xPosOpponentGoalLine;
xPosOwnGoalPost = -xPosOpponentGoalPost;
xPosOwnGoal = -xPosOpponentGoal;
xPosOwnFieldBorder = -xPosOpponentFieldBorder;
xPosReturnFromPenalty = xPosOwnPenaltyMark;

$y_positions
yPosRightGoal = -yPosLeftGoal;
yPosRightGoalArea = -yPosLeftGoalArea;
yPosRightPenaltyArea = -yPosLeftPenaltyArea;
yPosRightTouchline = -yPosLeftTouchline;
yPosRightReturnFromPenalty = -yPosLeftReturnFromPenalty;
yPosRightFieldBorder = -yPosLeftFieldBorder;

fieldLinesWidth = $field_lines_width;
centerCircleRadius = $center_circle_radius;
goalPostRadius = $goal_post_radius;
crossBarRadius = goalPostRadius;
goalHeight = $goal_height;
markRadius = $mark_radius;
hasCornerArcs = $has_corner_arcs;
cornerArcRadius = $corner_arc_radius;

goalFrameLines = [
$goal_frame_lines
];

fieldLines = [
$field_lines
];

centerCircle = {
  center = {x = xPosHalfwayLine; y = 0;};
  radius = centerCircleRadius;
  numOfSegments = 16;
};

marksAreCrosses = false;

marks = [
  {x = xPosHalfwayLine; y = 0;},
  {x = xPosOwnPenaltyMark; y = 0;},
  {x = xPosOpponentPenaltyMark; y = 0;},
];

corners = {
$corners
};
""")


def cfg_point(x, y):
    return f"{{x = {x}; y = {y};}}"


def goal_frame_lines():
    """The lines of the goal frames (the goalposts and the back bar)."""
    entries = []
    for side in ("Own", "Opponent"):
        post, back = f"xPos{side}GoalPost", f"xPos{side}Goal"
        for (x1, y1), (x2, y2) in (((post, "yPosLeftGoal"), (back, "yPosLeftGoal")),
                                   ((back, "yPosLeftGoal"), (back, "yPosRightGoal")),
                                   ((post, "yPosRightGoal"), (back, "yPosRightGoal"))):
            entries.append(f"  {{\n    from = {cfg_point(x1, y1)};\n    to   = {cfg_point(x2, y2)};\n  }}")
    return ",\n".join(entries) + ","


def field_lines():
    """The field lines as groups of connected segments (aligned in x within a group) with a comment per group."""
    def group(*segments):
        width = max(len(x) for segment in segments for x, _ in segment) + 1
        entries = []
        for (x1, y1), (x2, y2) in segments:
            entries.append(f"    from = {{x = {(x1 + ';').ljust(width)} y = {y1};}};\n"
                           f"    to   = {{x = {(x2 + ';').ljust(width)} y = {y2};}};\n")
        return "  {\n" + "  },{\n".join(entries) + "  }"

    def area(name, side):  # three sides of an area (goal area or penalty area) in front of one goal
        return group(((f"xPos{side}GoalLine", f"yPosLeft{name}"), (f"xPos{side}{name}", f"yPosLeft{name}")),
                     ((f"xPos{side}{name}", f"yPosLeft{name}"), (f"xPos{side}{name}", f"yPosRight{name}")),
                     ((f"xPos{side}{name}", f"yPosRight{name}"), (f"xPos{side}GoalLine", f"yPosRight{name}")))

    chunks = [
        ("  // field border lines",
         group((("xPosOpponentGoalLine", "yPosRightTouchline"), ("xPosOpponentGoalLine", "yPosLeftTouchline")),
               (("xPosOpponentGoalLine", "yPosLeftTouchline"), ("xPosOwnGoalLine", "yPosLeftTouchline")),
               (("xPosOwnGoalLine", "yPosLeftTouchline"), ("xPosOwnGoalLine", "yPosRightTouchline")),
               (("xPosOwnGoalLine", "yPosRightTouchline"), ("xPosOpponentGoalLine", "yPosRightTouchline")))),
        ("// halfway line", group((("xPosHalfwayLine", "yPosLeftTouchline"), ("xPosHalfwayLine", "yPosRightTouchline")))),
        ("// goal areas", area("GoalArea", "Own")),
        (None, area("GoalArea", "Opponent")),
        ("// penalty areas", area("PenaltyArea", "Own")),
        (None, area("PenaltyArea", "Opponent")),
    ]
    return "\n".join(f"{comment}\n" * (comment is not None) + entries + ("," if i < len(chunks) - 1 else "") + "\n"
                     for i, (comment, entries) in enumerate(chunks)).rstrip("\n")


def corners():
    """The positions of the line corners that are expected at the intersections of the lines."""
    circle = [("xPosHalfwayLine", "centerCircleRadius"), ("xPosHalfwayLine", "-centerCircleRadius")]

    def goal_line(side):  # where the penalty area and the goal area meet the goal line
        return [(f"xPos{side}GoalLine", f"yPos{y}") for y in ("LeftPenaltyArea", "RightPenaltyArea", "LeftGoalArea", "RightGoalArea")]

    def l_corner(near, far, side):
        return circle + goal_line(near) + [("xPosHalfwayLine", f"yPos{side}Touchline"), (f"xPos{near}GoalLine", f"yPos{side}Touchline"),
                                           (f"xPos{far}PenaltyArea", f"yPos{side}PenaltyArea"), (f"xPos{far}GoalArea", f"yPos{side}GoalArea")]

    lists = {
        "xCorner": circle,
        "tCorner0": circle + goal_line("Own"),
        "tCorner90": circle + [("xPosHalfwayLine", "yPosRightTouchline")],
        "tCorner180": circle + goal_line("Opponent"),
        "tCorner270": circle + [("xPosHalfwayLine", "yPosLeftTouchline")],
        "lCorner0": l_corner("Own", "Opponent", "Right"),
        "lCorner90": l_corner("Opponent", "Own", "Right"),
        "lCorner180": l_corner("Opponent", "Own", "Left"),
        "lCorner270": l_corner("Own", "Opponent", "Left"),
    }
    return "\n\n".join("  %s = [\n%s\n  ];" % (name, ",\n".join("    " + cfg_point(x, y) for x, y in points))
                        for name, points in lists.items())


def generate_field_dimensions(d):
    return FIELD_DIMENSIONS_TEMPLATE.substitute(
        x_positions=assignments(
            ("xPosOpponentFieldBorder", mm(d.goal_line_x + d.border_strip_width), "origin -> far field boundary"),
            ("xPosOpponentGoal", mm(d.goal_x + d.net_offset_x), "origin -> middle far side of goal"),
            ("xPosOpponentGoalPost", mm(d.goal_x), "origin -> middle goal post"),
            ("xPosOpponentGoalLine", mm(d.goal_line_x), "origin -> middle goal line"),
            ("xPosOpponentGoalArea", mm(d.goal_area_x), "origin -> middle near goal area line"),
            ("xPosOpponentPenaltyMark", mm(d.penalty_mark_x), "origin -> middle penalty mark"),
            ("xPosOpponentPenaltyArea", mm(d.penalty_area_x), "origin -> middle near penalty area line")),
        y_positions=assignments(
            ("yPosLeftFieldBorder", mm(d.touchline_y + d.border_strip_width), "origin -> side field boundary"),
            ("yPosLeftReturnFromPenalty", mm(d.touchline_y + 0.5), "origin -> 50cm beyond touchline"),
            ("yPosLeftTouchline", mm(d.touchline_y), "origin -> middle touchline"),
            ("yPosLeftPenaltyArea", mm(d.penalty_area_y), "origin -> middle left penalty area line"),
            ("yPosLeftGoalArea", mm(d.goal_area_y), "origin -> middle left goal area line"),
            ("yPosLeftGoal", mm(d.goalpost_y), "origin -> middle left goal post")),
        field_lines_width=mm(d.line_width),
        center_circle_radius=mm(d.center_circle_radius),
        goal_post_radius=mm(d.goalpost_radius),
        goal_height=mm(d.goal_height + d.goalpost_diameter),
        mark_radius=mm(d.penalty_mark_radius),
        has_corner_arcs="false" if d.corner_arc_radius is None else "true",
        corner_arc_radius=mm(d.corner_arc_radius or 0),
        goal_frame_lines=goal_frame_lines(),
        field_lines=field_lines(),
        corners=corners())


def main():
    parser = argparse.ArgumentParser(description="Generate a SimRobot scene and/or FieldDimensions for a specified field")
    parser.add_argument("parameters", type=Path, help="Path to a JSON file with measurements of the field")
    parser.add_argument("--scene", type=Path, help="Path to the .rsi3 file to generate")
    parser.add_argument("--field-dimensions", type=Path, help="Path to the .cfg file to generate")
    args = parser.parse_args()
    if not (args.scene or args.field_dimensions):
        parser.error("at least one of --scene and --field-dimensions is required")

    dimensions = Dimensions.from_parameters(json.loads(args.parameters.read_text()))
    if args.scene:
        args.scene.write_text(generate_scene(dimensions))
    if args.field_dimensions:
        args.field_dimensions.write_text(generate_field_dimensions(dimensions))


if __name__ == "__main__":
    main()
