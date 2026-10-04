#!/usr/bin/env python3
"""Generate meshes/uwb_anchor.dae, the RViz model of a LinkTrack anchor on a tripod.

Origin = the UWB antenna phase center, i.e. the anchor coordinate. The module and
antenna sit around the origin, the pole and tripod extend down to z = -STAND_HEIGHT.
No vendor CAD model is published, so this is built from simple solids, with the
module roughly at LinkTrack size (scaled up a bit so it stays visible from afar).

  python3 make_anchor_mesh.py      # rewrites uwb_anchor.dae next to this script
"""

import math
import os

STAND_HEIGHT = 1.75    # [m] from the antenna down to the tripod feet, = -floor_z in the launch
SEGMENTS = 24

MATERIALS = {
    'stand': (0.62, 0.64, 0.68),
    'module': (0.10, 0.35, 0.80),
    'antenna': (0.08, 0.08, 0.08),
    'led': (1.00, 0.55, 0.05),
}


def box(cx, cy, cz, sx, sy, sz):
    x0, x1 = cx - sx / 2, cx + sx / 2
    y0, y1 = cy - sy / 2, cy + sy / 2
    z0, z1 = cz - sz / 2, cz + sz / 2
    v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
         (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    quads = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    return v, [t for a, b, c, d in quads for t in ((a, b, c), (a, c, d))]


def cylinder(p0, p1, r0, r1=None):
    """Closed (possibly tapered) cylinder between points p0 and p1."""
    r1 = r0 if r1 is None else r1
    ax = [p1[i] - p0[i] for i in range(3)]
    length = math.sqrt(sum(a * a for a in ax))
    ax = [a / length for a in ax]
    # two unit vectors perpendicular to the axis
    ref = (0.0, 0.0, 1.0) if abs(ax[2]) < 0.9 else (1.0, 0.0, 0.0)
    u = [ax[1] * ref[2] - ax[2] * ref[1], ax[2] * ref[0] - ax[0] * ref[2], ax[0] * ref[1] - ax[1] * ref[0]]
    n = math.sqrt(sum(a * a for a in u))
    u = [a / n for a in u]
    w = [ax[1] * u[2] - ax[2] * u[1], ax[2] * u[0] - ax[0] * u[2], ax[0] * u[1] - ax[1] * u[0]]
    v = []
    for p, r in ((p0, r0), (p1, r1)):
        for k in range(SEGMENTS):
            a = 2 * math.pi * k / SEGMENTS
            v.append(tuple(p[i] + r * (math.cos(a) * u[i] + math.sin(a) * w[i]) for i in range(3)))
    v += [tuple(p0), tuple(p1)]
    c0, c1 = 2 * SEGMENTS, 2 * SEGMENTS + 1
    tris = []
    for k in range(SEGMENTS):
        a, b = k, (k + 1) % SEGMENTS
        tris += [(a, b, SEGMENTS + b), (a, SEGMENTS + b, SEGMENTS + a)]
        tris += [(c0, b, a), (c1, SEGMENTS + a, SEGMENTS + b)]
    return v, tris


def build():
    parts = {name: ([], []) for name in MATERIALS}

    def add(material, solid):
        verts, tris = parts[material]
        base = len(verts)
        verts.extend(solid[0])
        tris.extend(tuple(base + i for i in t) for t in solid[1])

    # module: ~LinkTrack box, antenna on top, the antenna center is the origin
    add('module', box(0, 0, -0.07, 0.10, 0.07, 0.03))
    add('led', box(0.051, 0, -0.065, 0.004, 0.02, 0.008))
    add('antenna', cylinder((0, 0, -0.055), (0, 0, 0.06), 0.008, 0.006))
    add('antenna', cylinder((0, 0, 0.06), (0, 0, 0.075), 0.010))
    # mount, pole, tripod
    add('stand', box(0, 0, -0.095, 0.04, 0.04, 0.02))
    add('stand', cylinder((0, 0, -0.105), (0, 0, -STAND_HEIGHT + 0.35), 0.014))
    hub = -STAND_HEIGHT + 0.35
    add('stand', cylinder((0, 0, hub + 0.03), (0, 0, hub - 0.03), 0.03))
    for k in range(3):
        a = 2 * math.pi * k / 3
        foot = (0.45 * math.cos(a), 0.45 * math.sin(a), -STAND_HEIGHT)
        add('stand', cylinder((0, 0, hub), foot, 0.012, 0.009))
        add('stand', cylinder((foot[0], foot[1], foot[2] + 0.01), (foot[0], foot[1], foot[2]), 0.025))
    return parts


def to_collada(parts):
    effects, materials, geoms, nodes = [], [], [], []
    for name, (verts, tris) in parts.items():
        r, g, b = MATERIALS[name]
        effects.append(f'''    <effect id="{name}-effect"><profile_COMMON><technique sid="common"><phong>
      <diffuse><color>{r} {g} {b} 1</color></diffuse>
      <specular><color>0.3 0.3 0.3 1</color></specular><shininess><float>20</float></shininess>
    </phong></technique></profile_COMMON></effect>''')
        materials.append(f'    <material id="{name}-material" name="{name}">'
                         f'<instance_effect url="#{name}-effect"/></material>')
        pos = ' '.join(f'{c:.5f}' for p in verts for c in p)
        idx = ' '.join(str(i) for t in tris for i in t)
        geoms.append(f'''    <geometry id="{name}-mesh" name="{name}"><mesh>
      <source id="{name}-pos"><float_array id="{name}-pos-array" count="{3 * len(verts)}">{pos}</float_array>
        <technique_common><accessor source="#{name}-pos-array" count="{len(verts)}" stride="3">
          <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
        </accessor></technique_common></source>
      <vertices id="{name}-vtx"><input semantic="POSITION" source="#{name}-pos"/></vertices>
      <triangles material="{name}-mat" count="{len(tris)}">
        <input semantic="VERTEX" source="#{name}-vtx" offset="0"/><p>{idx}</p></triangles>
    </mesh></geometry>''')
        nodes.append(f'''      <node id="{name}" name="{name}"><instance_geometry url="#{name}-mesh">
        <bind_material><technique_common>
          <instance_material symbol="{name}-mat" target="#{name}-material"/>
        </technique_common></bind_material></instance_geometry></node>''')
    nl = '\n'
    return f'''<?xml version="1.0" encoding="utf-8"?>
<!-- generated by make_anchor_mesh.py, do not edit -->
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
  <library_effects>
{nl.join(effects)}
  </library_effects>
  <library_materials>
{nl.join(materials)}
  </library_materials>
  <library_geometries>
{nl.join(geoms)}
  </library_geometries>
  <library_visual_scenes><visual_scene id="scene" name="scene">
{nl.join(nodes)}
  </visual_scene></library_visual_scenes>
  <scene><instance_visual_scene url="#scene"/></scene>
</COLLADA>
'''


if __name__ == '__main__':
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'uwb_anchor.dae')
    with open(out, 'w') as f:
        f.write(to_collada(build()))
    print(out)
