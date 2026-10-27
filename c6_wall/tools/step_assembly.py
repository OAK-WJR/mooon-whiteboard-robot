#!/usr/bin/env python3
"""Minimal STEP (AP214/AP242) assembly reader: world-space point clouds per part instance.

    python3 tools/step_assembly.py "Assembly 1.step" [name-regex]

Follows NEXT_ASSEMBLY_USAGE_OCCURRENCE / CONTEXT_DEPENDENT_SHAPE_REPRESENTATION /
ITEM_DEFINED_TRANSFORMATION from the root product down, and collects the
CARTESIAN_POINTs reachable from each part's shape representation. Good enough for
bounding boxes and centroids; it is not a B-rep kernel.
"""
import re
import sys
from collections import defaultdict

import numpy as np

ENT = re.compile(r"#(\d+)\s*=\s*(.*?);\s*(?=#\d+\s*=|ENDSEC)", re.S)
REF = re.compile(r"#(\d+)")


def load(path):
    text = open(path, errors="ignore").read()
    body = re.sub(r"/\*.*?\*/", "", text[text.index("DATA;") + 5:], flags=re.S)  # ST-Developer comments
    ents = {}
    for m in ENT.finditer(body):
        ents[int(m.group(1))] = re.sub(r"\s+", " ", m.group(2).strip())
    return ents


def kind(s):
    return s[:s.index("(")].strip() if not s.startswith("(") else "COMPLEX"


def args(s):
    return s[s.index("(") + 1:s.rindex(")")]


def refs(s):
    return [int(x) for x in REF.findall(s)]


class Assembly:
    def __init__(self, path):
        self.e = load(path)
        self.points = {}
        e = self.e
        self.by_kind = defaultdict(list)
        for i, s in e.items():
            self.by_kind[kind(s)].append(i)
        prod_name = {}
        for i, s in e.items():
            if s.startswith("PRODUCT("):
                m = re.match(r"PRODUCT\(\s*'((?:[^']|'')*)'", s)
                prod_name[i] = m.group(1).replace("''", "'") if m else s[8:60]
        pdf_prod = {i: refs(s)[0] for i, s in e.items() if s.startswith("PRODUCT_DEFINITION_FORMATION")}
        self.pd_name = {}
        for i in self.by_kind["PRODUCT_DEFINITION"]:
            r = refs(e[i])
            self.pd_name[i] = prod_name.get(pdf_prod.get(r[0]), "?")
        pds_target = {i: refs(e[i])[-1] for i in self.by_kind["PRODUCT_DEFINITION_SHAPE"]}
        self.pd_rep = {}
        for i in self.by_kind["SHAPE_DEFINITION_REPRESENTATION"]:
            pds, rep = refs(e[i])[:2]
            self.pd_rep[pds_target[pds]] = rep
        self.rep_links = defaultdict(list)  # untransformed shape_representation_relationship
        for i in self.by_kind["SHAPE_REPRESENTATION_RELATIONSHIP"]:
            a, b = refs(e[i])[:2]
            self.rep_links[a].append(b)
            self.rep_links[b].append(a)
        self.children = defaultdict(list)
        nauo_child = {}
        for i in self.by_kind["NEXT_ASSEMBLY_USAGE_OCCURRENCE"]:
            parent, child = refs(e[i])[:2]
            nauo_child[i] = (parent, child)
        for i in self.by_kind["CONTEXT_DEPENDENT_SHAPE_REPRESENTATION"]:
            rr, pds = refs(e[i])[:2]
            nauo = pds_target[pds]
            parent, child = nauo_child[nauo]
            s = e[rr]
            m = re.search(r"REPRESENTATION_RELATIONSHIP\s*\('[^']*'\s*,\s*'[^']*'\s*,\s*#(\d+)\s*,\s*#(\d+)\s*\)", s)
            rep1, rep2 = int(m.group(1)), int(m.group(2))
            idt = int(re.search(r"REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION\s*\(\s*#(\d+)", s).group(1))
            ax1, ax2 = refs(e[idt])[:2]
            # Item 1 lives in rep_1, item 2 in rep_2; the operator maps rep_1 onto rep_2.
            T = self.placement(ax2) @ np.linalg.inv(self.placement(ax1))
            if self.pd_rep.get(child) != rep1:  # rep_1 is the parent: invert
                T = np.linalg.inv(T)
            self.children[parent].append((child, T, e[nauo].split("'")[3] if e[nauo].count("'") >= 4 else ""))
        all_children = {c for v in self.children.values() for c, _, _ in v}
        self.roots = [p for p in self.pd_name if p not in all_children]

    def point(self, i):
        inner = re.search(r"\(([^()]*)\)\s*\)\s*$", self.e[i]).group(1)  # the coordinate tuple
        return np.array([float(v) for v in inner.split(",")][:3])

    direction = point

    def placement(self, i):
        r = refs(self.e[i])
        o = self.point(r[0])
        z = self.direction(r[1]) if len(r) > 1 else np.array([0, 0, 1.0])
        x = self.direction(r[2]) if len(r) > 2 else np.array([1.0, 0, 0])
        z = z / np.linalg.norm(z)
        x = x - z * (x @ z)
        x /= np.linalg.norm(x)
        y = np.cross(z, x)
        M = np.eye(4)
        M[:3, 0], M[:3, 1], M[:3, 2], M[:3, 3] = x, y, z, o
        return M

    def rep_points(self, rep):
        if rep in self.points:
            return self.points[rep]
        seen, stack, pts = set(), [rep], []
        for linked in self.rep_links.get(rep, []):
            if kind(self.e[linked]) != "SHAPE_REPRESENTATION" or linked == rep:
                stack.append(linked)
        while stack:
            i = stack.pop()
            if i in seen or i not in self.e:
                continue
            seen.add(i)
            s = self.e[i]
            k = kind(s)
            if k == "CARTESIAN_POINT":
                pts.append(self.point(i))
                continue
            if k in ("SHAPE_REPRESENTATION_RELATIONSHIP", "STYLED_ITEM", "OVER_RIDING_STYLED_ITEM", "COMPLEX",
                     "GEOMETRIC_REPRESENTATION_CONTEXT", "AXIS2_PLACEMENT_3D") and i != rep:
                if k != "AXIS2_PLACEMENT_3D":
                    continue
            if k == "AXIS2_PLACEMENT_3D" and i != rep:
                continue  # frames are not geometry
            stack.extend(r for r in refs(s) if r not in seen)
        P = np.array(pts) if pts else np.zeros((0, 3))
        self.points[rep] = P
        return P

    def walk(self, pd=None, T=None, path=()):
        """Yields (path names, pd, world transform) for every node."""
        if pd is None:
            for r in self.roots:
                yield from self.walk(r, np.eye(4), ())
            return
        name = self.pd_name.get(pd, "?")
        path = path + (name,)
        yield path, pd, T
        for child, Tc, _ in self.children.get(pd, []):
            yield from self.walk(child, T @ Tc, path)

    def world_points(self, pd, T):
        rep = self.pd_rep.get(pd)
        if rep is None:
            return np.zeros((0, 3))
        P = self.rep_points(rep)
        return P @ T[:3, :3].T + T[:3, 3] if len(P) else P


if __name__ == "__main__":
    a = Assembly(sys.argv[1])
    pat = re.compile(sys.argv[2] if len(sys.argv) > 2 else ".")
    for path, pd, T in a.walk():
        if pat.search(path[-1]):
            P = a.world_points(pd, T)
            if len(P):
                print(" / ".join(path[1:]), len(P), np.round(P.min(0) * 1000, 2), np.round(P.max(0) * 1000, 2))
