/*
 * slicer: Better slicing for multi-color extrusion.
 * Copyright (C) 2017  Terrence Cole
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/// Slice Space Partition
///
/// Similar to BSP, in that it allows for fast space searching, but
/// makes that search efficient only for slices. This avoids the face
/// explosion that BSP results in when run against the sort of fluid
/// surfaces common in 3d printing. It does not naively allow for efficient
/// merging, but it does allow us to easily sub-divide the problem into
/// manageable sub-meshes.

use alga::general::Inverse;
use errors::{Error, Result, ResultExt};
use itertools::Itertools;
use nalgebra::{normalize, Point3, Vector3};
use std::f32;
use stl::{StlMesh, StlTriangle};

pub struct SliceSettings {
    // In mm.
    pub layer_height: f32,
}

impl SliceSettings {
    pub fn new() -> SliceSettings {
        SliceSettings {
            layer_height: 0.1
        }
    }

    pub fn layer_height(mut self, mm: f32) -> Self {
        self.layer_height = mm;
        self
    }
}

#[derive(Clone, Debug)]
pub struct Vertex {
    position: Point3<f32>,
}

impl Vertex {
    pub fn new(position: &Point3<f32>) -> Self {
        Vertex { position: position.clone() }
    }

    pub fn invert(&mut self) {}
}

#[derive(Clone, Debug)]
pub struct Plane {
    normal: Vector3<f32>,
    d: f32,
}

// Bit flags for tracking orientation.
const COPLANAR: u8 = 0;
const FRONT: u8 = 1;
const BACK: u8 = 2;
const SPANNING: u8 = 3;
const EPSILON: f32 = 1e-3;

impl Plane {
    pub fn from_points(p0: &Point3<f32>, p1: &Point3<f32>, p2: &Point3<f32>) -> Self {
        assert!(p0 != p1);
        assert!(p0 != p2);
        assert!(p1 != p2);
        let p01 = *p1 - *p0;
        let p02 = *p2 - *p0;
        let nv = normalize(&p01.cross(&p02));
        let d = -nv.dot(&p0.coords);
        assert!(nv[0] != f32::NAN);
        assert!(nv[1] != f32::NAN);
        assert!(nv[2] != f32::NAN);
        assert!(d != f32::NAN);
        Plane { normal: nv, d: d }
    }

    fn distance_to(&self, p: &Point3<f32>) -> f32 {
        self.normal.dot(&p.coords) + self.d
    }

    pub fn invert(&mut self) {
        self.normal.inverse_mut();
        self.d = -self.d;
    }

    pub fn split_triangle(&self, triangle: Triangle, front: &mut Vec<Triangle>, back: &mut Vec<Triangle>) {
        let mut triangle_type = 0u8;
        let mut vertex_types: [u8; 3] = [0, 0, 0];
        for (i, vertex) in triangle.vertices.iter().enumerate() {
            let t = self.distance_to(&vertex.position);
            let ty = if t < -EPSILON {
                BACK
            } else if t > EPSILON {
                FRONT
            } else {
                COPLANAR
            };
            triangle_type |= ty;
            vertex_types[i] = ty;
        }

        match triangle_type {
            COPLANAR => {
                if self.normal.dot(&triangle.plane.normal) > 0f32 {
                    front.push(triangle)
                } else {
                    back.push(triangle)
                }
            }
            FRONT => {
                front.push(triangle);
            }
            BACK => {
                back.push(triangle);
            }
            SPANNING => {
                self.split_spanning_polygon(vertex_types,
                                            triangle,
                                            front,
                                            back);
            }
            _ => panic!("impossible polygon type when splitting"),
        }
    }

    fn split_spanning_polygon(&self,
                              vertex_types: [u8; 3],
                              triangle: Triangle,
                              front_tris: &mut Vec<Triangle>,
                              back_tris: &mut Vec<Triangle>) {
        assert!(triangle.vertices.len() == 3);

        let mut front = Vec::<Vertex>::new();
        let mut back = Vec::<Vertex>::new();
        for i in 0..3 {
            let ref vi = triangle.vertices[i];
            let ti = vertex_types[i];

            match ti {
                FRONT => front.push(vi.clone()),
                BACK => back.push(vi.clone()),
                COPLANAR => {
                    front.push(vi.clone());
                    back.push(vi.clone());
                }
                _ => panic!("impossible vertex type when splitting spanning poly"),
            }

            let tj = vertex_types[(i + 1) % vertex_types.len()];
            if (ti | tj) == SPANNING {
                let ref vj = triangle.vertices[(i + 1) % triangle.vertices.len()];
                let i2j = vj.position - vi.position;
                let t = -self.distance_to(&vi.position) / self.normal.dot(&i2j);

                // Note that this split quantizes the line's position where it
                // crosses the split plane to a resolution of f32.
                let v_split = Vertex { position: vi.position + (i2j * t) };

                // This may create a degenerate polygon if the quantization collapsed to
                // the same coordinate as one of the inputs; _build_tris_for_split is
                // responsible for detecting and rejecting this case.
                front.push(v_split.clone());
                back.push(v_split);
            }
        }

        self._build_tris_for_split(front, front_tris, triangle.attribute);
        self._build_tris_for_split(back, back_tris, triangle.attribute);
    }

    fn _build_tris_for_split(&self, verts: Vec<Vertex>, tris: &mut Vec<Triangle>, attr: u8) {
        // The above algorithm may create a degenerate polygon if one point or edge
        // is co-planar with the split plane, so we ignore the 1 and 2 cases.
        if verts.len() == 3 {
            if let Some((a, b, c)) = verts.into_iter().tuples().next() {
                if let Some(tri) = Triangle::from_vertices(a, b, c, attr) {
                    tris.push(tri);
                }
            }
        } else if verts.len() == 4 {
            // Quantization may throw our points out of co-planarity, so we always
            // reduce to triangles, even though this results in more intermediate objects.
            if let Some((a, b, c, d)) = verts.into_iter().tuples().next() {
                let l02 = (a.position - c.position).norm_squared();
                let l13 = (b.position - d.position).norm_squared();
                if l02 > l13 {
                    if let Some(tri) = Triangle::from_vertices(a.clone(), b, c.clone(), attr) {
                        tris.push(tri);
                    }
                    if let Some(tri) = Triangle::from_vertices(a, c, d, attr) {
                        tris.push(tri);
                    }
                } else {
                    if let Some(tri) = Triangle::from_vertices(a, b.clone(), d.clone(), attr) {
                        tris.push(tri);
                    }
                    if let Some(tri) = Triangle::from_vertices(b, c, d, attr) {
                        tris.push(tri);
                    }
                }
            }
        } else if verts.len() > 4 {
            panic!("unexpectedly large number of vertices");
        }
    }
}

#[derive(Clone, Debug)]
pub struct Triangle {
    plane: Plane,
    vertices: Vec<Vertex>,
    attribute: u8,
}

impl Triangle {
    pub fn from_vertices(v0: Vertex, v1: Vertex, v2: Vertex, attr: u8) -> Option<Self> {
        // Detect degenerate cases.
        if v0.position == v1.position || v0.position == v2.position || v1.position == v2.position {
            return None;
        }
        let plane = Plane::from_points(&v0.position, &v1.position, &v2.position);
        return Some(Triangle {
            plane: plane,
            vertices: vec![v0, v1, v2],
            attribute: attr,
        });
    }

    pub fn from_points(p0: &Point3<f32>,
                       p1: &Point3<f32>,
                       p2: &Point3<f32>,
                       attr: u8)
                       -> Option<Self> {
        return Self::from_vertices(Vertex::new(p0), Vertex::new(p1), Vertex::new(p2), attr);
    }

    pub fn set_attribute(&mut self, attr: u8) {
        self.attribute = attr;
    }

    pub fn invert(&mut self) {
        self.plane.invert();
        for vert in self.vertices.iter_mut() {
            vert.invert();
        }
        self.vertices.reverse();
    }
}

enum SliceBranch {
    Tree(Box<SplitTreeNode>),
    Leaf(Vec<Triangle>)
}

#[derive(Debug)]
struct Extents {
    lo: Point3<f32>,
    hi: Point3<f32>,
}

impl Extents {
    fn from_meshes(meshes: &Vec<StlMesh>) -> Self {
        let mut lo0 = f32::MAX;
        let mut hi0 = f32::MIN;
        let mut lo1 = f32::MAX;
        let mut hi1 = f32::MIN;
        let mut lo2 = f32::MAX;
        let mut hi2 = f32::MIN;
        for mesh in meshes.iter() {
            for tris in mesh.tris.iter() {
                for vertex in tris.verts.iter() {
                    if vertex.coords[0] < lo0 {
                        lo0 = vertex.coords[0];
                    }
                    if vertex.coords[0] > hi0 {
                        hi0 = vertex.coords[0];
                    }
                    if vertex.coords[1] < lo1 {
                        lo1 = vertex.coords[1];
                    }
                    if vertex.coords[1] > hi1 {
                        hi1 = vertex.coords[1];
                    }
                    if vertex.coords[2] < lo2 {
                        lo2 = vertex.coords[2];
                    }
                    if vertex.coords[2] > hi2 {
                        hi2 = vertex.coords[2];
                    }
                }
            }
        }
        return Self {
            lo: Point3::new(lo0, lo1, lo2),
            hi: Point3::new(hi0, hi1, hi2),
        };
    }
}

struct SplitTreeNode {
    offset: usize,
    lo_offset: usize,
    hi_offset: usize,

    above: SliceBranch,
    below: SliceBranch,
}

struct LayerIterator<'t> {
    root: &'t SplitTreeNode,
    current: usize
}

impl SplitTreeNode {
    fn split(lo_offset: usize, hi_offset: usize, mut tris: Vec<Triangle>, settings: &SliceSettings) -> Box<Self> {
        let offset = Self::_bisect_offset(lo_offset, hi_offset);
        let position = offset as f32 * settings.layer_height;
        let v0 = Point3::new(0f32, 0f32, position);
        let v1 = Point3::new(1f32, 0f32, position);
        let v2 = Point3::new(0f32, 1f32, position);
        let plane = Plane::from_points(&v0, &v1, &v2);

        let mut above = vec![];
        let mut below = vec![];
        for tri in tris.drain(..) {
            plane.split_triangle(tri, &mut above, &mut below);
        }

        let child_below = if offset - lo_offset == 1 {
            SliceBranch::Leaf(below)
        } else {
            SliceBranch::Tree(SplitTreeNode::split(lo_offset, offset, below, settings))
        };
        let child_above = if hi_offset - offset == 1 {
            SliceBranch::Leaf(above)
        } else {
            SliceBranch::Tree(SplitTreeNode::split(offset, hi_offset, above, settings))
        };

        return Box::new(SplitTreeNode {
            offset: offset,
            lo_offset: lo_offset,
            hi_offset: hi_offset,
            above: child_above,
            below: child_below,
        });
    }

    fn _bisect_offset(bottom: usize, top: usize) -> usize {
        bottom + (((top - bottom) as f32 / 2f32).floor() as usize)
    }

    fn layers(&self) -> LayerIterator {
        LayerIterator {
            root: self,
            current: 0,
        }
    }

    fn steal_triangles(mut self) -> Vec<Triangle> {
        let mut out = match self.below {
            SliceBranch::Leaf(tris) => tris,
            SliceBranch::Tree(node) => node.steal_triangles()
        };
        match self.above {
            SliceBranch::Leaf(ref mut tris) => out.append(tris),
            SliceBranch::Tree(node) => out.append(&mut node.steal_triangles())
        };
        return out;
    }
}

impl<'t> Iterator for LayerIterator<'t> {
    type Item = &'t Vec<Triangle>;

    fn next(&mut self) -> Option<Self::Item> {
        None
    }
}

pub struct SSP {
    layer_count: usize,
    roots: Vec<Box<SplitTreeNode>>,
}

impl SSP {
    pub fn new(meshes: Vec<StlMesh>, settings: SliceSettings) -> Self {
        let extents = Extents::from_meshes(&meshes);
        let height = extents.hi[2] - extents.lo[2];
        let layer_count = (height / settings.layer_height).ceil() as usize;

        let mut roots = vec![];
        for (i, mesh) in meshes.iter().enumerate() {
            let mut tris = vec![];
            for stl_tri in mesh.tris.iter() {
                let tri = match Triangle::from_points(&stl_tri.verts[0], &stl_tri.verts[1], &stl_tri.verts[2], i as u8) {
                    Some(tri) => tri,
                    None => continue,
                };
                tris.push(tri);
            }
            let root = SplitTreeNode::split(0, layer_count, tris, &settings);
            roots.push(root);
        }

        return SSP {
            layer_count: layer_count,
            roots: roots,
        };
    }

    pub fn convert_to_triangles(mut self) -> Vec<Triangle> {
        let mut out = vec![];
        for root in self.roots {
            out.append(&mut root.steal_triangles());
        }
        return out;
    }

    pub fn convert_to_stl(mut self, name: &str) -> StlMesh {
        let mut tris: Vec<StlTriangle> = Vec::new();
        for t in self.convert_to_triangles() {
            let mut u = StlTriangle::new(t.vertices[0].position,
                                         t.vertices[1].position,
                                         t.vertices[2].position,
                                         Point3::from_coordinates(t.plane.normal));
            match t.attribute {
                1 => u.set_color(0, 97, 207),
                2 => u.set_color(0, 207, 97),
                _ => u.set_color(255, 0, 255),
            }
            tris.push(u);
        }
        return StlMesh::from_tris(name, tris);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs::File;

    #[test]
    fn test_slice_settings() {
        let settings = SliceSettings::new()
                            .layer_height(0.2);
    }

    #[test]
    fn bench_octopus() {
        let fn0 = "test_data/2color/from_web/octopus2/octopus_part1_origin.stl";
        let fn1 = "test_data/2color/from_web/octopus2/octopus_part2_origin.stl";
        let mut fp0 = File::open(fn0).expect("failed to open part1");
        let mut fp1 = File::open(fn1).expect("failed to open part2");
        let stl0 = StlMesh::from_file(&mut fp0).expect("failed to parse part1");
        let stl1 = StlMesh::from_file(&mut fp1).expect("failed to parse part2");
        let ssp = SSP::new(vec![stl0, stl1], SliceSettings::new());
        let stl = ssp.convert_to_stl("test");
        println!("Tris count is: {}", stl.tris.len());
        let mut fp = File::create("target/bench_octopus.stl").unwrap();
        stl.to_binary_file(&mut fp).unwrap();
    }
}
