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
use std::f32;
use stl::{StlMesh, StlTriangle};
use nalgebra::{normalize, Point3, Vector3};

#[derive(Clone)]
struct Plane {
    normal: Vector3<f32>,
    d: f32,
}

impl Plane {
    fn from_stl_triangle(stl_tri: &StlTriangle) -> Plane {
        assert!(stl_tri.verts[0] != stl_tri.verts[1]);
        assert!(stl_tri.verts[0] != stl_tri.verts[2]);
        assert!(stl_tri.verts[1] != stl_tri.verts[2]);
        let p01 = stl_tri.verts[1] - stl_tri.verts[0];
        let p02 = stl_tri.verts[2] - stl_tri.verts[0];
        let nv = normalize(&p01.cross(&p02));
        let d = -nv.dot(&stl_tri.verts[0].coords);
        assert!(nv[0] != f32::NAN);
        assert!(nv[1] != f32::NAN);
        assert!(nv[2] != f32::NAN);
        assert!(d != f32::NAN);
        return Plane { normal: nv, d: d };
    }

    pub fn distance_to(&self, p: &Point3<f32>) -> f32 {
        self.normal.dot(&p.coords) + self.d
    }
}

#[derive(Clone)]
struct IntermediateTriangle {
    plane: Plane,
    points: [Point3<f32>; 3],
}

const EPSILON: f32 = 1e-3;

fn points_equal(a: &Point3<f32>, b: &Point3<f32>) -> bool {
    return relative_eq!(a.coords[0], b.coords[0]) &&
        relative_eq!(a.coords[1], b.coords[1]) &&
        relative_eq!(a.coords[2], b.coords[2]);
}

impl IntermediateTriangle {
    fn from_stl_triangle(stl_tri: &StlTriangle) -> IntermediateTriangle {
        return IntermediateTriangle {
                   plane: Plane::from_stl_triangle(stl_tri),
                   points: stl_tri.verts,
               };
    }

    fn is_coplanar(&self, other: &IntermediateTriangle) -> bool {
        return self.plane.distance_to(&other.points[0]).abs() < EPSILON &&
               self.plane.distance_to(&other.points[1]).abs() < EPSILON &&
               self.plane.distance_to(&other.points[2]).abs() < EPSILON;
    }

    fn shares_vertices_with(&self, other: &IntermediateTriangle) -> bool {
        let s0 = points_equal(&self.points[0], &other.points[2]) &&
                 points_equal(&self.points[1], &other.points[1]) &&
                 points_equal(&self.points[2], &other.points[0]);
        let s1 = points_equal(&self.points[0], &other.points[0]) &&
                 points_equal(&self.points[1], &other.points[2]) &&
                 points_equal(&self.points[2], &other.points[1]);
        let s2 = points_equal(&self.points[0], &other.points[1]) &&
                 points_equal(&self.points[1], &other.points[0]) &&
                 points_equal(&self.points[2], &other.points[2]);
        return s0 || s1 || s2;
    }
}

struct IntermediateMesh {
    tris: Vec<IntermediateTriangle>,
}

impl IntermediateMesh {
    fn from_stl_mesh(stl_mesh: &StlMesh) -> IntermediateMesh {
        let mut tris = Vec::new();
        for stl_tri in stl_mesh.tris.iter() {
            tris.push(IntermediateTriangle::from_stl_triangle(stl_tri));
        }
        return IntermediateMesh { tris: tris };
    }
}

struct MergedTriangle {
    indices: [u32; 3],
    normal: Vector3<f32>,
}

pub struct MergedMesh {
    vertices: Vec<Point3<f32>>,
    tris: Vec<MergedTriangle>,
}

struct CoplanarIndex {
    mesh_index: u32,
    tri_index: u32,

    triangle: IntermediateTriangle,
    split: Vec<IntermediateTriangle>,
}

pub fn merge_meshes(stl_meshes: &Vec<StlMesh>) -> StlMesh {
    let mut meshes = Vec::new();

    // Compute planes for everything.
    // FIXME: do this in parallel.
    for stl_mesh in stl_meshes.iter() {
        meshes.push(IntermediateMesh::from_stl_mesh(stl_mesh));
    }

    let mut coplanar = Vec::new();
    for (i, mesh_a) in meshes.iter().enumerate() {
        for (j, mesh_b) in meshes.iter().enumerate() {
            if j <= i {
                continue;
            }
            println!("Finding co-planar between {} and {}", i, j);

            for (ta_offset, tri_a) in mesh_a.tris.iter().enumerate() {
                for (tb_offset, tri_b) in mesh_b.tris.iter().enumerate() {
                    if tri_a.is_coplanar(&tri_b) {
                        if tri_a.shares_vertices_with(&tri_b) {
                            println!("find identical at: {}, {}", ta_offset, tb_offset);
                            continue;
                        }
                        coplanar.push(CoplanarIndex {
                                          mesh_index: i as u32,
                                          tri_index: ta_offset as u32,
                                          triangle: tri_a.clone(),
                                          split: Vec::new(),
                                      });
                        coplanar.push(CoplanarIndex {
                                          mesh_index: j as u32,
                                          tri_index: tb_offset as u32,
                                          triangle: tri_b.clone(),
                                          split: Vec::new(),
                                      });
                    }
                }
            }
        }
    }

    // Push all coplanar tris into a mesh for testing.
    println!("Building stl with intermediate: {} coplanar",
             coplanar.len());
    let mut stltris = Vec::new();
    for co in coplanar.drain(..) {
        stltris.push(StlTriangle {
                         verts: co.triangle.points.clone(),
                         normal: Point3::new(co.triangle.plane.normal[0],
                                             co.triangle.plane.normal[1],
                                             co.triangle.plane.normal[2]),
                         color: Some([255, 255, 255]),
                     });
    }
    return StlMesh {
               name: "merged".to_owned(),
               tris: stltris,
           };

    /*
    return MergedMesh {
               vertices: Vec::new(),
               tris: Vec::new(),
           };
    */
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_something() {}
}
