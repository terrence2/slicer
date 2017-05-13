mod errors {
    error_chain!{}
}

use errors::Result;
use geometry::Plane;
use nalgebra::Point3;
use stl::StlMesh;

pub struct Triangle {
    pub verts: [u32; 3],
    pub normal: u32,
    pub tag: u8,
}

impl Triangle {
    fn vert(&self, mesh: &Mesh, i: usize) -> Point3<f32> {
        let index = self.verts[i] as usize;
        return mesh.verts[index];
    }
}

pub struct Mesh {
    pub verts: Vec<Point3<f32>>,
    pub normals: Vec<Point3<f32>>,
    pub tris: Vec<Triangle>,
}

fn push_unique_point(points: &mut Vec<Point3<f32>>, point: Point3<f32>) -> u32 {
    for (i, p) in points.iter().enumerate() {
        if relative_eq(p, &point) {
            return i as u32;
        }
    }
    points.push(point);
    return (points.len() as u32) - 1u32;
}

fn relative_eq(p0: &Point3<f32>, p1: &Point3<f32>) -> bool {
    return relative_eq!(p0.x, p1.x) && relative_eq!(p0.y, p1.y) && relative_eq!(p0.z, p1.z);
}

impl Mesh {
    pub fn from_stl(stl: StlMesh, tag: u8) -> Result<Mesh> {
        let mut mesh = Mesh {
            verts: Vec::new(),
            normals: Vec::new(),
            tris: Vec::new(),
        };
        for stl_tri in stl.tris {
            let tri = Triangle {
                normal: push_unique_point(&mut mesh.normals, stl_tri.normal),
                verts: [push_unique_point(&mut mesh.verts, stl_tri.verts[0]),
                        push_unique_point(&mut mesh.verts, stl_tri.verts[1]),
                        push_unique_point(&mut mesh.verts, stl_tri.verts[2])],
                tag: tag,
            };
            mesh.tris.push(tri);
        }
        return Ok(mesh);
    }

    /// Import the vertices and faces of `other` into this mesh, cutting out co-planar, intersecting
    /// faces to result in the minimal mesh. Note: this is not a CSG union: the meshes must be non-
    /// interpenetrating.
    pub fn union_non_overlapping(mut self, other: &Mesh) -> Result<Mesh> {
        for tri0 in self.tris.iter() {
            let pi0 = Plane::from_triangle(&tri0.vert(&self, 0),
                                           &tri0.vert(&self, 1),
                                           &tri0.vert(&self, 2));
            for tri1 in other.tris.iter() {
                if relative_eq!(pi0.distance_to(&tri1.vert(&other, 0)), 0.0f32) &&
                   relative_eq!(pi0.distance_to(&tri1.vert(&other, 1)), 0.0f32) &&
                   relative_eq!(pi0.distance_to(&tri1.vert(&other, 2)), 0.0f32) {
                    // Co-planar
                }
            }
        }
        bail!("not implemented")
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use std::fs::File;

    #[test]
    fn test_simplify_mesh() {
        let mut fp = File::open("test_data/cube_scad.stl").unwrap();
        let stl = super::StlMesh::from_file(&mut fp).unwrap();
        let m = super::Mesh::from_stl(stl, 0).unwrap();
        assert_eq!(m.normals.len(), 6);
        assert_eq!(m.verts.len(), 8);
        assert_eq!(m.tris.len(), 12);
    }
}
