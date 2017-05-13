use stl::StlMesh;
use errors::{Result};
use nalgebra::Point3;

pub struct Triangle {
    pub verts: [u32; 3],
    pub normal: u32,
    pub tag: u8,
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
    return relative_eq!(p0.x, p1.x) &&
           relative_eq!(p0.y, p1.y) &&
           relative_eq!(p0.z, p1.z);
}

impl Mesh {
    pub fn from_stl(stl: StlMesh, tag: u8) -> Result<Mesh> {
        let mut mesh = Mesh {
            verts: Vec::new(),
            normals: Vec::new(),
            tris: Vec::new()
        };
        for stl_tri in stl.tris {
            let tri = Triangle {
                normal: push_unique_point(&mut mesh.normals, stl_tri.normal),
                verts: [
                    push_unique_point(&mut mesh.verts, stl_tri.verts[0]),
                    push_unique_point(&mut mesh.verts, stl_tri.verts[1]),
                    push_unique_point(&mut mesh.verts, stl_tri.verts[2]),
                ],
                tag: tag,
            };
            mesh.tris.push(tri);
        }
        return Ok(mesh);
    }

    pub fn merge(mut self, other: &Mesh) -> Mesh {
        for tri0 in self.tris.iter() {
            for tri1 in other.tris.iter() {
                

            }
        }
        self
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use std::fs::File;

    #[test]
    fn test_use_it() {
        let mut fp = File::open("test_data/cube_scad.stl").unwrap();
        let stl = super::StlMesh::from_file(&mut fp).unwrap();
        let m = super::Mesh::from_stl(stl).unwrap();
        assert_eq!(m.normals.len(), 6);
        assert_eq!(m.verts.len(), 8);
        assert_eq!(m.tris.len(), 12);
    }
}
