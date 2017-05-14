mod errors {
    error_chain!{}
}

use errors::Result;
use geometry as geo;
use nalgebra::Point3;
use stl::StlMesh;

#[derive(Debug)]
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

    /// Extract vertex data from the mesh for fast local access for geometry tests.
    fn as_geometry(&self, mesh: &Mesh) -> geo::Triangle {
        geo::Triangle::new(&self.vert(mesh, 0),
                           &self.vert(mesh, 1),
                           &self.vert(mesh, 2))
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
    pub fn union_non_overlapping(&self, other: &Mesh) -> Result<Mesh> {
        for mesh_tri0 in self.tris.iter() {
            let geo_tri0 = mesh_tri0.as_geometry(&self);
            let pi0 = geo::Plane::from_triangle(&geo_tri0);
            for mesh_tri1 in other.tris.iter() {
                if relative_eq!(pi0.distance_to(&mesh_tri1.vert(other, 0)), 0.0f32) &&
                   relative_eq!(pi0.distance_to(&mesh_tri1.vert(other, 1)), 0.0f32) &&
                   relative_eq!(pi0.distance_to(&mesh_tri1.vert(other, 2)), 0.0f32) {
                    // Co-planar
                    let geo_tri1 = mesh_tri1.as_geometry(other).invert();
                    println!("Found coplanar faces: {:?} and {:?}", geo_tri0, geo_tri1);
                    let next_tri0 = geo_tri0.clip_with(&geo_tri1);
                    // Remove tri0 an tri1.
                    // Subtract tri1 from tri0 and re-insert all resulting tris into mesh0.
                    // Subtract tri0 from tri1 and re-insert all resulting tris into mesh1.
                }
            }
        }
        bail!("not implemented")
    }
}

#[cfg(test)]
mod test {
    use super::Mesh;
    use std::fs::File;

    #[test]
    fn test_simplify_mesh() {
        let mut fp = File::open("test_data/single_color/cube_scad.stl").unwrap();
        let stl = super::StlMesh::from_file(&mut fp).unwrap();
        let m = super::Mesh::from_stl(stl, 0).unwrap();
        assert_eq!(m.normals.len(), 6);
        assert_eq!(m.verts.len(), 8);
        assert_eq!(m.tris.len(), 12);
    }

    fn load_mesh(filename: &str, tag: u8) -> Mesh {
        let mut fp = File::open(filename).unwrap();
        let stl = super::StlMesh::from_file(&mut fp).unwrap();
        return Mesh::from_stl(stl, tag).unwrap();
    }

    #[test]
    fn test_union_nesting() {
        let m0 = load_mesh("test_data/2color/nesting/0.stl", 0);
        let m1 = load_mesh("test_data/2color/nesting/1.stl", 1);
        let _ = m0.union_non_overlapping(&m1);
    }
}
