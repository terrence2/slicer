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

fn relative_eq(p0: &Point3<f32>, p1: &Point3<f32>) -> bool {
    return relative_eq!(p0.x, p1.x) && relative_eq!(p0.y, p1.y) && relative_eq!(p0.z, p1.z);
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
}
