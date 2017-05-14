mod errors {
    error_chain!{}
}

use nalgebra::{distance, Point3, Vector3};
use std::f32;

pub fn min_f32(a: f32, b: f32) -> f32 {
    if a < b { a } else { b }
}

pub fn max_f32(a: f32, b: f32) -> f32 {
    if a > b { a } else { b }
}

pub struct Plane {
    normal: Vector3<f32>,
    d: f32,
}

impl Plane {
    pub fn from_triangle(p0: &Point3<f32>, p1: &Point3<f32>, p2: &Point3<f32>) -> Self {
        let nv = (*p1 - *p0).cross(&(*p2 - *p0));
        Plane {
            normal: nv,
            d: -nv.dot(&p0.coords),
        }
    }

    pub fn distance_to(&self, p: &Point3<f32>) -> f32 {
        self.normal.dot(&p.coords) + self.d
    }
}

pub struct Triangle {
    verts: [Point3<f32>; 3],
}

#[derive(Debug)]
pub struct PointIntersection {
    pub nearest_point_in: Point3<f32>,
    pub distance: f32,
    pub intersects: bool,
}

impl Triangle {
    pub fn new(p0: &Point3<f32>, p1: &Point3<f32>, p2: &Point3<f32>) -> Triangle {
        Triangle { verts: [p0.clone(), p1.clone(), p2.clone()] }
    }

    pub fn intersect_with_point(&self, p: &Point3<f32>) -> PointIntersection {
        let mut out = PointIntersection {
            nearest_point_in: Point3::origin(),
            distance: f32::MAX,
            intersects: false,
        };
        for i in 0..3 {
            let a = self.verts[i];
            let b = self.verts[(i + 1) % 3];

            let len2 = (b - a).norm_squared();
            let t = (p - a).dot(&(b - a)) / len2;
            let t_clamp = max_f32(0f32, min_f32(1f32, t));
            let nearest = a + (t_clamp * (b - a));
            let dist = distance(p, &nearest);
            if dist < out.distance {
                out.distance = dist;
                let norm = (b - a).cross(&(p - a));
                out.intersects = norm.z <= 0f32 && relative_eq!(t, t_clamp);
                out.nearest_point_in = nearest;
            }
        }

        if out.intersects {
            out.distance = 0f32;
            out.nearest_point_in = p.clone();
        }
        println!("OUT: {:?}", out);
        return out;
    }

    pub fn clip_with(&mut self, clip: &Triangle) {
        // Check if all points in clip are inside self.
        //   foreach vert in clip, add edge to two nearest verts in self.
        for clip_vert in clip.verts.iter() {
            let _ = self.intersect_with_point(clip_vert);
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_plane_distance() {
        let p0 = Point3::new(0f32, 0f32, 0f32);
        let p1 = Point3::new(1f32, 0f32, 0f32);
        let p2 = Point3::new(0f32, 1f32, 0f32);
        let plane = Plane::from_triangle(&p0, &p1, &p2);
        let p = Point3::new(0f32, 0f32, 1f32);
        assert_eq!(1.0f32, plane.distance_to(&p));
    }

    fn assert_point_inside_tri(tri: &Triangle, p: &Point3<f32>) {
        let inside = tri.intersect_with_point(p);
        assert_eq!(inside.intersects, true);
        assert_eq!(inside.distance, 0f32);
        assert_eq!(inside.nearest_point_in, *p)
    }

    fn assert_point_outside_tri(tri: &Triangle, p: &Point3<f32>, nearest: &Point3<f32>) {
        let result = tri.intersect_with_point(p);
        assert_eq!(result.intersects, false);
        assert_eq!(result.nearest_point_in, *nearest);
        assert_eq!(result.distance, (result.nearest_point_in - *p).norm());
        assert_eq!(result.distance, (*nearest - *p).norm());
    }

    #[test]
    fn test_triangle_point_intersection() {
        let tri = Triangle::new(&Point3::new(0f32, 0f32, 0f32),
                                &Point3::new(0f32, 1f32, 0f32),
                                &Point3::new(1f32, 0f32, 0f32));
        assert_point_inside_tri(&tri, &Point3::new(0.1f32, 0.1f32, 0f32));
        assert_point_inside_tri(&tri, &Point3::new(0.5f32, 0f32, 0f32));
        assert_point_inside_tri(&tri, &Point3::new(0f32, 0.5f32, 0f32));
        assert_point_inside_tri(&tri, &Point3::new(0f32, 0f32, 0f32));
        assert_point_inside_tri(&tri, &Point3::new(1f32, 0f32, 0f32));
        assert_point_inside_tri(&tri, &Point3::new(0f32, 1f32, 0f32));

        assert_point_outside_tri(&tri,
                                 &Point3::new(-1f32, 0.5f32, 0f32),
                                 &Point3::new(0f32, 0.5f32, 0f32));
    }
}
