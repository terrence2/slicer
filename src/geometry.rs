mod errors { error_chain! {} }

use errors::{Result, ResultExt};
use nalgebra::{Point3, Vector3};

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
