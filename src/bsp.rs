use nalgebra::{normalize, Point3, Vector3};
use alga::general::Inverse;
use std::{mem, f32, u32};
use stl::{StlMesh, StlTriangle};
use itertools::Itertools;

#[derive(Clone, Debug)]
pub struct Vertex {
    position: Point3<f32>,
    normal: Vector3<f32>,
}

impl Vertex {
    pub fn new(position: &Point3<f32>, normal: &Vector3<f32>) -> Self {
        Vertex {
            position: position.clone(),
            normal: normal.clone(),
        }
    }

    pub fn invert(&mut self) {
        self.normal.inverse_mut();
    }
}

#[derive(Clone, Debug)]
pub struct Plane {
    normal: Vector3<f32>,
    d: f32,
}

#[derive(Debug)]
pub struct PlaneSplitResult {
    coplanar_front: Vec<Triangle>,
    coplanar_back: Vec<Triangle>,
    front: Vec<Triangle>,
    back: Vec<Triangle>,
}

// Bit flags for tracking orientation.
const COPLANAR: u8 = 0;
const FRONT: u8 = 1;
const BACK: u8 = 2;
const SPANNING: u8 = 3;
const EPSILON: f32 = 1e-5;

impl Plane {
    pub fn from_points(p0: &Point3<f32>, p1: &Point3<f32>, p2: &Point3<f32>) -> Self {
        let p01 = *p1 - *p0;
        let p02 = *p2 - *p0;
        let nv = normalize(&p01.cross(&p02));
        let d = -nv.dot(&p0.coords);
        assert!(nv[0] != f32::NAN);
        assert!(nv[1] != f32::NAN);
        assert!(nv[2] != f32::NAN);
        assert!(d != f32::NAN);
        Plane {
            normal: nv,
            d: d,
        }
    }

    fn distance_to(&self, p: &Point3<f32>) -> f32 {
        self.normal.dot(&p.coords) + self.d
    }

    pub fn invert(&mut self) {
        self.normal.inverse_mut();
        self.d = -self.d;
    }

    pub fn split_polygon(&self, triangle: Triangle, expect_coplanar: bool) -> PlaneSplitResult {
        let mut result = PlaneSplitResult {
            coplanar_front: Vec::new(),
            coplanar_back: Vec::new(),
            front: Vec::new(),
            back: Vec::new(),
        };
        if expect_coplanar {
            println!("self: {:?}", self);
        }

        let mut polygon_type = 0u8;
        let mut vertex_types = Vec::<u8>::new();
        for (i, vertex) in triangle.vertices.iter().enumerate() {
            let t = self.distance_to(&vertex.position);
            if expect_coplanar && !relative_eq!(t, 0f32) {
                println!("plane: {:?}", self);
                println!("v{}: {}: {:?}", i, t, vertex.position.coords);
            }
            let ty = if t < -EPSILON {
                BACK
            } else if t > EPSILON {
                FRONT
            } else {
                COPLANAR
            };
            polygon_type |= ty;
            vertex_types.push(ty);
        }

        match polygon_type {
            COPLANAR => {
                if self.normal.dot(&triangle.plane.normal) > 0f32 {
                    result.coplanar_front.push(triangle);
                } else {
                    result.coplanar_back.push(triangle);
                }
            }
            FRONT => {
                result.front.push(triangle);
            }
            BACK => {
                result.back.push(triangle);
            }
            SPANNING => {
                self.split_spanning_polygon(vertex_types, triangle, &mut result.front, &mut result.back);
                /*
                let (f, b) = self.split_spanning_polygon(vertex_types, triangle);
                if f.is_some() {
                    result.front.push(f.unwrap());
                }
                if b.is_some() {
                    result.back.push(b.unwrap());
                }
                */
            }
            _ => panic!("impossible polygon type when splitting"),
        }

        return result;
    }

    fn split_spanning_polygon(&self,
                              vertex_types: Vec<u8>,
                              polygon: Triangle,
                              front_tris: &mut Vec<Triangle>,
                              back_tris: &mut Vec<Triangle>) {
        assert!(polygon.vertices.len() > 2);
        assert!(vertex_types.len() == polygon.vertices.len());

        let mut front = Vec::<Vertex>::new();
        let mut back = Vec::<Vertex>::new();
        for i in 0..polygon.vertices.len() {
            let ref vi = polygon.vertices[i];
            let ti = vertex_types[i];

            match ti {
                FRONT => front.push(vi.clone()),
                BACK => back.push(vi.clone()),
                COPLANAR => {
                    back.push(vi.clone());
                    front.push(vi.clone());
                }
                _ => panic!("impossible vertex type when splitting spanning poly"),
            }

            let tj = vertex_types[(i + 1) % vertex_types.len()];
            if (ti | tj) == SPANNING {
                let ref vj = polygon.vertices[(i + 1) % polygon.vertices.len()];
                let i2j = vj.position - vi.position;
                let t = -self.distance_to(&vi.position) / self.normal.dot(&i2j);

                // Note that this split quantizes the line's position where it
                // crosses the split plane to a resolution of f32.
                let v_split = Vertex {
                    position: vi.position + (i2j * t),
                    normal: vi.normal,
                };

                back.push(v_split.clone());
                front.push(v_split);
            }
        }

        self._build_tris_for_split(front, front_tris, polygon.attribute);
        self._build_tris_for_split(back, back_tris, polygon.attribute);
    }

    fn _build_tris_for_split(&self, mut verts: Vec<Vertex>, tris: &mut Vec<Triangle>, attr: u8) {
        // The above algorithm may create a degenerate polygon if one point or edge
        // is co-planar with the split plane, so we ignore the 1 and 2 cases.
        if verts.len() == 3 {
            if let Some((a, b, c)) = verts.into_iter().tuples().next() {
                tris.push(Triangle::from_vertices(a, b, c, attr));
            }
        } else if verts.len() == 4 {
            // Quantization may throw our points out of co-planarity, so we always
            // reduce to triangles, even though this results in more intermediate objects.
            if let Some((a, b, c, d)) = verts.into_iter().tuples().next() {
                tris.push(Triangle::from_vertices(a.clone(), b, c.clone(), attr));
                tris.push(Triangle::from_vertices(a, c, d, attr));
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

    fn new_raw2(plane: &Plane, vertices: Vec<Vertex>, attr: u8) -> (bool, Self) {
        let mut different = false;
        let new_plane = Plane::from_points(&vertices[0].position, &vertices[1].position, &vertices[2].position);
        for i in 0..vertices.len() - 3 {
            let tmp = Plane::from_points(&vertices[i + 0].position,
                                         &vertices[i + 1].position,
                                         &vertices[i + 2].position);
            if plane.normal != tmp.normal || plane.d != tmp.d {
                println!("At i{}", i);
            }
            assert_eq!(new_plane.normal, tmp.normal);
            assert_eq!(new_plane.d, tmp.d);
        }
        (different, Triangle {
            plane: new_plane,
            vertices: vertices,
            attribute: attr,
        })
    }

    fn new_raw(plane: &Plane, v0: Vertex, v1: Vertex, v2: Vertex, attr: u8) -> Self {
        let tmp = Plane::from_points(&v0.position, &v1.position, &v2.position);
        assert_eq!(plane.normal, tmp.normal);
        assert_eq!(plane.d, tmp.d);
        Triangle {
            plane: plane.clone(),
            vertices: vec![v0, v1, v2],
            attribute: attr,
        }
    }

    pub fn from_vertices(v0: Vertex, v1: Vertex, v2: Vertex, attr: u8) -> Self {
        let plane = Plane::from_points(&v0.position, &v1.position, &v2.position);
        return Triangle {
            plane: plane,
            vertices: vec![v0, v1, v2],
            attribute: attr,
        };
    }

    pub fn from_points(p0: &Point3<f32>, p1: &Point3<f32>, p2: &Point3<f32>, attr: u8) -> Self {
        let plane = Plane::from_points(p0, p1, p2);
        let vertices = vec![Vertex::new(p0, &plane.normal),
                            Vertex::new(p1, &plane.normal),
                            Vertex::new(p2, &plane.normal)];
        return Triangle {
                   plane: plane,
                   vertices: vertices,
                   attribute: 0,
               };
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

    /*
    pub fn new_pair_from_quad(v0: Vertex, v1: Vertex, v2: Vertex, v3: Vertex, attr: u8) -> (Self, Self) {
        let l02 = (v0.position - v2.position).norm_squared();
        let l13 = (v1.position - v3.position).norm_squared();
        let a0;
        let a1;
        let a2;
        let b0;
        let b1;
        let b2;
        if l02 < l13 {
            a0 = v0.clone();
            a1 = v1;
            a2 = v2.clone();
            b0 = v0;
            b1 = v2;
            b2 = v3;
            return (Triangle::from_vertices(
                                      a0,
                                      a1,
                                      a2,
                                      attr),
                    Triangle::from_vertices(
                                      b0,
                                      b1,
                                      b2,
                                      attr));
        } else {
            a0 = v0;
            a1 = v1.clone();
            a2 = v3.clone();
            b0 = v1;
            b1 = v2;
            b2 = v3;
            return (Triangle::new_raw(&Plane::from_points(&a0.position, &a1.position, &a2.position),
                                      a0,
                                      a1,
                                      a2,
                                      attr),
                    Triangle::new_raw(&Plane::from_points(&b0.position, &b1.position, &b2.position),
                                      b0,
                                      b1,
                                      b2,
                                      attr));
        }
    }
    */

    pub fn to_triangles(&self) -> Vec<Triangle> {
        assert!(self.vertices.len() > 2);
        if self.vertices.len() == 3 {
            return vec![self.clone()];
        }
        if self.vertices.len() == 4 {
            let l02 = (self.vertices[0].position - self.vertices[2].position).norm_squared();
            let l13 = (self.vertices[1].position - self.vertices[3].position).norm_squared();
            if l02 < l13 {
                return vec![Triangle::new_raw(&self.plane,
                                              self.vertices[0].clone(),
                                              self.vertices[1].clone(),
                                              self.vertices[2].clone(),
                                              self.attribute),
                            Triangle::new_raw(&self.plane,
                                              self.vertices[0].clone(),
                                              self.vertices[2].clone(),
                                              self.vertices[3].clone(),
                                              self.attribute)];
            }
            return vec![Triangle::new_raw(&self.plane,
                                          self.vertices[0].clone(),
                                          self.vertices[1].clone(),
                                          self.vertices[3].clone(),
                                          self.attribute),
                        Triangle::new_raw(&self.plane,
                                          self.vertices[1].clone(),
                                          self.vertices[2].clone(),
                                          self.vertices[3].clone(),
                                          self.attribute)];
        }
        panic!("triangle with more than 4 vertices");
    }
}

pub struct BspTree {
    nodes: Vec<BspNode>,
}

pub struct BspNode {
    plane: Option<Plane>,
    front: Option<BspNodeId>,
    back: Option<BspNodeId>,
    polygons: Vec<Triangle>,
}

#[derive(Clone)]
pub struct BspNodeId {
    index: usize,
}

impl BspTree {
    pub fn new_from_stl(stl: &StlMesh, attr: u8) -> BspTree {
        // TODO: this recomputes the normal. Can we take it safely from
        // the STL file, or is that going to result in havoc down the road?
        // We need to time this phase on a heavy file to find out.
        let mut polygons = vec![];
        for t in stl.tris.iter() {
            let mut p = Triangle::from_points(&t.verts[0], &t.verts[1], &t.verts[2], 0);
            p.set_attribute(attr);
            /*
            let p0 = &t.verts[1];
            let p1 = &t.verts[2];
            let p2 = &t.verts[0];
            let nvn = (*p1 - *p0).cross(&(*p2 - *p0)).normalize();
            let normed = t.normal.clone();
            //normed.normalize();
            println!("{} == {} == {}", p.plane.normal[0], t.normal.coords[0], nvn[0]);
            println!("{} == {} == {}", p.plane.normal[1], t.normal.coords[1], nvn[1]);
            println!("{} == {} == {}", p.plane.normal[2], t.normal.coords[2], nvn[2]);
            println!("{}; {}; {}", p0.coords, p1.coords, p2.coords);
            assert!(relative_eq!(p.plane.normal[0], t.normal.coords[0]));
            assert!(relative_eq!(p.plane.normal[1], t.normal.coords[1]));
            assert!(relative_eq!(p.plane.normal[2], t.normal.coords[2]));
            */
            polygons.push(p);
        }
        let mut bsp = Self::new();
        bsp.get_root().add_polygons(polygons, &mut bsp);
        return bsp;
    }

    pub fn convert_to_stl(&self, name: &str) -> StlMesh {
        let mut tris: Vec<StlTriangle> = Vec::new();
        for poly in self.get_polygons() {
            for t in poly.to_triangles() {
                let mut u = StlTriangle::new(t.vertices[0].position,
                                             t.vertices[1].position,
                                             t.vertices[2].position,
                                             Point3::from_coordinates(t.plane.normal));
                match t.attribute {
                    1 => u.set_color(0, 97, 207),
                    2 => u.set_color(0, 207, 97),
                    _ => u.set_color(255, 0, 255)
                }
                tris.push(u);
            }
        }
        return StlMesh::from_tris(name, tris);
    }

    pub fn new() -> Self {
        let mut tree = BspTree { nodes: Vec::new() };
        tree.create_node();
        return tree;
    }

    pub fn get_root(&self) -> BspNodeId {
        assert!(!self.nodes.is_empty());
        return BspNodeId { index: 0 };
    }

    pub fn invert(&mut self) {
        self.get_root().invert(self);
    }

    pub fn union_with(&mut self, mut other: BspTree) {
        self.clip_to(&mut other);
        other.clip_to(self);
        other.invert();
        other.clip_to(self);
        other.invert();
        self.get_root().add_polygons(other.get_polygons(), self);
    }

    pub fn get_polygons(&self) -> Vec<Triangle> {
        let mut out = Vec::new();
        self.get_root().get_polygons(&mut out, self);
        return out;
    }

    fn clip_to(&mut self, clip: &mut BspTree) {
        self.get_root().clip_to(&clip.get_root(), clip, self);
    }

    fn create_node(&mut self) -> BspNodeId {
        let next_index = self.nodes.len();
        self.nodes
            .push(BspNode {
                      plane: None,
                      front: None,
                      back: None,
                      polygons: Vec::new(),
                  });
        return BspNodeId { index: next_index };
    }
}

impl BspNodeId {
    pub fn get_polygons(&self, polygons: &mut Vec<Triangle>, arena: &BspTree) {
        {
            let self_borrow = arena.nodes.get(self.index).expect("unknown id");
            let mut tmp = self_borrow.polygons.clone();
            polygons.append(&mut tmp);
        }

        let (front_nodeid_opt, back_nodeid_opt) = self.get_front_and_back(arena);
        if let Some(front_nodeid) = front_nodeid_opt {
            front_nodeid.get_polygons(polygons, arena);
        }
        if let Some(back_nodeid) = back_nodeid_opt {
            back_nodeid.get_polygons(polygons, arena);
        }
    }

    pub fn invert(&self, arena: &mut BspTree) {
        {
            let self_borrow = arena.nodes.get_mut(self.index).expect("unknown id");
            if let Some(plane) = self_borrow.plane.as_mut() {
                plane.invert();
            }
            for poly in self_borrow.polygons.iter_mut() {
                poly.invert();
            }
        }

        let (front_nodeid_opt, back_nodeid_opt) = self.get_front_and_back(arena);
        if let Some(front_nodeid) = front_nodeid_opt {
            front_nodeid.invert(arena);
        }
        if let Some(back_nodeid) = back_nodeid_opt {
            back_nodeid.invert(arena);
        }

        let (front_nodeid_opt, back_nodeid_opt) = self.get_front_and_back(arena);
        let self_borrow = arena.nodes.get_mut(self.index).expect("unknown id");
        self_borrow.front = back_nodeid_opt;
        self_borrow.back = front_nodeid_opt;
    }

    pub fn add_polygons(&self, mut polygons: Vec<Triangle>, arena: &mut BspTree) {
        println!("adding {} polys", polygons.len());
        if polygons.is_empty() {
            return;
        }

        // Borrow arena to use self's node to split the polygons. Add any co-planar
        // polygons to our list and split the rest into front and back lists.
        let mut front = Vec::<Triangle>::new();
        let mut back = Vec::<Triangle>::new();
        {
            let self_borrow = arena.nodes.get_mut(self.index).expect("unknown id");

            let mut using_poly0_plane = false;
            if self_borrow.plane.is_none() {
                // TODO: find and use a good heuristic for our initial split plane.
                //self_borrow.plane = Some(polygons[0].plane.clone());
                self_borrow.plane = Some(Plane::from_points(
                    &polygons[0].vertices[0].position,
                    &polygons[0].vertices[1].position,
                    &polygons[0].vertices[2].position));
                using_poly0_plane = true;
                let test_plane = Plane::from_points(
                    &polygons[0].vertices[0].position,
                    &polygons[0].vertices[1].position,
                    &polygons[0].vertices[2].position,
                );
                assert_eq!(test_plane.normal, polygons[0].plane.normal);
                assert_eq!(test_plane.d, polygons[0].plane.d);

                println!("test plane: {:?}", test_plane);
                println!("poly plane: {:?}", polygons[0].plane);

                let dt0 = test_plane.distance_to(&polygons[0].vertices[0].position);
                let dt1 = test_plane.distance_to(&polygons[0].vertices[1].position);
                let dt2 = test_plane.distance_to(&polygons[0].vertices[2].position);
                println!("distance to test plane: {}, {}, {}", dt0, dt1, dt2);

                let dv0 = polygons[0].plane.distance_to(&polygons[0].vertices[0].position);
                let dv1 = polygons[0].plane.distance_to(&polygons[0].vertices[1].position);
                let dv2 = polygons[0].plane.distance_to(&polygons[0].vertices[2].position);
                println!("disvance to poly plane: {}, {}, {}", dv0, dv1, dv2);

                println!("test plane: {:?}", test_plane);
                println!("poly plane: {:?}", polygons[0].plane);
            }

            let plane: &Plane = self_borrow.plane.as_ref().expect("not none");
            for poly in polygons.drain(..) {
                let mut result = plane.split_polygon(poly, using_poly0_plane);
                if using_poly0_plane {
                    using_poly0_plane = false;
                    assert_eq!(result.front.len(), 0);
                    assert_eq!(result.back.len(), 0); // FAILING HERE!
                    assert!(result.coplanar_front.len() + result.coplanar_back.len() > 0);
                }
                front.append(&mut result.front);
                back.append(&mut result.back);
                self_borrow.polygons.append(&mut result.coplanar_front);
                self_borrow.polygons.append(&mut result.coplanar_back);
            }
        }
        println!("split polys: f: {}, b: {}", front.len(), back.len());

        // Get or create new front and back nodes as needed.
        let (mut front_nodeid_opt, mut back_nodeid_opt) = self.get_front_and_back(arena);
        if front_nodeid_opt.is_none() && front.len() > 0 {
            front_nodeid_opt = Some(arena.create_node());
        }
        if back_nodeid_opt.is_none() && back.len() > 0 {
            back_nodeid_opt = Some(arena.create_node());
        }
        if let Some(ref mut front_nodeid) = front_nodeid_opt {
            front_nodeid.add_polygons(front, arena);
        }
        if let Some(ref mut back_nodeid) = back_nodeid_opt {
            back_nodeid.add_polygons(back, arena);
        }

        // Re-set the node links (under a new borrow) in case we had to create them above.
        {
            let self_borrow = arena.nodes.get_mut(self.index).expect("unknown id");
            self_borrow.front = front_nodeid_opt;
            self_borrow.back = back_nodeid_opt;
        }
    }

    fn clip_to(&self, clip: &BspNodeId, clip_arena: &BspTree, arena: &mut BspTree) {
        // Call clipPolygons on `clip` with `self.polygons`, to remove any parts of
        // them that are in the other BSP's internal volume.
        let mut polygons;
        {
            let self_borrow = arena.nodes.get_mut(self.index).expect("unknown id");
            polygons = mem::replace(&mut self_borrow.polygons, Vec::new());
        }
        {
            polygons = clip.clip_polygons(polygons, clip_arena);
        }
        {
            let self_borrow = arena.nodes.get_mut(self.index).expect("unknown id");
            self_borrow.polygons = polygons;
        }

        let (front_nodeid_opt, back_nodeid_opt) = self.get_front_and_back(arena);
        if let Some(front_nodeid) = front_nodeid_opt {
            front_nodeid.clip_to(clip, clip_arena, arena);
        }
        if let Some(back_nodeid) = back_nodeid_opt {
            back_nodeid.clip_to(clip, clip_arena, arena);
        }
    }

    // Remove any members of `polygons`, or parts of `polygons` that are inside this volume.
    fn clip_polygons(&self,
                     mut polygons: Vec<Triangle>,
                     arena: &BspTree)
                     -> Vec<Triangle> {
        if polygons.len() == 0 {
            return Vec::new();
        }

        let mut front: Vec<Triangle> = Vec::new();
        let mut back: Vec<Triangle> = Vec::new();
        {
            let self_borrow = arena.nodes.get(self.index).expect("unknown id");
            if self_borrow.plane.is_none() {
                return Vec::new();
            }

            for polygon in polygons.drain(..) {
                let mut result = self_borrow
                    .plane
                    .as_ref()
                    .expect("not none")
                    .split_polygon(polygon, false);
                front.append(&mut result.front);
                front.append(&mut result.coplanar_front);
                back.append(&mut result.back);
                back.append(&mut result.coplanar_back);
            }
        }

        let (front_nodeid_opt, back_nodeid_opt) = self.get_front_and_back(arena);
        if let Some(front_nodeid) = front_nodeid_opt {
            front = front_nodeid.clip_polygons(front, arena);
        }
        if let Some(back_nodeid) = back_nodeid_opt {
            back = back_nodeid.clip_polygons(back, arena);
        } else {
            back = Vec::new();
        }

        front.append(&mut back);
        return front;
    }

    fn get_front_and_back(&self, arena: &BspTree) -> (Option<BspNodeId>, Option<BspNodeId>) {
        let mut front_nodeid_opt = None;
        let mut back_nodeid_opt = None;
        let self_borrow = arena.nodes.get(self.index).expect("unknown id");
        if let Some(f) = self_borrow.front.as_ref() {
            front_nodeid_opt = Some(BspNodeId { index: f.index });
        }
        if let Some(f) = self_borrow.back.as_ref() {
            back_nodeid_opt = Some(BspNodeId { index: f.index });
        }
        return (front_nodeid_opt, back_nodeid_opt);
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_plane_distance_orthogonal_origin() {
        let p0 = Point3::new(0f32, 0f32, 0f32);
        let p1 = Point3::new(1f32, 0f32, 0f32);
        let p2 = Point3::new(0f32, 1f32, 0f32);
        let plane = Plane::from_points(&p0, &p1, &p2);
        let p = Point3::new(0f32, 0f32, 1f32);
        assert_eq!(1.0f32, plane.distance_to(&p));
    }

    #[test]
    fn test_plane_split_middle() {
        let plane = Plane::from_points(&Point3::new(0f32, 0f32, 0f32),
                                       &Point3::new(1f32, 0f32, 0f32),
                                       &Point3::new(0f32, 1f32, 0f32));
        let poly = Triangle::from_points(&Point3::new(0f32, 0f32, -1f32),
                                         &Point3::new(0f32, 0f32, 1f32),
                                         &Point3::new(0f32, 1f32, 0f32), 0);
        let result = plane.split_polygon(poly, false);
        assert_eq!(result.front.len(), 1);
        assert_eq!(result.front[0].vertices.len(), 3);
        assert_eq!(result.back.len(), 1);
        assert_eq!(result.back[0].vertices.len(), 3);
        assert_eq!(result.coplanar_front.len(), 0);
        assert_eq!(result.coplanar_back.len(), 0);
        assert!(result.front[0].vertices[0].position == Point3::new(0f32, 0f32, 0f32));
        assert!(result.front[0].vertices[1].position == Point3::new(0f32, 0f32, 1f32));
        assert!(result.front[0].vertices[2].position == Point3::new(0f32, 1f32, 0f32));
        assert!(result.back[0].vertices[0].position == Point3::new(0f32, 0f32, -1f32));
        assert!(result.back[0].vertices[1].position == Point3::new(0f32, 0f32, 0f32));
        assert!(result.back[0].vertices[2].position == Point3::new(0f32, 1f32, 0f32));
    }

    #[test]
    fn test_plane_split_coplanar_front() {
        let plane = Plane::from_points(&Point3::new(0f32, 0f32, 0f32),
                                       &Point3::new(1f32, 0f32, 0f32),
                                       &Point3::new(0f32, 1f32, 0f32));
        let poly = Triangle::from_points(&Point3::new(0f32, 0f32, 0f32),
                                         &Point3::new(1f32, 0f32, 0f32),
                                         &Point3::new(0f32, 1f32, 0f32), 0);
        let result = plane.split_polygon(poly, false);
        assert_eq!(result.front.len(), 0);
        assert_eq!(result.back.len(), 0);
        assert_eq!(result.coplanar_front.len(), 1);
        assert_eq!(result.coplanar_front[0].vertices.len(), 3);
        assert_eq!(result.coplanar_back.len(), 0);
        assert!(result.coplanar_front[0].vertices[0].position == Point3::new(0f32, 0f32, 0f32));
        assert!(result.coplanar_front[0].vertices[1].position == Point3::new(1f32, 0f32, 0f32));
        assert!(result.coplanar_front[0].vertices[2].position == Point3::new(0f32, 1f32, 0f32));
    }

    #[test]
    fn test_plane_split_coplanar_back() {
        let plane = Plane::from_points(&Point3::new(0f32, 0f32, 0f32),
                                       &Point3::new(1f32, 0f32, 0f32),
                                       &Point3::new(0f32, 1f32, 0f32));
        let poly = Triangle::from_points(&Point3::new(0f32, 1f32, 0f32),
                                         &Point3::new(1f32, 0f32, 0f32),
                                         &Point3::new(0f32, 0f32, 0f32), 0);
        let result = plane.split_polygon(poly, false);
        assert_eq!(result.front.len(), 0);
        assert_eq!(result.back.len(), 0);
        assert_eq!(result.coplanar_front.len(), 0);
        assert_eq!(result.coplanar_back.len(), 1);
        assert_eq!(result.coplanar_back[0].vertices.len(), 3);
        assert!(result.coplanar_back[0].vertices[0].position == Point3::new(0f32, 1f32, 0f32));
        assert!(result.coplanar_back[0].vertices[1].position == Point3::new(1f32, 0f32, 0f32));
        assert!(result.coplanar_back[0].vertices[2].position == Point3::new(0f32, 0f32, 0f32));
    }

    use std::fs::File;
    use stl::{StlTriangle, StlMesh};

    fn load_cube_text() -> StlMesh {
        let mut fp = File::open("test_data/single_color/cube_text.stl").unwrap();
        let m = StlMesh::from_file(&mut fp).unwrap();
        assert_eq!(m.tris.len(), 12);
        assert_eq!(m.radius(), 3f32.sqrt());
        return m;
    }

    fn save_polygons(polygons: &Vec<Triangle>, name: &str) {
        let mut tris: Vec<StlTriangle> = Vec::new();
        for poly in polygons {
            for t in poly.to_triangles() {
                let mut u = StlTriangle::new(t.vertices[0].position,
                                             t.vertices[1].position,
                                             t.vertices[2].position,
                                             Point3::from_coordinates(t.plane.normal));
                match t.attribute {
                    1 => u.set_color(0, 97, 207),
                    2 => u.set_color(0, 207, 97),
                    _ => u.set_color(255, 0, 255)
                }
                tris.push(u);
            }
        }
        let stl = StlMesh::from_tris(name, tris);
        let mut fp = File::create(format!("target/{}.stl", name)).unwrap();
        stl.to_binary_file(&mut fp).unwrap();
    }

    fn save_bsp(bsp: &BspTree, name: &str) {
        let polys = bsp.get_polygons();
        save_polygons(&polys, name);
    }

    #[test]
    fn test_bsp_create() {
        let bsp = BspTree::new_from_stl(&load_cube_text(), 1);
        save_bsp(&bsp, "test_bsp_create");
    }

    #[test]
    fn test_bsp_clip_to() {
        let mut bsp = BspTree::new_from_stl(&load_cube_text(), 1);

        let mut polygons1 = Vec::<Triangle>::new();
        polygons1.push(Triangle::from_points(&Point3::new(10f32, 0.5f32, 10f32),
                                             &Point3::new(10f32, 0.5f32, -10f32),
                                             &Point3::new(-10f32, 0.5f32, -10f32), 0));
        polygons1.push(Triangle::from_points(&Point3::new(10f32, 0.5f32, 10f32),
                                             &Point3::new(-10f32, 0.5f32, -10f32),
                                             &Point3::new(-10f32, 0.5f32, 10f32), 0));
        let mut bsp_offset = BspTree::new();
        bsp_offset
            .get_root()
            .add_polygons(polygons1, &mut bsp_offset);

        bsp.clip_to(&mut bsp_offset);

        save_bsp(&bsp, "test_bsp_clip_to");
    }

    #[test]
    fn test_bsp_inverted_clip_to() {
        let mut bsp = BspTree::new_from_stl(&load_cube_text(), 1);

        let mut polygons1 = Vec::<Triangle>::new();
        polygons1.push(Triangle::from_points(&Point3::new(10f32, 0.5f32, 10f32),
                                             &Point3::new(10f32, 0.5f32, -10f32),
                                             &Point3::new(-10f32, 0.5f32, -10f32), 0));
        polygons1.push(Triangle::from_points(&Point3::new(10f32, 0.5f32, 10f32),
                                             &Point3::new(-10f32, 0.5f32, -10f32),
                                             &Point3::new(-10f32, 0.5f32, 10f32), 0));
        let mut bsp_offset = BspTree::new();
        bsp_offset
            .get_root()
            .add_polygons(polygons1, &mut bsp_offset);

        bsp_offset.invert();
        bsp.clip_to(&mut bsp_offset);

        save_bsp(&bsp, "test_bsp_invert_clip_to");
    }

    #[test]
    fn test_bsp_union() {
        let mut bsp0 = BspTree::new_from_stl(&load_cube_text(), 1);

        let m = load_cube_text();
        let mut polygons0 = Vec::<Triangle>::new();
        for t in m.tris.iter() {
            let mut v0 = t.verts[0].clone();
            let mut v1 = t.verts[1].clone();
            let mut v2 = t.verts[2].clone();
            for i in 0..3 {
                v0.coords.as_mut_slice()[i] -= 0.5f32;
                v1.coords.as_mut_slice()[i] -= 0.5f32;
                v2.coords.as_mut_slice()[i] -= 0.5f32;
            }
            let mut poly = Triangle::from_points(&v0, &v1, &v2, 2);
            polygons0.push(poly);
        }
        let mut bsp1 = BspTree::new();
        bsp1.get_root().add_polygons(polygons0, &mut bsp1);

        bsp0.union_with(bsp1);
        save_bsp(&bsp0, "test_bsp_union");
    }
}
