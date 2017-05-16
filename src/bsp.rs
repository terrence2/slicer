use nalgebra::{Point3, Vector3};
use alga::general::Inverse;
use std::{mem, u32};

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
    coplanar_front: Vec<ConvexPolygon>,
    coplanar_back: Vec<ConvexPolygon>,
    front: Vec<ConvexPolygon>,
    back: Vec<ConvexPolygon>,
}

// Bit flags for tracking orientation.
const COPLANAR: u8 = 0;
const FRONT: u8 = 1;
const BACK: u8 = 2;
const SPANNING: u8 = 3;

impl Plane {
    pub fn from_points(p0: &Point3<f32>, p1: &Point3<f32>, p2: &Point3<f32>) -> Self {
        let nv = (*p1 - *p0).cross(&(*p2 - *p0)).normalize();
        Plane {
            normal: nv,
            d: -nv.dot(&p0.coords),
        }
    }

    pub fn invert(&mut self) {
        self.normal.inverse_mut();
        self.d = -self.d;
    }

    pub fn split_polygon(&self, polygon: ConvexPolygon) -> PlaneSplitResult {
        let mut result = PlaneSplitResult {
            coplanar_front: Vec::new(),
            coplanar_back: Vec::new(),
            front: Vec::new(),
            back: Vec::new(),
        };

        let mut polygon_type = 0u8;
        let mut vertex_types = Vec::<u8>::new();
        for vertex in polygon.vertices.iter() {
            let t = self.distance_to(&vertex.position);
            let ty = if relative_eq!(t, 0f32) {
                COPLANAR
            } else if t < 0f32 {
                BACK
            } else {
                FRONT
            };
            polygon_type |= ty;
            vertex_types.push(ty);
        }

        match polygon_type {
            COPLANAR => {
                if self.normal.dot(&polygon.plane.normal) > 0f32 {
                    result.coplanar_front.push(polygon);
                } else {
                    result.coplanar_back.push(polygon);
                }
            }
            FRONT => {
                result.front.push(polygon);
            }
            BACK => {
                result.back.push(polygon);
            }
            SPANNING => {
                let (f, b) = self.split_spanning_polygon(vertex_types, polygon);
                if f.is_some() {
                    result.front.push(f.unwrap());
                }
                if b.is_some() {
                    result.back.push(b.unwrap());
                }
            }
            _ => panic!("impossible polygon type when splitting"),
        }

        return result;
    }

    fn split_spanning_polygon(&self,
                              vertex_types: Vec<u8>,
                              polygon: ConvexPolygon)
                              -> (Option<ConvexPolygon>, Option<ConvexPolygon>) {
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
                let v_split = Vertex {
                    position: vi.position + (i2j * t),
                    normal: vi.normal,
                };
                back.push(v_split.clone());
                front.push(v_split);
            }
        }

        // Note that we may create a degenerate polygon if one edge is co-planar.
        let f = if front.len() > 2 {
            Some(ConvexPolygon::new_raw(&polygon.plane, front))
        } else {
            None
        };
        let b = if back.len() > 2 {
            Some(ConvexPolygon::new_raw(&polygon.plane, back))
        } else {
            None
        };
        return (f, b);
    }

    fn distance_to(&self, p: &Point3<f32>) -> f32 {
        self.normal.dot(&p.coords) + self.d
    }
}

#[derive(Clone, Debug)]
pub struct ConvexPolygon {
    plane: Plane,
    vertices: Vec<Vertex>,
}

impl ConvexPolygon {
    fn new_raw(plane: &Plane, vertices: Vec<Vertex>) -> ConvexPolygon {
        ConvexPolygon {
            plane: plane.clone(),
            vertices: vertices,
        }
    }

    pub fn from_points(p0: &Point3<f32>, p1: &Point3<f32>, p2: &Point3<f32>) -> Self {
        let plane = Plane::from_points(p0, p1, p2);
        let verts = vec![Vertex::new(p0, &plane.normal),
                         Vertex::new(p1, &plane.normal),
                         Vertex::new(p2, &plane.normal)];
        return ConvexPolygon {
                   plane: plane,
                   vertices: verts,
               };
    }

    pub fn from_points4(p0: &Point3<f32>,
                        p1: &Point3<f32>,
                        p2: &Point3<f32>,
                        p3: &Point3<f32>)
                        -> Self {
        let plane = Plane::from_points(p0, p1, p2);
        let verts = vec![Vertex::new(p0, &plane.normal),
                         Vertex::new(p1, &plane.normal),
                         Vertex::new(p2, &plane.normal),
                         Vertex::new(p3, &plane.normal)];
        return ConvexPolygon {
                   plane: plane,
                   vertices: verts,
               };
    }

    pub fn invert(&mut self) {
        self.plane.invert();
        for vert in self.vertices.iter_mut() {
            vert.invert();
        }
        self.vertices.reverse();
    }

    pub fn to_triangles(&self) -> Vec<ConvexPolygon> {
        assert!(self.vertices.len() > 2);
        if self.vertices.len() == 3 {
            return vec![self.clone()];
        }
        if self.vertices.len() == 4 {
            let l02 = (self.vertices[0].position - self.vertices[2].position).norm_squared();
            let l13 = (self.vertices[1].position - self.vertices[3].position).norm_squared();
            if l02 < l13 {
                return vec![ConvexPolygon::new_raw(&self.plane,
                                                   vec![self.vertices[0].clone(),
                                                        self.vertices[1].clone(),
                                                        self.vertices[2].clone()]),
                            ConvexPolygon::new_raw(&self.plane,
                                                   vec![self.vertices[0].clone(),
                                                        self.vertices[2].clone(),
                                                        self.vertices[3].clone()])];
            }
            return vec![ConvexPolygon::new_raw(&self.plane,
                                               vec![self.vertices[0].clone(),
                                                    self.vertices[1].clone(),
                                                    self.vertices[3].clone()]),
                        ConvexPolygon::new_raw(&self.plane,
                                               vec![self.vertices[1].clone(),
                                                    self.vertices[2].clone(),
                                                    self.vertices[3].clone()])];
        }
        // FIXME: pick a better algorithm than fan.
        let mut polys = Vec::new();
        for i in 1..self.vertices.len() - 1 {
            polys.push(ConvexPolygon::new_raw(&self.plane,
                                              vec![self.vertices[0].clone(),
                                                   self.vertices[i].clone(),
                                                   self.vertices[i + 1].clone()]));
        }
        return polys;
    }
}

pub struct BspTree {
    nodes: Vec<BspNode>,
}

pub struct BspNode {
    plane: Option<Plane>,
    front: Option<BspNodeId>,
    back: Option<BspNodeId>,
    polygons: Vec<ConvexPolygon>,
}

#[derive(Clone)]
pub struct BspNodeId {
    index: usize,
}

impl BspTree {
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

    pub fn union(&mut self, other: &mut BspTree) -> Vec<ConvexPolygon> {
        let mut a = BspTree::new();
        a.get_root().add_polygons(self.get_polygons(), &mut a);

        let mut b = BspTree::new();
        b.get_root().add_polygons(other.get_polygons(), &mut b);

        a.clip_to(&mut b);
        b.clip_to(&mut a);
        b.invert();
        b.clip_to(&mut a);
        b.invert();

        let mut polys = a.get_polygons();
        polys.append(&mut b.get_polygons());
        //a.get_root().add_polygons(b.get_polygons(), &mut a);

        return polys;
    }

    pub fn get_polygons(&self) -> Vec<ConvexPolygon> {
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
    pub fn get_polygons(&self, polygons: &mut Vec<ConvexPolygon>, arena: &BspTree) {
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

    pub fn add_polygons(&self, mut polygons: Vec<ConvexPolygon>, arena: &mut BspTree) {
        if polygons.is_empty() {
            return;
        }

        // Borrow arena to use self's node to split the polygons. Add any co-planar
        // polygons to our list and split the rest into front and back lists.
        let mut front = Vec::<ConvexPolygon>::new();
        let mut back = Vec::<ConvexPolygon>::new();
        {
            let self_borrow = arena.nodes.get_mut(self.index).expect("unknown id");

            if self_borrow.plane.is_none() {
                // TODO: find and use a good heuristic for our initial split plane.
                self_borrow.plane = Some(polygons[0].plane.clone());
            }

            let plane: &Plane = self_borrow.plane.as_ref().expect("not none");
            for poly in polygons.drain(..) {
                let mut result = plane.split_polygon(poly);
                front.append(&mut result.front);
                back.append(&mut result.back);
                self_borrow.polygons.append(&mut result.coplanar_front);
                self_borrow.polygons.append(&mut result.coplanar_back);
            }
        }

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
                     mut polygons: Vec<ConvexPolygon>,
                     arena: &BspTree)
                     -> Vec<ConvexPolygon> {
        if polygons.len() == 0 {
            return Vec::new();
        }

        let mut front: Vec<ConvexPolygon> = Vec::new();
        let mut back: Vec<ConvexPolygon> = Vec::new();
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
                    .split_polygon(polygon);
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
    fn test_plane_distance() {
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
        let poly = ConvexPolygon::from_points(&Point3::new(0f32, 0f32, -1f32),
                                              &Point3::new(0f32, 0f32, 1f32),
                                              &Point3::new(0f32, 1f32, 0f32));
        let result = plane.split_polygon(poly);
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
        let poly = ConvexPolygon::from_points(&Point3::new(0f32, 0f32, 0f32),
                                              &Point3::new(1f32, 0f32, 0f32),
                                              &Point3::new(0f32, 1f32, 0f32));
        let result = plane.split_polygon(poly);
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
        let poly = ConvexPolygon::from_points(&Point3::new(0f32, 1f32, 0f32),
                                              &Point3::new(1f32, 0f32, 0f32),
                                              &Point3::new(0f32, 0f32, 0f32));
        let result = plane.split_polygon(poly);
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

    fn load_cube_bsp() -> BspTree {
        let m = load_cube_text();
        let polygons0 = m.tris
            .iter()
            .map(|t| ConvexPolygon::from_points(&t.verts[0], &t.verts[1], &t.verts[2]))
            .collect::<Vec<ConvexPolygon>>();
        let mut bsp = BspTree::new();
        bsp.get_root().add_polygons(polygons0, &mut bsp);
        return bsp;
    }

    fn save_polygons(polygons: &Vec<ConvexPolygon>, name: &str) {
        let mut tris: Vec<StlTriangle> = Vec::new();
        for poly in polygons {
            for t in poly.to_triangles() {
                tris.push(StlTriangle::new(t.vertices[0].position,
                                           t.vertices[1].position,
                                           t.vertices[2].position,
                                           Point3::from_coordinates(t.plane.normal)));
            }
        }
        let stl = StlMesh::from_tris(name, tris);
        let mut fp = File::create(format!("target/{}.stl", name)).unwrap();
        stl.to_file(&mut fp).unwrap();
    }

    fn save_bsp(bsp: &mut BspTree, name: &str) {
        let polys = bsp.get_polygons();
        save_polygons(&polys, name);
    }

    #[test]
    fn test_bsp_create() {
        let mut bsp = load_cube_bsp();
        save_bsp(&mut bsp, "test_bsp_create");
    }

    #[test]
    fn test_bsp_clip_to() {
        let mut bsp = load_cube_bsp();

        let mut polygons1 = Vec::<ConvexPolygon>::new();
        polygons1.push(ConvexPolygon::from_points4(&Point3::new(10f32, 0.5f32, 10f32),
                                                   &Point3::new(10f32, 0.5f32, -10f32),
                                                   &Point3::new(-10f32, 0.5f32, -10f32),
                                                   &Point3::new(-10f32, 0.5f32, 10f32)));
        let mut bsp_offset = BspTree::new();
        bsp_offset
            .get_root()
            .add_polygons(polygons1, &mut bsp_offset);

        bsp.clip_to(&mut bsp_offset);

        save_bsp(&mut bsp, "test_bsp_clip_to");
    }

    #[test]
    fn test_bsp_inverted_clip_to() {
        let mut bsp = load_cube_bsp();

        let mut polygons1 = Vec::<ConvexPolygon>::new();
        polygons1.push(ConvexPolygon::from_points4(&Point3::new(10f32, 0.5f32, 10f32),
                                                   &Point3::new(10f32, 0.5f32, -10f32),
                                                   &Point3::new(-10f32, 0.5f32, -10f32),
                                                   &Point3::new(-10f32, 0.5f32, 10f32)));
        let mut bsp_offset = BspTree::new();
        bsp_offset
            .get_root()
            .add_polygons(polygons1, &mut bsp_offset);

        bsp_offset.invert();
        bsp.clip_to(&mut bsp_offset);

        save_bsp(&mut bsp, "test_bsp_invert_clip_to");
    }

    #[test]
    fn test_bsp_union() {
        let mut bsp0 = load_cube_bsp();

        let m = load_cube_text();
        let mut polygons0 = Vec::<ConvexPolygon>::new();
        for t in m.tris.iter() {
            let mut v0 = t.verts[0].clone();
            let mut v1 = t.verts[1].clone();
            let mut v2 = t.verts[2].clone();
            for i in 0..3 {
                v0.coords.as_mut_slice()[i] -= 0.5f32;
                v1.coords.as_mut_slice()[i] -= 0.5f32;
                v2.coords.as_mut_slice()[i] -= 0.5f32;
            }
            polygons0.push(ConvexPolygon::from_points(&v0, &v1, &v2));
        }
        let mut bsp1 = BspTree::new();
        bsp1.get_root().add_polygons(polygons0, &mut bsp1);

        let mut result = bsp0.union(&mut bsp1);
        save_polygons(&result, "test_bsp_union");
    }
}
