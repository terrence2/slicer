use byteorder::{LittleEndian, WriteBytesExt};
use nalgebra::{distance, Point3};
use nom::{le_u16, le_u32, le_f32, space, multispace};
use std::f32;
use std::fs::File;
use std::io::{Read, Write};
use std::str;

mod errors {
    error_chain!{}
}
use errors::{Result, ResultExt};

#[derive(Debug)]
pub struct StlTriangle {
    pub verts: [Point3<f32>; 3],
    pub normal: Point3<f32>,
    pub color: [u8; 3], // 8-bit RGB format.
    pub is_color_valid: bool,
}
impl StlTriangle {
    pub fn new(v0: Point3<f32>, v1: Point3<f32>, v2: Point3<f32>, norm: Point3<f32>) -> Self {
        StlTriangle {
            verts: [v0, v1, v2],
            normal: norm,
            color: [0, 0, 0],
            is_color_valid: false,
        }
    }

    pub fn set_color(&mut self, r: u8, g: u8, b: u8) {
        self.color = [r, g, b];
        self.is_color_valid = true;
    }

    #[allow(dead_code)]
    pub fn clear_color(&mut self) {
        self.is_color_valid = false;
    }
}

fn is_newline(c: u8) -> bool {
    c == '\n' as u8
}

fn is_multispace(c: u8) -> bool {
    for s in [' ', '\t', '\n', '\r'].into_iter() {
        if c == *s as u8 {
            return true;
        }
    }
    return false;
}

named!(get_float <&[u8], f32>, do_parse!(
    n: map_res!(map_res!(take_till!(is_multispace),
                         str::from_utf8), str::parse::<f32>) >>
    (n)
));

named!(get_vec3 <&[u8], Point3<f32>>, do_parse!(
    many1!(space) >>
    f0: get_float >>
    many1!(space) >>
    f1: get_float >>
    many1!(space) >>
    f2: get_float >>
    (Point3::new(f0, f1, f2))
));

named!(get_normal <&[u8], Point3<f32>>, do_parse!(
    tag!("normal") >> norm: get_vec3 >> many0!(multispace) >>
    (norm)
));

named!(get_vertex <&[u8], Point3<f32>>, do_parse!(
    tag!("vertex") >> v: get_vec3 >> many1!(multispace) >>
    (v)
));

named!(get_ascii_triangle <&[u8], StlTriangle>, do_parse!(
    tag!("facet") >> many1!(multispace) >>
        norm: get_normal >>
        tag!("outer") >> many1!(space) >> tag!("loop") >> many1!(multispace) >>
            v0: get_vertex >>
            v1: get_vertex >>
            v2: get_vertex >>
        tag!("endloop") >> many1!(multispace) >>
    tag!("endfacet") >> many1!(multispace) >>
    (StlTriangle::new(v0, v1, v2, norm))
));

named!(parse_ascii_stl <&[u8], StlMesh>, do_parse!(
    tag!("solid") >>
    many0!(space) >>
    name: map_res!(take_till!(is_newline), str::from_utf8) >> many0!(multispace) >>
    tris: many1!(get_ascii_triangle) >>
    tag!("endsolid") >>
    (StlMesh { name: name.to_owned(), tris: tris })
));

named!(get_binary_triangle <&[u8], StlTriangle>, do_parse!(
    fs: count!(le_f32, 12) >>
    attrs: count!(le_u16, 1) >>
    (StlTriangle::new(
            Point3::new(fs[3],  fs[4],  fs[5]),
            Point3::new(fs[6],  fs[7],  fs[8]),
            Point3::new(fs[9], fs[10], fs[11]),
            Point3::new(fs[0],  fs[1],  fs[2])))
));

named!(parse_binary_stl <&[u8], StlMesh>, do_parse!(
    header: take!(80) >>
    tris: length_count!(le_u32, get_binary_triangle) >>
    (StlMesh { name: "<name unknown>".to_owned(), tris: tris })
));

#[allow(dead_code)]
fn max4(a: f32, b: f32, c: f32, d: f32) -> f32 {
    let mut m = a;
    if b > m {
        m = b;
    } else if c > m {
        m = c;
    } else if d > m {
        m = d;
    }
    return m;
}

#[derive(Debug)]
pub struct StlMesh {
    pub name: String,
    pub tris: Vec<StlTriangle>,
}

impl StlMesh {
    pub fn from_file(fp: &mut File) -> Result<StlMesh> {
        let mut s: Vec<u8> = Vec::new();
        fp.read_to_end(&mut s).chain_err(|| "file read failed")?;

        if str::from_utf8(&s[0..5]).unwrap() == "solid" {
            let (_, mesh) = parse_ascii_stl(&s).unwrap();
            return Ok(mesh);
        }

        let (_, mesh) = parse_binary_stl(&s).unwrap();
        return Ok(mesh);
    }

    pub fn from_tris(name: &str, tris: Vec<StlTriangle>) -> Self {
        StlMesh {
            name: name.to_owned(),
            tris: tris,
        }
    }

    #[allow(dead_code)]
    pub fn to_text_file(&self, fp: &mut File) -> Result<()> {
        let mut buf = format!("solid {}\n", self.name);
        for tri in self.tris.iter() {
            buf += &format!("  facet normal {} {} {}\n",
                            tri.normal[0],
                            tri.normal[1],
                            tri.normal[2]);
            buf += &format!("    outer loop\n");
            buf += &format!("      verts {} {} {}\n",
                            tri.verts[0][0],
                            tri.verts[0][1],
                            tri.verts[0][2]);
            buf += &format!("      verts {} {} {}\n",
                            tri.verts[1][0],
                            tri.verts[1][1],
                            tri.verts[1][2]);
            buf += &format!("      verts {} {} {}\n",
                            tri.verts[2][0],
                            tri.verts[2][1],
                            tri.verts[2][2]);
            buf += &format!("    end loop\n");
            buf += &format!("  end facet\n");
        }
        buf += "end solid";
        fp.write_all(buf.into_bytes().as_slice())
            .chain_err(|| "failed to write")?;
        return Ok(());
    }

    pub fn to_binary_file(&self, fp: &mut File) -> Result<()> {
        let mut buf = Vec::<u8>::new();
        for _ in 0..80 {
            buf.write_u8(0).chain_err(|| "make header")?;
        }
        buf.write_u32::<LittleEndian>(self.tris.len() as u32)
            .chain_err(|| "format ntris")?;
        for tri in self.tris.iter() {
            for i in 0..3 {
                buf.write_f32::<LittleEndian>(tri.normal[i])
                    .chain_err(|| "format normal")?;
            }
            for i in 0..3 {
                for j in 0..3 {
                    buf.write_f32::<LittleEndian>(tri.verts[i][j])
                        .chain_err(|| "format vert")?;
                }
            }
            if tri.is_color_valid {
                // Convert to 15-bit BGR format and set the "valid" bit.
                let r: u16 = tri.color[0] as u16 >> 3;
                let g: u16 = tri.color[1] as u16 >> 3;
                let b: u16 = tri.color[2] as u16 >> 3;
                let clr = (1 << 15) | (r << 10) | (g << 5) | b;
                buf.write_u16::<LittleEndian>(clr)
                    .chain_err(|| "format attr")?;
            } else {
                buf.write_u16::<LittleEndian>(0)
                    .chain_err(|| "format attr")?;
            }
        }
        fp.write_all(buf.as_slice())
            .chain_err(|| "failed to write")?;
        return Ok(());
    }

    #[allow(dead_code)]
    pub fn radius(&self) -> f32 {
        let mut r = 1.0f32;
        for tri in self.tris.iter() {
            r = max4(r,
                     distance(&tri.verts[0], &Point3::origin()),
                     distance(&tri.verts[1], &Point3::origin()),
                     distance(&tri.verts[2], &Point3::origin()));
        }
        return r;
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_read_scad() {
        let mut fp = File::open("test_data/single_color/cube_scad.stl").unwrap();
        let m = super::StlMesh::from_file(&mut fp).unwrap();
        assert_eq!(m.tris.len(), 12);
        assert_eq!(m.radius(), 3f32.sqrt());
    }

    #[test]
    fn test_read_bin() {
        let mut fp = File::open("test_data/single_color/cube_bin.stl").unwrap();
        let m = super::StlMesh::from_file(&mut fp).unwrap();
        assert_eq!(m.tris.len(), 12);
        assert_eq!(m.radius(), 3f32.sqrt());
    }

    #[test]
    fn test_read_text() {
        let mut fp = File::open("test_data/single_color/cube_text.stl").unwrap();
        let m = super::StlMesh::from_file(&mut fp).unwrap();
        assert_eq!(m.tris.len(), 12);
        assert_eq!(m.radius(), 3f32.sqrt());
    }
}
