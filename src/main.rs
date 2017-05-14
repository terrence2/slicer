#[macro_use]
extern crate approx;
#[macro_use]
extern crate clap;
#[macro_use]
extern crate error_chain;
#[macro_use]
extern crate nom;
extern crate nalgebra;

mod errors {
    error_chain!{}
}
mod geometry;
mod mesh;
mod stl;

use mesh::Mesh;
use std::fs::File;
use stl::StlMesh;
use errors::{Result, ResultExt};

quick_main!(run);
fn run() -> Result<()> {
    let parser = clap_app!(slicer =>
        (version: "0.1.0")
        (author: "Terrence Cole <terrence.d.cole@gmail.com>")
        (about: "Slice multi-color models smarter.")
        (@arg INPUT: +required ... "Sets the input file(s) to use.")
    );
    let matches = parser.get_matches();

    // Clap ensures that there is at least one INPUT value to unwrap.
    let filenames = matches
        .values_of("INPUT")
        .unwrap()
        .collect::<Vec<&str>>();

    // Load all meshes, tagging faces from each mesh with the offset: i.e. the extruder number.
    let mut meshes = Vec::<Mesh>::new();
    for (i, filename) in filenames.iter().enumerate() {
        let mut fp = File::open(filename)
            .chain_err(|| format!("failed to open source file: {}", filename))?;
        let stl = StlMesh::from_file(&mut fp)
            .chain_err(|| "failed to load stl file")?;
        let mesh = Mesh::from_stl(stl, i as u8)
            .chain_err(|| format!("failed to reify mesh: {}", filename))?;
        meshes.push(mesh);
    }

    // Union all meshes, preserving the face tags.
    let mut mesh = meshes.pop().unwrap();
    for other in meshes.iter() {
        mesh = mesh.union_non_overlapping(other)
            .chain_err(|| "failed to merge mesh")?;
    }

    println!("  Verts: {}", mesh.verts.len());
    println!("  Norms: {}", mesh.normals.len());
    Ok(())
}
