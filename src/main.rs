extern crate alga;
#[macro_use]
extern crate approx;
extern crate byteorder;
#[macro_use]
extern crate clap;
#[macro_use]
extern crate error_chain;
#[macro_use]
extern crate itertools;
#[macro_use]
extern crate nom;
extern crate nalgebra;

mod errors {
    error_chain!{}
}
mod bsp;
mod geometry;
mod mesh;
mod stl;

use bsp::BspTree;
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
    let filenames = matches.values_of("INPUT").unwrap().collect::<Vec<&str>>();

    // Load all meshes, tagging faces from each mesh with the offset: i.e. the extruder number.
    let mut merged: Option<BspTree> = None;
    for (i, filename) in filenames.iter().enumerate() {
        let mut fp = File::open(filename)
            .chain_err(|| format!("failed to open source file: {}", filename))?;

        println!("Loading STL from file: {}", filename);
        let stl = StlMesh::from_file(&mut fp)
            .chain_err(|| "failed to load stl file")?;

        println!("Creating BSP tree...");
        let bsp = BspTree::new_from_stl(&stl, i as u8);

        println!("Merging with existing mesh...");
        if let Some(ref mut target) = merged {
            target.union_with(bsp);
        } else {
            merged = Some(bsp);
        }
    }

    if let Some(bsp) = merged {
        println!("Converting merged mesh to STL...");
        let stl = bsp.convert_to_stl("slicer merge mesh");

        println!("Writing merged mesh to: a.stl");
        let mut fp = File::create("a.stl").unwrap();
        stl.to_binary_file(&mut fp).unwrap();
    }

    // Union all meshes, preserving the face tags.
    //   Build a BSP of the first mesh and then union into it.
    //let mut mesh = meshes.pop().unwrap();
    //for other in meshes.iter() {
        //mesh = mesh.union_non_overlapping(other)
        //    .chain_err(|| "failed to merge mesh")?;
    //}

    // Build an infill mesh somehow, tagging faces with "don't care" extruder setting.
    // BSP should be able to carve these to the right shape by doing something like
    // infill.subtract(mesh.invert()).union(infill). I'm not sure how BSP will handle
    // the internal polygons. We'll need to figure something out.

    // Take slices of the model at each successive height interval, keeping each 2D edge
    // tagged with the same tag as the face the slice came from.
    // Unsolved problem: finding top layers if they land in-between slices.
    //                   Take a look at what other slicers are doing.

    // Plan a route through each successive layer that minimizes filament switches and
    // ensures that at least the minimum extrusion occurs before printing outer walls.
    // If not enough infill needs to be printed to satisfy this condition, keep track of
    // the extra that is needed.

    // Inject additional infill at heights that need extra extrusion to satisfy the color
    // requirements. Repeat the last two steps until we cannot subdivide further, or
    // we satisfy our extrusion minimums.

    // Take the excess extrusion requirements and compute the minimum radius for each
    // layer. Build a variable-radius cylinder, with some minimal width and back-fill
    // with adequate supporting structures. Place it outside the main structure.

    // Add the wipe tower's paths into the main paths.

    // (optional) Do a layer-by-layer physical analysis of the main structure's static
    // forces. Ensure that there is adequate support to print.


    //println!("  Verts: {}", mesh.verts.len());
    //println!("  Norms: {}", mesh.normals.len());
    Ok(())
}

