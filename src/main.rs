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
extern crate alga;
#[macro_use]
extern crate approx;
extern crate byteorder;
#[macro_use]
extern crate clap;
extern crate cpuprofiler;
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
mod merge;
mod mesh;
mod stl;

use bsp::BspTree;
use cpuprofiler::PROFILER;
use merge::{merge_meshes, MergedMesh};
use mesh::Mesh;
use std::fs::File;
use std::thread;
use stl::StlMesh;
use errors::{Error, Result, ResultExt};

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

    /*
    let stl = load_and_merge_meshes_smart(filenames.clone())
        .chain_err(|| "Failed to load meshes")?;
    */

    let bsp = load_and_merge_meshes(filenames)
        .chain_err(|| "Failed to load meshes")?;

    println!("Converting merged mesh to STL...");
    let stl = bsp.convert_to_stl("slicer merge mesh");

    println!("Writing merged mesh to: a.stl");
    let mut fp = File::create("a.stl").unwrap();
    stl.to_binary_file(&mut fp).unwrap();


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

fn load_and_merge_meshes(filenames: Vec<&str>) -> Result<BspTree> {

    // Load all meshes, tagging faces from each mesh with the offset: i.e. the extruder number.
    let mut handles = Vec::<thread::JoinHandle<Result<BspTree>>>::new();
    for (i, filename) in filenames.into_iter().enumerate() {
        println!("Loading STL from file: {}", filename);

        // Note: we have to load a copy of the string into a stack slot here so that we
        // can safely capture it in the child thread.
        let owned_filename = filename.to_owned();

        // BSP creation may use excessive amounts of stack space if we get unlucky,
        // so we set a fairly enormous stack here. On the plus side, we're not going
        // to be loading more than a handful of models in parallel.
        // TODO: make the stack size configurable.
        let handle = thread::Builder::new()
            .name("BSP_loader".to_owned())
            .stack_size(8 * 1024 * 1024 * 1024)
            .spawn(move || load_mesh(owned_filename, i as u8))
            .chain_err(|| "spawning a thread to load a BSP failed")?;
        handles.push(handle);
    }

    PROFILER.lock().unwrap().start("./foo.profile").expect("couldn't start");
    let mut merged: Option<BspTree> = None;
    for handle in handles {
        let bsp = match handle.join() {
            Err(e) => {
                return Err(*e.downcast::<Error>().unwrap());
            }
            Ok(m) => m.chain_err(|| "failed to join with child thread")?,
        };
        if let Some(ref mut target) = merged {
            println!("merging {} into {}", bsp.name, target.name);
            target.union_with(bsp);
        } else {
            merged = Some(bsp);
        }
    }
    PROFILER.lock().unwrap().stop().expect("couldn't stop");

    return Ok(merged.expect("no meshes to load should be handled by clap"));
}

fn load_mesh(filename: String, attr: u8) -> Result<BspTree> {
    let mut fp = File::open(&filename)
        .chain_err(|| format!("failed to open source file: {}", &filename))?;

    let stl = StlMesh::from_file(&mut fp)
        .chain_err(|| "failed to load stl file")?;

    return Ok(BspTree::new_from_stl(&stl, attr));
}

fn load_and_merge_meshes_smart(filenames: Vec<&str>) -> Result<StlMesh> {

    // Load all meshes, tagging faces from each mesh with the offset: i.e. the extruder number.
    let mut stl_meshes = Vec::new();
    for (i, filename) in filenames.into_iter().enumerate() {
        let mut fp = File::open(&filename)
            .chain_err(|| format!("failed to open source file: {}", &filename))?;

        let stl_mesh = StlMesh::from_file(&mut fp)
            .chain_err(|| format!("failed to load stl mesh: {}", &filename))?;

        stl_meshes.push(stl_mesh);
    }

    return Ok(merge_meshes(&stl_meshes));
}
