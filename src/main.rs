#[macro_use] extern crate approx;
#[macro_use] extern crate clap;
#[macro_use] extern crate error_chain;
#[macro_use] extern crate nom;
extern crate nalgebra;

mod errors { error_chain! {} }
mod mesh;
mod stl;

use std::fs::File;
use std::io::Read;
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

    let filenames = matches.values_of("INPUT").unwrap().collect::<Vec<&str>>();
    for filename in filenames {
        let mut fp = File::open(filename).chain_err(|| "failed to open source files")?;
        let stl = StlMesh::from_file(&mut fp).chain_err(|| "failed to load stl file")?;
    }

    //let foo = m.intersect_plane();

    println!("Hello, world!");
    Ok(())
}
