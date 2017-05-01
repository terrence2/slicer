#[macro_use] extern crate approx;
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
    let mut fp = File::open("test_data/cube_bin.stl").unwrap();
    let stl = StlMesh::from_file(&mut fp).unwrap();

    //let foo = m.intersect_plane();

    println!("Hello, world!");
    Ok(())
}
