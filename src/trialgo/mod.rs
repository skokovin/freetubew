use std::fs::File;
use std::io::{BufWriter, Write};
use std::ops::Sub;
use cgmath::InnerSpace;
use ruststep::ast::Name;
use truck_base::cgmath64::{Point3, Vector3};

pub mod analyzepl;
pub mod steputils;
pub mod pathfinder;
pub mod bendpipe;
pub mod foran_utils;
pub fn intersect_line_by_plane(cylinder_dir_vec: &Vector3, radius_vec: &Vector3, plane_point: &Point3, line_p0: &Point3, line_p1: &Point3) -> Point3 {
    //https://stackoverflow.com/questions/5666222/3d-line-plane-intersection
    //n: normal vector of the Plane
    // V0: any point that belongs to the Plane
    // P0: end point 1 of the segment P0P1
    // P1:  end point 2 of the segment P0P1
    let n = cylinder_dir_vec.cross(radius_vec.clone());
    let w = line_p0.sub(plane_point);
    let u = line_p1.sub(line_p0);
    let nw = -n.dot(w);
    let du = n.dot(u);
    let si = nw / du;
    let res: Point3 = line_p0 + si * u;
    //println!("{:?}", res);
    res
}

pub fn project_point_to_vec(proj_vector: &Vector3, point_on_proj_vector: &Point3, point_to_project: &Point3) -> Point3 {
    //https://stackoverflow.com/questions/9368436/3d-perpendicular-point-on-line-from-3d-point
    let pq: Vector3 = point_to_project.sub(point_on_proj_vector);
    let w2: Vector3 = pq - proj_vector * pq.dot(proj_vector.clone()) / proj_vector.magnitude2();
    let point: Point3 = point_to_project - w2;
    //println!("{:?} {:?}", point, 0);
    point
}

pub fn float_range(start: f64, threshold: f64, step_size: f64) -> impl Iterator<Item=f64> {
    std::iter::successors(Some(start), move |&prev| {
        let next = prev + step_size;
        (next < threshold).then_some(next)
    })
}
pub fn export_to_pt(points: &Vec<cgmath::Point3<f64>>, counter: i32) {
    let path = format!("d:\\pipe_project\\{}.pt", counter);
    match File::create(path) {
        Ok(file) => {
            let mut writer = BufWriter::new(file);
            points.iter().for_each(|p| {
                let line = format!("{} {} {} \r\n", p.x, p.y, p.z);
                writer.write_all(&line.as_bytes());
            });
            writer.flush();
        }
        Err(_) => {}
    }
}

pub fn export_to_pt_str(points: &Vec<cgmath::Point3<f64>>, counter: &str) {
    let path = format!("d:\\pipe_project\\{}.pt", counter);
    match File::create(path) {
        Ok(file) => {
            let mut writer = BufWriter::new(file);
            points.iter().for_each(|p| {
                let line = format!("{} {} {} \r\n", p.x, p.y, p.z);
                writer.write_all(&line.as_bytes());
            });
            writer.flush();
        }
        Err(_) => {}
    }
}

fn name_to_id(name: Name) -> u64 {
    match name {
        Name::Entity(id) => { id }
        Name::Value(_) => { 0 }
        Name::ConstantEntity(_) => { 0 }
        Name::ConstantValue(_) => { 0 }
    }
}

pub fn round_by_dec(x: f64, decimals: u32) -> f64 {
    let y = 10i64.pow(decimals);
    (x * y as f64).round() / y as f64
}