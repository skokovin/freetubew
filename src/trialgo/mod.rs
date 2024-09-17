use std::fs::File;
use std::io::{BufWriter, Write};
use std::ops::Sub;
use cgmath::{Basis3, InnerSpace};
use itertools::Itertools;
use ruststep::ast::Name;
use truck_base::cgmath64::{Point3, Vector3};
use truck_meshalgo::prelude::*;
use truck_polymesh::{Faces, PolygonMesh, StandardAttributes, StandardVertex};
use crate::device::{MeshVertex, RawMesh, StepVertexBuffer, Triangle};
use crate::trialgo::pathfinder::CncOps;

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

pub fn polygon_mesh_from_triangles(triangles: &Vec<Triangle>) -> PolygonMesh {
    let mut positions: Vec<Point3> = vec![];
    let mut tri_normals: Vec<Vector3> = vec![];
    let mut indx: Vec<[StandardVertex; 3]> = vec![];
    triangles.iter().for_each(|tri| {
        positions.extend_from_slice(tri.as_p64().as_slice());
        let n: Vector3 = Vector3::new(tri.normal.x as f64, tri.normal.y as f64, tri.normal.z as f64);
        tri_normals.push(n.clone());
        tri_normals.push(n.clone());
        tri_normals.push(n.clone());
    });


    (0..positions.len()).chunks(3).into_iter().for_each(|ch| {
        let ii = ch.collect_vec();
        indx.push([ii[0].into(), ii[1].into(), ii[2].into()]);
    });

    let mut polymesh = PolygonMesh::new(
        StandardAttributes {
            positions: positions,
            normals: tri_normals,
            ..Default::default()
        },
        Faces::from_tri_and_quad_faces(
            indx,
            Vec::new(),
        ),
    );

    polymesh.triangulate()
        .normalize_normals()
        .remove_unused_attrs()
        .remove_degenerate_faces()
        .add_smooth_normals(std::f64::consts::PI / 6.0, true)
        .remove_unused_attrs();
    polymesh
}
pub fn gen_cap(cp: Point3, dir_up: Vector3, dir_radius: Vector3, r: f64, id: i32, reverse_normal:bool) -> StepVertexBuffer {
    let mut triangles: Vec<Triangle> = vec![];
    let dir_radius_normalized = dir_radius.normalize();
    let step=4;
    for i in (0..360).step_by(step) {
        let start_ang: Rad<f64> = Rad::from(Deg(i as f64));
        let end_ang: Rad<f64> = Rad::from(Deg((i + step) as f64));

        let rotation_s: Basis3<f64> = Rotation3::from_axis_angle(dir_up, start_ang);
        let rotation_e: Basis3<f64> = Rotation3::from_axis_angle(dir_up, end_ang);
        let s_angle = rotation_s.rotate_vector(dir_radius_normalized);
        let e_angle = rotation_e.rotate_vector(dir_radius_normalized);
        let p1 = cp + s_angle * r;
        let p2 = cp + e_angle * r;
        if(reverse_normal){
            triangles.push(Triangle::from_f64_without_normals(cp, p1, p2));
        }else{
            triangles.push(Triangle::from_f64_without_normals(cp, p2, p1));
        }

    }
    let pm = polygon_mesh_from_triangles(&triangles);
    let (verts, indx, bbx, triangles) = CncOps::convert_polymesh(&pm);

    let mesh = RawMesh {
        id: id as i32,
        vertex_normal: verts,
        indx: indx,
        bbx: bbx,
        triangles: triangles,
    };
    let mut index: u32 = 0;
    let mut indxes: Vec<u32> = vec![];
    let mut buffer: Vec<MeshVertex> = vec![];
    mesh.vertex_normal.chunks(6).for_each(|vn| {
        let mv = MeshVertex {
            position: [vn[0], vn[1], vn[2], 1.0],
            normal: [vn[3], vn[4], vn[5], 0.0],
            id: id as i32,
        };
        buffer.push(mv);
        indxes.push(index);
        index = index + 1;
    });
    let sv = StepVertexBuffer {
        buffer,
        indxes,
    };
    sv
}
