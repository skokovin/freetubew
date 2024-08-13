use std::f64::consts::PI;
use serde_json::from_slice;
use std::str;
use std::str::Utf8Error;
use cgmath::{Basis3, InnerSpace, Matrix3, Rad, Rotation, Rotation3, Vector3};
use rand::random;
use truck_base::cgmath64::Point3;
use truck_geometry::prelude::Plane;

use crate::trialgo::bendpipe::{BendToro, MainCircle, MainCylinder};
use crate::trialgo::{export_to_pt, round_by_dec};
use crate::trialgo::pathfinder::CncOps;

const FAR_POINT: Point3 = Point3::new(600326.0, 843392.1, 932431.7963);
const E: i32 = 0;
const P: i32 = 1;
#[test]
fn run() {
    let f = include_bytes!("../bin/fout.json");
    match str::from_utf8(f) {
        Ok(s) => {
            println!("{:?}", s)
        }
        Err(e) => { println!("{:?}", e) }
    }
    //let v:Vec<f64>=serde_json::from_slice(&f);
}

#[derive(Debug, Clone)]
pub struct RawPL {
    ctype: i32,
    a11: f64,
    a12: f64,
    a13: f64,
    a21: f64,
    a22: f64,
    a23: f64,
    a31: f64,
    a32: f64,
    a33: f64,
    a41: f64,
    a42: f64,
    a43: f64,
    v1: f64,
    v2: f64,
    v3: f64,
    v4: f64,
    v5: f64,
}

pub fn extract_pipeline(arr: Vec<f32>) -> CncOps {
    let mut cyls: Vec<MainCylinder> = vec![];
    let mut toros: Vec<BendToro> = vec![];
    let rd = convert_to_raw(&arr);
    let mut last_index: usize = rd.iter().len() - 1;
    let mut count: usize = 0;
    rd.iter().for_each(|item| {
        match item.ctype {
            E => {
                if (count == 0) {
                    let tor: BendToro = extract_tor(item, 1);
                    let extra_point = tor.ca.loc.clone() + tor.ca.dir * 20.0;
                    let mut extra_cyl = MainCylinder {
                        id: count as u64,
                        ca: MainCircle {
                            id: random(),
                            radius: tor.r,
                            loc: tor.ca.loc.clone(),
                            dir: tor.ca.dir,
                            radius_dir: tor.ca.radius_dir,
                        },
                        cb: MainCircle {
                            id: random(),
                            radius: tor.r,
                            loc: extra_point,
                            dir: tor.ca.dir,
                            radius_dir: tor.ca.radius_dir,
                        },
                        h: 20.0,
                        r: tor.r,
                        r_gr_id: tor.r_gr_id,
                        ca_tor: u64::MAX,
                        cb_tor: u64::MAX,
                        triangles: vec![],
                    };
                    count = count + 1;
                    last_index = last_index + 1;
                    //extra_cyl.triangulate();
                    //extra_cyl.to_obj();
                    //tor.to_obj();
                    cyls.push(extra_cyl);
                    toros.push(tor);
                } else if (count == last_index) {
                    let tor = extract_tor(item, count as u64);
                    count = count + 1;
                    let extra_point = tor.cb.loc.clone() + tor.cb.dir * 20.0;
                    let mut extra_cyl = MainCylinder {
                        id: count as u64,
                        ca: MainCircle {
                            id: random(),
                            radius: tor.r,
                            loc: tor.cb.loc.clone(),
                            dir: tor.cb.dir,
                            radius_dir: tor.cb.radius_dir,
                        },
                        cb: MainCircle {
                            id: random(),
                            radius: tor.r,
                            loc: extra_point,
                            dir: tor.cb.dir,
                            radius_dir: tor.cb.radius_dir,
                        },
                        h: 20.0,
                        r: tor.r,
                        r_gr_id: tor.r_gr_id,
                        ca_tor: u64::MAX,
                        cb_tor: u64::MAX,
                        triangles: vec![],
                    };
                    //extra_cyl.triangulate();
                    //extra_cyl.to_obj();
                    //tor.to_obj();
                    toros.push(tor);
                    cyls.push(extra_cyl);
                } else {
                    let tor = extract_tor(item, count as u64);
                    //tor.to_obj();
                    toros.push(tor);
                }
            }
            P => {
                let cyl: MainCylinder = extract_cyl(item, count as u64);
                //cyl.to_obj();
                cyls.push(cyl);
            }
            _ => {}
        }
        count = count + 1;
    });
    MainCylinder::init_tors(&mut cyls, &toros);
    let mut ops = CncOps::new(&cyls, &toros);
    ops.triangulate_all();
    //ops.all_to_one_obj();
    ops.calculate_lra();
    ops
}

fn convert_to_raw(arr: &Vec<f32>) -> Vec<RawPL> {
    let mut ret: Vec<RawPL> = vec![];
    if (!arr.is_empty()) {
        arr.chunks(18).for_each(|v| {
            let item = RawPL {
                ctype: v[0] as i32,
                a11: v[1] as f64,
                a12: v[2] as f64,
                a13: v[3] as f64,
                a21: v[4] as f64,
                a22: v[5] as f64,
                a23: v[6] as f64,
                a31: v[7] as f64,
                a32: v[8] as f64,
                a33: v[9] as f64,
                a41: v[10] as f64,
                a42: v[11] as f64,
                a43: v[12] as f64,
                v1: v[13] as f64,
                v2: v[14] as f64,
                v3: v[15] as f64,
                v4: v[16] as f64,
                v5: v[17] as f64,
            };
            ret.push(item);
        })
    }
    ret
}

fn extract_cyl(raw: &RawPL, id: u64) -> MainCylinder {
    let v_one: Vector3<f64> = Vector3::new(0.0, 0.0, 1.0);
    let m3: Matrix3<f64> = Matrix3::new(
        raw.a11, raw.a12, raw.a13,
        raw.a21, raw.a22, raw.a23,
        raw.a31, raw.a32, raw.a33,
    );
    let dir = (m3 * v_one).normalize();
    let middle_p: Point3 = Point3::new(raw.a41, raw.a42, raw.a43);
    let d = raw.v2 / 2.0;
    let p1 = middle_p.clone() + dir * d;
    let p2 = middle_p.clone() + dir * -d;
    let r = raw.v1;
    let plane = Plane::new(p1, p2, FAR_POINT);
    let n = plane.normal().normalize();
    let divider = 100000000.0;
    let hashr: u64 = (round_by_dec(r, 5) * divider) as u64;
    let mut mc = MainCylinder {
        id: id,
        ca: MainCircle {
            id: random(),
            radius: r,
            loc: p1,
            dir,
            radius_dir: n.clone(),
        },
        cb: MainCircle {
            id: random(),
            radius: r,
            loc: p2,
            dir,
            radius_dir: n.clone(),
        },
        h: raw.v2,
        r: r,
        r_gr_id: hashr,
        ca_tor: u64::MAX,
        cb_tor: u64::MAX,
        triangles: vec![],
    };
    //mc.triangulate();
    mc
}

fn extract_tor(raw: &RawPL, id: u64) -> BendToro {
    let r_pipe = raw.v1;
    let r_bend = raw.v2;
    let angle_bend = raw.v3;
    let v_one_z: Vector3<f64> = Vector3::new(0.0, 0.0, 1.0);
    let v_one_y: Vector3<f64> = Vector3::new(0.0, 1.0, 0.0);
    let v_one_x: Vector3<f64> = Vector3::new(1.0, 0.0, 0.0);

    let m3: Matrix3<f64> = Matrix3::new(
        raw.a11, raw.a12, raw.a13,
        raw.a21, raw.a22, raw.a23,
        raw.a31, raw.a32, raw.a33,
    );


    let p0: Point3 = Point3::new(raw.a41, raw.a42, raw.a43);
    let arc_len = r_bend * angle_bend;
    let hord_len = (arc_len / (2.0 * r_bend)).sin() * r_bend * 2.0;
    let hord_angle = ((2.0 * PI - angle_bend) - angle_bend) * 0.5;
    let point_angle = (PI - hord_angle) * 0.5;
    let len_to_point = (hord_len / (hord_angle.sin())) * point_angle.sin();


    let dir_z = (m3 * v_one_z).normalize();
    let dir_y = (m3 * v_one_y).normalize();
    let dir_x = (m3 * v_one_x).normalize();
    let rotation: Basis3<f64> = Rotation3::from_axis_angle(dir_z, Rad(hord_angle));
    let dir_x2 = rotation.rotate_vector(dir_x.clone());

    let sp = p0.clone() + dir_x * len_to_point;
    let ep = p0.clone() + dir_x2 * len_to_point;
    let cp = sp.clone() + dir_y * r_bend;
    let plane = Plane::new(cp, sp, ep);
    let bend_plane_norm = plane.normal().normalize();

    let divider = 100000000.0;
    let hashr: u64 = (round_by_dec(r_pipe, 5) * divider) as u64;

    let mut toro = BendToro {
        id: id,
        r: r_pipe,
        bend_radius: r_bend,
        bend_center_point: cp,
        bend_plane_norm: bend_plane_norm,
        radius_dir: dir_z,
        ca: MainCircle {
            id: random(),
            radius: r_pipe,
            loc: sp,
            dir: dir_x,
            radius_dir: dir_z,
        },
        cb: MainCircle {
            id: random(),
            radius: r_pipe,
            loc: ep,
            dir: -dir_x2,
            radius_dir: dir_z,
        },
        r_gr_id: hashr,
        triangles: vec![],
    };

    //toro.triangulate(&-dir_x);

    toro
}