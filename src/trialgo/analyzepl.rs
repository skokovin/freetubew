use std::fs::File;
use std::io::Read;
use std::ops::Sub;
use cgmath::Vector3;
use encoding_rs::WINDOWS_1251;
use encoding_rs_io::DecodeReaderBytesBuilder;
use glob::glob;
use rand::random;
use ruststep::tables::PlaceHolder;
use serde::de::Unexpected::Str;
use truck_base::cgmath64::Point3;
use truck_stepio::r#in::{FaceBoundHolder, NonRationalBSplineSurfaceHolder, Table};
use crate::trialgo::bendpipe::{BendToro, MainCircle, MainCylinder};
use crate::trialgo::name_to_id;
use crate::trialgo::pathfinder::{CncOps, OpElem};
use crate::trialgo::steputils::{extract_circles_bounds, extract_circles_bounds_tourus, extract_plane_points, extract_position3d};

pub const DIVIDER: f64 = 100000000.0;
//pub const TOLE: f64 = 0.00001;
pub const TOLE: f64 = 0.01;
pub const MAX_BEND_RADIUS: f64 = 500.0;
pub const TESS_TOL_ANGLE: f64 = 180.0;
pub const TESS_TOR_STEP: u64 = 20;
pub const EXTRA_LEN_CALC: f64 = 3.0;
pub const EXTRA_R_CALC: f64 = 1.2;
pub static mut STPSCALE: f64 = 1.0;

pub fn analyze_bin(arr: &Vec<u8>) -> Option<CncOps> {
    let mut transcoded = DecodeReaderBytesBuilder::new()
        .encoding(Some(WINDOWS_1251))
        .build(arr.as_slice());
    let mut buf: Vec<u8> = vec![];
    let mut stp: String = String::new();
    match transcoded.read_to_end(&mut buf) {
        Ok(b) => {
            match String::from_utf8(buf) {
                Ok(cont) => {
                    stp = cont.clone();
                }
                Err(e) => { println!("{:?}", e) }
            }
        }
        Err(e) => {
            println!("{:?}", e)
        }
    }
    unsafe { STPSCALE = extact_scale(&stp); }
    let exchange = ruststep::parser::parse(&stp).unwrap();
    let table = Table::from_data_section(&exchange.data[0]);
    match extract_main_radius(&table) {
        None => { None }
        Some((cyls, bends)) => {
            //MainCylinder::flush_to_pt_files(&cyls);
            //BendToro::all_to_pt_file(&bends);

            let ops: CncOps = CncOps::new(&cyls, &bends);
            if(!ops.ops.is_empty()){
                let extra_len_pts: Vec<Point3> = extract_plane_points(&table);

                let mut extra_len_ops = ops.calculate_extra_len(&extra_len_pts);
                extra_len_ops.triangulate_all();
                //ops.all_to_one_obj();
                //ops.flush_to_pt_file();
                Some(extra_len_ops)
            }else{
                None
            }


        }
    }
}


pub fn analyze() {
    glob("d:\\pipe_project\\tests\\1214710 New Tube Leg Prototype 01 A.00.stp").unwrap().for_each(|entry| {
        match entry {
            Ok(path) => {
                let stp: String = readstr_by_path(path.as_os_str().to_str().unwrap());
                unsafe {
                    STPSCALE = extact_scale(&stp);
                }

                let exchange = ruststep::parser::parse(&stp).unwrap();
                let table = Table::from_data_section(&exchange.data[0]);

                match extract_main_radius(&table) {
                    None => {}
                    Some((cyls, bends)) => {
                        //MainCylinder::flush_to_pt_files(&cyls);
                        //BendToro::all_to_pt_file(&bends);

                        let extra_len_pts: Vec<Point3> = extract_plane_points(&table);
                        let ops: CncOps = CncOps::new(&cyls, &bends);
                        let mut extra_len_ops = ops.calculate_extra_len(&extra_len_pts);
                        extra_len_ops.triangulate_all();
                        //ops.all_to_one_obj();
                        //ops.flush_to_pt_file();
                        extra_len_ops.triangulate_all();
                        extra_len_ops.all_to_one_obj();
                        //ops.flush_to_pt_file();
                    }
                }
            }
            Err(_) => {
                println!("NO PATH");
            }
        }
    });
}
pub fn readstr_by_path(path: &str) -> String {
    let mut s: String = String::new();
    //match File::open("d:\\pipe_project\\tests\\Dapper_6_truba.stp") {
    match File::open(path) {
        Ok(file) => {
            let mut transcoded = DecodeReaderBytesBuilder::new()
                .encoding(Some(WINDOWS_1251))
                .build(file);
            let mut buf: Vec<u8> = vec![];
            match transcoded.read_to_end(&mut buf) {
                Ok(b) => {
                    match String::from_utf8(buf) {
                        Ok(cont) => {
                            s = cont.clone();
                        }
                        Err(e) => { println!("{:?}", e) }
                    }
                }
                Err(e) => {
                    println!("{:?}", e)
                }
            }
        }
        Err(_) => {}
    }
    s
}
pub fn extact_scale(stp: &String) -> f64 {
    let mut scale: f64 = 1.0;
    stp.split(";").for_each(|line| {
        if (line.contains("CONVERSION_BASED_UNIT")) {
            if (line.to_uppercase().contains("'METRE'")) {
                scale = 1000.0;
            } else if (line.to_uppercase().contains("'INCH'")) {
                scale = 25.4;
            }
        }
    });
    scale
}
pub fn extract_main_radius(table: &Table) -> Option<(Vec<MainCylinder>, Vec<BendToro>)> {
    let (cyls, bend_toros): (Vec<MainCylinder>, Vec<BendToro>) = extract_all_cylynders(table);
    match MainCylinder::calculate_main_diam(&cyls) {
        None => {
            None
        }
        Some(md_cyls) => {
            let cyls_no_dubs = MainCylinder::remove_dublicates(&md_cyls);
            let cyls_merged = MainCylinder::merge(&cyls_no_dubs);
            let main_radius_id = cyls_merged[0].r_gr_id;
            let mut bends: Vec<BendToro> = vec![];
            let bend_toros_no_dublicates = BendToro::remove_dublicates(&bend_toros);
            bend_toros_no_dublicates.iter().for_each(|bend| {
                if (bend.r_gr_id == main_radius_id) {
                    bends.push(bend.clone());
                }
            });
            let merged_tors = BendToro::merge(&bends);
            Some((cyls_merged, merged_tors))
        }
    }
}
pub fn extract_all_cylynders(table: &Table) -> (Vec<MainCylinder>, Vec<(BendToro)>) {
    let mut ret: Vec<(MainCylinder)> = vec![];
    let mut ret_bend: Vec<(BendToro)> = vec![];
    table.shell.iter().for_each(|(k, v)| {
        v.cfs_faces.iter().for_each(|face_holder| {
            match face_holder {
                PlaceHolder::Ref(name) => {
                    let id = name_to_id(name.clone());
                    match table.face_surface.get(&id) {
                        None => { println!("NOT FOUND") }
                        Some(face_holder) => {
                            let face_bounds: &Vec<PlaceHolder<FaceBoundHolder>> = &face_holder.bounds;
                            match &face_holder.face_geometry {
                                PlaceHolder::Ref(name) => {
                                    let mut found = false;
                                    let id = name_to_id(name.clone());
                                    match table.cylindrical_surface.get(&id) {
                                        None => {
                                            //println!("NOT FOUND {:?}",id)
                                        }
                                        Some(cyl) => {
                                            found = true;
                                            let r = cyl.radius * unsafe { STPSCALE };
                                            let (loc, dir, dir_rad) = extract_position3d(table, &cyl.position);
                                            match extract_circles_bounds(table, face_bounds, &loc.unwrap(), &dir.unwrap(), id, r) {
                                                None => {}
                                                Some(cyl) => {
                                                    ret.push(cyl)
                                                }
                                            };
                                        }
                                    }
                                    match table.toroidal_surface.get(&id) {
                                        None => {}
                                        Some(toro) => {
                                            found = true;
                                            let r = toro.minor_radius * unsafe { STPSCALE };
                                            let r_bend = toro.major_radius * unsafe { STPSCALE };
                                            let (loc, bend_plane_normal, dir_rad) = extract_position3d(table, &toro.position);
                                            if (toro.major_radius * unsafe { STPSCALE } > MAX_BEND_RADIUS) {
                                                //TODO potentional error with big tole
                                                match extract_circles_bounds(table, face_bounds, &loc.unwrap(), &bend_plane_normal.unwrap(), id, r) {
                                                    None => {}
                                                    Some(cyl) => {
                                                        ret.push(cyl)
                                                    }
                                                };
                                            } else {
                                                match extract_circles_bounds_tourus(table,
                                                                                    face_bounds,
                                                                                    &loc.unwrap(),
                                                                                    &bend_plane_normal.unwrap(),
                                                                                    &dir_rad.unwrap(),
                                                                                    id.clone(),
                                                                                    r_bend,
                                                                                    r) {
                                                    None => {}
                                                    Some(bend_toro) => {
                                                        //println!("ID {:?} {:?}",bend_toro.r_gr_id,bend_toro.r);
                                                        ret_bend.push(bend_toro);
                                                    }
                                                };
                                            }
                                        }
                                    }
                                    match table.plane.get(&id) {
                                        None => {}
                                        Some(plane) => {
                                            found = true;
                                        }
                                    }
                                    match table.conical_surface.get(&id) {
                                        None => {}
                                        Some(conic) => {
                                            found = true;
                                        }
                                    }
                                    match table.rational_b_spline_surface.get(&id) {
                                        None => {}
                                        Some(surf) => {
                                            found = true;
                                            let weights = &surf.weights_data;
                                            match &surf.non_rational_b_spline_surface {
                                                PlaceHolder::Ref(nr_name) => {}
                                                PlaceHolder::Owned(bssurf) => {
                                                    match bssurf {
                                                        NonRationalBSplineSurfaceHolder::BSplineSurfaceWithKnots(s) => {
                                                            //println!("UniformSurface {:?}",s.control_points_list.len());
                                                        }
                                                        NonRationalBSplineSurfaceHolder::UniformSurface(_) => {
                                                            //println!("UniformSurface")
                                                        }
                                                        NonRationalBSplineSurfaceHolder::QuasiUniformSurface(_) => {
                                                            //println!("QuasiUniformSurface")
                                                        }
                                                        NonRationalBSplineSurfaceHolder::BezierSurface(_) => {
                                                            //println!("BezierSurface")
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                    if (!found) {
                                        println!("ID SURF NOT FOUND {:?}", id);
                                    }
                                }
                                PlaceHolder::Owned(_) => {}
                            }
                        }
                    }
                }
                PlaceHolder::Owned(_) => {}
            }
        })
    });
    (ret, ret_bend)
}
