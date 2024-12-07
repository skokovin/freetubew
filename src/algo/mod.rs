pub mod cnc;
use crate::algo::cnc::{gen_cyl, LRACLR};
use crate::device::{MeshVertex, StepVertexBuffer};
use cgmath::num_traits::real::Real;
use cgmath::{Basis3, Deg, InnerSpace, MetricSpace, Rad, Rotation, Rotation3};
use encoding_rs::WINDOWS_1251;
use encoding_rs_io::DecodeReaderBytesBuilder;
use itertools::{ChunkBy, Itertools};
use rand::{random, Rng};
use ruststep::ast::{Exchange, Name};
use ruststep::tables::PlaceHolder;
use ruststep::tables::PlaceHolder::Ref;
use serde::{Deserialize, Serialize};
use shipyard::Component;
use std::collections::{HashMap, HashSet};
use std::f64::consts::PI;
use std::fs::File;
use std::io::{BufWriter, Read, Write};
use std::ops::{Mul, Sub};
use std::vec::IntoIter;
use cgmath::num_traits::abs;
use cgmath::num_traits::float::FloatCore;
use log::warn;
use ruststep::parser;
use truck_base::bounding_box::BoundingBox;
use truck_base::cgmath64::{Point3, Vector3};
use truck_geometry::nurbs::NurbsCurve;
use truck_geometry::prelude::{BSplineCurve, Plane};
use truck_meshalgo::prelude::*;
use truck_stepio::r#in::{Axis2Placement3dHolder, Axis2PlacementHolder, BSplineCurveWithKnots, CartesianPoint, CartesianPointHolder, CurveAnyHolder, DirectionHolder, FaceBoundHolder, NonRationalBSplineCurveHolder, NonRationalBSplineSurfaceHolder, Table, VectorHolder, VertexPointHolder};
use nom::Finish;

pub const PI36_FLOAT_RANGE: [f64; 36] = {
    let mut v: [f64; 36] = [0.0; 36];
    let mut i = 0;
    loop{
        if i >= 35 {
            v[i]=2.0*PI;
            break;
        }
        v[i]=2.0*PI/36.0 * i as f64;
        i += 1;
    }
    v
};
pub const HALF_PI18_FLOAT_RANGE: [f64; 18] = {
    let mut v: [f64; 18] = [0.0; 18];
    let mut i = 0;
    loop{
        if i >= 17 {
            v[i]=PI;
            break;
        }
        v[i]=PI/18.0 * i as f64;
        i += 1;
    }
    v
};
pub const HALF_PI4_FLOAT_RANGE: [f64; 4] = {
    let mut v: [f64; 4] = [0.0; 4];
    let mut i = 0;
    loop{
        if i >= 3 {
            v[i]=PI;
            break;
        }
        v[i]=PI/4.0 * i as f64;
        i += 1;
    }
    v
};
pub const P_FORWARD: Vector3 = Vector3::new(1.0, 0.0, 0.0);
pub const P_FORWARD_REVERSE: Vector3 = Vector3::new(-1.0, 0.0, 0.0);
pub const P_RIGHT: Vector3 = Vector3::new(0.0, 1.0, 0.0);
pub const P_RIGHT_REVERSE: Vector3 = Vector3::new(0.0, -1.0, 0.0);
pub const P_UP: Vector3 = Vector3::new(0.0, 0.0, 1.0);
pub const P_UP_REVERSE: Vector3 = Vector3::new(0.0, 0.0, -1.0);
pub const ROT_DIR_CCW: f64 = -1.0;
pub const TESS_TOL_ANGLE: f64 = 18.0;
pub const TESS_TOL_TOR_ANGLE: f64 = 18.0;
pub const TESS_TOR_STEP: u64 = 20;
pub const Z_FIGHTING_FACTOR: f32 = 1.0;
const CAP_TRIANGULATION: Rad<f64> = Rad(PI / 180.0);
pub const TOLE: f64 = 0.9;
pub const EXTRA_LEN_CALC: f64 = 3.0;
pub const EXTRA_R_CALC: f64 = 1.2;
pub const MAX_BEND_RADIUS: f64 = 500.0;
pub const DIVIDER: f64 = 100000000.0;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Triangle {
    pub p: [cgmath::Point3<f32>; 3],
    pub normal: cgmath::Vector3<f32>,
}
impl Triangle {
    pub fn new(p0: cgmath::Point3<f32>, p1: cgmath::Point3<f32>, p2: cgmath::Point3<f32>) -> Self {
        let u = p1.sub(p0);
        let v = p2.sub(p0);
        let normal: cgmath::Vector3<f32> = cgmath::Vector3::new(
            u.y * v.z - u.z * v.y,
            u.z * v.x - u.x * v.z,
            u.x * v.y - u.y * v.x,
        );
        Self {
            p: [p0, p1, p2],
            normal: normal,
        }
    }
    pub fn from_f64_without_normals(
        p0_64: cgmath::Point3<f64>,
        p1_64: cgmath::Point3<f64>,
        p2_64: cgmath::Point3<f64>,
    ) -> Self {
        let p0: cgmath::Point3<f32> = cgmath::Point3::new(p0_64.x as f32, p0_64.y as f32, p0_64.z as f32);
        let p1: cgmath::Point3<f32> = cgmath::Point3::new(p1_64.x as f32, p1_64.y as f32, p1_64.z as f32);
        let p2: cgmath::Point3<f32> = cgmath::Point3::new(p2_64.x as f32, p2_64.y as f32, p2_64.z as f32);

        let u = p1.sub(p0);
        let v = p2.sub(p0);
        let normal: cgmath::Vector3<f32> = cgmath::Vector3::new(
            u.y * v.z - u.z * v.y,
            u.z * v.x - u.x * v.z,
            u.x * v.y - u.y * v.x,
        );
        Self {
            p: [p0, p1, p2],
            normal: normal,
        }
    }

    pub fn as_p64(&self) -> [truck_base::cgmath64::Point3; 3] {
        let p64: [truck_base::cgmath64::Point3; 3] = [
            cgmath::Point3::new(self.p[0].x as f64, self.p[0].y as f64, self.p[0].z as f64),
            cgmath::Point3::new(self.p[1].x as f64, self.p[1].y as f64, self.p[1].z as f64),
            cgmath::Point3::new(self.p[2].x as f64, self.p[2].y as f64, self.p[2].z as f64),
        ];
        p64
    }
}

#[derive(Clone)]
pub struct RawMesh {
    pub id: i32,
    pub vertex_normal: Vec<f32>,
    pub indx: Vec<i32>,
    pub bbx: BoundingBox<cgmath::Point3<f64>>,
    pub triangles: Vec<Triangle>,
}
impl Default for RawMesh {
    fn default() -> Self {
        RawMesh {
            id: -999,
            vertex_normal: vec![],
            indx: vec![],
            bbx: BoundingBox::default(),
            triangles: vec![],
        }
    }
}

#[derive(Clone)]
pub struct MainCircle {
    pub id: u64,
    pub radius: f64,
    pub loc: Point3,
    pub dir: Vector3,
    pub radius_dir: Vector3,
    pub r_gr_id: u64,
}
impl MainCircle {
    pub fn gen_points(&self) -> Vec<Point3> {
        let mut pts_a: Vec<Point3> = vec![];
        let mut pts_b: Vec<Point3> = vec![];
        HALF_PI18_FLOAT_RANGE.iter().for_each(|angle| {
            let rotation: Basis3<f64> = Rotation3::from_axis_angle(self.dir, Rad(*angle));
            let nv = rotation.rotate_vector(self.radius_dir);
            let p_a = self.loc + nv * self.radius;
            pts_a.push(p_a);
            let p_b = self.loc + nv * -self.radius;
            pts_b.push(p_b);
        });
        let first = pts_a[0].clone();
        pts_b.remove(0);
        pts_a.extend(pts_b);
        pts_a.push(first);
        pts_a
    }
    pub fn gen_points_low_res(&self) -> Vec<Point3> {
        let mut pts_a: Vec<Point3> = vec![];
        let mut pts_b: Vec<Point3> = vec![];
        HALF_PI4_FLOAT_RANGE.iter().for_each(|angle| {
            let rotation: Basis3<f64> = Rotation3::from_axis_angle(self.dir, Rad(*angle));
            let nv = rotation.rotate_vector(self.radius_dir);
            let p_a = self.loc + nv * self.radius;
            pts_a.push(p_a);
            let p_b = self.loc + nv * -self.radius;
            pts_b.push(p_b);
        });
        let first = pts_a[0].clone();
        pts_b.remove(0);
        pts_a.extend(pts_b);
        pts_a.push(first);
        pts_a
    }

    fn find_next(&self, others: &Vec<MainCircle>) -> Option<MainCircle> {
        let mut candidates: Vec<(f64, MainCircle)> = vec![];
        others.iter().for_each(|other| {
            if (self.id != other.id && (self.radius - other.radius).abs() < TOLE) {
                let p1 = project_point_to_vec(&self.dir, &self.loc, &other.loc);
                let d1 = other.loc.distance(p1);
                let d2 = p1.distance(self.loc);
                if (d1 < TOLE && d2 > TOLE) {
                    candidates.push((d1, other.clone()));
                }
            }
        });
        if (candidates.is_empty()) {
            None
        } else {
            if (candidates.len() == 1) {
                Some(candidates[0].1.clone())
            } else {
                candidates.sort_by(|(dist, cc), (dist1, cc1)| dist.partial_cmp(dist1).unwrap());
                Some(candidates[0].1.clone())
            }
        }
    }
    pub fn is_same_pos(&self, other: &MainCircle) -> bool {
        let is_r = (self.radius - other.radius).abs() < TOLE;
        let dist = self.loc.distance(other.loc) < TOLE;
        let is_coplanar = self.dir.normalize().dot(other.dir.normalize()).abs() - 1.0 < TOLE;
        if (is_r && dist && is_coplanar) {
            true
        } else {
            false
        }
    }
    pub fn remove_dublicates(circles: &Vec<MainCircle>) -> Vec<MainCircle> {
        let mut ret: Vec<MainCircle> = vec![];
        let mut is_exist = false;
        circles.iter().for_each(|cyl_candidate| {
            ret.iter().for_each(|ret_cyl| {
                let is_same = cyl_candidate.is_same_pos(ret_cyl);
                if (!is_exist) {
                    is_exist = is_same;
                }
            });
            if (!is_exist) {
                ret.push(cyl_candidate.clone());
            } else {
                is_exist = false;
            }
        });
        ret
    }
}

#[derive(Clone, Component)]
pub struct MainCylinder {
    pub id: u64,
    pub ca: MainCircle,
    pub cb: MainCircle,
    pub h: f64,
    pub r: f64,
    pub r_gr_id: u64,
    pub ca_tor: u64,
    pub cb_tor: u64,
    pub step_vertex_buffer: StepVertexBuffer,
    pub bbx: BoundingBox<Point3>,
}
impl MainCylinder {
    pub fn from_len(h: f64, r: f64, id: u32) -> MainCylinder {
        let ca = MainCircle {
            id: rand::thread_rng().gen_range(0..1024),
            radius: r,
            loc: Point3::new(0.0, 0.0, 0.0),
            dir: -P_FORWARD,
            radius_dir: P_UP,
            r_gr_id: (round_by_dec(r, 5) * DIVIDER) as u64,
        };

        let cb = MainCircle {
            id: rand::thread_rng().gen_range(0..1024),
            radius: r,
            loc: Point3::new(0.0, 0.0, 0.0) - P_FORWARD * h,
            dir: -P_FORWARD,
            radius_dir: P_UP,
            r_gr_id: (round_by_dec(r, 5) * DIVIDER) as u64,
        };
        let mut mc = MainCylinder {
            id: id as u64,
            ca: ca,
            cb: cb,
            h: h,
            r: r,
            r_gr_id: (round_by_dec(r, 5) * DIVIDER) as u64,
            ca_tor: u64::MAX,
            cb_tor: u64::MAX,

            step_vertex_buffer: StepVertexBuffer::default(),
            bbx: BoundingBox::default(),
        };
        mc.triangulate();
        mc
    }
    pub fn find_by_pt(pt: &Point3, cyls: &Vec<MainCylinder>) -> Option<MainCylinder> {
        let mut found: Option<MainCylinder> = None;
        cyls.iter().for_each(|cyl| match found {
            None => {
                if (pt.distance(cyl.ca.loc) < TOLE) {
                    found = Some(cyl.clone());
                }
                if (pt.distance(cyl.cb.loc) < TOLE) {
                    found = Some(cyl.clone());
                }
            }
            Some(_) => {}
        });
        found
    }
    pub fn gen_points(&self) -> Vec<Point3> {
        let mut pts: Vec<Point3> = vec![];
        HALF_PI18_FLOAT_RANGE.iter().for_each(|angle| {
            let rotation: Basis3<f64> = Rotation3::from_axis_angle(self.ca.dir, Rad(*angle));
            let nv = rotation.rotate_vector(self.ca.radius_dir.clone());
            let p = self.ca.loc.clone() + nv * self.ca.radius;
            pts.push(p);
        });
        HALF_PI18_FLOAT_RANGE.iter().for_each(|angle| {
            let rotation: Basis3<f64> = Rotation3::from_axis_angle(self.cb.dir, Rad(*angle));
            let nv = rotation.rotate_vector(self.cb.radius_dir.clone());
            let p = self.cb.loc.clone() + nv * self.cb.radius;
            pts.push(p);
        });
        pts.push(self.ca.loc.clone());
        pts.push(self.cb.loc.clone());
        pts
    }
    pub fn merge_me(&self, other: &MainCylinder) -> Option<MainCylinder> {
        if (self.id != other.id && self.r_gr_id == other.r_gr_id) {
            if (self.is_other_overlaps_me(other)) {
                let mut acc: Vec<(f64, MainCircle)> = vec![];
                let control_point = self.ca.loc + self.get_dir() * 5000000.0;

                let d1 = self.ca.loc.distance(control_point);
                acc.push((d1, self.ca.clone()));
                let d2 = self.cb.loc.distance(control_point);
                acc.push((d2, self.cb.clone()));

                let d3 = other.ca.loc.distance(control_point);
                acc.push((d3, other.ca.clone()));
                let d4 = other.cb.loc.distance(control_point);
                acc.push((d4, other.cb.clone()));

                acc.sort_by(|(dist, cc), (dist1, cc1)| dist.partial_cmp(dist1).unwrap());

                let ca = acc.first().unwrap().1.clone();
                let cb = acc.last().unwrap().1.clone();
                let d = ca.loc.distance(cb.loc);

                let new_c: MainCylinder = MainCylinder {
                    id: rand::thread_rng().gen_range(0..1024),
                    ca: ca,
                    cb: cb,
                    h: d,
                    r: self.r,
                    r_gr_id: self.r_gr_id,
                    ca_tor: u64::MAX,
                    cb_tor: u64::MAX,

                    step_vertex_buffer: StepVertexBuffer::default(),
                    bbx: BoundingBox::default(),
                };

                //println!("OVERLAPS!!!");
                Some(new_c)
            } else if (self.ca.loc.distance(other.ca.loc) < TOLE) {
                let new_h = self.cb.loc.distance(other.cb.loc);
                let new_c: MainCylinder = MainCylinder {
                    id: rand::thread_rng().gen_range(0..1024),
                    ca: other.cb.clone(),
                    cb: self.cb.clone(),
                    h: new_h,
                    r: self.r,
                    r_gr_id: self.r_gr_id,
                    ca_tor: u64::MAX,
                    cb_tor: u64::MAX,
                    step_vertex_buffer: StepVertexBuffer::default(),
                    bbx: BoundingBox::default(),
                };
                Some(new_c)
            } else if (self.ca.loc.distance(other.cb.loc) < TOLE) {
                let new_h = self.cb.loc.distance(other.ca.loc);
                let new_c: MainCylinder = MainCylinder {
                    id: rand::thread_rng().gen_range(0..1024),
                    ca: other.ca.clone(),
                    cb: self.cb.clone(),
                    h: new_h,
                    r: self.r,
                    r_gr_id: self.r_gr_id,
                    ca_tor: u64::MAX,
                    cb_tor: u64::MAX,
                    step_vertex_buffer: StepVertexBuffer::default(),
                    bbx: BoundingBox::default(),
                };
                Some(new_c)
            } else if (self.cb.loc.distance(other.ca.loc) < TOLE) {
                let new_h = self.ca.loc.distance(other.cb.loc);
                let new_c: MainCylinder = MainCylinder {
                    id: rand::thread_rng().gen_range(0..1024),
                    ca: self.ca.clone(),
                    cb: other.cb.clone(),
                    h: new_h,
                    r: self.r,
                    r_gr_id: self.r_gr_id,
                    ca_tor: u64::MAX,
                    cb_tor: u64::MAX,
                    step_vertex_buffer: StepVertexBuffer::default(),
                    bbx: BoundingBox::default(),
                };
                Some(new_c)
            } else if (self.cb.loc.distance(other.cb.loc) < TOLE) {
                let new_h = self.ca.loc.distance(other.ca.loc);
                let new_c: MainCylinder = MainCylinder {
                    id: rand::thread_rng().gen_range(0..1024),
                    ca: self.ca.clone(),
                    cb: other.ca.clone(),
                    h: new_h,
                    r: self.r,
                    r_gr_id: self.r_gr_id,
                    ca_tor: u64::MAX,
                    cb_tor: u64::MAX,
                    step_vertex_buffer: StepVertexBuffer::default(),
                    bbx: BoundingBox::default(),
                };
                Some(new_c)
            } else {
                None
            }
        } else {
            None
        }
    }
    pub fn get_next(&self, hashids: &HashSet<u64>, cyls: &Vec<MainCylinder>, ) -> Option<MainCylinder> {
        let mut ret: Option<MainCylinder> = None;
        cyls.iter().for_each(|c| {
            if (self.id != c.id && !((c.ca_tor == u64::MAX || c.cb_tor == u64::MAX) && (self.ca_tor == u64::MAX || self.cb_tor == u64::MAX)))
            {
                //if (self.id != c.id) {
                match ret {
                    None => {
                        if (self.ca_tor == c.ca_tor || self.ca_tor == c.cb_tor || self.cb_tor == c.ca_tor || self.cb_tor == c.cb_tor)
                        {
                            match hashids.get(&c.id) {
                                None => ret = Some(c.clone()),
                                Some(_) => {}
                            }
                        }
                    }
                    Some(_) => {}
                }
            }
        });
        ret
    }
    pub fn find_ends(cyls: &Vec<MainCylinder>) -> Option<Vec<MainCylinder>> {
        let mut ret: Vec<MainCylinder> = vec![];
        cyls.iter().for_each(|c| {
            if (c.ca_tor == u64::MAX || c.cb_tor == u64::MAX) {
                ret.push(c.clone());
            }
        });
        if (ret.len() == 2) {
            Some(ret)
        } else {
            warn!("FOUND NOT 2 ENDS {:?}", ret.len());
            None
        }
    }
    pub fn init_tors(cyls: &mut Vec<MainCylinder>, bends: &Vec<BendToro>) {
        cyls.iter_mut().for_each(|c| {
            match BendToro::find_by_pt(&c.ca.loc, bends) {
                None => {}
                Some(toro) => c.ca_tor = toro.id,
            }
            match BendToro::find_by_pt(&c.cb.loc, bends) {
                None => {}
                Some(toro) => c.cb_tor = toro.id,
            }
            //println!("CA {:?} {:?} {:?}",c.id,c.ca_tor,c.cb_tor);
        })
    }
    pub fn is_same_pos(&self, other: &MainCylinder) -> bool {
        if ((self.h - other.h).abs() < TOLE && self.r_gr_id == other.r_gr_id) {
            let a = other.ca.loc.distance(self.ca.loc) < TOLE;
            let b = other.cb.loc.distance(self.cb.loc) < TOLE;
            let c = other.ca.loc.distance(self.cb.loc) < TOLE;
            let d = other.cb.loc.distance(self.ca.loc) < TOLE;
            if ((a && b) || (c && d)) {
                true
            } else {
                false
            }
        } else {
            false
        }
    }
    pub fn is_other_overlaps_me(&self, other: &MainCylinder) -> bool {
        if (!self.is_same_pos(other)) {
            let a0 = (self.ca.loc.distance(other.ca.loc) + self.cb.loc.distance(other.ca.loc) - self.ca.loc.distance(self.cb.loc)).abs() < TOLE;
            let b0 = (self.ca.loc.distance(other.cb.loc) + self.cb.loc.distance(other.cb.loc) - self.ca.loc.distance(self.cb.loc)).abs() < TOLE;
            if (a0 || b0 && a0 != b0) {
                true
            } else {
                false
            }
        } else {
            false
        }
    }
    pub fn is_connected_me(&self, other: &MainCylinder) -> bool {
        if (self.id != other.id) {
            let a = other.ca.loc.distance(self.ca.loc) < TOLE;
            let b = other.ca.loc.distance(self.cb.loc) < TOLE;
            let c = other.cb.loc.distance(self.cb.loc) < TOLE;
            let d = other.cb.loc.distance(self.ca.loc) < TOLE;
            a || b || c || d
        } else {
            false
        }
    }
    pub fn remove_dublicates(cyls: &Vec<MainCylinder>) -> Vec<MainCylinder> {
        let mut ret: Vec<MainCylinder> = vec![];
        let mut is_exist = false;
        cyls.iter().for_each(|cyl_candidate| {
            ret.iter().for_each(|ret_cyl| {
                let is_same = cyl_candidate.is_same_pos(ret_cyl);
                if (!is_exist) {
                    is_exist = is_same;
                }
            });
            if (!is_exist) {
                ret.push(cyl_candidate.clone());
            } else {
                is_exist = false;
            }
        });
        ret
    }
    pub fn calculate_main_diam(cyls_raw: &Vec<MainCylinder>) -> Option<Vec<MainCylinder>> {
        let mut cyls: Vec<MainCylinder> = vec![];
        cyls_raw.clone().iter_mut().for_each(|c| {
            let nh = c.ca.loc.distance(c.cb.loc);
            c.h = nh;

            cyls.push(c.clone());
        });

        let mut ret: Vec<MainCylinder> = vec![];
        let mut rad_group: HashMap<u64, f64> = HashMap::new();
        cyls.iter().for_each(|cyl| match rad_group.get_mut(&cyl.r_gr_id) {
            None => {
                rad_group.insert(cyl.r_gr_id.clone(), cyl.h);
            }
            Some(dist) => {
                *dist = *dist + cyl.h;
            }
        });

        let mut rad_group_vec: Vec<(u64, f64)> = vec![];
        rad_group.iter().for_each(|(id, dist)| {
            rad_group_vec.push((id.clone(), dist.clone()));
        });

        if (rad_group_vec.is_empty()) {
            None
        } else if (rad_group_vec.len() == 1) {
            cyls.iter().for_each(|c| {
                if (c.r_gr_id == rad_group_vec[0].0) {
                    ret.push(c.clone())
                }
            });
            Some(ret)
        } else {
            let mut last_id: u64 = 0;
            let mut lasd_dist: f64 = 0.0;
            let mut tole: f64 = 0.001;
            //rad_group_vec.sort_by(|(id, dist), (id1, dist1)| id.cmp(id1));
            /*    rad_group_vec.iter().for_each(|rg|{
                warn!("id {:?} dist {:?}",rg.0, rg.1);
            });*/

            rad_group_vec.sort_by(|(id, dist), (id1, dist1)| dist.partial_cmp(dist1).unwrap());
            rad_group_vec.inverse().iter().for_each(|(id, dist)| {
                if ((dist.clone() * 1000.0) as u32 >= (lasd_dist * 1000.0) as u32) {
                    if (id.clone() > last_id) {
                        last_id = id.clone();
                        lasd_dist = dist.clone();
                    }
                }
            });

            cyls.iter().for_each(|c| {
                if (c.r_gr_id == last_id) {
                    ret.push(c.clone())
                }
            });
            Some(ret)
        }
    }
    pub fn get_dir(&self) -> Vector3 {
        self.cb.loc.sub(self.ca.loc).normalize()
    }
    pub fn merge(main_cyls: &Vec<MainCylinder>) -> Vec<MainCylinder> {
        let mut ret: Vec<MainCylinder> = vec![];
        let mut merged_ids: HashSet<u64> = HashSet::new();
        let mut merged: Vec<MainCylinder> = vec![];
        main_cyls.iter().for_each(|candidate| match merged_ids.get(&candidate.id) {
            None => main_cyls.iter().for_each(|other| match merged_ids.get(&other.id) {
                None => match candidate.merge_me(other) {
                    None => {}
                    Some(merged_cyl) => {
                        merged.push(merged_cyl);
                        merged_ids.insert(other.id);
                        merged_ids.insert(candidate.id);
                    }
                },
                Some(_) => {}
            }),
            Some(_) => {}
        });
        main_cyls.iter().for_each(|cyl| match merged_ids.get(&cyl.id) {
            None => {
                ret.push(cyl.clone());
            }
            Some(_) => {}
        });
        ret.extend_from_slice(merged.as_slice());
        ret
    }
    pub fn reverse_my_points(&mut self) {
        let tmp = self.ca.clone();
        self.ca = self.cb.clone();
        self.cb = tmp;
    }
    pub fn reverse_my_ends(&mut self) {
        let tmp = self.ca_tor.clone();
        self.ca_tor = self.cb_tor.clone();
        self.cb_tor = tmp;
    }
    pub fn triangulate(&mut self) {
        let mut index: u32 = 0;
        let mut indxes: Vec<u32> = vec![];
        let mut buffer: Vec<MeshVertex> = vec![];
        let main_dir = self.cb.loc.sub(self.ca.loc).normalize();
        let h = self.h;
        let tesspoints = self.ca.gen_points();

        for i in 0..tesspoints.iter().len() - 2 {
            let p0 = tesspoints[i];
            let p1 = tesspoints[i + 1];
            let p2 = p0.clone() + main_dir * h;
            let p3 = p1.clone() + main_dir * h;

            self.bbx.push(p0);
            self.bbx.push(p1);
            self.bbx.push(p2);
            self.bbx.push(p3);

            let plane = Plane::new(p0.clone(), p1.clone(), p3.clone());
            let n = plane.normal().normalize();
            let n_arr = [n.x as f32, n.y as f32, n.z as f32, 0.0];
            let r_dir = p0.sub(self.ca.loc);
            let is_coplanar = n.dot(r_dir);
            if (is_coplanar < 0.0) {
                {
                    let mv0 = MeshVertex {
                        position: [p0.x as f32, p0.y as f32, p0.z as f32, 1.0],
                        normal: n_arr,
                        id: self.id as i32,
                        tex_coords: [0.0, 0.0],
                    };
                    buffer.push(mv0);
                    indxes.push(index);
                    index = index + 1;

                    let mv1 = MeshVertex {
                        position: [p3.x as f32, p3.y as f32, p3.z as f32, 1.0],
                        normal: n_arr,
                        id: self.id as i32,
                        tex_coords: [0.0, 0.0],
                    };
                    buffer.push(mv1);
                    indxes.push(index);
                    index = index + 1;

                    let mv2 = MeshVertex {
                        position: [p1.x as f32, p1.y as f32, p1.z as f32, 1.0],
                        normal: n_arr,
                        id: self.id as i32,
                        tex_coords: [0.0, 0.0],
                    };
                    buffer.push(mv2);
                    indxes.push(index);
                    index = index + 1;
                }
                {
                    let mv0 = MeshVertex {
                        position: [p0.x as f32, p0.y as f32, p0.z as f32, 1.0],
                        normal: n_arr,
                        id: self.id as i32,
                        tex_coords: [0.0, 0.0],
                    };
                    buffer.push(mv0);
                    indxes.push(index);
                    index = index + 1;

                    let mv1 = MeshVertex {
                        position: [p2.x as f32, p2.y as f32, p2.z as f32, 1.0],
                        normal: n_arr,
                        id: self.id as i32,
                        tex_coords: [0.0, 0.0],
                    };
                    buffer.push(mv1);
                    indxes.push(index);
                    index = index + 1;

                    let mv2 = MeshVertex {
                        position: [p3.x as f32, p3.y as f32, p3.z as f32, 1.0],
                        normal: n_arr,
                        id: self.id as i32,
                        tex_coords: [0.0, 0.0],
                    };
                    buffer.push(mv2);
                    indxes.push(index);
                    index = index + 1;
                }
            } else {
                {
                    let mv0 = MeshVertex {
                        position: [p0.x as f32, p0.y as f32, p0.z as f32, 1.0],
                        normal: n_arr,
                        id: self.id as i32,
                        tex_coords: [0.0, 0.0],
                    };
                    buffer.push(mv0);
                    indxes.push(index);
                    index = index + 1;

                    let mv1 = MeshVertex {
                        position: [p1.x as f32, p1.y as f32, p1.z as f32, 1.0],
                        normal: n_arr,
                        id: self.id as i32,
                        tex_coords: [0.0, 0.0],
                    };
                    buffer.push(mv1);
                    indxes.push(index);
                    index = index + 1;

                    let mv2 = MeshVertex {
                        position: [p3.x as f32, p3.y as f32, p3.z as f32, 1.0],
                        normal: n_arr,
                        id: self.id as i32,
                        tex_coords: [0.0, 0.0],
                    };
                    buffer.push(mv2);
                    indxes.push(index);
                    index = index + 1;
                }
                {
                    let mv0 = MeshVertex {
                        position: [p0.x as f32, p0.y as f32, p0.z as f32, 1.0],
                        normal: n_arr,
                        id: self.id as i32,
                        tex_coords: [0.0, 0.0],
                    };
                    buffer.push(mv0);
                    indxes.push(index);
                    index = index + 1;

                    let mv1 = MeshVertex {
                        position: [p3.x as f32, p3.y as f32, p3.z as f32, 1.0],
                        normal: n_arr,
                        id: self.id as i32,
                        tex_coords: [0.0, 0.0],
                    };
                    buffer.push(mv1);
                    indxes.push(index);
                    index = index + 1;

                    let mv2 = MeshVertex {
                        position: [p2.x as f32, p2.y as f32, p2.z as f32, 1.0],
                        normal: n_arr,
                        id: self.id as i32,
                        tex_coords: [0.0, 0.0],
                    };
                    buffer.push(mv2);
                    indxes.push(index);
                    index = index + 1;
                }
            }

            let inv_normal = self.ca.dir.mul(-1.0);
            let inv_normal_arr = [
                inv_normal.x as f32,
                inv_normal.y as f32,
                inv_normal.z as f32,
                0.0,
            ];

            {
                let cap0 = MeshVertex {
                    position: [p0.x as f32, p0.y as f32, p0.z as f32, 1.0],
                    normal: inv_normal_arr,
                    id: self.id as i32,
                    tex_coords: [0.0, 0.0],
                };
                buffer.push(cap0);
                indxes.push(index);
                index = index + 1;

                let cap1 = MeshVertex {
                    position: [p1.x as f32, p1.y as f32, p1.z as f32, 1.0],
                    normal: inv_normal_arr,
                    id: self.id as i32,
                    tex_coords: [0.0, 0.0],
                };
                buffer.push(cap1);
                indxes.push(index);
                index = index + 1;

                let cap2 = MeshVertex {
                    position: [
                        self.ca.loc.x as f32,
                        self.ca.loc.y as f32,
                        self.ca.loc.z as f32,
                        1.0,
                    ],
                    normal: inv_normal_arr,
                    id: self.id as i32,
                    tex_coords: [0.0, 0.0],
                };
                buffer.push(cap2);
                indxes.push(index);
                index = index + 1;
            }

            let cap_normal = self.ca.dir;
            let cap_normal_arr = [
                cap_normal.x as f32,
                cap_normal.y as f32,
                cap_normal.z as f32,
                0.0,
            ];

            {
                let cap0 = MeshVertex {
                    position: [p2.x as f32, p2.y as f32, p2.z as f32, 1.0],
                    normal: cap_normal_arr,
                    id: self.id as i32,
                    tex_coords: [0.0, 0.0],
                };
                buffer.push(cap0);
                indxes.push(index);
                index = index + 1;

                let cap1 = MeshVertex {
                    position: [p3.x as f32, p3.y as f32, p3.z as f32, 1.0],
                    normal: cap_normal_arr,
                    id: self.id as i32,
                    tex_coords: [0.0, 0.0],
                };
                buffer.push(cap1);
                indxes.push(index);
                index = index + 1;

                let cap2 = MeshVertex {
                    position: [
                        self.cb.loc.x as f32,
                        self.cb.loc.y as f32,
                        self.cb.loc.z as f32,
                        1.0,
                    ],
                    normal: cap_normal_arr,
                    id: self.id as i32,
                    tex_coords: [0.0, 0.0],
                };
                buffer.push(cap2);
                indxes.push(index);
                index = index + 1;
            }
        }

        self.step_vertex_buffer = StepVertexBuffer { buffer, indxes };
    }
    pub fn triangulate_low_res(&mut self) -> Vec<Point3> {
        let mut index: u32 = 0;
        let mut indxes: Vec<u32> = vec![];
        let mut buffer: Vec<MeshVertex> = vec![];
        let main_dir = self.cb.loc.sub(self.ca.loc).normalize();
        let h = self.h;
        let tesspoints: Vec<Point3> = self.ca.gen_points_low_res();

        for i in 0..tesspoints.iter().len() - 2 {
            let p0 = tesspoints[i];
            let p1 = tesspoints[i + 1];
            let p2 = p0.clone() + main_dir * h;
            let p3 = p1.clone() + main_dir * h;

            self.bbx.push(p0);
            self.bbx.push(p1);
            self.bbx.push(p2);
            self.bbx.push(p3);

            let plane = Plane::new(p0.clone(), p1.clone(), p3.clone());
            let n = plane.normal().normalize();
            let n_arr = [n.x as f32, n.y as f32, n.z as f32, 0.0];
            let r_dir = p0.sub(self.ca.loc);
            let is_coplanar = n.dot(r_dir);
            if (is_coplanar < 0.0) {
                {
                    let mv0 = MeshVertex {
                        position: [p0.x as f32, p0.y as f32, p0.z as f32, 1.0],
                        normal: n_arr,
                        id: self.id as i32,
                        tex_coords: [0.0, 0.0],
                    };
                    buffer.push(mv0);
                    indxes.push(index);
                    index = index + 1;

                    let mv1 = MeshVertex {
                        position: [p3.x as f32, p3.y as f32, p3.z as f32, 1.0],
                        normal: n_arr,
                        id: self.id as i32,
                        tex_coords: [0.0, 0.0],
                    };
                    buffer.push(mv1);
                    indxes.push(index);
                    index = index + 1;

                    let mv2 = MeshVertex {
                        position: [p1.x as f32, p1.y as f32, p1.z as f32, 1.0],
                        normal: n_arr,
                        id: self.id as i32,
                        tex_coords: [0.0, 0.0],
                    };
                    buffer.push(mv2);
                    indxes.push(index);
                    index = index + 1;
                }
                {
                    let mv0 = MeshVertex {
                        position: [p0.x as f32, p0.y as f32, p0.z as f32, 1.0],
                        normal: n_arr,
                        id: self.id as i32,
                        tex_coords: [0.0, 0.0],
                    };
                    buffer.push(mv0);
                    indxes.push(index);
                    index = index + 1;

                    let mv1 = MeshVertex {
                        position: [p2.x as f32, p2.y as f32, p2.z as f32, 1.0],
                        normal: n_arr,
                        id: self.id as i32,
                        tex_coords: [0.0, 0.0],
                    };
                    buffer.push(mv1);
                    indxes.push(index);
                    index = index + 1;

                    let mv2 = MeshVertex {
                        position: [p3.x as f32, p3.y as f32, p3.z as f32, 1.0],
                        normal: n_arr,
                        id: self.id as i32,
                        tex_coords: [0.0, 0.0],
                    };
                    buffer.push(mv2);
                    indxes.push(index);
                    index = index + 1;
                }
            } else {
                {
                    let mv0 = MeshVertex {
                        position: [p0.x as f32, p0.y as f32, p0.z as f32, 1.0],
                        normal: n_arr,
                        id: self.id as i32,
                        tex_coords: [0.0, 0.0],
                    };
                    buffer.push(mv0);
                    indxes.push(index);
                    index = index + 1;

                    let mv1 = MeshVertex {
                        position: [p1.x as f32, p1.y as f32, p1.z as f32, 1.0],
                        normal: n_arr,
                        id: self.id as i32,
                        tex_coords: [0.0, 0.0],
                    };
                    buffer.push(mv1);
                    indxes.push(index);
                    index = index + 1;

                    let mv2 = MeshVertex {
                        position: [p3.x as f32, p3.y as f32, p3.z as f32, 1.0],
                        normal: n_arr,
                        id: self.id as i32,
                        tex_coords: [0.0, 0.0],
                    };
                    buffer.push(mv2);
                    indxes.push(index);
                    index = index + 1;
                }
                {
                    let mv0 = MeshVertex {
                        position: [p0.x as f32, p0.y as f32, p0.z as f32, 1.0],
                        normal: n_arr,
                        id: self.id as i32,
                        tex_coords: [0.0, 0.0],
                    };
                    buffer.push(mv0);
                    indxes.push(index);
                    index = index + 1;

                    let mv1 = MeshVertex {
                        position: [p3.x as f32, p3.y as f32, p3.z as f32, 1.0],
                        normal: n_arr,
                        id: self.id as i32,
                        tex_coords: [0.0, 0.0],
                    };
                    buffer.push(mv1);
                    indxes.push(index);
                    index = index + 1;

                    let mv2 = MeshVertex {
                        position: [p2.x as f32, p2.y as f32, p2.z as f32, 1.0],
                        normal: n_arr,
                        id: self.id as i32,
                        tex_coords: [0.0, 0.0],
                    };
                    buffer.push(mv2);
                    indxes.push(index);
                    index = index + 1;
                }
            }

            let inv_normal = self.ca.dir.mul(-1.0);
            let inv_normal_arr = [
                inv_normal.x as f32,
                inv_normal.y as f32,
                inv_normal.z as f32,
                0.0,
            ];

            {
                let cap0 = MeshVertex {
                    position: [p0.x as f32, p0.y as f32, p0.z as f32, 1.0],
                    normal: inv_normal_arr,
                    id: self.id as i32,
                    tex_coords: [0.0, 0.0],
                };
                buffer.push(cap0);
                indxes.push(index);
                index = index + 1;

                let cap1 = MeshVertex {
                    position: [p1.x as f32, p1.y as f32, p1.z as f32, 1.0],
                    normal: inv_normal_arr,
                    id: self.id as i32,
                    tex_coords: [0.0, 0.0],
                };
                buffer.push(cap1);
                indxes.push(index);
                index = index + 1;

                let cap2 = MeshVertex {
                    position: [
                        self.ca.loc.x as f32,
                        self.ca.loc.y as f32,
                        self.ca.loc.z as f32,
                        1.0,
                    ],
                    normal: inv_normal_arr,
                    id: self.id as i32,
                    tex_coords: [0.0, 0.0],
                };
                buffer.push(cap2);
                indxes.push(index);
                index = index + 1;
            }

            let cap_normal = self.ca.dir;
            let cap_normal_arr = [
                cap_normal.x as f32,
                cap_normal.y as f32,
                cap_normal.z as f32,
                0.0,
            ];

            {
                let cap0 = MeshVertex {
                    position: [p2.x as f32, p2.y as f32, p2.z as f32, 1.0],
                    normal: cap_normal_arr,
                    id: self.id as i32,
                    tex_coords: [0.0, 0.0],
                };
                buffer.push(cap0);
                indxes.push(index);
                index = index + 1;

                let cap1 = MeshVertex {
                    position: [p3.x as f32, p3.y as f32, p3.z as f32, 1.0],
                    normal: cap_normal_arr,
                    id: self.id as i32,
                    tex_coords: [0.0, 0.0],
                };
                buffer.push(cap1);
                indxes.push(index);
                index = index + 1;

                let cap2 = MeshVertex {
                    position: [
                        self.cb.loc.x as f32,
                        self.cb.loc.y as f32,
                        self.cb.loc.z as f32,
                        1.0,
                    ],
                    normal: cap_normal_arr,
                    id: self.id as i32,
                    tex_coords: [0.0, 0.0],
                };
                buffer.push(cap2);
                indxes.push(index);
                index = index + 1;
            }
        }

        self.step_vertex_buffer = StepVertexBuffer { buffer, indxes };
        tesspoints
    }
    pub fn triangulate_with_start_index(&mut self, index: u32) -> u32 {
        let mut index: u32 = index;
        let mut indxes: Vec<u32> = vec![];
        let mut buffer: Vec<MeshVertex> = vec![];
        let main_dir = self.cb.loc.sub(self.ca.loc).normalize();
        let h = self.h;
        let tesspoints = self.ca.gen_points();

        for i in 0..tesspoints.iter().len() - 2 {
            let p0 = tesspoints[i];
            let p1 = tesspoints[i + 1];
            let p2 = p0.clone() + main_dir * h;
            let p3 = p1.clone() + main_dir * h;

            self.bbx.push(p0);
            self.bbx.push(p1);
            self.bbx.push(p2);
            self.bbx.push(p3);

            let plane = Plane::new(p0.clone(), p1.clone(), p3.clone());
            let n = plane.normal().normalize();
            let mut n_arr = [n.x as f32, n.y as f32, n.z as f32, 0.0];
            let r_dir = p0.sub(self.ca.loc);
            let is_coplanar = n.dot(r_dir);
            if (is_coplanar < 0.0) {
                n_arr = [-n_arr[0], -n_arr[1], -n_arr[2], 0.0];
            }
            {
                let mv0 = MeshVertex {
                    position: [p0.x as f32, p0.y as f32, p0.z as f32, 1.0],
                    normal: n_arr,
                    id: self.id as i32,
                    tex_coords: [0.0, 0.0],
                };
                buffer.push(mv0);
                indxes.push(index);
                index = index + 1;

                let mv1 = MeshVertex {
                    position: [p1.x as f32, p1.y as f32, p1.z as f32, 1.0],
                    normal: n_arr,
                    id: self.id as i32,
                    tex_coords: [0.0, 0.0],
                };
                buffer.push(mv1);
                indxes.push(index);
                index = index + 1;

                let mv2 = MeshVertex {
                    position: [p3.x as f32, p3.y as f32, p3.z as f32, 1.0],
                    normal: n_arr,
                    id: self.id as i32,
                    tex_coords: [0.0, 0.0],
                };
                buffer.push(mv2);
                indxes.push(index);
                index = index + 1;
            }
            {
                let mv0 = MeshVertex {
                    position: [p0.x as f32, p0.y as f32, p0.z as f32, 1.0],
                    normal: n_arr,
                    id: self.id as i32,
                    tex_coords: [0.0, 0.0],
                };
                buffer.push(mv0);
                indxes.push(index);
                index = index + 1;

                let mv1 = MeshVertex {
                    position: [p3.x as f32, p3.y as f32, p3.z as f32, 1.0],
                    normal: n_arr,
                    id: self.id as i32,
                    tex_coords: [0.0, 0.0],
                };
                buffer.push(mv1);
                indxes.push(index);
                index = index + 1;

                let mv2 = MeshVertex {
                    position: [p2.x as f32, p2.y as f32, p2.z as f32, 1.0],
                    normal: n_arr,
                    id: self.id as i32,
                    tex_coords: [0.0, 0.0],
                };
                buffer.push(mv2);
                indxes.push(index);
                index = index + 1;
            }

            let inv_normal = self.ca.dir.mul(-1.0);
            let inv_normal_arr = [
                inv_normal.x as f32,
                inv_normal.y as f32,
                inv_normal.z as f32,
                0.0,
            ];

            {
                let cap0 = MeshVertex {
                    position: [p0.x as f32, p0.y as f32, p0.z as f32, 1.0],
                    normal: inv_normal_arr,
                    id: self.id as i32,
                    tex_coords: [0.0, 0.0],
                };
                buffer.push(cap0);
                indxes.push(index);
                index = index + 1;

                let cap1 = MeshVertex {
                    position: [p1.x as f32, p1.y as f32, p1.z as f32, 1.0],
                    normal: inv_normal_arr,
                    id: self.id as i32,
                    tex_coords: [0.0, 0.0],
                };
                buffer.push(cap1);
                indxes.push(index);
                index = index + 1;

                let cap2 = MeshVertex {
                    position: [
                        self.ca.loc.x as f32,
                        self.ca.loc.y as f32,
                        self.ca.loc.z as f32,
                        1.0,
                    ],
                    normal: inv_normal_arr,
                    id: self.id as i32,
                    tex_coords: [0.0, 0.0],
                };
                buffer.push(cap2);
                indxes.push(index);
                index = index + 1;
            }

            let cap_normal = self.ca.dir;
            let cap_normal_arr = [
                cap_normal.x as f32,
                cap_normal.y as f32,
                cap_normal.z as f32,
                0.0,
            ];

            {
                let cap0 = MeshVertex {
                    position: [p2.x as f32, p2.y as f32, p2.z as f32, 1.0],
                    normal: cap_normal_arr,
                    id: self.id as i32,
                    tex_coords: [0.0, 0.0],
                };
                buffer.push(cap0);
                indxes.push(index);
                index = index + 1;

                let cap1 = MeshVertex {
                    position: [p3.x as f32, p3.y as f32, p3.z as f32, 1.0],
                    normal: cap_normal_arr,
                    id: self.id as i32,
                    tex_coords: [0.0, 0.0],
                };
                buffer.push(cap1);
                indxes.push(index);
                index = index + 1;

                let cap2 = MeshVertex {
                    position: [
                        self.cb.loc.x as f32,
                        self.cb.loc.y as f32,
                        self.cb.loc.z as f32,
                        1.0,
                    ],
                    normal: cap_normal_arr,
                    id: self.id as i32,
                    tex_coords: [0.0, 0.0],
                };
                buffer.push(cap2);
                indxes.push(index);
                index = index + 1;
            }
        }

        self.step_vertex_buffer = StepVertexBuffer { buffer, indxes };
        index
    }
    pub fn triangulate_with_start_index_no_cap(&mut self, index: u32) -> u32 {
        let mut index: u32 = index;
        let mut indxes: Vec<u32> = vec![];
        let mut buffer: Vec<MeshVertex> = vec![];
        let main_dir = self.cb.loc.sub(self.ca.loc).normalize();
        let h = self.h;
        let tesspoints = self.ca.gen_points();

        for i in 0..tesspoints.iter().len() - 2 {
            let p0 = tesspoints[i];
            let p1 = tesspoints[i + 1];
            let p2 = p0.clone() + main_dir * h;
            let p3 = p1.clone() + main_dir * h;

            self.bbx.push(p0);
            self.bbx.push(p1);
            self.bbx.push(p2);
            self.bbx.push(p3);

            let plane = Plane::new(p0.clone(), p1.clone(), p3.clone());
            let n = plane.normal().normalize();
            let mut n_arr = [n.x as f32, n.y as f32, n.z as f32, 0.0];
            let r_dir = p0.sub(self.ca.loc);
            let is_coplanar = n.dot(r_dir);
            if (is_coplanar < 0.0) {
                //let n_rev = plane.normal().normalize().mul(-1.0);
                n_arr = [-n_arr[0], -n_arr[1], -n_arr[2], 0.0];
            }
            {
                let mv0 = MeshVertex {
                    position: [p0.x as f32, p0.y as f32, p0.z as f32, 1.0],
                    normal: n_arr,
                    id: self.id as i32,
                    tex_coords: [0.0, 0.0],
                };
                buffer.push(mv0);
                indxes.push(index);
                index = index + 1;

                let mv1 = MeshVertex {
                    position: [p1.x as f32, p1.y as f32, p1.z as f32, 1.0],
                    normal: n_arr,
                    id: self.id as i32,
                    tex_coords: [0.0, 0.0],
                };
                buffer.push(mv1);
                indxes.push(index);
                index = index + 1;

                let mv2 = MeshVertex {
                    position: [p3.x as f32, p3.y as f32, p3.z as f32, 1.0],
                    normal: n_arr,
                    id: self.id as i32,
                    tex_coords: [0.0, 0.0],
                };
                buffer.push(mv2);
                indxes.push(index);
                index = index + 1;
            }
            {
                let mv0 = MeshVertex {
                    position: [p0.x as f32, p0.y as f32, p0.z as f32, 1.0],
                    normal: n_arr,
                    id: self.id as i32,
                    tex_coords: [0.0, 0.0],
                };
                buffer.push(mv0);
                indxes.push(index);
                index = index + 1;

                let mv1 = MeshVertex {
                    position: [p3.x as f32, p3.y as f32, p3.z as f32, 1.0],
                    normal: n_arr,
                    id: self.id as i32,
                    tex_coords: [0.0, 0.0],
                };
                buffer.push(mv1);
                indxes.push(index);
                index = index + 1;

                let mv2 = MeshVertex {
                    position: [p2.x as f32, p2.y as f32, p2.z as f32, 1.0],
                    normal: n_arr,
                    id: self.id as i32,
                    tex_coords: [0.0, 0.0],
                };
                buffer.push(mv2);
                indxes.push(index);
                index = index + 1;
            }
        }

        self.step_vertex_buffer = StepVertexBuffer { buffer, indxes };
        index
    }
    pub fn recalculate_h(&mut self) {
        self.h = self.cb.loc.distance(self.ca.loc);
    }
}
#[derive(Clone, Component)]
pub struct BendToro {
    pub id: u64,
    pub r: f64,
    pub bend_radius: f64,
    pub bend_center_point: Point3,
    pub bend_plane_norm: Vector3,
    pub radius_dir: Vector3,
    pub ca: MainCircle,
    pub cb: MainCircle,
    pub r_gr_id: u64,
    pub step_vertex_buffer: StepVertexBuffer,
    pub bbx: BoundingBox<Point3>,
}
impl BendToro {
    pub fn from_angle(radians_angle: f64, bend_radius: f64, r: f64, id: u32) -> BendToro {
        let start_point: Point3 = Point3::new(0.0, 0.0, 0.0);
        let dorn_point: Point3 = Point3::new(0.0, r + bend_radius, 0.0);
        let bend_plane_norm: Vector3 = P_UP;

        let rotation: Basis3<f64> = Rotation3::from_axis_angle(bend_plane_norm, -Rad(radians_angle));
        let p_tmp = rotation.rotate_point(Point3::new(0.0, -r - bend_radius, 0.0));
        let end_point = Point3::new(p_tmp.x, p_tmp.y + r + bend_radius, p_tmp.z);

        let ca = MainCircle {
            id: rand::thread_rng().gen_range(0..1024),
            radius: r,
            loc: start_point,
            dir: -P_FORWARD,
            radius_dir: P_UP,
            r_gr_id: (round_by_dec(r, 5) * DIVIDER) as u64,
        };
        let cb = MainCircle {
            id: rand::thread_rng().gen_range(0..1024),
            radius: r,
            loc: end_point,
            dir: -P_FORWARD,
            radius_dir: P_UP,
            r_gr_id: (round_by_dec(r, 5) * DIVIDER) as u64,
        };

        let mut tor = BendToro {
            id: rand::thread_rng().gen_range(0..1024),
            r: r,
            bend_radius: bend_radius,
            bend_center_point: dorn_point,
            bend_plane_norm: bend_plane_norm,
            radius_dir: bend_plane_norm,
            ca: ca,
            cb: cb,
            r_gr_id: (round_by_dec(r, 5) * DIVIDER) as u64,
            step_vertex_buffer: StepVertexBuffer::default(),
            bbx: BoundingBox::default(),
        };
        tor.triangulate(&P_FORWARD_REVERSE);
        tor
    }
    pub fn angle(&self) -> Rad<f64> {
        let sv = self.ca.loc.sub(self.bend_center_point);
        let ev = self.cb.loc.sub(self.bend_center_point);
        sv.angle(ev)
    }
    pub fn is_same_pos(&self, other: &BendToro) -> bool {
        if (self.r_gr_id == other.r_gr_id) {
            let a = other.ca.loc.distance(self.ca.loc) < TOLE;
            let b = other.cb.loc.distance(self.cb.loc) < TOLE;
            let c = other.ca.loc.distance(self.cb.loc) < TOLE;
            let d = other.cb.loc.distance(self.ca.loc) < TOLE;
            if ((a && b) || (c && d)) {
                true
            } else {
                false
            }
        } else {
            false
        }
    }
    pub fn remove_dublicates(cyls: &Vec<BendToro>) -> Vec<BendToro> {
        let mut ret: Vec<BendToro> = vec![];
        let mut is_exist = false;
        cyls.iter().for_each(|cyl_candidate| {
            ret.iter().for_each(|ret_cyl| {
                let is_same = cyl_candidate.is_same_pos(ret_cyl);
                if (!is_exist) {
                    is_exist = is_same;
                }
            });
            if (!is_exist) {
                ret.push(cyl_candidate.clone());
            } else {
                is_exist = false;
            }
        });
        ret
    }
    pub fn find_by_pt(pt: &Point3, toros: &Vec<BendToro>) -> Option<BendToro> {
        let mut ret_tor: Option<BendToro> = None;
        toros.iter().for_each(|t| match ret_tor {
            None => {
                if (pt.clone().distance(t.ca.loc) < TOLE) {
                    ret_tor = Some((t.clone()));
                }
                if (pt.clone().distance(t.cb.loc) < TOLE) {
                    ret_tor = Some((t.clone()));
                }
            }
            Some(_) => {}
        });
        ret_tor
    }
    pub fn merge(main_toros: &Vec<BendToro>) -> Vec<BendToro> {
        let mut ret: Vec<BendToro> = vec![];
        let mut merged: HashSet<u64> = HashSet::new();
        let mut new_tors: Vec<BendToro> = vec![];
        main_toros.iter().for_each(|im| match merged.get(&im.id) {
            None => {
                main_toros.iter().for_each(|other| match merged.get(&other.id) {
                    None => {
                        if (im.id != other.id) {
                            let mut has_same_points = false;
                            let mut new_circles: Vec<MainCircle> = vec![];
                            if (!has_same_points) {
                                has_same_points = im.ca.is_same_pos(&other.ca);
                                if (has_same_points) {
                                    new_circles.push(im.cb.clone());
                                    new_circles.push(other.cb.clone());
                                }
                            }
                            if (!has_same_points) {
                                has_same_points = im.ca.is_same_pos(&other.cb);
                                if (has_same_points) {
                                    new_circles.push(im.cb.clone());
                                    new_circles.push(other.ca.clone());
                                }
                            }
                            if (!has_same_points) {
                                has_same_points = im.cb.is_same_pos(&other.ca);
                                if (has_same_points) {
                                    new_circles.push(im.ca.clone());
                                    new_circles.push(other.cb.clone());
                                }
                            }
                            if (!has_same_points) {
                                has_same_points = im.cb.is_same_pos(&other.cb);
                                if (has_same_points) {
                                    new_circles.push(im.ca.clone());
                                    new_circles.push(other.ca.clone());
                                }
                            }
                            if (has_same_points) {
                                merged.insert(im.id.clone());
                                merged.insert(other.id.clone());

                                let new_tor = BendToro {
                                    id: rand::thread_rng().gen_range(0..1024),
                                    r: im.r,
                                    bend_radius: im.bend_radius,
                                    bend_center_point: im.bend_center_point.clone(),
                                    bend_plane_norm: im.bend_plane_norm.clone(),
                                    radius_dir: im.radius_dir.clone(),
                                    ca: new_circles[0].clone(),
                                    cb: new_circles[1].clone(),
                                    r_gr_id: im.r_gr_id,
                                    step_vertex_buffer: StepVertexBuffer::default(),
                                    bbx: BoundingBox::default(),
                                };

                                new_tors.push(new_tor);
                            }
                        }
                    }
                    Some(_) => {}
                });
            }
            Some(_) => {}
        });
        main_toros.iter().for_each(|mt| match merged.get(&mt.id) {
            None => {
                ret.push(mt.clone());
            }
            Some(_) => {}
        });
        ret.extend_from_slice(new_tors.as_slice());
        ret
    }
    pub fn reverse_my_points(&mut self) {
        let tmp = self.ca.clone();
        self.ca = self.cb.clone();
        self.cb = tmp;
    }
    pub fn up_dir(&self) -> cgmath::Vector3<f64> {
        let plane = Plane::new(self.bend_center_point, self.ca.loc, self.cb.loc.clone());
        let n = plane.normal();
        let tmp_p = self.bend_center_point.clone() + n * self.bend_radius;
        tmp_p.sub(self.bend_center_point).normalize()
    }
    pub fn triangulate(&mut self, prev_dir: &Vector3) {
        let mut index: u32 = 0;
        let mut indxes: Vec<u32> = vec![];
        let mut buffer: Vec<MeshVertex> = vec![];

        let bend_s_dir = self.ca.loc.sub(self.bend_center_point);
        let bend_e_dir = self.cb.loc.sub(self.bend_center_point);
        let bend_diag_dir = self.cb.loc.sub(self.ca.loc).normalize();
        //println!("ANGLE {:?}", bend_e_dir.angle(bend_s_dir).0);
        let angle_step = bend_s_dir.angle(bend_e_dir).0 / TESS_TOR_STEP as f64;
        let angle_step_rev = (2.0 * PI - bend_s_dir.angle(bend_e_dir).0) / TESS_TOR_STEP as f64;
        let up_dir = self.up_dir().normalize();

        let mut anchors: Vec<Point3> = {
            let mut anchors_stright: Vec<Point3> = vec![];
            let mut anchors_rev: Vec<Point3> = vec![];

            let mut curr_angle_stright = 0.0;
            for i in 0..=TESS_TOR_STEP {
                let rotation: Basis3<f64> = Rotation3::from_axis_angle(up_dir, Rad(curr_angle_stright));
                let nv = rotation.rotate_vector(bend_s_dir.clone());
                let p = self.bend_center_point.clone() + nv;
                anchors_stright.push(p);
                curr_angle_stright = curr_angle_stright + angle_step;
            }

            let mut curr_angle_rev = 0.0;
            for i in 0..=TESS_TOR_STEP {
                let rotation: Basis3<f64> = Rotation3::from_axis_angle(up_dir, Rad(-curr_angle_rev));
                let nv = rotation.rotate_vector(bend_s_dir.clone());
                let p = self.bend_center_point.clone() + nv;
                anchors_rev.push(p);
                curr_angle_rev = curr_angle_rev + angle_step_rev;
            }

            let p_dir_stright = anchors_stright[1].sub(anchors_stright[0]);

            let is_coplanar = p_dir_stright.dot(prev_dir.clone());
            if (is_coplanar < 0.0) {
                anchors_rev
            } else {
                anchors_stright
            }
        };

        let mut circles: Vec<MainCircle> = vec![];
        for i in 0..anchors.len() - 1 {
            let pc0 = anchors[i];
            let pc1 = anchors[i + 1];
            let c_dir_0 = self.bend_center_point.clone().sub(pc0).normalize();
            let c_dir_1 = self.bend_center_point.clone().sub(pc1).normalize();

            let r_dir_0 = ((pc0.clone() + up_dir * self.r).sub(pc0)).normalize();
            let r_dir_1 = ((pc1.clone() + up_dir * self.r).sub(pc1)).normalize();
            let cdir0 = up_dir.cross(c_dir_0).normalize();
            let cdir1 = up_dir.cross(c_dir_1).normalize();
            let mc0 = MainCircle {
                id: rand::thread_rng().gen_range(0..1024),
                radius: self.r,
                loc: pc0,
                dir: cdir0,
                radius_dir: r_dir_0,
                r_gr_id: (round_by_dec(self.r, 5) * DIVIDER) as u64,
            };
            let mc1 = MainCircle {
                id: rand::thread_rng().gen_range(0..1024),
                radius: self.r,
                loc: pc1,
                dir: cdir1,
                radius_dir: r_dir_1,
                r_gr_id: (round_by_dec(self.r, 5) * DIVIDER) as u64,
            };
            circles.push(mc0);
            circles.push(mc1);
        }
        for i in 0..circles.len() - 1 {
            //let c0 = circles[i].gen_points();
            //let c1 = circles[i + 1].gen_points();

            let c0 = circles[i].gen_points();
            let c1 = circles[i + 1].gen_points();

            for j in 0..c0.len() - 1 {
                let p0 = c0[j];
                let p1 = c0[j + 1];
                let p2 = c1[j];
                let p3 = c1[j + 1];
                self.bbx.push(p0);
                self.bbx.push(p1);
                self.bbx.push(p2);
                self.bbx.push(p3);

                let plane = Plane::new(p0.clone(), p3.clone(), p1.clone());
                let n = plane.normal().normalize();
                let n_arr = [n.x as f32, n.y as f32, n.z as f32, 0.0];
                let r_dir = p0.sub(circles[i].loc);
                let is_coplanar = n.dot(r_dir);
                if (is_coplanar > 0.0) {
                    {
                        let mv0 = MeshVertex {
                            position: [p0.x as f32, p0.y as f32, p0.z as f32, 1.0],
                            normal: n_arr,
                            id: self.id as i32,
                            tex_coords: [0.0, 0.0],
                        };
                        buffer.push(mv0);
                        indxes.push(index);
                        index = index + 1;

                        let mv1 = MeshVertex {
                            position: [p3.x as f32, p3.y as f32, p3.z as f32, 1.0],
                            normal: n_arr,
                            id: self.id as i32,
                            tex_coords: [0.0, 0.0],
                        };
                        buffer.push(mv1);
                        indxes.push(index);
                        index = index + 1;

                        let mv2 = MeshVertex {
                            position: [p1.x as f32, p1.y as f32, p1.z as f32, 1.0],
                            normal: n_arr,
                            id: self.id as i32,
                            tex_coords: [0.0, 0.0],
                        };
                        buffer.push(mv2);
                        indxes.push(index);
                        index = index + 1;
                    }
                    {
                        let mv0 = MeshVertex {
                            position: [p0.x as f32, p0.y as f32, p0.z as f32, 1.0],
                            normal: n_arr,
                            id: self.id as i32,
                            tex_coords: [0.0, 0.0],
                        };
                        buffer.push(mv0);
                        indxes.push(index);
                        index = index + 1;

                        let mv1 = MeshVertex {
                            position: [p2.x as f32, p2.y as f32, p2.z as f32, 1.0],
                            normal: n_arr,
                            id: self.id as i32,
                            tex_coords: [0.0, 0.0],
                        };
                        buffer.push(mv1);
                        indxes.push(index);
                        index = index + 1;

                        let mv2 = MeshVertex {
                            position: [p3.x as f32, p3.y as f32, p3.z as f32, 1.0],
                            normal: n_arr,
                            id: self.id as i32,
                            tex_coords: [0.0, 0.0],
                        };
                        buffer.push(mv2);
                        indxes.push(index);
                        index = index + 1;
                    }
                } else {
                    {
                        let mv0 = MeshVertex {
                            position: [p0.x as f32, p0.y as f32, p0.z as f32, 1.0],
                            normal: n_arr,
                            id: self.id as i32,
                            tex_coords: [0.0, 0.0],
                        };
                        buffer.push(mv0);
                        indxes.push(index);
                        index = index + 1;

                        let mv1 = MeshVertex {
                            position: [p1.x as f32, p1.y as f32, p1.z as f32, 1.0],
                            normal: n_arr,
                            id: self.id as i32,
                            tex_coords: [0.0, 0.0],
                        };
                        buffer.push(mv1);
                        indxes.push(index);
                        index = index + 1;

                        let mv2 = MeshVertex {
                            position: [p3.x as f32, p3.y as f32, p3.z as f32, 1.0],
                            normal: n_arr,
                            id: self.id as i32,
                            tex_coords: [0.0, 0.0],
                        };
                        buffer.push(mv2);
                        indxes.push(index);
                        index = index + 1;
                    }
                    {
                        let mv0 = MeshVertex {
                            position: [p0.x as f32, p0.y as f32, p0.z as f32, 1.0],
                            normal: n_arr,
                            id: self.id as i32,
                            tex_coords: [0.0, 0.0],
                        };
                        buffer.push(mv0);
                        indxes.push(index);
                        index = index + 1;

                        let mv1 = MeshVertex {
                            position: [p3.x as f32, p3.y as f32, p3.z as f32, 1.0],
                            normal: n_arr,
                            id: self.id as i32,
                            tex_coords: [0.0, 0.0],
                        };
                        buffer.push(mv1);
                        indxes.push(index);
                        index = index + 1;

                        let mv2 = MeshVertex {
                            position: [p2.x as f32, p2.y as f32, p2.z as f32, 1.0],
                            normal: n_arr,
                            id: self.id as i32,
                            tex_coords: [0.0, 0.0],
                        };
                        buffer.push(mv2);
                        indxes.push(index);
                        index = index + 1;
                    }
                }
            }
        }
        self.step_vertex_buffer = StepVertexBuffer { buffer, indxes };
    }
    pub fn triangulate_with_start_index(&mut self, index: u32) -> u32 {
        let mut index: u32 = index;
        let mut indxes: Vec<u32> = vec![];
        let mut buffer: Vec<MeshVertex> = vec![];

        let bend_s_dir = self.ca.loc.sub(self.bend_center_point);
        let bend_e_dir = self.cb.loc.sub(self.bend_center_point);

        let bend_diag_dir = self.cb.loc.sub(self.ca.loc).normalize();
        //println!("ANGLE {:?}", bend_e_dir.angle(bend_s_dir).0);
        let angle_step = bend_s_dir.angle(bend_e_dir).0 / TESS_TOR_STEP as f64;
        let angle_step_rev = (2.0 * PI - bend_s_dir.angle(bend_e_dir).0) / TESS_TOR_STEP as f64;
        //let up_dir = self.up_dir().normalize();
        let up_dir = self.bend_plane_norm;
        let mut anchors: Vec<Point3> = {
            let mut anchors_stright: Vec<Point3> = vec![];
            let mut anchors_rev: Vec<Point3> = vec![];

            let mut curr_angle_stright = 0.0;
            for i in 0..=TESS_TOR_STEP {
                let rotation: Basis3<f64> = Rotation3::from_axis_angle(up_dir, Rad(curr_angle_stright));
                let nv = rotation.rotate_vector(bend_s_dir.clone());
                let p = self.bend_center_point.clone() + nv;
                anchors_stright.push(p);
                curr_angle_stright = curr_angle_stright + angle_step;
            }

            let mut curr_angle_rev = 0.0;
            for i in 0..=TESS_TOR_STEP {
                let rotation: Basis3<f64> = Rotation3::from_axis_angle(up_dir, Rad(-curr_angle_rev));
                let nv = rotation.rotate_vector(bend_s_dir.clone());
                let p = self.bend_center_point.clone() + nv;
                anchors_rev.push(p);
                curr_angle_rev = curr_angle_rev + angle_step_rev;
            }

            let p_dir_stright = anchors_stright[1].sub(anchors_stright[0]);
            anchors_stright
            //anchors_rev
        };

        let mut circles: Vec<MainCircle> = vec![];
        for i in 0..anchors.len() - 1 {
            let pc0 = anchors[i];
            let pc1 = anchors[i + 1];
            let c_dir_0 = self.bend_center_point.clone().sub(pc0).normalize();
            let c_dir_1 = self.bend_center_point.clone().sub(pc1).normalize();

            let r_dir_0 = ((pc0.clone() + up_dir * self.r).sub(pc0)).normalize();
            let r_dir_1 = ((pc1.clone() + up_dir * self.r).sub(pc1)).normalize();
            let cdir0 = up_dir.cross(c_dir_0).normalize();
            let cdir1 = up_dir.cross(c_dir_1).normalize();
            let mc0 = MainCircle {
                id: rand::thread_rng().gen_range(0..1024),
                radius: self.r,
                loc: pc0,
                dir: cdir0,
                radius_dir: r_dir_0,
                r_gr_id: (round_by_dec(self.r, 5) * DIVIDER) as u64,
            };
            let mc1 = MainCircle {
                id: rand::thread_rng().gen_range(0..1024),
                radius: self.r,
                loc: pc1,
                dir: cdir1,
                radius_dir: r_dir_1,
                r_gr_id: (round_by_dec(self.r, 5) * DIVIDER) as u64,
            };
            circles.push(mc0);
            circles.push(mc1);
        }
        for i in 0..circles.len() - 1 {
            let c0 = circles[i].gen_points();
            let c1 = circles[i + 1].gen_points();

            for j in 0..c0.len() - 1 {
                let p0 = c0[j];
                let p1 = c0[j + 1];
                let p2 = c1[j];
                let p3 = c1[j + 1];
                self.bbx.push(p0);
                self.bbx.push(p1);
                self.bbx.push(p2);
                self.bbx.push(p3);

                let plane = Plane::new(p0.clone(), p3.clone(), p1.clone());
                let n = plane.normal().normalize();
                let n_arr = [n.x as f32, n.y as f32, n.z as f32, 0.0];
                let r_dir = p0.sub(circles[i].loc);
                let is_coplanar = n.dot(r_dir);
                if (is_coplanar > 0.0) {
                    {
                        let mv0 = MeshVertex {
                            position: [p0.x as f32, p0.y as f32, p0.z as f32, 1.0],
                            normal: n_arr,
                            id: self.id as i32,
                            tex_coords: [0.0, 0.0],
                        };
                        buffer.push(mv0);
                        indxes.push(index);
                        index = index + 1;

                        let mv1 = MeshVertex {
                            position: [p3.x as f32, p3.y as f32, p3.z as f32, 1.0],
                            normal: n_arr,
                            id: self.id as i32,
                            tex_coords: [0.0, 0.0],
                        };
                        buffer.push(mv1);
                        indxes.push(index);
                        index = index + 1;

                        let mv2 = MeshVertex {
                            position: [p1.x as f32, p1.y as f32, p1.z as f32, 1.0],
                            normal: n_arr,
                            id: self.id as i32,
                            tex_coords: [0.0, 0.0],
                        };
                        buffer.push(mv2);
                        indxes.push(index);
                        index = index + 1;
                    }
                    {
                        let mv0 = MeshVertex {
                            position: [p0.x as f32, p0.y as f32, p0.z as f32, 1.0],
                            normal: n_arr,
                            id: self.id as i32,
                            tex_coords: [0.0, 0.0],
                        };
                        buffer.push(mv0);
                        indxes.push(index);
                        index = index + 1;

                        let mv1 = MeshVertex {
                            position: [p2.x as f32, p2.y as f32, p2.z as f32, 1.0],
                            normal: n_arr,
                            id: self.id as i32,
                            tex_coords: [0.0, 0.0],
                        };
                        buffer.push(mv1);
                        indxes.push(index);
                        index = index + 1;

                        let mv2 = MeshVertex {
                            position: [p3.x as f32, p3.y as f32, p3.z as f32, 1.0],
                            normal: n_arr,
                            id: self.id as i32,
                            tex_coords: [0.0, 0.0],
                        };
                        buffer.push(mv2);
                        indxes.push(index);
                        index = index + 1;
                    }
                } else {
                    {
                        let mv0 = MeshVertex {
                            position: [p0.x as f32, p0.y as f32, p0.z as f32, 1.0],
                            normal: n_arr,
                            id: self.id as i32,
                            tex_coords: [0.0, 0.0],
                        };
                        buffer.push(mv0);
                        indxes.push(index);
                        index = index + 1;

                        let mv1 = MeshVertex {
                            position: [p1.x as f32, p1.y as f32, p1.z as f32, 1.0],
                            normal: n_arr,
                            id: self.id as i32,
                            tex_coords: [0.0, 0.0],
                        };
                        buffer.push(mv1);
                        indxes.push(index);
                        index = index + 1;

                        let mv2 = MeshVertex {
                            position: [p3.x as f32, p3.y as f32, p3.z as f32, 1.0],
                            normal: n_arr,
                            id: self.id as i32,
                            tex_coords: [0.0, 0.0],
                        };
                        buffer.push(mv2);
                        indxes.push(index);
                        index = index + 1;
                    }
                    {
                        let mv0 = MeshVertex {
                            position: [p0.x as f32, p0.y as f32, p0.z as f32, 1.0],
                            normal: n_arr,
                            id: self.id as i32,
                            tex_coords: [0.0, 0.0],
                        };
                        buffer.push(mv0);
                        indxes.push(index);
                        index = index + 1;

                        let mv1 = MeshVertex {
                            position: [p3.x as f32, p3.y as f32, p3.z as f32, 1.0],
                            normal: n_arr,
                            id: self.id as i32,
                            tex_coords: [0.0, 0.0],
                        };
                        buffer.push(mv1);
                        indxes.push(index);
                        index = index + 1;

                        let mv2 = MeshVertex {
                            position: [p2.x as f32, p2.y as f32, p2.z as f32, 1.0],
                            normal: n_arr,
                            id: self.id as i32,
                            tex_coords: [0.0, 0.0],
                        };
                        buffer.push(mv2);
                        indxes.push(index);
                        index = index + 1;
                    }
                }
            }
        }
        self.step_vertex_buffer = StepVertexBuffer { buffer, indxes };
        index
    }
    pub fn gen_points(&self) -> Vec<Point3> {
        let mut pts: Vec<Point3> = vec![];
        HALF_PI18_FLOAT_RANGE.iter().for_each(|angle| {
            let rotation: Basis3<f64> = Rotation3::from_axis_angle(self.ca.dir, Rad(*angle));
            let nv = rotation.rotate_vector(self.ca.radius_dir.clone());
            let p = self.ca.loc.clone() + nv * self.ca.radius;
            pts.push(p);
        });

        HALF_PI18_FLOAT_RANGE.iter().for_each(|angle| {
            let rotation: Basis3<f64> = Rotation3::from_axis_angle(self.cb.dir, Rad(*angle));
            let nv = rotation.rotate_vector(self.cb.radius_dir.clone());
            let p = self.cb.loc.clone() + nv * self.cb.radius;
            pts.push(p);
        });
        pts.push(self.ca.loc.clone());
        pts.push(self.cb.loc.clone());
        pts
    }

    pub fn default() -> Self {
        Self {
            id: 100000,
            r: 0.0,
            bend_radius: 0.0,
            bend_center_point: Point3::origin(),
            bend_plane_norm: Vector3::unit_x(),
            radius_dir: Vector3::unit_x(),
            ca: MainCircle {
                id: 0,
                radius: 0.0,
                loc: Point3::origin(),
                dir: Vector3::unit_x(),
                radius_dir: Vector3::unit_x(),
                r_gr_id: 0,
            },
            cb: MainCircle {
                id: 0,
                radius: 0.0,
                loc: Point3::origin(),
                dir: Vector3::unit_x(),
                radius_dir: Vector3::unit_x(),
                r_gr_id: 0,
            },
            r_gr_id: 0,
            step_vertex_buffer: Default::default(),
            bbx: Default::default(),
        }
    }
}
pub fn intersect_line_by_plane(cylinder_dir_vec: &Vector3, radius_vec: &Vector3, plane_point: &Point3, line_p0: &Point3, line_p1: &Point3, ) -> Point3 {
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
pub fn project_point_to_vec(proj_vector: &Vector3, point_on_proj_vector: &Point3, point_to_project: &Point3, ) -> Point3 {
    //https://stackoverflow.com/questions/9368436/3d-perpendicular-point-on-line-from-3d-point
    let pq: Vector3 = point_to_project.sub(point_on_proj_vector);
    let w2: Vector3 = pq - proj_vector * pq.dot(proj_vector.clone()) / proj_vector.magnitude2();
    let point: Point3 = point_to_project - w2;
    //println!("{:?} {:?}", point, 0);
    point
}

pub fn perpendicular_rand_dir(src:&Vector3)->Vector3{
    //https://math.stackexchange.com/questions/137362/how-to-find-perpendicular-vector-to-another-vector
    let nx=src.z.copysign(src.x);
    let ny=src.z.copysign(src.y);
    let nz=-(src.x.abs()+src.y.abs()).copysign(src.z);
    Vector3::new(nx, ny, nz)
}
pub fn round_by_dec(x: f64, decimals: u32) -> f64 {
    let y = 10i64.pow(decimals);
    (x * y as f64).round() / y as f64
}
pub fn extract_vertex(t: &Table, vertex: &PlaceHolder<VertexPointHolder>, scale: f64) -> Option<Point3> {
    match vertex {
        Ref(name) => match t.vertex_point.get(&name_to_id(name.clone())) {
            None => None,
            Some(vtx) => extract_cartesian_point(t, &vtx.vertex_geometry, scale),
        },
        PlaceHolder::Owned(_) => None,
    }
}
pub fn extract_plane_points(table: &Table, scale: f64) -> Vec<Point3> {
    let mut points: Vec<Point3> = vec![];
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
                                    let id = name_to_id(name.clone());
                                    match table.plane.get(&id) {
                                        None => {}
                                        Some(plane) => {
                                            face_bounds.iter().for_each(|bound_holder| {
                                                match bound_holder {
                                                    PlaceHolder::Ref(bound_holder_name) => {
                                                        match table.face_bound.get(&name_to_id(bound_holder_name.clone())) {
                                                            None => {}
                                                            Some(face_bound_holder) => {
                                                                match &face_bound_holder.bound {
                                                                    PlaceHolder::Ref(name) => {
                                                                        match table.edge_loop.get(&name_to_id(name.clone())) {
                                                                            None => {}
                                                                            Some(edge_loop) => {
                                                                                edge_loop.edge_list.iter().for_each(|curve_holder| {
                                                                                    match curve_holder {
                                                                                        PlaceHolder::Ref(name) => {
                                                                                            match table.oriented_edge.get(&name_to_id(name.clone())) {
                                                                                                None => {}
                                                                                                Some(oe_holder) => {
                                                                                                    match &oe_holder.edge_element {
                                                                                                        PlaceHolder::Ref(name) => {
                                                                                                            match table.edge_curve.get(&name_to_id(name.clone())) {
                                                                                                                None => {}
                                                                                                                Some(c) => {
                                                                                                                    let sp = extract_vertex(&table, &c.edge_start, scale).unwrap();
                                                                                                                    let ep = extract_vertex(&table, &c.edge_end, scale).unwrap();
                                                                                                                    points.push(sp);
                                                                                                                    points.push(ep);
                                                                                                                }
                                                                                                            }
                                                                                                        }
                                                                                                        PlaceHolder::Owned(_) => {}
                                                                                                    }
                                                                                                }
                                                                                            }
                                                                                        }
                                                                                        PlaceHolder::Owned(_) => {}
                                                                                    }
                                                                                });
                                                                            }
                                                                        }
                                                                    }
                                                                    PlaceHolder::Owned(_) => {}
                                                                }
                                                            }
                                                        }
                                                    }
                                                    PlaceHolder::Owned(_) => {}
                                                }
                                            });
                                        }
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
    points
}
pub fn analyze_stp(_stp: &Vec<u8>) -> Vec<LRACLR> {
    let mut transcoded = DecodeReaderBytesBuilder::new().encoding(Some(WINDOWS_1251)).build(_stp.as_slice());
    let mut buf: Vec<u8> = vec![];
    let mut stp: String = String::new();
    match transcoded.read_to_end(&mut buf) {
        Ok(b) => match String::from_utf8(buf) {
            Ok(cont) => {
                stp = cont.clone();
            }
            Err(e) => {
                println!("{:?}", e)
            }
        },
        Err(e) => {
            println!("{:?}", e)
        }
    }
    let scale = extact_scale(&stp);

    let fixed_stp: String =stp.replace(",(),", ",'',");

    let exchange = ruststep::parser::parse(&fixed_stp).unwrap();
    let table: Table = Table::from_data_section(&exchange.data[0]);

    let (cyls, tors) = extract_cyls(&table, scale);

    let cyls_no_dubs = MainCylinder::remove_dublicates(&cyls);
    let cyls_merged = MainCylinder::merge(&cyls_no_dubs);
    let bend_toros_no_dublicates: Vec<BendToro> = BendToro::remove_dublicates(&tors);
    let merged_tors = BendToro::merge(&bend_toros_no_dublicates);
    let racalculated_tors: Vec<BendToro> = recalc_tors_tole(&cyls_merged, &merged_tors);
    let lracmd: Vec<LRACLR> = find_bending_surface(&cyls_merged, &racalculated_tors,&table,scale);

    lracmd

}
pub fn extract_cyls(table: &Table, scale: f64) -> (Vec<MainCylinder>, Vec<BendToro>) {
    let mut toros: Vec<BendToro> = vec![];
    let mut cilinders: Vec<MainCylinder> = vec![];

    table.shell.iter().for_each(|(k, v)| {
        let mut counter = 0;
        v.cfs_faces.iter().for_each(|face_holder| {
            let mut points: Vec<Point3> = vec![];
            let mut candidates: Vec<(MainCircle)> = vec![];
            match face_holder {
                PlaceHolder::Ref(name) => {
                    let id = name_to_id(name.clone());
                    match table.face_surface.get(&id) {
                        None => {}
                        Some(face_holder) => {

                            let face_bounds: &Vec<PlaceHolder<FaceBoundHolder>> = &face_holder.bounds;
                            //warn!("{:?}",face_bounds.len());
                            let mut oriented_edge_ids: Vec<u64> = vec![];
                            face_bounds.iter().for_each(|bound_holder| {
                                match bound_holder {
                                    PlaceHolder::Ref(bound_holder_name) => {
                                        match table.face_bound.get(&name_to_id(bound_holder_name.clone())) {
                                            None => {}
                                            Some(face_bound_holder) => {
                                                match &face_bound_holder.bound {
                                                    PlaceHolder::Ref(name) => {
                                                        match table.edge_loop.get(&name_to_id(name.clone())) {
                                                            None => {}
                                                            Some(edge_loop) => {
                                                                //warn!("edge_loop.edge_list.len {:?}",edge_loop.edge_list.len());
                                                                edge_loop.edge_list.iter().for_each(|curve_holder| {
                                                                    match curve_holder {
                                                                        PlaceHolder::Ref(name) => {
                                                                            match table.oriented_edge.get(&name_to_id(name.clone())) {
                                                                                None => {}
                                                                                Some(oe_holder) => {
                                                                                    oriented_edge_ids.push(name_to_id(name.clone()));
                                                                                    let otientation = oe_holder.orientation.clone();
                                                                                    match &oe_holder.edge_element {
                                                                                        PlaceHolder::Ref(name) => {
                                                                                            match table.edge_curve.get(&name_to_id(name.clone())) {
                                                                                                None => {}
                                                                                                Some(c) => {
                                                                                                    let sp = extract_vertex(&table, &c.edge_start, scale).unwrap();
                                                                                                    let ep = extract_vertex(&table, &c.edge_end, scale).unwrap();
                                                                                                    points.push(sp.clone());
                                                                                                    points.push(ep.clone());


                                                                                                    let curve_geom: &PlaceHolder<CurveAnyHolder> = &c.edge_geometry;
                                                                                                    match curve_geom {
                                                                                                        PlaceHolder::Ref(name) => {
                                                                                                            let curve_id = &name_to_id(name.clone());
                                                                                                            let mut found = false;
                                                                                                            match table.circle.get(curve_id) {
                                                                                                                None => {}
                                                                                                                Some(circle) => {
                                                                                                                    found = true;
                                                                                                                    let circle_r = (circle.radius * scale);

                                                                                                                    let (loc, dir, dir_rad) = extract_position(&table, &circle.position, scale);
                                                                                                                    let mc: MainCircle = MainCircle {
                                                                                                                        id: curve_id.clone(),
                                                                                                                        radius: circle_r,
                                                                                                                        loc: loc.unwrap().clone(),
                                                                                                                        dir: dir.unwrap().normalize(),
                                                                                                                        radius_dir: dir_rad.unwrap().normalize(),
                                                                                                                        r_gr_id: (round_by_dec(circle_r, 5) * DIVIDER) as u64,
                                                                                                                    };
                                                                                                                   // borderlines.extend_from_slice(mc.gen_lines(&sp,&ep,otientation).as_slice());
                                                                                                                    candidates.push(mc);
                                                                                                                }
                                                                                                            }
                                                                                                            match table.rational_b_spline_curve.get(curve_id) {
                                                                                                                None => {}
                                                                                                                Some(spline) => {
                                                                                                                    found = true;
                                                                                                                    let wgts = &spline.weights_data;

                                                                                                                    //warn!("C {:?}",b_spline_curve_with_knots.l);
                                                                                                                    match &spline.non_rational_b_spline_curve {
                                                                                                                        PlaceHolder::Ref(name) => {}
                                                                                                                        PlaceHolder::Owned(v) => {
                                                                                                                            match v {
                                                                                                                                NonRationalBSplineCurveHolder::BSplineCurveWithKnots(spline) => {
                                                                                                                                    let mut contrl_points: Vec<CartesianPoint> = vec![];
                                                                                                                                    spline.control_points_list.iter().for_each(|cp| {
                                                                                                                                        let pnt = extract_cartesian_point(&table, &cp, scale).unwrap();
                                                                                                                                        let pp = CartesianPoint {
                                                                                                                                            label: "".to_string(),
                                                                                                                                            coordinates: Vec::from([pnt.x, pnt.y, pnt.z]),
                                                                                                                                        };
                                                                                                                                        contrl_points.push(pp);
                                                                                                                                    });

                                                                                                                                    let sspl: BSplineCurveWithKnots = BSplineCurveWithKnots {
                                                                                                                                        label: "".to_string(),
                                                                                                                                        degree: spline.degree,
                                                                                                                                        control_points_list: contrl_points,
                                                                                                                                        curve_form: spline.curve_form.clone(),
                                                                                                                                        closed_curve: spline.closed_curve,
                                                                                                                                        self_intersect: spline.self_intersect,
                                                                                                                                        knot_multiplicities: spline.knot_multiplicities.clone(),
                                                                                                                                        knots: spline.knots.clone(),
                                                                                                                                        knot_spec: spline.knot_spec.clone(),
                                                                                                                                    };
                                                                                                                                    let bsc: BSplineCurve<Point3> = (&sspl).try_into().unwrap();
                                                                                                                                    let res: NurbsCurve<Vector4> = NurbsCurve::try_from_bspline_and_weights(bsc, wgts.clone()).unwrap();
                                                                                                                                    let mut tess_points: Vec<Point3> = vec![];
                                                                                                                                    for t in (0..=10) {
                                                                                                                                        let tess_point = res.subs(t as f64 / 10.0);
                                                                                                                                        tess_points.push(tess_point);
                                                                                                                                        //points.push(tess_point);
                                                                                                                                    }
                                                                                                                              /*      for i in 0..tess_points.len()-1 {
                                                                                                                                        let bl:BorderLine=BorderLine::new(tess_points[i].clone(), tess_points[i+1].clone());
                                                                                                                                        borderlines.push(bl);
                                                                                                                                    }*/

                                                                                                                                                                                                                                                      match nurbs_to_circle(&tess_points) {
                                                                                                                                        None => {
                                                                                                                                            points.extend(tess_points)
                                                                                                                                        }
                                                                                                                                        Some(mc) => {
                                                                                                                                            candidates.push(mc);
                                                                                                                                        }
                                                                                                                                    }
                                                                                                                                }
                                                                                                                                NonRationalBSplineCurveHolder::BezierCurve(c) => {
                                                                                                                                    warn!("BezierCurve");
                                                                                                                                }
                                                                                                                                NonRationalBSplineCurveHolder::QuasiUniformCurve(c) => {
                                                                                                                                    warn!("QuasiUniformCurve");
                                                                                                                                }
                                                                                                                                NonRationalBSplineCurveHolder::UniformCurve(c) => {
                                                                                                                                    warn!("UniformCurve");
                                                                                                                                }
                                                                                                                            }
                                                                                                                        }
                                                                                                                    }
                                                                                                                }
                                                                                                            }
                                                                                                            match table.b_spline_curve_with_knots.get(curve_id) {
                                                                                                                None => {}
                                                                                                                Some(spline) => {
                                                                                                                    found = true;
                                                                                                                    let mut contrl_points: Vec<CartesianPoint> = vec![];
                                                                                                                    spline.control_points_list.iter().for_each(|cp| {
                                                                                                                        let pnt = extract_cartesian_point(&table, &cp, scale).unwrap();
                                                                                                                        let pp = CartesianPoint {
                                                                                                                            label: "".to_string(),
                                                                                                                            coordinates: Vec::from([pnt.x, pnt.y, pnt.z]),
                                                                                                                        };
                                                                                                                        contrl_points.push(pp);
                                                                                                                    });

                                                                                                                    let sspl: BSplineCurveWithKnots = BSplineCurveWithKnots {
                                                                                                                        label: "".to_string(),
                                                                                                                        degree: spline.degree,
                                                                                                                        control_points_list: contrl_points,
                                                                                                                        curve_form: spline.curve_form.clone(),
                                                                                                                        closed_curve: spline.closed_curve,
                                                                                                                        self_intersect: spline.self_intersect,
                                                                                                                        knot_multiplicities: spline.knot_multiplicities.clone(),
                                                                                                                        knots: spline.knots.clone(),
                                                                                                                        knot_spec: spline.knot_spec.clone(),
                                                                                                                    };

                                                                                                                    let res: BSplineCurve<Point3> = (&sspl).try_into().unwrap();
                                                                                                                    let mut tess_points: Vec<Point3> = vec![];


                                                                                                                    for t in (0..=10) {
                                                                                                                        let tess_point = res.subs(t as f64 / 10.0);
                                                                                                                        tess_points.push(tess_point);
                                                                                                                        //points.push(tess_point);
                                                                                                                    }
                                                                                                    /*                for i in 0..tess_points.len()-1 {
                                                                                                                        let bl:BorderLine=BorderLine::new(tess_points[i].clone(), tess_points[i+1].clone());
                                                                                                                        borderlines.push(bl);
                                                                                                                    }*/
                                                                                                                    match nurbs_to_circle(&tess_points) {
                                                                                                                        None => {
                                                                                                                            points.extend(tess_points)
                                                                                                                        }
                                                                                                                        Some(mc) => {
                                                                                                                            candidates.push(mc);
                                                                                                                        }
                                                                                                                    }
                                                                                                                }
                                                                                                            }
                                                                                                            match table.ellipse.get(curve_id) {
                                                                                                                None => {}
                                                                                                                Some(ellipse) => {
                                                                                                                    found = true;
                                                                                                                    let (loc, dir, dir_rad) = extract_position(&table, &ellipse.position, scale);
                                                                                                                    let axe1 = ellipse.semi_axis_1;
                                                                                                                    let axe2 = ellipse.semi_axis_2;

                                                                                                                    warn!("SEMIAXE {:?} {:?}",axe1, axe2);
                                                                                                                    let p1 = loc.unwrap() + dir_rad.unwrap() * axe1;
                                                                                                                    let p2 = loc.unwrap() + dir_rad.unwrap() * axe2;
                                                                                                                    points.push(p1);
                                                                                                                    points.push(p2);
                                                                                                                }
                                                                                                            }
                                                                                                            match table.line.get(curve_id) {
                                                                                                                None => {}
                                                                                                                Some(line) => {
                                                                                                                    found = true;
                                                                                                     /*               let ptn=extract_cartesian_point(&table,&line.pnt,scale).unwrap();
                                                                                                                    let dir=extract_vector(&table,&line.dir,scale).unwrap();
                                                                                                                    let ptn2=ptn+dir;
                                                                                                                    borderlines.push(BorderLine::new(ptn,ptn2));*/

                                                                                                                }
                                                                                                            }

                                                                                                            if (!found) {
                                                                                                                println!("name ID {:?} ", &name_to_id(name.clone()))
                                                                                                            }

                                                                                                            //println!("name ID {:?} ", &name_to_id(name.clone()))
                                                                                                        }
                                                                                                        PlaceHolder::Owned(_) => {}
                                                                                                    }

                                                                                                }
                                                                                            }
                                                                                        }
                                                                                        PlaceHolder::Owned(_) => {}
                                                                                    }
                                                                                }
                                                                            }
                                                                        }
                                                                        PlaceHolder::Owned(_) => {}
                                                                    }
                                                                });
                                                            }
                                                        }
                                                    }
                                                    PlaceHolder::Owned(_) => {}
                                                }
                                            }
                                        }
                                    }
                                    PlaceHolder::Owned(_) => {
                                        warn!("Owned Owned");
                                    }
                                }
                            });
                        }
                    }
                }
                PlaceHolder::Owned(_) => {}
            };


            let mut no_dubs = remove_circle_dublicates(&candidates);
            no_dubs.sort_by(|a, b| a.r_gr_id.cmp(&b.r_gr_id));
            counter = counter + 1;
            no_dubs.iter().chunk_by(|c| c.r_gr_id).into_iter().for_each(|(k, v)| {
                 let vec = v.into_iter().collect_vec();
                 let mut nv: Vec<MainCircle> = vec![];
                 vec.iter().for_each(|c| {
                     nv.push(c.clone().clone());
                 });
                 let count = nv.len();
                 match count {
                     1 => {
                         do_cyl_1(&nv[0], &points);
                     }
                     2 => {
                         let (cyls, tors) = do_cyl_2(&nv[0], &nv[1], &points);
                         cilinders.extend(cyls);
                         toros.extend(tors);
                     }

                     0 => {}
                     _ => {
                         nv.iter().for_each(|c| {
                             //warn!("no_dubs {:?}  {:?} ",counter, c.radius);
                             let cyls = do_cyl_1(&c, &points);
                             cilinders.extend(cyls);
                         });
                     }
                 }
             });


/*            {
                if (no_dubs.len() == 2) {
                    let (cyls, tors) = do_cyl_2(&no_dubs[0], &no_dubs[1], &points);
                    cilinders.extend(cyls);
                    toros.extend(tors);
                }

                if (no_dubs.len() == 1) {
                    let cyls = do_cyl_1(&no_dubs[0], &points);
                    cilinders.extend(cyls);
                }

                if (no_dubs.len() == 3) {
                    if ((no_dubs[0].radius - no_dubs[1].radius).abs() < TOLE) {
                        let (cyls, tors) = do_cyl_2(&no_dubs[0], &no_dubs[1], &points);
                        cilinders.extend(cyls);
                        toros.extend(tors);
                    } else if ((no_dubs[0].radius - no_dubs[2].radius).abs() < TOLE) {
                        let rr = no_dubs[0].radius;
                        let (cyls, tors) = do_cyl_2(&no_dubs[0], &no_dubs[2], &points);
                        cilinders.extend(cyls);
                        toros.extend(tors);
                    } else if ((no_dubs[1].radius - no_dubs[2].radius).abs() < TOLE) {
                        let (cyls, tors) = do_cyl_2(&no_dubs[1], &no_dubs[2], &points);
                        cilinders.extend(cyls);
                        toros.extend(tors);
                    }
                }

                if (no_dubs.len() > 3) {
                    no_dubs.iter().for_each(|c| {
                        warn!("no_dubs {:?}  {:?} ",counter, c.radius);
                        let cyls = do_cyl_1(&c, &points);
                        cilinders.extend(cyls);
                    });
                }
            }*/

        });
    });

    (cilinders, toros)
}
pub fn extract_tors(table: &Table, scale: f64, cyls: &Vec<MainCylinder>, radius: f64) -> Vec<BendToro> {
    let mut toros: Vec<BendToro> = vec![];

    table.shell.iter().for_each(|(k, v)| {
        let mut counter = 0;
        v.cfs_faces.iter().for_each(|face_holder| {
            let mut points: Vec<Point3> = vec![];
            let mut candidates: Vec<(MainCircle)> = vec![];

            match face_holder {
                PlaceHolder::Ref(name) => {
                    let id = name_to_id(name.clone());
                    match table.face_surface.get(&id) {
                        None => {}
                        Some(face_holder) => {
                            let face_bounds: &Vec<PlaceHolder<FaceBoundHolder>> = &face_holder.bounds;
                            //warn!("{:?}",face_bounds.len());
                            let mut oriented_edge_ids: Vec<u64> = vec![];
                            face_bounds.iter().for_each(|bound_holder| {
                                match bound_holder {
                                    PlaceHolder::Ref(bound_holder_name) => {
                                        match table.face_bound.get(&name_to_id(bound_holder_name.clone())) {
                                            None => {}
                                            Some(face_bound_holder) => {
                                                match &face_bound_holder.bound {
                                                    PlaceHolder::Ref(name) => {
                                                        match table.edge_loop.get(&name_to_id(name.clone())) {
                                                            None => {}
                                                            Some(edge_loop) => {
                                                                //warn!("edge_loop.edge_list.len {:?}",edge_loop.edge_list.len());
                                                                edge_loop.edge_list.iter().for_each(|curve_holder| {
                                                                    match curve_holder {
                                                                        PlaceHolder::Ref(name) => {
                                                                            match table.oriented_edge.get(&name_to_id(name.clone())) {
                                                                                None => {}
                                                                                Some(oe_holder) => {
                                                                                    oriented_edge_ids.push(name_to_id(name.clone()));
                                                                                    let otientation = oe_holder.orientation.clone();
                                                                                    match &oe_holder.edge_element {
                                                                                        PlaceHolder::Ref(name) => {
                                                                                            match table.edge_curve.get(&name_to_id(name.clone())) {
                                                                                                None => {}
                                                                                                Some(c) => {
                                                                                                    let curve_geom: &PlaceHolder<CurveAnyHolder> = &c.edge_geometry;
                                                                                                    match curve_geom {
                                                                                                        PlaceHolder::Ref(name) => {
                                                                                                            let curve_id = &name_to_id(name.clone());
                                                                                                            let mut found = false;
                                                                                                            match table.circle.get(curve_id) {
                                                                                                                None => {}
                                                                                                                Some(circle) => {
                                                                                                                    found = true;
                                                                                                                    let circle_r = circle.radius * scale;
                                                                                                                    if (abs(circle_r - radius) < TOLE) {
                                                                                                                        let (loc, dir, dir_rad) = extract_position(&table, &circle.position, scale);
                                                                                                                        let mc: MainCircle = MainCircle {
                                                                                                                            id: curve_id.clone(),
                                                                                                                            radius: radius,
                                                                                                                            loc: loc.unwrap().clone(),
                                                                                                                            dir: dir.unwrap().normalize(),
                                                                                                                            radius_dir: dir_rad.unwrap().normalize(),
                                                                                                                            r_gr_id: (round_by_dec(radius, 5) * DIVIDER) as u64,
                                                                                                                        };
                                                                                                                        candidates.push(mc);
                                                                                                                    }
                                                                                                                }
                                                                                                            }
                                                                                                            match table.rational_b_spline_curve.get(curve_id) {
                                                                                                                None => {}
                                                                                                                Some(spline) => {
                                                                                                                    found = true;
                                                                                                                    let wgts = &spline.weights_data;

                                                                                                                    //warn!("C {:?}",b_spline_curve_with_knots.l);
                                                                                                                    match &spline.non_rational_b_spline_curve {
                                                                                                                        PlaceHolder::Ref(name) => {}
                                                                                                                        PlaceHolder::Owned(v) => {
                                                                                                                            match v {
                                                                                                                                NonRationalBSplineCurveHolder::BSplineCurveWithKnots(spline) => {
                                                                                                                                    let mut contrl_points: Vec<CartesianPoint> = vec![];
                                                                                                                                    spline.control_points_list.iter().for_each(|cp| {
                                                                                                                                        let pnt = extract_cartesian_point(&table, &cp, scale).unwrap();
                                                                                                                                        let pp = CartesianPoint {
                                                                                                                                            label: "".to_string(),
                                                                                                                                            coordinates: Vec::from([pnt.x, pnt.y, pnt.z]),
                                                                                                                                        };
                                                                                                                                        contrl_points.push(pp);
                                                                                                                                    });

                                                                                                                                    let sspl: BSplineCurveWithKnots = BSplineCurveWithKnots {
                                                                                                                                        label: "".to_string(),
                                                                                                                                        degree: spline.degree,
                                                                                                                                        control_points_list: contrl_points,
                                                                                                                                        curve_form: spline.curve_form.clone(),
                                                                                                                                        closed_curve: spline.closed_curve,
                                                                                                                                        self_intersect: spline.self_intersect,
                                                                                                                                        knot_multiplicities: spline.knot_multiplicities.clone(),
                                                                                                                                        knots: spline.knots.clone(),
                                                                                                                                        knot_spec: spline.knot_spec.clone(),
                                                                                                                                    };
                                                                                                                                    let bsc: BSplineCurve<Point3> = (&sspl).try_into().unwrap();
                                                                                                                                    let res: NurbsCurve<Vector4> = NurbsCurve::try_from_bspline_and_weights(bsc, wgts.clone()).unwrap();
                                                                                                                                    let mut tess_points: Vec<Point3> = vec![];
                                                                                                                                    for t in (0..=10) {
                                                                                                                                        let tess_point = res.subs(t as f64 / 10.0);
                                                                                                                                        tess_points.push(tess_point);
                                                                                                                                        //points.push(tess_point);
                                                                                                                                    }
                                                                                                                                    if tess_points.len() > 10 {
                                                                                                                                        let c = circum_center(tess_points[0], tess_points[2], tess_points[5]);
                                                                                                                                        let r = c.sub(tess_points[0]).magnitude();

                                                                                                                                        if (abs(r - radius) < TOLE) {
                                                                                                                                            let mc: MainCircle = MainCircle {
                                                                                                                                                id: curve_id.clone(),
                                                                                                                                                radius: radius,
                                                                                                                                                loc: c,
                                                                                                                                                dir: truck_geometry::prelude::Plane::new(tess_points[0], tess_points[5], tess_points[7]).normal().normalize(),
                                                                                                                                                radius_dir: c.sub(tess_points[0]).normalize(),
                                                                                                                                                r_gr_id: (round_by_dec(radius, 5) * DIVIDER) as u64,
                                                                                                                                            };
                                                                                                                                            candidates.push(mc);
                                                                                                                                        } else {
                                                                                                                                            points.extend(tess_points)
                                                                                                                                        }
                                                                                                                                    }
                                                                                                                                }
                                                                                                                                NonRationalBSplineCurveHolder::BezierCurve(c) => {
                                                                                                                                    warn!("BezierCurve");
                                                                                                                                }
                                                                                                                                NonRationalBSplineCurveHolder::QuasiUniformCurve(c) => {
                                                                                                                                    warn!("QuasiUniformCurve");
                                                                                                                                }
                                                                                                                                NonRationalBSplineCurveHolder::UniformCurve(c) => {
                                                                                                                                    warn!("UniformCurve");
                                                                                                                                }
                                                                                                                            }
                                                                                                                        }
                                                                                                                    }
                                                                                                                }
                                                                                                            }
                                                                                                            match table.b_spline_curve_with_knots.get(curve_id) {
                                                                                                                None => {}
                                                                                                                Some(spline) => {
                                                                                                                    found = true;
                                                                                                                    let mut contrl_points: Vec<CartesianPoint> = vec![];
                                                                                                                    spline.control_points_list.iter().for_each(|cp| {
                                                                                                                        let pnt = extract_cartesian_point(&table, &cp, scale).unwrap();
                                                                                                                        let pp = CartesianPoint {
                                                                                                                            label: "".to_string(),
                                                                                                                            coordinates: Vec::from([pnt.x, pnt.y, pnt.z]),
                                                                                                                        };
                                                                                                                        contrl_points.push(pp);
                                                                                                                    });

                                                                                                                    let sspl: BSplineCurveWithKnots = BSplineCurveWithKnots {
                                                                                                                        label: "".to_string(),
                                                                                                                        degree: spline.degree,
                                                                                                                        control_points_list: contrl_points,
                                                                                                                        curve_form: spline.curve_form.clone(),
                                                                                                                        closed_curve: spline.closed_curve,
                                                                                                                        self_intersect: spline.self_intersect,
                                                                                                                        knot_multiplicities: spline.knot_multiplicities.clone(),
                                                                                                                        knots: spline.knots.clone(),
                                                                                                                        knot_spec: spline.knot_spec.clone(),
                                                                                                                    };

                                                                                                                    let res: BSplineCurve<Point3> = (&sspl).try_into().unwrap();
                                                                                                                    let mut tess_points: Vec<Point3> = vec![];
                                                                                                                    for t in (0..=10) {
                                                                                                                        let tess_point = res.subs(t as f64 / 10.0);
                                                                                                                        tess_points.push(tess_point);
                                                                                                                        //points.push(tess_point);
                                                                                                                    }
                                                                                                                    if tess_points.len() > 10 {
                                                                                                                        let c = circum_center(tess_points[0], tess_points[2], tess_points[5]);
                                                                                                                        let r = c.sub(tess_points[0]).magnitude();
                                                                                                                        if (abs(r - radius) < TOLE) {
                                                                                                                            let mc: MainCircle = MainCircle {
                                                                                                                                id: curve_id.clone(),
                                                                                                                                radius: radius,
                                                                                                                                loc: c,
                                                                                                                                dir: truck_geometry::prelude::Plane::new(tess_points[0], tess_points[5], tess_points[7]).normal().normalize(),
                                                                                                                                radius_dir: c.sub(tess_points[0]).normalize(),
                                                                                                                                r_gr_id: (round_by_dec(radius, 5) * DIVIDER) as u64,
                                                                                                                            };
                                                                                                                            candidates.push(mc);
                                                                                                                        } else {
                                                                                                                            points.extend(tess_points)
                                                                                                                        }
                                                                                                                    }
                                                                                                                }
                                                                                                            }
                                                                                                            match table.ellipse.get(curve_id) {
                                                                                                                None => {}
                                                                                                                Some(ellipse) => {
                                                                                                                    found = true;
                                                                                                                    let (loc, dir, dir_rad) = extract_position(&table, &ellipse.position, scale);
                                                                                                                    let axe1 = ellipse.semi_axis_1;
                                                                                                                    let axe2 = ellipse.semi_axis_1;
                                                                                                                    let p1 = loc.unwrap() + dir_rad.unwrap() * axe1;
                                                                                                                    let p2 = loc.unwrap() + dir_rad.unwrap() * axe2;
                                                                                                                    points.push(p1);
                                                                                                                    points.push(p2);
                                                                                                                }
                                                                                                            }
                                                                                                            match table.line.get(curve_id) {
                                                                                                                None => {}
                                                                                                                Some(line) => {
                                                                                                                    found = true;
                                                                                                                }
                                                                                                            }

                                                                                                            if (!found) {
                                                                                                                println!("name ID {:?} ", &name_to_id(name.clone()))
                                                                                                            }

                                                                                                            //println!("name ID {:?} ", &name_to_id(name.clone()))
                                                                                                        }
                                                                                                        PlaceHolder::Owned(_) => {}
                                                                                                    }
                                                                                                    let sp = extract_vertex(&table, &c.edge_start, scale).unwrap();
                                                                                                    let ep = extract_vertex(&table, &c.edge_end, scale).unwrap();
                                                                                                    //points.push(sp);
                                                                                                    //points.push(ep);
                                                                                                }
                                                                                            }
                                                                                        }
                                                                                        PlaceHolder::Owned(_) => {}
                                                                                    }
                                                                                }
                                                                            }
                                                                        }
                                                                        PlaceHolder::Owned(_) => {}
                                                                    }
                                                                });
                                                            }
                                                        }
                                                    }
                                                    PlaceHolder::Owned(_) => {}
                                                }
                                            }
                                        }
                                    }
                                    PlaceHolder::Owned(_) => {}
                                }
                            });
                        }
                    }
                }
                PlaceHolder::Owned(_) => {}
            }
            let no_dubs = remove_circle_dublicates(&candidates);
            /*            no_dubs.iter().for_each(|c|{
                            points.extend(c.gen_points());
                        });*/
            //export_to_pt_str(&points,counter.to_string().as_str());
            //warn!("{:?} {:?}", counter,no_dubs.len());
            counter = counter + 1;
            let mut dist1 = f64::MAX;
            let mut dist2 = f64::MAX;
            if (no_dubs.len() == 2) {
                let circle1_orig: &MainCircle = &no_dubs[0];
                let circle2_orig = &no_dubs[1];
                let plane_v = circle1_orig.dir.cross(circle2_orig.dir);
                if (plane_v.magnitude() > TOLE) {
                    //TOR THERE
                    let mut is_1_found = false;
                    let mut is_2_found = false;
                    let mut circle1: MainCircle = MainCircle {
                        id: 0,
                        radius,
                        loc: Point3::new(0.0, 0.0, 0.0),
                        dir: Vector3::new(0.0, 0.0, 0.0),
                        radius_dir: Vector3::new(0.0, 0.0, 0.0),
                        r_gr_id: (round_by_dec(radius, 5) * DIVIDER) as u64,
                    };
                    let mut circle2: MainCircle = MainCircle {
                        id: 0,
                        radius,
                        loc: Point3::new(0.0, 0.0, 0.0),
                        dir: Vector3::new(0.0, 0.0, 0.0),
                        radius_dir: Vector3::new(0.0, 0.0, 0.0),
                        r_gr_id: (round_by_dec(radius, 5) * DIVIDER) as u64,
                    };
                    cyls.iter().for_each(|c| {
                        let d11 = circle1_orig.loc.distance(c.ca.loc);
                        if (d11 < dist1) { dist1 = d11 };
                        let d12 = circle1_orig.loc.distance(c.ca.loc);
                        if (d12 < dist1) { dist1 = d12 };

                        let d21 = circle2_orig.loc.distance(c.ca.loc);
                        if (d21 < dist2) { dist2 = d21 };
                        let d22 = circle2_orig.loc.distance(c.ca.loc);
                        if (d22 < dist2) { dist2 = d22 };

                        if (circle1_orig.is_same_pos(&c.ca)) {
                            is_1_found = true;
                            circle1 = c.ca.clone();
                        }
                        if (circle1_orig.is_same_pos(&c.cb)) {
                            is_1_found = true;
                            circle1 = c.cb.clone();
                        }

                        if (circle2_orig.is_same_pos(&c.ca)) {
                            is_2_found = true;
                            circle2 = c.ca.clone();
                        }
                        if (circle2_orig.is_same_pos(&c.cb)) {
                            is_2_found = true;
                            circle2 = c.cb.clone();
                        }
                    });

                    if (is_1_found && is_2_found) {
                        let up_v = plane_v.normalize();
                        let catet1 = circle1.dir.cross(up_v).normalize();
                        let p90 = project_point_to_vec(&catet1, &circle1.loc, &circle2.loc);
                        let catet2 = circle2.loc.sub(p90).magnitude();
                        let catet2_dir = circle2.loc.sub(p90).normalize();
                        let hypotenuze = circle2.dir.cross(up_v).normalize();
                        let angle = hypotenuze.angle(catet2_dir);
                        let hypotenuze_len = (catet2 / angle.cos()).round();
                        let bend_radius_centre = circle2.loc - hypotenuze.mul(hypotenuze_len);

                        let t = BendToro {
                            id: rand::thread_rng().gen_range(0..1024),
                            r: circle1.radius.abs(),
                            bend_radius: hypotenuze_len.abs(),
                            bend_center_point: bend_radius_centre,
                            bend_plane_norm: up_v,
                            radius_dir: circle1.radius_dir,
                            ca: circle1.clone(),
                            cb: circle2.clone(),
                            r_gr_id: (round_by_dec(circle1.radius, 5) * DIVIDER) as u64,
                            step_vertex_buffer: StepVertexBuffer::default(),
                            bbx: Default::default(),
                        };
                        toros.push(t);
                    } else {
                        //warn!("NOTFOUND {:?} {:?}", dist1, dist2);
                    }
                }
            }
        });
    });
    toros
}
pub fn recalc_tors_tole(cyls: &Vec<MainCylinder>, tors: &Vec<BendToro>) -> Vec<BendToro> {
    let mut toros: Vec<BendToro> = vec![];
    tors.iter().for_each(|t| {
        let mut dist1 = f64::MAX;
        let mut dist2 = f64::MAX;
        let radius = t.r;

        let circle1_orig: &MainCircle = &t.ca;
        let circle2_orig: &MainCircle = &t.cb;
        //let plane_v = circle1_orig.dir.cross(circle2_orig.dir);
        //if (plane_v.magnitude() > TOLE) {
        //TOR THERE
        let mut is_1_found = false;
        let mut is_2_found = false;
        let mut circle1: MainCircle = MainCircle {
            id: rand::thread_rng().gen_range(0..1024),
            radius,
            loc: Point3::new(0.0, 0.0, 0.0),
            dir: Vector3::new(0.0, 0.0, 0.0),
            radius_dir: Vector3::new(0.0, 0.0, 0.0),
            r_gr_id: (round_by_dec(radius, 5) * DIVIDER) as u64,
        };
        let mut circle2: MainCircle = MainCircle {
            id: rand::thread_rng().gen_range(0..1024),
            radius,
            loc: Point3::new(0.0, 0.0, 0.0),
            dir: Vector3::new(0.0, 0.0, 0.0),
            radius_dir: Vector3::new(0.0, 0.0, 0.0),
            r_gr_id: (round_by_dec(radius, 5) * DIVIDER) as u64,
        };
        cyls.iter().for_each(|c| {
            if (c.r_gr_id == t.r_gr_id) {
                let d11 = circle1_orig.loc.distance(c.ca.loc);
                if (d11 < dist1) { dist1 = d11 };
                let d12 = circle1_orig.loc.distance(c.ca.loc);
                if (d12 < dist1) { dist1 = d12 };

                let d21 = circle2_orig.loc.distance(c.ca.loc);
                if (d21 < dist2) { dist2 = d21 };
                let d22 = circle2_orig.loc.distance(c.ca.loc);
                if (d22 < dist2) { dist2 = d22 };

                if (circle1_orig.is_same_pos(&c.ca) && !is_1_found) {
                    is_1_found = true;
                    circle1 = c.ca.clone();
                }
                if (circle1_orig.is_same_pos(&c.cb) && !is_1_found) {
                    is_1_found = true;
                    circle1 = c.cb.clone();
                }

                if (circle2_orig.is_same_pos(&c.ca) && !is_2_found) {
                    is_2_found = true;
                    circle2 = c.ca.clone();
                }
                if (circle2_orig.is_same_pos(&c.cb) && !is_2_found) {
                    is_2_found = true;
                    circle2 = c.cb.clone();
                }
            }
        });

        if (is_1_found && is_2_found) {
            let up_v = t.bend_plane_norm.normalize();
            let catet1 = circle1.dir.cross(up_v).normalize();
            let p90 = project_point_to_vec(&catet1, &circle1.loc, &circle2.loc);
            let catet2 = circle2.loc.sub(p90).magnitude();
            let catet2_dir = circle2.loc.sub(p90).normalize();
            let hypotenuze = circle2.dir.cross(up_v).normalize();
            let angle = hypotenuze.angle(catet2_dir);
            let hypotenuze_len = (catet2 / angle.cos()).round();
            let bend_radius_centre = circle2.loc - hypotenuze.mul(hypotenuze_len);

            let t = BendToro {
                id: rand::thread_rng().gen_range(0..1024),
                r: circle1.radius.abs(),
                bend_radius: hypotenuze_len.abs(),
                bend_center_point: bend_radius_centre,
                bend_plane_norm: up_v,
                radius_dir: up_v,
                ca: circle1.clone(),
                cb: circle2.clone(),
                r_gr_id: (round_by_dec(circle1.radius, 5) * DIVIDER) as u64,
                step_vertex_buffer: StepVertexBuffer::default(),
                bbx: Default::default(),
            };
            toros.push(t);
        } else {
            //warn!("NOTFOUND {:?} {:?}", dist1, dist2);
        }
        //}
    });

    toros
}
pub fn circum_center(pt0: Point3, pt1: Point3, pt2: Point3) -> Point3 {
    let (vec0, vec1) = (pt1 - pt0, pt2 - pt0);
    let (a2, ab, b2) = (vec0.dot(vec0), vec0.dot(vec1), vec1.dot(vec1));
    let ff = a2 * b2 - ab * ab;
    let (det, u, v) = (a2 * b2 - ab * ab, a2 * b2 - ab * b2, a2 * b2 - ab * a2);
    pt0 + u / (2.0 * det) * vec0 + v / (2.0 * det) * vec1
}
pub fn remove_circle_dublicates(circles: &Vec<MainCircle>) -> Vec<MainCircle> {
    let mut ret: Vec<MainCircle> = vec![];
    circles.iter().for_each(|c| {
        let mut is_same = false;
        ret.iter().for_each(|c_ret| {
            if (!is_same) {
                is_same = c.is_same_pos(c_ret);
            }
        });
        if (!is_same) {
            ret.push(c.clone());
        }
    });
    ret
}
pub fn do_cyl_1(circle: &MainCircle, points: &Vec<Point3>) -> Vec<MainCylinder> {
    let mut circles: Vec<MainCylinder> = vec![];
    if (!points.is_empty()) {
        let mut point_a: Point3 = Point3::new(f64::MAX, 0.0, 0.0);
        let mut point_b: Point3 = Point3::new(f64::MIN, 0.0, 0.0);
        let mut dist_a = 0.0;
        let mut dist_b = 0.0;
        points.iter().for_each(|p| {
            let pp = project_point_to_vec(&circle.dir, &circle.loc, p);
            if ((pp.sub(p).magnitude() - circle.radius).abs() < TOLE) {
                let v = pp.sub(circle.loc);
                let dist = v.magnitude();
                if (dist > TOLE) {
                    if (circle.dir.dot(v).signum().clone() < 0.0) {
                        if (dist > dist_b) {
                            dist_b = dist;
                            point_b = pp;
                        }
                    } else {
                        if (dist > dist_a) {
                            dist_a = dist;
                            point_a = pp;
                        }
                    }
                }
            }
        });

        if (dist_a > TOLE && dist_b > TOLE) {
            let mut c1 = circle.clone();
            let mut c2 = circle.clone();
            c1.loc = point_a;
            c2.loc = point_b;
            let r = c1.radius;
            let nc: MainCylinder = MainCylinder {
                id: rand::thread_rng().gen_range(0..1024),
                ca: c1,
                cb: c2,
                h: dist_a + dist_b,
                r: r,
                r_gr_id: (round_by_dec(r, 5) * DIVIDER) as u64,
                ca_tor: u64::MAX,
                cb_tor: u64::MAX,
                step_vertex_buffer: StepVertexBuffer::default(),
                bbx: Default::default(),
            };
            circles.push(nc);
        } else if (dist_a > TOLE) {
            let mut c1 = circle.clone();
            let mut c2 = circle.clone();
            c1.loc = point_a;
            let r = c1.radius;
            let nc: MainCylinder = MainCylinder {
                id: rand::thread_rng().gen_range(0..1024),
                ca: c1,
                cb: c2,
                h: dist_a,
                r: r,
                r_gr_id: (round_by_dec(r, 5) * DIVIDER) as u64,
                ca_tor: u64::MAX,
                cb_tor: u64::MAX,
                step_vertex_buffer: StepVertexBuffer::default(),
                bbx: Default::default(),
            };
            circles.push(nc);
        } else if (dist_b > TOLE) {
            let mut c1 = circle.clone();
            let mut c2 = circle.clone();
            c2.loc = point_b;
            let r = c1.radius;
            let nc: MainCylinder = MainCylinder {
                id: rand::thread_rng().gen_range(0..1024),
                ca: c1,
                cb: c2,
                h: dist_b,
                r: r,
                r_gr_id: (round_by_dec(r, 5) * DIVIDER) as u64,
                ca_tor: u64::MAX,
                cb_tor: u64::MAX,
                step_vertex_buffer: StepVertexBuffer::default(),
                bbx: Default::default(),
            };
            circles.push(nc);
        }
    }
    circles
}
pub fn do_cyl_2(circle1: &MainCircle, circle2: &MainCircle, points: &Vec<Point3>) -> (Vec<MainCylinder>, Vec<BendToro>) {
    let mut toros: Vec<BendToro> = vec![];
    let mut circles: Vec<MainCylinder> = vec![];
    if ((circle1.radius - circle2.radius).abs() < TOLE) {
        let plane_v = circle1.dir.cross(circle2.dir);
        let angle = circle1.dir.angle(circle2.dir);
        if (Deg::from(angle) > Deg(3.0)) {
            //TOR
            let up_v = plane_v.normalize();
            let catet1 = circle1.dir.cross(up_v).normalize();
            let p90 = project_point_to_vec(&catet1, &circle1.loc, &circle2.loc);
            let catet2 = circle2.loc.sub(p90).magnitude();
            let catet2_dir = circle2.loc.sub(p90).normalize();
            let hypotenuze = circle2.dir.cross(up_v).normalize();
            let angle = hypotenuze.angle(catet2_dir);
            let hypotenuze_len = (catet2 / angle.cos()).round();
            let bend_radius_centre = circle2.loc - hypotenuze.mul(hypotenuze_len);
            let plane: Vector3 = Plane::new(bend_radius_centre, circle2.loc, circle1.loc).normal().normalize();
            if (plane.dot(up_v).abs() - 1.0 < TOLE) {
                let r = circle1.radius;
                let dir_radius1 = circle1.loc.sub(bend_radius_centre).normalize();
                let dir_radius2 = circle2.loc.sub(bend_radius_centre).normalize();
                let dir1 = dir_radius1.cross(plane);
                let dir2 = dir_radius2.cross(plane);
                let c1: MainCircle = MainCircle {
                    id: circle1.id,
                    radius: r,
                    loc: circle1.loc,
                    dir: dir1,
                    radius_dir: dir_radius1,
                    r_gr_id: (round_by_dec(r, 5) * DIVIDER) as u64,
                };
                let c2: MainCircle = MainCircle {
                    id: circle2.id,
                    radius: r,
                    loc: circle2.loc,
                    dir: dir2,
                    radius_dir: dir_radius2,
                    r_gr_id: (round_by_dec(r, 5) * DIVIDER) as u64,
                };
                if( hypotenuze_len.abs()<MAX_BEND_RADIUS){
                    let t = BendToro {
                        id: rand::thread_rng().gen_range(0..1024),
                        r: c1.radius,
                        bend_radius: hypotenuze_len.abs(),
                        bend_center_point: bend_radius_centre,
                        bend_plane_norm: plane,
                        radius_dir: plane,
                        ca: c1,
                        cb: c2,
                        r_gr_id: (round_by_dec(r, 5) * DIVIDER) as u64,
                        step_vertex_buffer: StepVertexBuffer::default(),
                        bbx: Default::default(),
                    };
                    toros.push(t);
                }else{
                    let dir = circle2.loc.sub(circle1.loc);
                    if (dir.magnitude() > TOLE) {
                        let some_p: Point3 = Point3::new(52369.33, 4596.66, 8899.36);
                        let proj_point: Point3 = project_point_to_vec(&dir, &circle1.loc, &some_p);
                        let radius_dir = some_p.sub(proj_point).normalize();


                        let r = circle1.radius;
                        let ca = MainCircle {
                            id: rand::thread_rng().gen_range(0..1024),
                            radius: r,
                            loc: circle1.loc.clone(),
                            dir: dir.normalize(),
                            radius_dir: radius_dir.clone(),
                            r_gr_id: (round_by_dec(r, 5) * DIVIDER) as u64,
                        };
                        let cb = MainCircle {
                            id: rand::thread_rng().gen_range(0..1024),
                            radius: r,
                            loc: circle2.loc.clone(),
                            dir: dir.normalize(),
                            radius_dir: radius_dir.clone(),
                            r_gr_id: (round_by_dec(r, 5) * DIVIDER) as u64,
                        };

                        let mut cb_clone = cb.clone();
                        let mut ca_clone = ca.clone();
                        cb_clone.dir = dir.normalize();
                        ca_clone.dir = dir.normalize();
                        let nc: MainCylinder = MainCylinder {
                            id: rand::thread_rng().gen_range(0..1024),
                            ca: cb_clone,
                            cb: ca_clone,
                            h: dir.magnitude(),
                            r: r,
                            r_gr_id: (round_by_dec(r, 5) * DIVIDER) as u64,
                            ca_tor: u64::MAX,
                            cb_tor: u64::MAX,
                            step_vertex_buffer: StepVertexBuffer::default(),
                            bbx: Default::default(),
                        };
                        circles.push(nc);
                    } else {
                        let pts = circle2.gen_points();
                        let mut in_pts = points.clone();
                        in_pts.extend_from_slice(&pts);
                        circles.extend(do_cyl_1(circle1, &in_pts));
                    }
                }

            }
        } else {
            let dir = circle2.loc.sub(circle1.loc);
            if (dir.magnitude() > TOLE) {
                let some_p: Point3 = Point3::new(52369.33, 4596.66, 8899.36);
                let proj_point: Point3 = project_point_to_vec(&dir, &circle1.loc, &some_p);
                let radius_dir = some_p.sub(proj_point).normalize();


                let r = circle1.radius;
                let ca = MainCircle {
                    id: rand::thread_rng().gen_range(0..1024),
                    radius: r,
                    loc: circle1.loc.clone(),
                    dir: dir.normalize(),
                    radius_dir: radius_dir.clone(),
                    r_gr_id: (round_by_dec(r, 5) * DIVIDER) as u64,
                };
                let cb = MainCircle {
                    id: rand::thread_rng().gen_range(0..1024),
                    radius: r,
                    loc: circle2.loc.clone(),
                    dir: dir.normalize(),
                    radius_dir: radius_dir.clone(),
                    r_gr_id: (round_by_dec(r, 5) * DIVIDER) as u64,
                };

                let mut cb_clone = cb.clone();
                let mut ca_clone = ca.clone();
                cb_clone.dir = dir.normalize();
                ca_clone.dir = dir.normalize();
                let nc: MainCylinder = MainCylinder {
                    id: rand::thread_rng().gen_range(0..1024),
                    ca: cb_clone,
                    cb: ca_clone,
                    h: dir.magnitude(),
                    r: r,
                    r_gr_id: (round_by_dec(r, 5) * DIVIDER) as u64,
                    ca_tor: u64::MAX,
                    cb_tor: u64::MAX,
                    step_vertex_buffer: StepVertexBuffer::default(),
                    bbx: Default::default(),
                };
                circles.push(nc);
            } else {
                let pts = circle2.gen_points();
                let mut in_pts = points.clone();
                in_pts.extend_from_slice(&pts);
                circles.extend(do_cyl_1(circle1, &in_pts));
            }
        }
    }
    (circles, toros)
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
pub fn extract_cartesian_point(t: &Table, point: &PlaceHolder<CartesianPointHolder>, scale: f64, ) -> Option<Point3> {
    match point {
        Ref(name) => match t.cartesian_point.get(&name_to_id(name.clone())) {
            None => None,
            Some(p) => {
                let x = unsafe { p.coordinates[0] * scale };
                let y = unsafe { p.coordinates[1] * scale };
                let z = unsafe { p.coordinates[2] * scale };
                Some(Point3::new(x, y, z))
            }
        },
        PlaceHolder::Owned(_) => None,
    }
}
pub fn extract_vector(t: &Table, vector: &PlaceHolder<VectorHolder>, scale: f64, ) -> Option<Vector3> {
    match vector {
        Ref(name) => match t.vector.get(&name_to_id(name.clone())) {
            None => None,
            Some(v) => {
                let mag=v.magnitude;
                let dir=extract_direction(t,&Some(v.orientation.clone()),scale).unwrap().normalize();
                Some(dir.mul(mag))
            }
        },
        PlaceHolder::Owned(_) => None,
    }
}
pub fn extract_position(t: &Table, pos: &PlaceHolder<Axis2PlacementHolder>, scale: f64, ) -> (Option<Point3>, Option<Vector3>, Option<Vector3>) {
    match pos {
        Ref(name) => match t.axis2_placement_3d.get(&name_to_id(name.clone())) {
            None => (None, None, None),
            Some(p) => {
                let dir_ref: Option<Vector3> = extract_direction(t, &p.ref_direction, scale);
                let dir: Option<Vector3> = extract_direction(t, &p.axis, scale);
                let loc: Option<Point3> = extract_cartesian_point(t, &p.location, scale);
                (loc, dir, dir_ref)
            }
        },
        PlaceHolder::Owned(_) => (None, None, None),
    }
}
pub fn extract_position3d(t: &Table, pos: &PlaceHolder<Axis2Placement3dHolder>, scale: f64, ) -> (Option<Point3>, Option<Vector3>, Option<Vector3>) {
    match pos {
        Ref(name) => match t.axis2_placement_3d.get(&name_to_id(name.clone())) {
            None => (None, None, None),
            Some(p) => {
                let dir_ref: Option<Vector3> = extract_direction(t, &p.ref_direction, scale);
                let dir: Option<Vector3> = extract_direction(t, &p.axis, scale);
                let loc: Option<Point3> = extract_cartesian_point(t, &p.location, scale);
                (loc, dir, dir_ref)
            }
        },
        PlaceHolder::Owned(_) => (None, None, None),
    }
}
pub fn extract_direction(t: &Table, _pos: &Option<PlaceHolder<DirectionHolder>>, scale: f64, ) -> Option<Vector3> {
    match _pos {
        None => None,
        Some(pos) => match pos {
            Ref(name) => match t.direction.get(&name_to_id(name.clone())) {
                None => None,
                Some(p) => {
                    let x = p.direction_ratios[0] * ROT_DIR_CCW;
                    let y = p.direction_ratios[1] * ROT_DIR_CCW;
                    let z = p.direction_ratios[2] * ROT_DIR_CCW;
                    Some(unsafe {
                        Vector3::new(x, y, z).mul(scale).normalize()
                    })
                }
            },
            PlaceHolder::Owned(_) => None,
        },
    }
}
fn name_to_id(name: Name) -> u64 {
    match name {
        Name::Entity(id) => id,
        Name::Value(_) => 0,
        Name::ConstantEntity(_) => 0,
        Name::ConstantValue(_) => 0,
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
pub fn nurbs_to_circle(nurbs_points: &Vec<Point3>) -> Option<MainCircle> {
    let p1 = circum_center(nurbs_points[0], nurbs_points[1], nurbs_points[2]);
    let p2 = circum_center(nurbs_points[1], nurbs_points[2], nurbs_points[3]);
    let p3 = circum_center(nurbs_points[2], nurbs_points[3], nurbs_points[4]);

    let p4 = circum_center(nurbs_points[3], nurbs_points[4], nurbs_points[5]);
    let p5 = circum_center(nurbs_points[4], nurbs_points[5], nurbs_points[6]);
    let p6 = circum_center(nurbs_points[5], nurbs_points[6], nurbs_points[7]);

    let p7 = circum_center(nurbs_points[6], nurbs_points[7], nurbs_points[8]);
    let p8 = circum_center(nurbs_points[7], nurbs_points[8], nurbs_points[9]);
    let p9 = circum_center(nurbs_points[8], nurbs_points[9], nurbs_points[10]);

    let dx = (p1.x + p2.x + p3.x + p4.x + p5.x + p6.x + p7.x + p8.x + p9.x) / 9.0;
    let dy = (p1.y + p2.y + p3.y + p4.y + p5.x + p6.y + p7.y + p8.y + p9.y) / 9.0;
    let dz = (p1.z + p2.z + p3.z + p4.z + p5.z + p6.z + p7.z + p8.z + p9.z) / 9.0;

    let cp: Point3 = Point3::new(dx, dy, dz);

    let r0 = cp.distance(nurbs_points[0]);
    let r1 = cp.distance(nurbs_points[1]);
    let r2 = cp.distance(nurbs_points[2]);
    let r3 = cp.distance(nurbs_points[3]);
    let r4 = cp.distance(nurbs_points[4]);
    let r5 = cp.distance(nurbs_points[5]);
    let r6 = cp.distance(nurbs_points[6]);
    let r7 = cp.distance(nurbs_points[7]);
    let r8 = cp.distance(nurbs_points[8]);
    let r9 = cp.distance(nurbs_points[9]);
    let r10 = cp.distance(nurbs_points[10]);

    let r_aver = (r0 + r1 + r2 + r3 + r4 + r5 + r6 + r7 + r8 + r9 + r10) / 11.0;

    let d_err = ((r_aver - r0).abs() + (r_aver - r1).abs() + (r_aver - r2).abs() + (r_aver - r3).abs() + (r_aver - r4).abs() + (r_aver - r5).abs() + (r_aver - r6).abs() + (r_aver - r7).abs() + (r_aver - r8).abs() + (r_aver - r9).abs() + (r_aver - r10).abs()) / 11.0;

    if (d_err < TOLE) {
        let r = r_aver.round();
        let dir = Plane::new(p1, p5, p8).normal().normalize();
        let radius_dir = p1.sub(cp).normalize();
        let circle: MainCircle = MainCircle {
            id: rand::thread_rng().gen_range(0..1024),
            radius: r,
            loc: cp,
            dir: dir,
            radius_dir: radius_dir,
            r_gr_id: (round_by_dec(r, 5) * DIVIDER) as u64,
        };
        //warn!("CIRCLE {:?}",circle.radius);
        Some(circle)
    } else {
        None
    }
}
pub fn find_bending_surface(cyls: &Vec<MainCylinder>, tors: &Vec<(BendToro)>, table: &Table, scale: f64) -> Vec<LRACLR> {
    let mut cc: Vec<MainCylinder> = cyls.clone();
    let mut tt: Vec<BendToro> = tors.clone();

    cc.sort_by(|a, b| a.r_gr_id.cmp(&b.r_gr_id));
    tt.sort_by(|a, b| a.r_gr_id.cmp(&b.r_gr_id));

    let mut chm: HashMap<u64, Vec<&MainCylinder>> = HashMap::new();
    cc.iter().chunk_by(|c| c.r_gr_id).into_iter().for_each(|(k, v)| {
        let vec = v.into_iter().collect_vec();
        chm.insert(k, vec);
    });
    let mut thm: HashMap<u64, Vec<&BendToro>> = HashMap::new();
    tt.iter().chunk_by(|c| c.r_gr_id).into_iter().for_each(|(k, v)| {
        let vec = v.into_iter().collect_vec();
        thm.insert(k, vec);
    });
    let mut radius_gr: u64 = 0;
    let mut op_qty: usize = 0;

    chm.iter().for_each(|(k, v)| {
        match thm.get(k) {
            None => {}
            Some(tors_same_r) => {

                if (v.len() == tors_same_r.len() + 1) {
                    let ops= tors_same_r.len()+v.len();
                    if(ops>op_qty){
                            radius_gr = k.clone();
                            op_qty=ops;
                    }else if (ops==op_qty) {
                        if (k.clone() > radius_gr) {
                            radius_gr = k.clone();
                            op_qty=ops;
                        }
                    }
                    //warn!("T {:?} C {:?} GR {:?} R {:?} OP {:?}", tors_same_r.len(), v.len(),k,radius_gr,op_qty);
                }

            }
        }
    });
    warn!("RADIUS GR {:?}", radius_gr);
    let mut ncyls: Vec<MainCylinder> = vec![];
    let mut ntors: Vec<(BendToro)> = vec![];
    if (radius_gr > 0) {
        //let mut allpts: Vec<Point3> = vec![];
        match chm.get(&radius_gr) {
            None => {}
            Some(c) => {
                c.iter().for_each(|nc| {
                    //allpts.extend(nc.gen_points());
                    //warn!("CYL R {:?}  {:?}  {:?}", nc.r, nc.ca.radius, nc.cb.radius);
                    //export_to_pt_str(&nc.gen_points(), nc.id.to_string().as_str());
                    ncyls.push(nc.clone().clone());
                });
            }
        }
        match thm.get(&radius_gr) {
            None => {}
            Some(c) => {
                c.iter().for_each(|nc| {
                    //allpts.extend(nc.gen_points());
                    //warn!("TOR R {:?}  {:?}  {:?}", nc.r, nc.ca.radius, nc.cb.radius);
                    //export_to_pt_str(&nc.gen_points(), nc.id.to_string().as_str());
                    ntors.push(nc.clone().clone());
                });
            }
        }
        // export_to_pt_str(&allpts, "dada");
    }


    let lracmd: Vec<LRACLR> = find_next_tor(&ncyls, &ntors,table,scale);

    //(ncyls, ntors)
    lracmd
}
fn find_next_tor(_cyls: &Vec<MainCylinder>, _tors: &Vec<(BendToro)>, table: &Table, scale: f64) -> Vec<LRACLR> {
    let mut lracmds: Vec<LRACLR> = vec![];
    let mut cc = _cyls.clone();
    let mut tors_bank: HashMap<u64, BendToro> = HashMap::new();
    let mut glob_counter = 0;
    _tors.iter().for_each(|t| {
        let mut nt = t.clone();
        nt.id = glob_counter;
        tors_bank.insert(nt.id, nt);
        glob_counter = glob_counter + 1;
    });

    //generate chain
    cc.iter_mut().for_each(|c| {
        c.id = glob_counter;
        glob_counter = glob_counter + 1;
        tors_bank.iter().for_each(|(k, t)| {
            if (c.ca.loc.distance(t.ca.loc) < TOLE || c.ca.loc.distance(t.cb.loc) < TOLE) {
                c.ca_tor = t.id;
            } else if (c.cb.loc.distance(t.ca.loc) < TOLE || c.cb.loc.distance(t.cb.loc) < TOLE) {
                c.cb_tor = t.id;
            }
        })
    });

    let mut ends: Vec<MainCylinder> = vec![];
    cc.iter().for_each(|c| {
       // warn!("ca_tor cb_tor {:?}  {:?} {:?}",c.h.round(), c.ca_tor,c.cb_tor);
        if (c.ca_tor == u64::MAX || c.cb_tor == u64::MAX) {
            ends.push(c.clone());
        }
    });
    if (ends.len() == 2) {
        let (s,e)=calculate_extra_len(ends[0].clone(),ends[1].clone(),table,scale,&tors_bank);
        let mut start: MainCylinder =s.clone();//ends[0].clone();
        let mut tor_id: u64 = { if (start.ca_tor != u64::MAX) { start.ca_tor } else { start.cb_tor } };
        let end: MainCylinder = e.clone();//ends[0].clone();
        //warn!("start {:?} end {:?} {:?} {:?}",start.id,end.id,end.ca_tor,end.cb_tor);
        let mut prev_plane = P_UP;
        let mut counter = 0;
        let mut has_next = true;

        while (has_next) {
            match tors_bank.get(&tor_id) {
                None => {
                    has_next = false;
                    let lra = LRACLR {
                        id1: counter,
                        id2: counter + 1,
                        l: end.h,
                        lt: 0.0,
                        r: 0.0,
                        a: 0.0,
                        clr: 0.0,
                        pipe_radius: start.r,
                    };
                    lracmds.push(lra);
                }
                Some(t) => {
                    let (a, b, c) = gen_plane_points(&start, t);
                    let frwd = b.sub(a).normalize();
                    let bend_plane: Vector3 = Plane::new(a, b, c).normal().normalize();
                    let ba = t.ca.loc.sub(t.bend_center_point);
                    let bb = t.cb.loc.sub(t.bend_center_point);
                    let mut bend_angle = ba.angle(bb);
                    let lt = abs(bend_angle.0) * (t.bend_radius);
                    let a = Deg::from(bend_angle).0;
                    let r: Rad<f64> = {
                        if (counter == 0) {
                            prev_plane = bend_plane;
                            Rad(0.0)
                        } else {
                            let a = {
                                //https://stackoverflow.com/questions/14066933/direct-way-of-computing-the-clockwise-angle-between-two-vectors
                                //Plane embedded in 3D
                                let dot = prev_plane.dot(bend_plane);
                                //let det = prev.x*t.bend_plane_norm.y*t.ca.dir.z + t.bend_plane_norm.x*t.ca.dir.y*prev.z + t.ca.dir.x*prev.y*t.bend_plane_norm.z - prev.z*t.bend_plane_norm.y*t.ca.dir.x - t.bend_plane_norm.z*t.ca.dir.y*prev.x - t.ca.dir.z*prev.y*t.bend_plane_norm.x;
                                let det = Matrix3::from_cols(
                                    bend_plane,
                                    prev_plane,
                                    frwd,
                                ).determinant();
                                let angle = det.atan2(dot);
                                let d = prev_plane.dot(bend_plane);
                                //warn!("dot {:?}  {:?} {:?}",Deg::from(Rad(angle)),Deg::from(prev.angle(t.bend_plane_norm)),d_frwd);
                                if (d == -1.0) {
                                    //Rad(0.0)
                                    Rad(-angle)
                                } else {
                                    Rad(angle)
                                }
                            };
                            prev_plane = bend_plane;
                            a
                        }
                    };
                    let lra = LRACLR {
                        id1: counter,
                        id2: counter + 1,
                        l: start.h,
                        lt: lt,
                        r: Deg::from(r).0,
                        a: a.abs(),
                        clr: t.bend_radius.abs(),
                        pipe_radius: start.r,
                    };
                    //warn!("ROT {:?}",lra.r);
                    //warn!("c {:?} t {:?} L {:?}",start.id, tor_id, start.h.round());
                    lracmds.push(lra);

                    counter = counter + 2;
                    let mut found = false;

                    cc.iter().for_each(|c| {
                        if (!found && c.id != start.id && (c.ca_tor == tor_id || c.cb_tor == tor_id)) {
                            tor_id = { if (c.ca_tor != tor_id) { c.ca_tor } else { c.cb_tor } };
                            start = c.clone();
                            found = true;
                        }
                    });
                }
            }
        }

        //warn!("lra {:?}", lracmds);
    } else {
        warn!("ends_not_found {:?}", ends.len());
    }
    lracmds
}
fn gen_plane_points(s: &MainCylinder, b: &BendToro) -> (Point3, Point3, Point3,) {
    if (s.ca.loc.distance(b.ca.loc) < TOLE) {
        (s.cb.loc.clone(), s.ca.loc.clone(), b.cb.loc.clone())
    } else if (s.ca.loc.distance(b.cb.loc) < TOLE) {
        (s.cb.loc.clone(), s.ca.loc.clone(), b.ca.loc.clone())
    } else if (s.cb.loc.distance(b.cb.loc) < TOLE) {
        (s.ca.loc.clone(), s.cb.loc.clone(), b.ca.loc.clone())
    } else {
        (s.ca.loc.clone(), s.cb.loc.clone(), b.cb.loc.clone())
    }
}
fn calculate_extra_len(a:MainCylinder, b:MainCylinder, table: &Table, scale: f64, tors_bank: &HashMap<u64, BendToro>) -> (MainCylinder, MainCylinder) {
    let mut extra_len_pts: Vec<Point3> = vec![];
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
                                    let id = name_to_id(name.clone());
                                    match table.plane.get(&id) {
                                        None => {}
                                        Some(plane) => {
                                            face_bounds.iter().for_each(|bound_holder| {
                                                match bound_holder {
                                                    PlaceHolder::Ref(bound_holder_name) => {
                                                        match table.face_bound.get(&name_to_id(bound_holder_name.clone())) {
                                                            None => {}
                                                            Some(face_bound_holder) => {
                                                                match &face_bound_holder.bound {
                                                                    PlaceHolder::Ref(name) => {
                                                                        match table.edge_loop.get(&name_to_id(name.clone())) {
                                                                            None => {}
                                                                            Some(edge_loop) => {
                                                                                edge_loop.edge_list.iter().for_each(|curve_holder| {
                                                                                    match curve_holder {
                                                                                        PlaceHolder::Ref(name) => {
                                                                                            match table.oriented_edge.get(&name_to_id(name.clone())) {
                                                                                                None => {}
                                                                                                Some(oe_holder) => {
                                                                                                    match &oe_holder.edge_element {
                                                                                                        PlaceHolder::Ref(name) => {
                                                                                                            match table.edge_curve.get(&name_to_id(name.clone())) {
                                                                                                                None => {}
                                                                                                                Some(c) => {
                                                                                                                    let sp = extract_vertex(&table, &c.edge_start, scale).unwrap();
                                                                                                                    let ep = extract_vertex(&table, &c.edge_end, scale).unwrap();
                                                                                                                    extra_len_pts.push(sp);
                                                                                                                    extra_len_pts.push(ep);
                                                                                                                }
                                                                                                            }
                                                                                                        }
                                                                                                        PlaceHolder::Owned(_) => {}
                                                                                                    }
                                                                                                }
                                                                                            }
                                                                                        }
                                                                                        PlaceHolder::Owned(_) => {}
                                                                                    }
                                                                                });
                                                                            }
                                                                        }
                                                                    }
                                                                    PlaceHolder::Owned(_) => {}
                                                                }
                                                            }
                                                        }
                                                    }
                                                    PlaceHolder::Owned(_) => {}
                                                }
                                            });
                                        }
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
    let mut start=a.clone();
    let mut end=b.clone();

    let start_tor={
        if(start.ca_tor!=u64::MAX){
            tors_bank.get(&start.ca_tor).unwrap()
        }else{
            tors_bank.get(&start.cb_tor).unwrap()
        }
    };
    let end_tor={
        if(end.ca_tor!=u64::MAX){
            tors_bank.get(&end.ca_tor).unwrap()
        }else{
            tors_bank.get(&end.cb_tor).unwrap()
        }
    };

    {
        if(start.ca.loc.distance(start_tor.ca.loc) < TOLE || start.ca.loc.distance(start_tor.cb.loc) < TOLE){
            let dir = start.cb.loc.sub(start.ca.loc).normalize();
            let mut new_p: Point3 = Point3::new(0.0, 0.0, 0.0);
            let mut curr_d = 0.0;
            let find_r = start.r * EXTRA_R_CALC;
            let d = start.r * EXTRA_LEN_CALC;
            let sp = start.ca.loc.clone();
            let ep = sp.clone() + dir * d;
            extra_len_pts.iter().for_each(|p| {
                let projected_point = project_point_to_vec(&dir, &ep, &p);
                let new_r = projected_point.distance(*p);
                let new_d = sp.distance(projected_point);
                let new_dir = projected_point.sub(sp);
                let is_coplanar = new_dir.dot(dir);
                if (new_r < find_r && is_coplanar > 0.0) {
                    if (new_d > curr_d && new_d < d) {
                        new_p = projected_point;
                        curr_d = new_d;
                    }
                }
            });
            if curr_d != 0.0 {
                start.cb.loc = new_p;
                let h = start.ca.loc.distance(start.cb.loc);
                start.h = h;
            }


        }
        else if (start.cb.loc.distance(start_tor.ca.loc) < TOLE ||start.cb.loc.distance(start_tor.cb.loc) < TOLE) {
            let dir = start.ca.loc.sub(start.cb.loc).normalize();
            let mut new_p: Point3 = Point3::new(0.0, 0.0, 0.0);
            let mut curr_d = 0.0;
            let find_r = start.r * EXTRA_R_CALC;
            let d = start.r * EXTRA_LEN_CALC;
            let sp = start.ca.loc.clone();
            let ep = sp.clone() + dir * d;
            extra_len_pts.iter().for_each(|p| {
                let projected_point = project_point_to_vec(&dir, &ep, &p);
                let new_r = projected_point.distance(*p);
                let new_d = sp.distance(projected_point);
                let new_dir = projected_point.sub(sp);
                let is_coplanar = new_dir.dot(dir);
                if (new_r < find_r && is_coplanar > 0.0) {
                    if (new_d > curr_d && new_d < d) {
                        new_p = projected_point;
                        curr_d = new_d;
                    }
                }
            });
            if curr_d != 0.0 {
                start.ca.loc = new_p;
                let h = start.ca.loc.distance(start.cb.loc);
                start.h = h;
            }
        }
    }

    {
        if(end.ca.loc.distance(end_tor.ca.loc) < TOLE || end.ca.loc.distance(end_tor.cb.loc) < TOLE){
            let dir = end.cb.loc.sub(end.ca.loc).normalize();
            let mut new_p: Point3 = Point3::new(0.0, 0.0, 0.0);
            let mut curr_d = 0.0;
            let find_r = end.r * EXTRA_R_CALC;
            let d = end.r * EXTRA_LEN_CALC;
            let sp = end.ca.loc.clone();
            let ep = sp.clone() + dir * d;
            extra_len_pts.iter().for_each(|p| {
                let projected_point = project_point_to_vec(&dir, &ep, &p);
                let new_r = projected_point.distance(*p);
                let new_d = sp.distance(projected_point);
                let new_dir = projected_point.sub(sp);
                let is_coplanar = new_dir.dot(dir);
                if (new_r < find_r && is_coplanar > 0.0) {
                    if (new_d > curr_d && new_d < d) {
                        new_p = projected_point;
                        curr_d = new_d;
                    }
                }
            });
            if curr_d != 0.0 {
                end.cb.loc = new_p;
                let h = end.ca.loc.distance(end.cb.loc);
                end.h = h;
            }


        }
        else if (end.cb.loc.distance(end_tor.ca.loc) < TOLE ||end.cb.loc.distance(end_tor.cb.loc) < TOLE) {
            let dir = end.ca.loc.sub(end.cb.loc).normalize();
            let mut new_p: Point3 = Point3::new(0.0, 0.0, 0.0);
            let mut curr_d = 0.0;
            let find_r = end.r * EXTRA_R_CALC;
            let d = end.r * EXTRA_LEN_CALC;
            let sp = end.ca.loc.clone();
            let ep = sp.clone() + dir * d;
            extra_len_pts.iter().for_each(|p| {
                let projected_point = project_point_to_vec(&dir, &ep, &p);
                let new_r = projected_point.distance(*p);
                let new_d = sp.distance(projected_point);
                let new_dir = projected_point.sub(sp);
                let is_coplanar = new_dir.dot(dir);
                if (new_r < find_r && is_coplanar > 0.0) {
                    if (new_d > curr_d && new_d < d) {
                        new_p = projected_point;
                        curr_d = new_d;
                    }
                }
            });
            if curr_d != 0.0 {
                end.ca.loc = new_p;
                let h = end.ca.loc.distance(end.cb.loc);
                end.h = h;
            }
        }
    }

    (start,end)
}


