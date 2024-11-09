pub mod cnc;
use crate::algo::cnc::CncOps;
use crate::device::{MeshVertex, StepVertexBuffer};
use cgmath::num_traits::real::Real;
use cgmath::{Basis3, Deg, InnerSpace, MetricSpace, Rad, Rotation, Rotation3};
use encoding_rs::WINDOWS_1251;
use encoding_rs_io::DecodeReaderBytesBuilder;
use itertools::Itertools;
use rand::random;
use ruststep::ast::Name;
use ruststep::tables::PlaceHolder;
use ruststep::tables::PlaceHolder::Ref;
use serde::{Deserialize, Serialize};
use shipyard::Component;
use std::collections::{HashMap, HashSet};
use std::f64::consts::PI;
use std::io::{ Read, Write};
use std::ops::{Mul, Sub};
use truck_base::bounding_box::BoundingBox;
use truck_base::cgmath64::{Point3, Vector3};
use truck_geometry::prelude::{BSplineCurve, Plane};
use truck_meshalgo::prelude::*;
use truck_stepio::r#in::{
    Axis2Placement3dHolder, Axis2PlacementHolder, BSplineCurveWithKnots, CartesianPoint,
    CartesianPointHolder, CurveAnyHolder, DirectionHolder, FaceBoundHolder,
    NonRationalBSplineSurfaceHolder, Table, VertexPointHolder,
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
pub const TOLE: f64 = 0.01;
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
        let p0: cgmath::Point3<f32> =
            cgmath::Point3::new(p0_64.x as f32, p0_64.y as f32, p0_64.z as f32);
        let p1: cgmath::Point3<f32> =
            cgmath::Point3::new(p1_64.x as f32, p1_64.y as f32, p1_64.z as f32);
        let p2: cgmath::Point3<f32> =
            cgmath::Point3::new(p2_64.x as f32, p2_64.y as f32, p2_64.z as f32);

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
}
impl MainCircle {
    pub fn gen_points_with_tol(&self, step_size: f64) -> Vec<Point3> {
        let mut pts_a: Vec<Point3> = vec![];
        let mut pts_b: Vec<Point3> = vec![];
        float_range(0.0, PI, step_size).for_each(|angle| {
            let rotation: Basis3<f64> = Rotation3::from_axis_angle(self.dir, Rad(angle));
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
            id: random(),
            radius: r,
            loc: Point3::new(0.0, 0.0, 0.0),
            dir: -P_FORWARD,
            radius_dir: P_UP,
        };

        let cb = MainCircle {
            id: random(),
            radius: r,
            loc: Point3::new(0.0, 0.0, 0.0) - P_FORWARD * h,
            dir: -P_FORWARD,
            radius_dir: P_UP,
        };
        let mut mc = MainCylinder {
            id: id as u64,
            ca: ca,
            cb: cb,
            h: h,
            r: r,
            r_gr_id: 0,
            ca_tor: 0,
            cb_tor: 0,

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
        float_range(0.0, PI * 2.0, PI / 18.0).for_each(|angle| {
            let rotation: Basis3<f64> = Rotation3::from_axis_angle(self.ca.dir, Rad(angle));
            let nv = rotation.rotate_vector(self.ca.radius_dir.clone());
            let p = self.ca.loc.clone() + nv * self.ca.radius;
            pts.push(p);
        });

        float_range(0.0, PI * 2.0, PI / 18.0).for_each(|angle| {
            let rotation: Basis3<f64> = Rotation3::from_axis_angle(self.cb.dir, Rad(angle));
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
                    id: random(),
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
                    id: random(),
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
                    id: random(),
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
                    id: random(),
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
                    id: random(),
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
    pub fn get_next(
        &self,
        hashids: &HashSet<u64>,
        cyls: &Vec<MainCylinder>,
    ) -> Option<MainCylinder> {
        let mut ret: Option<MainCylinder> = None;
        cyls.iter().for_each(|c| {
            if (self.id != c.id
                && !((c.ca_tor == u64::MAX || c.cb_tor == u64::MAX)
                    && (self.ca_tor == u64::MAX || self.cb_tor == u64::MAX)))
            {
                //if (self.id != c.id) {
                match ret {
                    None => {
                        if (self.ca_tor == c.ca_tor
                            || self.ca_tor == c.cb_tor
                            || self.cb_tor == c.ca_tor
                            || self.cb_tor == c.cb_tor)
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
            let a0 = (self.ca.loc.distance(other.ca.loc) + self.cb.loc.distance(other.ca.loc)
                - self.ca.loc.distance(self.cb.loc))
            .abs()
                < TOLE;
            let b0 = (self.ca.loc.distance(other.cb.loc) + self.cb.loc.distance(other.cb.loc)
                - self.ca.loc.distance(self.cb.loc))
            .abs()
                < TOLE;
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
        cyls.iter()
            .for_each(|cyl| match rad_group.get_mut(&cyl.r_gr_id) {
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
        main_cyls
            .iter()
            .for_each(|candidate| match merged_ids.get(&candidate.id) {
                None => main_cyls
                    .iter()
                    .for_each(|other| match merged_ids.get(&other.id) {
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
        main_cyls
            .iter()
            .for_each(|cyl| match merged_ids.get(&cyl.id) {
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
        let tesspoints = self.ca.gen_points_with_tol(PI / TESS_TOL_ANGLE);

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
    pub fn triangulate_with_start_index(&mut self, index: u32) -> u32 {
        let mut index: u32 = index;
        let mut indxes: Vec<u32> = vec![];
        let mut buffer: Vec<MeshVertex> = vec![];
        let main_dir = self.cb.loc.sub(self.ca.loc).normalize();
        let h = self.h;
        let tesspoints = self.ca.gen_points_with_tol(PI / TESS_TOL_ANGLE);

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
        let tesspoints = self.ca.gen_points_with_tol(PI / TESS_TOL_ANGLE);

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

        let rotation: Basis3<f64> =
            Rotation3::from_axis_angle(bend_plane_norm, -Rad(radians_angle));
        let p_tmp = rotation.rotate_point(Point3::new(0.0, -r - bend_radius, 0.0));
        let end_point = Point3::new(p_tmp.x, p_tmp.y + r + bend_radius, p_tmp.z);

        let ca = MainCircle {
            id: random(),
            radius: r,
            loc: start_point,
            dir: -P_FORWARD,
            radius_dir: P_UP,
        };
        let cb = MainCircle {
            id: random(),
            radius: r,
            loc: end_point,
            dir: -P_FORWARD,
            radius_dir: P_UP,
        };

        let mut tor = BendToro {
            id: id as u64,
            r: r,
            bend_radius: bend_radius,
            bend_center_point: dorn_point,
            bend_plane_norm: bend_plane_norm,
            radius_dir: bend_plane_norm,
            ca: ca,
            cb: cb,
            r_gr_id: 0,
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
                main_toros
                    .iter()
                    .for_each(|other| match merged.get(&other.id) {
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
                                        id: random(),
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
                let rotation: Basis3<f64> =
                    Rotation3::from_axis_angle(up_dir, Rad(curr_angle_stright));
                let nv = rotation.rotate_vector(bend_s_dir.clone());
                let p = self.bend_center_point.clone() + nv;
                anchors_stright.push(p);
                curr_angle_stright = curr_angle_stright + angle_step;
            }

            let mut curr_angle_rev = 0.0;
            for i in 0..=TESS_TOR_STEP {
                let rotation: Basis3<f64> =
                    Rotation3::from_axis_angle(up_dir, Rad(-curr_angle_rev));
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
                id: random(),
                radius: self.r,
                loc: pc0,
                dir: cdir0,
                radius_dir: r_dir_0,
            };
            let mc1 = MainCircle {
                id: random(),
                radius: self.r,
                loc: pc1,
                dir: cdir1,
                radius_dir: r_dir_1,
            };
            circles.push(mc0);
            circles.push(mc1);
        }
        for i in 0..circles.len() - 1 {
            //let c0 = circles[i].gen_points();
            //let c1 = circles[i + 1].gen_points();

            let c0 = circles[i].gen_points_with_tol(PI / TESS_TOL_TOR_ANGLE);
            let c1 = circles[i + 1].gen_points_with_tol(PI / TESS_TOL_TOR_ANGLE);

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
                let rotation: Basis3<f64> =
                    Rotation3::from_axis_angle(up_dir, Rad(curr_angle_stright));
                let nv = rotation.rotate_vector(bend_s_dir.clone());
                let p = self.bend_center_point.clone() + nv;
                anchors_stright.push(p);
                curr_angle_stright = curr_angle_stright + angle_step;
            }

            let mut curr_angle_rev = 0.0;
            for i in 0..=TESS_TOR_STEP {
                let rotation: Basis3<f64> =
                    Rotation3::from_axis_angle(up_dir, Rad(-curr_angle_rev));
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
                id: random(),
                radius: self.r,
                loc: pc0,
                dir: cdir0,
                radius_dir: r_dir_0,
            };
            let mc1 = MainCircle {
                id: random(),
                radius: self.r,
                loc: pc1,
                dir: cdir1,
                radius_dir: r_dir_1,
            };
            circles.push(mc0);
            circles.push(mc1);
        }
        for i in 0..circles.len() - 1 {
            //let c0 = circles[i].gen_points();
            //let c1 = circles[i + 1].gen_points();

            let c0 = circles[i].gen_points_with_tol(PI / TESS_TOL_TOR_ANGLE);
            let c1 = circles[i + 1].gen_points_with_tol(PI / TESS_TOL_TOR_ANGLE);

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
}

pub fn float_range(start: f64, threshold: f64, step_size: f64) -> impl Iterator<Item = f64> {
    std::iter::successors(Some(start), move |&prev| {
        let next = prev + step_size;
        (next < threshold).then_some(next)
    })
}

pub fn intersect_line_by_plane(
    cylinder_dir_vec: &Vector3,
    radius_vec: &Vector3,
    plane_point: &Point3,
    line_p0: &Point3,
    line_p1: &Point3,
) -> Point3 {
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
pub fn project_point_to_vec(
    proj_vector: &Vector3,
    point_on_proj_vector: &Point3,
    point_to_project: &Point3,
) -> Point3 {
    //https://stackoverflow.com/questions/9368436/3d-perpendicular-point-on-line-from-3d-point
    let pq: Vector3 = point_to_project.sub(point_on_proj_vector);
    let w2: Vector3 = pq - proj_vector * pq.dot(proj_vector.clone()) / proj_vector.magnitude2();
    let point: Point3 = point_to_project - w2;
    //println!("{:?} {:?}", point, 0);
    point
}
pub fn round_by_dec(x: f64, decimals: u32) -> f64 {
    let y = 10i64.pow(decimals);
    (x * y as f64).round() / y as f64
}

pub fn extract_vertex(t: &Table, vertex: &PlaceHolder<VertexPointHolder>,scale:f64) -> Option<Point3> {
    match vertex {
        Ref(name) => match t.vertex_point.get(&name_to_id(name.clone())) {
            None => None,
            Some(vtx) => extract_cartesian_point(t, &vtx.vertex_geometry,scale),
        },
        PlaceHolder::Owned(_) => None,
    }
}
pub fn extract_plane_points(table: &Table,scale:f64) -> Vec<Point3> {
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
                                                                                                                    let sp = extract_vertex(&table, &c.edge_start,scale).unwrap();
                                                                                                                    let ep = extract_vertex(&table, &c.edge_end,scale).unwrap();
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
pub fn analyze_bin(arr: &Vec<u8>) -> Option<CncOps> {
    let mut transcoded = DecodeReaderBytesBuilder::new()
        .encoding(Some(WINDOWS_1251))
        .build(arr.as_slice());
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
    let scale=extact_scale(&stp);
    let exchange = ruststep::parser::parse(&stp).unwrap();
    let table = Table::from_data_section(&exchange.data[0]);
    match extract_main_radius(&table,scale) {
        None => None,
        Some((cyls, bends)) => {
            let ops: CncOps = CncOps::new(&cyls, &bends);
            if (!ops.ops.is_empty()) {
                let extra_len_pts: Vec<Point3> = extract_plane_points(&table,scale);
                let mut extra_len_ops = ops.calculate_extra_len(&extra_len_pts);
                Some(extra_len_ops)
            } else {
                None
            }
        }
    }
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
pub fn extract_main_radius(table: &Table,scale:f64) -> Option<(Vec<MainCylinder>, Vec<BendToro>)> {
    let (cyls, bend_toros): (Vec<MainCylinder>, Vec<BendToro>) = extract_all_cylynders(table,scale);
    match MainCylinder::calculate_main_diam(&cyls) {
        None => None,
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
pub fn extract_cartesian_point(
    t: &Table,
    point: &PlaceHolder<CartesianPointHolder>,
    scale:f64,
) -> Option<Point3> {
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
pub fn extract_position(
    t: &Table,
    pos: &PlaceHolder<Axis2PlacementHolder>,
    scale:f64,
) -> (Option<Point3>, Option<Vector3>, Option<Vector3>) {
    match pos {
        Ref(name) => match t.axis2_placement_3d.get(&name_to_id(name.clone())) {
            None => (None, None, None),
            Some(p) => {
                let dir_ref: Option<Vector3> = extract_direction(t, &p.ref_direction,scale);
                let dir: Option<Vector3> = extract_direction(t, &p.axis,scale);
                let loc: Option<Point3> = extract_cartesian_point(t, &p.location,scale);
                (loc, dir, dir_ref)
            }
        },
        PlaceHolder::Owned(_) => (None, None, None),
    }
}
pub fn extract_position3d(
    t: &Table,
    pos: &PlaceHolder<Axis2Placement3dHolder>,
    scale:f64,
) -> (Option<Point3>, Option<Vector3>, Option<Vector3>) {
    match pos {
        Ref(name) => match t.axis2_placement_3d.get(&name_to_id(name.clone())) {
            None => (None, None, None),
            Some(p) => {
                let dir_ref: Option<Vector3> = extract_direction(t, &p.ref_direction,scale);
                let dir: Option<Vector3> = extract_direction(t, &p.axis,scale);
                let loc: Option<Point3> = extract_cartesian_point(t, &p.location,scale);
                (loc, dir, dir_ref)
            }
        },
        PlaceHolder::Owned(_) => (None, None, None),
    }
}
pub fn extract_direction(
    t: &Table,
    _pos: &Option<PlaceHolder<DirectionHolder>>,
    scale:f64,
) -> Option<Vector3> {
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
                        Vector3::new(x, y, z)
                            .mul(scale)
                            .normalize()
                    })
                }
            },
            PlaceHolder::Owned(_) => None,
        },
    }
}
pub fn extract_circles_bounds(
    table: &Table,
    bounds: &Vec<PlaceHolder<FaceBoundHolder>>,
    cyl_sp: &Point3,
    cyl_dir: &Vector3,
    cyl_id: u64,
    surf_radius: f64,
    scale: f64,
) -> Option<MainCylinder> {
    //println!("{:?}",surf_radius);
    let mut points: Vec<Point3> = vec![];
    let mut candidates: Vec<(MainCircle)> = vec![];
    let control_point = cyl_sp.clone() + cyl_dir * 5000000.0;
    let mut oriented_edge_ids: Vec<u64> = vec![];
    bounds.iter().for_each(|bound_holder| {
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
                                                                                            let circle_r = circle.radius *  scale;
                                                                                            let (loc, dir, dir_rad) = extract_position(table, &circle.position,scale);
                                                                                            let is_coplanar = dir.unwrap().dot(cyl_dir.clone()).abs() - 1.0 < TOLE;
                                                                                            //let dot=dir.unwrap().normalize().dot(cyl_dir.clone().normalize()).abs()-1.0;
                                                                                            //println!("cyl_id {:?} {:?}",cyl_id,is_coplanar);
                                                                                            if (is_coplanar) {
                                                                                                let mc: MainCircle = MainCircle {
                                                                                                    id: curve_id.clone(),
                                                                                                    radius: circle_r,
                                                                                                    loc: loc.unwrap().clone(),
                                                                                                    dir: dir.unwrap().normalize(),
                                                                                                    radius_dir: dir_rad.unwrap().normalize(),
                                                                                                };
                                                                                                //export_to_pt(&mc.gen_points(), curve_id.clone() as i32);
                                                                                                candidates.push(mc);
                                                                                            }
                                                                                        }
                                                                                    }
                                                                                    match table.b_spline_curve_with_knots.get(curve_id) {
                                                                                        None => {}
                                                                                        Some(spline) => {
                                                                                            found = true;
                                                                                            let mut contrl_points: Vec<CartesianPoint> = vec![];
                                                                                            spline.control_points_list.iter().for_each(|cp| {
                                                                                                let pnt = extract_cartesian_point(&table, &cp,scale).unwrap();
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
                                                                                            }
                                                                                            //export_to_pt(&tess_points, random());

                                                                                            tess_points.iter().for_each(|point| {
                                                                                                let pp = project_point_to_vec(cyl_dir, cyl_sp, point);
                                                                                                let dist = (point.distance(pp) - surf_radius).abs() < TOLE;
                                                                                                if (dist) {
                                                                                                    let mc: MainCircle = MainCircle {
                                                                                                        id: random(),
                                                                                                        radius: surf_radius,
                                                                                                        loc: pp.clone(),
                                                                                                        dir: cyl_dir.normalize(),
                                                                                                        radius_dir: point.sub(pp).normalize(),
                                                                                                    };
                                                                                                    //export_to_pt(&mc.gen_points(), curve_id.clone() as i32);
                                                                                                    candidates.push(mc);
                                                                                                }

                                                                                                //println!("{:?} {:?} {:?} {:?}",cyl_id,curve_id,dist,surf_radius);
                                                                                            });
                                                                                        }
                                                                                    }
                                                                                    match table.ellipse.get(curve_id) {
                                                                                        None => {}
                                                                                        Some(ellipse) => {
                                                                                            found = true;
                                                                                            let circle_r = surf_radius;
                                                                                            let (loc, dir, dir_rad) = extract_position(table, &ellipse.position,scale);
                                                                                            let is_coplanar = dir.unwrap().dot(cyl_dir.clone()).abs() - 1.0 < TOLE;
                                                                                            //let dot=dir.unwrap().normalize().dot(cyl_dir.clone().normalize()).abs()-1.0;
                                                                                            //println!("cyl_id {:?} {:?}",cyl_id,is_coplanar);
                                                                                            if (is_coplanar) {
                                                                                                let mc: MainCircle = MainCircle {
                                                                                                    id: curve_id.clone(),
                                                                                                    radius: circle_r,
                                                                                                    loc: loc.unwrap().clone(),
                                                                                                    dir: dir.unwrap().normalize(),
                                                                                                    radius_dir: dir_rad.unwrap().normalize(),
                                                                                                };
                                                                                                //export_to_pt(&mc.gen_points(), curve_id.clone() as i32);
                                                                                                candidates.push(mc);
                                                                                            }
                                                                                        }
                                                                                    }
                                                                                    match table.line.get(curve_id) {
                                                                                        None => {}
                                                                                        Some(line) => {
                                                                                            found = true;
                                                                                        }
                                                                                    }

                                                                                    if (!found) {}

                                                                                    //println!("name ID {:?} ", &name_to_id(name.clone()))
                                                                                }
                                                                                PlaceHolder::Owned(_) => {}
                                                                            }
                                                                            let sp = extract_vertex(&table, &c.edge_start,scale).unwrap();
                                                                            let ep = extract_vertex(&table, &c.edge_end,scale).unwrap();
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
    let mut candidates_dists: Vec<(f64, MainCircle)> = vec![];
    candidates.iter().for_each(|cp| {
        let dist = cp.loc.distance(control_point);
        candidates_dists.push((dist, cp.clone()));
    });
    if (candidates_dists.is_empty() || candidates_dists.len() == 1) {
        None
    } else {
        candidates_dists.sort_by(|(dist, cc), (dist1, cc1)| dist.partial_cmp(dist1).unwrap());
        let sp = candidates_dists.first().unwrap();
        let ep = candidates_dists.last().unwrap();
        let cyl: MainCylinder = MainCylinder {
            id: cyl_id,
            ca: sp.1.clone(),
            cb: ep.1.clone(),
            h: ep.0,
            r: sp.1.radius,
            r_gr_id: (round_by_dec(sp.1.radius, 5) * DIVIDER) as u64,
            ca_tor: u64::MAX,
            cb_tor: u64::MAX,
            step_vertex_buffer: StepVertexBuffer::default(),
            bbx: BoundingBox::default(),
        };
        Some(cyl)
    }
}
pub fn extract_circles_bounds_tourus(
    table: &Table,
    bounds: &Vec<PlaceHolder<FaceBoundHolder>>,
    bend_center_point: &Point3,
    bend_plane_normal: &Vector3,
    bend_radius_dir: &Vector3,
    tor_id: u64,
    bend_radius: f64,
    pipe_radius: f64,
    scale:f64,
) -> Option<BendToro> {
    let mut points: Vec<Point3> = vec![];
    let mut candidates: Vec<(MainCircle)> = vec![];
    let mut oriented_edge_ids: Vec<u64> = vec![];
    bounds.iter().for_each(|bound_holder| {
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
                                                                                            let (loc, dir, dir_rad) = extract_position(table, &circle.position,scale);
                                                                                            let is_same_radius = (circle_r - pipe_radius).abs() < TOLE;
                                                                                            //let dot=dir.unwrap().normalize().dot(cyl_dir.clone().normalize()).abs()-1.0;
                                                                                            //println!("cyl_id {:?} {:?}",cyl_id,is_coplanar);
                                                                                            if (is_same_radius) {
                                                                                                let mc: MainCircle = MainCircle {
                                                                                                    id: curve_id.clone(),
                                                                                                    radius: circle_r,
                                                                                                    loc: loc.unwrap().clone(),
                                                                                                    dir: dir.unwrap().normalize(),
                                                                                                    radius_dir: dir_rad.unwrap().normalize(),
                                                                                                };
                                                                                                //export_to_pt(&mc.gen_points(), curve_id.clone() as i32);
                                                                                                candidates.push(mc);
                                                                                            }
                                                                                        }
                                                                                    }

                                                                                    match table.b_spline_curve_with_knots.get(curve_id) {
                                                                                        None => {}
                                                                                        Some(spline) => {
                                                                                            found = true;
                                                                                            found = true;
                                                                                            let mut contrl_points: Vec<CartesianPoint> = vec![];
                                                                                            spline.control_points_list.iter().for_each(|cp| {
                                                                                                let pnt = extract_cartesian_point(&table, &cp,scale).unwrap();
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
                                                                                            }

                                                                                            let p_start = res.subs(0.0);
                                                                                            let p_middle = res.subs(0.5);
                                                                                            let p_end = res.subs(1.0);
                                                                                            let is_same_rad = (p_start.distance(p_middle) / 2.0 - pipe_radius).abs() < TOLE;

                                                                                            if (p_start.distance(p_end).abs() < TOLE && is_same_rad) {
                                                                                                let dir_to_center = (p_middle - p_start).normalize();
                                                                                                let cp = p_start.clone() + dir_to_center * pipe_radius;
                                                                                                let radius_dir = (p_start.clone() - cp.clone()).normalize();
                                                                                                let circle_plane = Plane::new(tess_points[0].clone(), tess_points[3].clone(), tess_points[8].clone());
                                                                                                let dir = circle_plane.normal().normalize();
                                                                                                let mc = MainCircle {
                                                                                                    id: random(),
                                                                                                    radius: pipe_radius,
                                                                                                    loc: cp.clone(),
                                                                                                    dir: dir.clone(),
                                                                                                    radius_dir: radius_dir.clone(),
                                                                                                };
                                                                                                //let is_same_radius = (circle_r - pipe_radius).abs() < TOLE;
                                                                                                candidates.push(mc);
                                                                                            }
                                                                                        }
                                                                                    }

                                                                                    match table.ellipse.get(curve_id) {
                                                                                        None => {}
                                                                                        Some(ellipse) => {
                                                                                            found = true;
                                                                                            println!("ellipse!!!")
                                                                                        }
                                                                                    }

                                                                                    if (!found) {
                                                                                        println!("!found {:?}", curve_id)
                                                                                    }
                                                                                }
                                                                                PlaceHolder::Owned(_) => {}
                                                                            }
                                                                            let sp = extract_vertex(&table, &c.edge_start,scale).unwrap();
                                                                            let ep = extract_vertex(&table, &c.edge_end,scale).unwrap();
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

    let candidates_remove_dubs = MainCircle::remove_dublicates(&candidates);
    if (candidates_remove_dubs.len() == 2) {
        let dist = candidates_remove_dubs[0]
            .loc
            .distance(candidates_remove_dubs[1].loc);
        if (dist > TOLE) {
            let bend: BendToro = BendToro {
                id: tor_id,
                r: pipe_radius,
                bend_radius: bend_radius,
                bend_center_point: bend_center_point.clone(),
                bend_plane_norm: bend_plane_normal.clone(),
                radius_dir: bend_radius_dir.clone(),
                ca: candidates_remove_dubs[0].clone(),
                cb: candidates_remove_dubs[1].clone(),
                r_gr_id: (round_by_dec(pipe_radius, 5) * DIVIDER) as u64,
                step_vertex_buffer: StepVertexBuffer::default(),
                bbx: BoundingBox::default(),
            };
            Some(bend)
        } else {
            None
        }
    } else {
        None
    }
}
pub fn extract_all_cylynders(table: &Table, scale:f64) -> (Vec<MainCylinder>, Vec<(BendToro)>) {
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
                                            let r = cyl.radius *  scale;
                                            let (loc, dir, dir_rad) = extract_position3d(table, &cyl.position,scale);
                                            match extract_circles_bounds(table, face_bounds, &loc.unwrap(), &dir.unwrap(), id, r,scale) {
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
                                            let r = toro.minor_radius *  scale;
                                            let r_bend = toro.major_radius *  scale;
                                            let (loc, bend_plane_normal, dir_rad) = extract_position3d(table, &toro.position,scale);
                                            if (toro.major_radius * scale > MAX_BEND_RADIUS) {
                                                //TODO potentional error with big tole
                                                match extract_circles_bounds(table, face_bounds, &loc.unwrap(), &bend_plane_normal.unwrap(), id, r,scale) {
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
                                                                                    r,
                                                                                    scale,) {
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
fn name_to_id(name: Name) -> u64 {
    match name {
        Name::Entity(id) => id,
        Name::Value(_) => 0,
        Name::ConstantEntity(_) => 0,
        Name::ConstantValue(_) => 0,
    }
}
pub fn angle_with_sign(v1: Vector3, v2: Vector3, plane_norm: Vector3) -> f64 {
    //https://stackoverflow.com/questions/14066933/direct-way-of-computing-the-clockwise-angle-between-two-vectors
    /*    let dot = -v1.dot(v2);
    let det = -Matrix3::from_cols(v1, v2, plane_norm).determinant();
    let angle = det.atan2(dot)+PI;
    angle*/

    let dot = v1.dot(v2);
    let det = Matrix3::from_cols(v1, v2, plane_norm).determinant();
    let angle = det.atan2(dot);
    angle
}
