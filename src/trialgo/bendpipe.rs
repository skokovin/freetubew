use std::collections::{HashMap, HashSet};
use std::f64::consts::PI;
use std::ops::Sub;
use cgmath::{Basis3, InnerSpace, MetricSpace, Rad, Rotation, Rotation3};
use itertools::Itertools;
use nanoid::nanoid;
use rand::random;
use truck_base::cgmath64::{Point3, Vector3};
use truck_geometry::prelude::Plane;
use truck_geotrait::Invertible;
use truck_meshalgo::filters::{NormalFilters, OptimizingFilter, StructuringFilter};
use truck_polymesh::{Faces, obj, PolygonMesh, StandardAttributes, StandardVertex};
use crate::device::Triangle;
use crate::trialgo::{export_to_pt, export_to_pt_str, float_range, project_point_to_vec, round_by_dec};
use crate::trialgo::analyzepl::{TESS_TOL_ANGLE, TESS_TOR_STEP, TOLE};
const P_FORWARD: Vector3 = Vector3::new(1.0, 0.0, 0.0);
const P_FORWARD_REVERSE: Vector3 = Vector3::new(-1.0, 0.0, 0.0);
const P_RIGHT: Vector3 = Vector3::new(0.0, 1.0, 0.0);
const P_UP: Vector3 = Vector3::new(0.0, 0.0, 1.0);
/*#[derive(Clone)]
pub struct Triangle {
    pub vtx: [Point3; 3],
    pub normal: Vector3,
}*/
#[derive(Clone)]
pub struct MainCircle {
    pub id: u64,
    pub radius: f64,
    pub loc: Point3,
    pub dir: Vector3,
    pub radius_dir: Vector3,
}
impl MainCircle {
    pub fn gen_points(&self) -> Vec<Point3> {
        let mut pts: Vec<Point3> = vec![];
        float_range(0.0, PI * 2.0, PI / 18.0).for_each(|angle| {
            let rotation: Basis3<f64> = Rotation3::from_axis_angle(self.dir, Rad(angle));
            let nv = rotation.rotate_vector(self.radius_dir.clone());
            let p = self.loc.clone() + nv * self.radius;
            pts.push(p);
        });
        pts
    }
    pub fn gen_points_with_cp(&self) -> Vec<Point3> {
        let mut pts: Vec<Point3> = vec![];
        float_range(0.0, PI * 2.0, PI / 18.0).for_each(|angle| {
            let rotation: Basis3<f64> = Rotation3::from_axis_angle(self.dir, Rad(angle));
            let nv = rotation.rotate_vector(self.radius_dir.clone());
            let p = self.loc.clone() + nv * self.radius;
            pts.push(p);
        });
        pts.push(self.loc.clone());
        pts
    }
    pub fn gen_points_with_tol(&self, step_size: f64) -> Vec<Point3> {
        let mut pts: Vec<Point3> = vec![];
        float_range(0.0, PI * 2.0, step_size).for_each(|angle| {
            let rotation: Basis3<f64> = Rotation3::from_axis_angle(self.dir, Rad(angle));
            let nv = rotation.rotate_vector(self.radius_dir.clone());
            let p = self.loc.clone() + nv * self.radius;
            pts.push(p);
        });
        pts.push(self.loc.clone());
        pts
    }
    pub fn to_pts_file(&self) {
        let mut pts: Vec<Point3> = vec![];
        float_range(0.0, PI * 2.0, PI / 18.0).for_each(|angle| {
            let rotation: Basis3<f64> = Rotation3::from_axis_angle(self.dir, Rad(angle));
            let nv = rotation.rotate_vector(self.radius_dir.clone());
            let p = self.loc.clone() + nv * self.radius;
            pts.push(p);
        });
        pts.push(self.loc.clone());
        export_to_pt(&pts, self.id as i32);
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
    pub fn extract_cylinders(circles: &Vec<MainCircle>) -> Vec<MainCylinder> {
        let mut cyls: Vec<MainCylinder> = vec![];
        let mut counter = 1;
        let divider = 100000000.0;
        circles.iter().for_each(|c| {
            let hashr: u64 = (round_by_dec(c.radius, 5) * divider) as u64;
            match c.find_next(&circles) {
                None => {}
                Some(other) => {
                    let h = c.loc.distance(other.loc);
                    let mc: MainCylinder = MainCylinder {
                        id: counter,
                        ca: c.clone(),
                        cb: other.clone(),
                        h: h,
                        r: c.radius,
                        r_gr_id: hashr,
                        ca_tor: u64::MAX,
                        cb_tor: u64::MAX,
                        triangles: vec![],
                    };
                    cyls.push(mc);
                    counter = counter + 1;
                }
            }
        });
        cyls
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
    pub fn by_3_pts(p1: Point3, p2: Point3, p3: Point3) {
        let D21x = p2.x - p1.x;
        let D21y = p2.y - p1.y;
        let D21z = p2.z - p1.z;
        let D31x = p3.x - p1.x;
        let D31y = p3.y - p1.y;
        let D31z = p3.z - p1.z;

        let F2 = 0.5 * (D21x * D21x + D21y * D21y + D21z * D21z);
        let F3 = 0.5 * (D31x * D31x + D31y * D31y + D31z * D31z);

        let M23xy = D21x * D31y - D21y * D31x;
        let M23yz = D21y * D31z - D21z * D31y;
        let M23xz = D21z * D31x - D21x * D31z;

        let F23x = F2 * D31x - F3 * D21x;
        let F23y = F2 * D31y - F3 * D21y;
        let F23z = F2 * D31z - F3 * D21z;

        let Cx = p1.x + (M23xy * F23y - M23xz * F23z) / (M23xy * M23xy + M23yz * M23yz + M23xz * M23xz);
        let Cy = p1.y + (M23yz * F23z - M23xy * F23x) / (M23xy * M23xy + M23yz * M23yz + M23xz * M23xz);
        let Cz = p1.z + (M23xz * F23x - M23yz * F23y) / (M23xy * M23xy + M23yz * M23yz + M23xz * M23xz);

        let cp = Point3::new(Cx, Cy, Cz);
        let plane = Plane::new(p1.clone(), p2.clone(), p3.clone());
        let plane_norm = plane.normal().normalize();

        let some_dir_p = cp.clone() + plane_norm * 10000.0;
        let dir = some_dir_p.sub(cp.clone()).normalize();
        let radius_dir = p1.sub(cp.clone()).normalize();
        let r = p1.distance(cp);
    }
}
#[derive(Clone)]
pub struct MainCylinder {
    pub id: u64,
    pub ca: MainCircle,
    pub cb: MainCircle,
    pub h: f64,
    pub r: f64,
    pub r_gr_id: u64,
    pub ca_tor: u64,
    pub cb_tor: u64,
    pub triangles: Vec<Triangle>,
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
            triangles: vec![],
        };
        mc.triangulate();
        mc
    }
    pub fn find_by_pt(pt: &Point3, cyls: &Vec<MainCylinder>) -> Option<MainCylinder> {
        let mut found: Option<MainCylinder> = None;
        cyls.iter().for_each(|cyl| {
            match found {
                None => {
                    if (pt.distance(cyl.ca.loc) < TOLE) {
                        found = Some(cyl.clone());
                    }
                    if (pt.distance(cyl.cb.loc) < TOLE) {
                        found = Some(cyl.clone());
                    }
                }
                Some(_) => {}
            }
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
    pub fn to_pts_file(&self) {
        let mut pts: Vec<Point3> = self.gen_points();
        let id = nanoid!(4);
        export_to_pt_str(&pts, id.as_str());
    }
    pub fn to_pts_file_with_id(&self) {
        let mut pts: Vec<Point3> = self.gen_points();

        export_to_pt_str(&pts, &self.id.to_string());
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
                    triangles: vec![],
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
                    triangles: vec![],
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
                    triangles: vec![],
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
                    triangles: vec![],
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
                    triangles: vec![],
                };
                Some(new_c)
            } else {
                None
            }
        } else {
            None
        }
    }
    pub fn get_next(&self, hashids: &HashSet<u64>, cyls: &Vec<MainCylinder>) -> Option<MainCylinder> {
        let mut ret: Option<MainCylinder> = None;
        cyls.iter().for_each(|c| {
            if (self.id != c.id && !((c.ca_tor == u64::MAX || c.cb_tor == u64::MAX) && (self.ca_tor == u64::MAX || self.cb_tor == u64::MAX))) {
                //if (self.id != c.id) {
                match ret {
                    None => {
                        if (self.ca_tor == c.ca_tor || self.ca_tor == c.cb_tor || self.cb_tor == c.ca_tor || self.cb_tor == c.cb_tor) {
                            match hashids.get(&c.id) {
                                None => {
                                    ret = Some(c.clone())
                                }
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
                Some(toro) => {
                    c.ca_tor = toro.id
                }
            }
            match BendToro::find_by_pt(&c.cb.loc, bends) {
                None => {}
                Some(toro) => {
                    c.cb_tor = toro.id
                }
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
        cyls.iter().for_each(|cyl| {
            match rad_group.get_mut(&cyl.r_gr_id) {
                None => {
                    rad_group.insert(cyl.r_gr_id.clone(), cyl.h);
                }
                Some(dist) => {
                    *dist = *dist + cyl.h;
                }
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
            //rad_group_vec.sort_by(|(id, dist), (id1, dist1)| id.cmp(id1));
            rad_group_vec.sort_by(|(id, dist), (id1, dist1)| dist.partial_cmp(dist1).unwrap());
            rad_group_vec.inverse().iter().for_each(|(id, dist)| {
                if (dist.clone() >= lasd_dist) {
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
        main_cyls.iter().for_each(|candidate| {
            match merged_ids.get(&candidate.id) {
                None => {
                    main_cyls.iter().for_each(|other| {
                        match merged_ids.get(&other.id) {
                            None => {
                                match candidate.merge_me(other) {
                                    None => {}
                                    Some(merged_cyl) => {
                                        merged.push(merged_cyl);
                                        merged_ids.insert(other.id);
                                        merged_ids.insert(candidate.id);
                                    }
                                }
                            }
                            Some(_) => {}
                        }
                    })
                }
                Some(_) => {}
            }
        });
        main_cyls.iter().for_each(|cyl| {
            match merged_ids.get(&cyl.id) {
                None => {
                    ret.push(cyl.clone());
                }
                Some(_) => {}
            }
        });
        ret.extend_from_slice(merged.as_slice());
        ret
    }
    pub fn flush_to_pt_files(cyls: &Vec<MainCylinder>) {
        cyls.iter().for_each(|cyl| {
            //println!("{:?}", cyl.r_gr_id);
            cyl.to_pts_file_with_id();
        });
    }
    pub fn flush_to_one_pt_files(cyls: &Vec<MainCylinder>) {
        let mut pts: Vec<Point3> = vec![];
        cyls.iter().for_each(|cyl| {
            pts.extend_from_slice(cyl.gen_points().as_slice());
        });
        export_to_pt(&pts, 9999);
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
        self.triangles = vec![];
        let main_dir = self.cb.loc.sub(self.ca.loc).normalize();
        //let h = self.cb.loc.distance(self.ca.loc);
        let h = self.h;
        let tesspoints = self.ca.gen_points_with_tol(PI / TESS_TOL_ANGLE);
        //export_to_pt(&tesspoints, self.id as i32);
        for i in 0..tesspoints.iter().len() - 2 {
            let p0 = tesspoints[i];

            let p1 = tesspoints[i + 1];
            let p2 = p0.clone() + main_dir * h;
            let p3 = p1.clone() + main_dir * h;
            let plane = Plane::new(p0.clone(), p1.clone(), p3.clone());
            let n = plane.normal().normalize();
            let r_dir = p0.sub(self.ca.loc);
            let is_coplanar = n.dot(r_dir);
            if (is_coplanar < 0.0) {
                self.triangles.push(Triangle::fromX64(p0.clone(), p3.clone(), p1.clone(), n.clone()));
                self.triangles.push(Triangle::fromX64(p0.clone(), p2.clone(), p3.clone(), n.clone()));
            } else {
                self.triangles.push(Triangle::fromX64(p0.clone(), p1.clone(), p3.clone(), n.clone()));
                self.triangles.push(Triangle::fromX64(p0.clone(), p3.clone(), p2.clone(), n.clone()));
            }
        }
    }
    pub fn to_polygon_mesh(&self) -> PolygonMesh {
        let mut positions: Vec<Point3> = vec![];
        let mut tri_normals: Vec<Vector3> = vec![];
        let mut indx: Vec<[StandardVertex; 3]> = vec![];
        self.triangles.iter().for_each(|tri| {
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
    pub fn to_obj(&self) {
        let mut positions: Vec<Point3> = vec![];
        let mut indx: Vec<[StandardVertex; 3]> = vec![];
        self.triangles.iter().for_each(|tri| {
            positions.extend_from_slice(tri.as_p64().as_slice());
        });
        (0..positions.len()).chunks(3).into_iter().for_each(|ch| {
            let ii = ch.collect_vec();
            indx.push([ii[0].into(), ii[1].into(), ii[2].into()]);
        });

        let mut polymesh = PolygonMesh::new(
            StandardAttributes {
                positions: positions,
                ..Default::default()
            },
            Faces::from_tri_and_quad_faces(
                indx,
                Vec::new(),
            ),
        );
        let path = format!("d:\\pipe_project\\cyl_{}.obj", self.id);
        let mut obj_file = std::fs::File::create(path).unwrap();
        obj::write(&polymesh, obj_file).unwrap();
    }
    pub fn recalculate_h(&mut self) {
        self.h = self.cb.loc.distance(self.ca.loc);
    }
    pub fn to_render_data(&self) {
        let mut mesh = self.to_polygon_mesh();
        let vrtx: &Vec<truck_base::cgmath64::Point3> = mesh.positions();
        let norms: &Vec<truck_base::cgmath64::Vector3> = mesh.normals();
    }
}
#[derive(Clone)]
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
    pub triangles: Vec<Triangle>,
}
impl BendToro {
    pub fn from_angle(radians_angle: f64, bend_radius: f64, r: f64, id: u32) -> BendToro {
        let start_point: Point3 = Point3::new(0.0, 0.0, 0.0);
        let dorn_point: Point3 = Point3::new(0.0, r+bend_radius, 0.0);
        let bend_plane_norm: Vector3 = Vector3::new(0.0, 0.0, 1.0);

        let rotation: Basis3<f64> = Rotation3::from_axis_angle(bend_plane_norm, -Rad(radians_angle));
        let p_tmp = rotation.rotate_point(Point3::new(0.0, -r-bend_radius, 0.0));
        let end_point = Point3::new(p_tmp.x, p_tmp.y + r+bend_radius, p_tmp.z);

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
            triangles: vec![],
        };
        tor.triangulate(&P_FORWARD_REVERSE);
        tor
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
    pub fn to_pts_file(&self) {
        let mut pts: Vec<Point3> = self.gen_points();
        let id = nanoid!(4);
        export_to_pt_str(&pts, id.as_str());
    }
    pub fn to_pts_file_with_id(&self) {
        let mut pts: Vec<Point3> = self.gen_points();
        export_to_pt_str(&pts, &self.id.to_string());
    }
    pub fn all_to_pt_file(bents: &Vec<BendToro>) {
        bents.iter().for_each(|bend| {
            bend.to_pts_file_with_id();
        });
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
        toros.iter().for_each(|t| {
            match ret_tor {
                None => {
                    if (pt.clone().distance(t.ca.loc) < TOLE) {
                        ret_tor = Some((t.clone()));
                    }
                    if (pt.clone().distance(t.cb.loc) < TOLE) {
                        ret_tor = Some((t.clone()));
                    }
                }
                Some(_) => {}
            }
        });
        ret_tor
    }
    pub fn merge(main_toros: &Vec<BendToro>) -> Vec<BendToro> {
        let mut ret: Vec<BendToro> = vec![];
        let mut merged: HashSet<u64> = HashSet::new();
        let mut new_tors: Vec<BendToro> = vec![];
        main_toros.iter().for_each(|im| {
            match merged.get(&im.id) {
                None => {
                    main_toros.iter().for_each(|other| {
                        match merged.get(&other.id) {
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
                                            triangles: vec![],
                                        };

                                        new_tors.push(new_tor);
                                    }
                                }
                            }
                            Some(_) => {}
                        }
                    });
                }
                Some(_) => {}
            }
        });
        main_toros.iter().for_each(|mt| {
            match merged.get(&mt.id) {
                None => {
                    ret.push(mt.clone());
                }
                Some(_) => {}
            }
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
            let c0 = circles[i].gen_points();
            let c1 = circles[i + 1].gen_points();

            for j in 0..c0.len() - 1 {
                let p0 = c0[j];
                let p1 = c0[j + 1];
                let p2 = c1[j];
                let p3 = c1[j + 1];
                let plane = Plane::new(p0.clone(), p1.clone(), p3.clone());
                let n = plane.normal().normalize();

                let r_dir = p0.sub(circles[i].loc);
                let is_coplanar = n.dot(r_dir);


                if (is_coplanar > 0.0) {
                    self.triangles.push(Triangle::fromX64(p0.clone(), p1.clone(), p3.clone(), n.clone()));
                    self.triangles.push(Triangle::fromX64(p0.clone(), p3.clone(), p2.clone(), n.clone()));
                } else {
                    self.triangles.push(Triangle::fromX64(p0.clone(), p3.clone(), p1.clone(), n.clone()));
                    self.triangles.push(Triangle::fromX64(p0.clone(), p2.clone(), p3.clone(), n.clone()));
                }
            }
        }
    }
    pub fn to_polygon_mesh(&self) -> PolygonMesh {
        let mut positions: Vec<Point3> = vec![];
        let mut tri_normals: Vec<Vector3> = vec![];
        let mut indx: Vec<[StandardVertex; 3]> = vec![];
        self.triangles.iter().for_each(|tri| {
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
    pub fn to_obj(&self) {
        let mut positions: Vec<Point3> = vec![];
        let mut indx: Vec<[StandardVertex; 3]> = vec![];
        self.triangles.iter().for_each(|tri| {
            positions.extend_from_slice(tri.as_p64().as_slice());
        });
        (0..positions.len()).chunks(3).into_iter().for_each(|ch| {
            let ii = ch.collect_vec();
            indx.push([ii[0].into(), ii[1].into(), ii[2].into()]);
        });

        let mut polymesh = PolygonMesh::new(
            StandardAttributes {
                positions: positions,
                ..Default::default()
            },
            Faces::from_tri_and_quad_faces(
                indx,
                Vec::new(),
            ),
        );
        let path = format!("d:\\pipe_project\\toro{}.obj", self.id);
        let mut obj_file = std::fs::File::create(path).unwrap();
        obj::write(&polymesh, obj_file).unwrap();
    }
}