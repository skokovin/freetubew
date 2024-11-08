use std::collections::{HashMap, HashSet};
use std::f64::consts::PI;
use std::fmt::{Display, Formatter};
use std::ops::{Mul, Sub};
use cgmath::{Basis3, Deg, InnerSpace, Matrix, Matrix3, MetricSpace, Rad, Rotation, Rotation3, SquareMatrix};
use cgmath::num_traits::{abs, signum};
use is_odd::IsOdd;
use itertools::Itertools;
use rand::random;
use truck_base::bounding_box::BoundingBox;
use truck_base::cgmath64::{Point3, Vector3};
use crate::algo::{project_point_to_vec, round_by_dec, BendToro, MainCircle, MainCylinder, EXTRA_LEN_CALC, EXTRA_R_CALC, P_FORWARD, P_FORWARD_REVERSE, P_RIGHT, P_UP, TOLE};
use crate::device::{MeshVertex, StepVertexBuffer};
use crate::device::graphics::{AnimState, BendParameters};

const L: i32 = 0;
const R: i32 = 1;
const A: i32 = 2;
const K: i32 = 3;
#[derive(Debug, Clone)]
pub struct LRACLR {
    pub id1: i32,
    pub id2: i32,
    pub l: f64,
    pub lt: f64,
    pub r: f64,
    pub a: f64,
    pub clr: f64,
    pub pipe_radius: f64,
}
impl LRACLR {
    pub fn default() -> Self {
        Self {
            id1: 0,
            id2: 0,
            l: 0.0,
            lt: 0.0,
            r: 0.0,
            a: 0.0,
            clr: 0.0,
            pipe_radius: 0.0,
        }
    }

    pub fn to_array(cmnd: &Vec<LRACLR>) -> Vec<i32> {
        let mut arr: Vec<i32> = vec![];
        cmnd.iter().for_each(|cmd| {
            let rounded0: i32 = (round_by_dec(cmd.l, 3) * 1000.0) as i32;
            let rounded1: i32 = (round_by_dec(cmd.lt, 3) * 1000.0) as i32;
            let rounded2: i32 = (round_by_dec(cmd.r, 3) * 1000.0) as i32;
            let rounded3: i32 = (round_by_dec(cmd.a, 3) * 1000.0) as i32;
            let rounded4: i32 = (round_by_dec(cmd.clr, 3) * 1000.0) as i32;
            let rounded5: i32 = (round_by_dec(cmd.pipe_radius, 3) * 1000.0) as i32;

            arr.push(cmd.id1);
            arr.push(cmd.id2);
            arr.push(rounded0);
            arr.push(rounded1);
            arr.push(rounded2);
            arr.push(rounded3);
            arr.push(rounded4);
            arr.push(rounded5);
        });
        arr
    }

    pub fn total_len_out_d(cmnd: &Vec<LRACLR>) -> (f64, f64) {
        let mut tl: f64 = 0.0;
        let mut outd: f64 = 0.0;
        cmnd.iter().for_each(|cmd| {
            tl = tl + cmd.l + cmd.lt;
            outd = cmd.pipe_radius;
        });
        (tl, outd)
    }
}

impl Display for LRACLR {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "L {} R {} A {} CLR {}", self.l, self.r, self.a, self.clr)
    }
}

#[derive(Clone)]
pub enum OpElem {
    CYL((MainCylinder)),
    TOR((BendToro)),
    Nothing,
}
#[derive(Clone)]
pub struct CncOps {
    pub ops: Vec<OpElem>,
    pub current_step: usize,

}
impl CncOps {
    pub fn new(cyls: &Vec<MainCylinder>, bends: &Vec<BendToro>) -> Self {
        let mut tot_ops: Vec<OpElem> = vec![];
        if (!cyls.is_empty() && !bends.is_empty()) {
            let mut tot_ops_cyls: Vec<OpElem> = vec![];
            let mut bends_db: HashMap<u64, BendToro> = HashMap::new();
            bends.iter().for_each(|b| {
                bends_db.insert(b.id, b.clone());
            });
            let mut cyls_path = cyls.clone();
            MainCylinder::init_tors(&mut cyls_path, bends);
            let mut cyls_path_cleared: Vec<MainCylinder> = vec![];
            cyls_path.iter().for_each(|c| {
                if (c.ca_tor != u64::MAX || c.cb_tor != u64::MAX) {
                    cyls_path_cleared.push(c.clone());
                }
            });
            if (cyls.len() == 2 && bends.len() == 1) {
                tot_ops_cyls.push(OpElem::CYL(cyls[0].clone()));
                tot_ops_cyls.push(OpElem::TOR(bends[0].clone()));
                tot_ops_cyls.push(OpElem::CYL(cyls[1].clone()));
            } else {
                match MainCylinder::find_ends(&cyls_path_cleared) {
                    None => {}
                    Some(ends) => {
                        let mut curr = ends[0].clone();
                        tot_ops_cyls.push(OpElem::CYL(ends[0].clone()));
                        let mut hashids: HashSet<u64> = HashSet::new();
                        hashids.insert(curr.id);
                        let mut inserted = true;
                        while (inserted) {
                            match curr.get_next(&hashids, &cyls_path_cleared) {
                                None => {
                                    inserted = false;
                                }
                                Some(next) => {
                                    //println!("NEXT curr {:?} next {:?} {:?} {:?}", curr.id, next.id, next.ca_tor, next.cb_tor);
                                    tot_ops_cyls.push(OpElem::CYL(next.clone()));
                                    hashids.insert(next.id);
                                    curr = next;
                                }
                            }
                        }
                    }
                }
            }
            tot_ops.push(tot_ops_cyls[0].clone());
            for i in 0..tot_ops_cyls.len() - 1 {
                let mut vec: Vec<u64> = vec![];
                let mut tor_indx: u64 = u64::MAX;
                match &tot_ops_cyls[i] {
                    OpElem::CYL(c) => {
                        vec.push(c.ca_tor);
                        vec.push(c.cb_tor);
                    }
                    OpElem::TOR(_) => {}
                    OpElem::Nothing => {}
                }
                match &tot_ops_cyls[i + 1] {
                    OpElem::CYL(c) => {
                        vec.push(c.ca_tor);
                        vec.push(c.cb_tor);
                    }
                    OpElem::TOR(_) => {}
                    OpElem::Nothing => {}
                }

                let mut hashids: HashMap<u64, i64> = HashMap::new();
                vec.iter().for_each(|a| {
                    match hashids.get_mut(a) {
                        None => {
                            if (*a != u64::MAX) {
                                hashids.insert(a.clone(), 1);
                            }
                        }
                        Some(v) => {
                            if (*a != u64::MAX) {
                                *v = *v + 1;
                            }
                        }
                    }
                });

                hashids.iter().for_each(|(k, v)| {
                    if (*v == 2) {
                        tor_indx = k.clone();
                    }
                });
                match bends_db.get(&tor_indx) {
                    None => {
                        tot_ops.push(tot_ops_cyls[i + 1].clone());
                    }
                    Some(t) => {
                        tot_ops.push(OpElem::TOR(t.clone()));
                        tot_ops.push(tot_ops_cyls[i + 1].clone());
                    }
                }
            }
            CncOps::fix_dirs(&mut tot_ops);
        }
        Self {
            ops: tot_ops,
            current_step: 0,
        }
    }
    fn fix_dirs(opers: &mut Vec<OpElem>) {
        let ops1 = opers[1].clone();
        let ops0 = &mut opers[0];
        match ops0 {
            OpElem::CYL(cyl) => {
                if (cyl.ca_tor != u64::MAX) {
                    cyl.reverse_my_ends();
                    match ops1 {
                        OpElem::CYL(_) => {}
                        OpElem::TOR(t) => {
                            if (!(cyl.cb.loc.distance(t.ca.loc).abs() < TOLE) && !(cyl.cb.loc.distance(t.cb.loc).abs() < TOLE)) {
                                cyl.reverse_my_points();
                            }
                        }
                        OpElem::Nothing => {}
                    }
                }
                cyl.recalculate_h();
            }
            OpElem::TOR(_) => {}
            OpElem::Nothing => {}
        }
        let mut lastop = opers[0].clone();
        for i in 1..opers.len() {
            let ops0 = &mut opers[i];
            match ops0 {
                OpElem::CYL(c_op) => {
                    match lastop {
                        OpElem::CYL(_) => {}
                        OpElem::TOR(t) => {
                            if (!(c_op.ca.loc.distance(t.cb.loc).abs() < TOLE)) {
                                c_op.reverse_my_points();
                            }
                            if (c_op.ca_tor != t.id) {
                                c_op.reverse_my_ends();
                            }
                        }
                        OpElem::Nothing => {}
                    }
                    c_op.recalculate_h();
                    lastop = ops0.clone();
                }
                OpElem::TOR(t) => {
                    match &lastop {
                        OpElem::CYL(cyl) => {
                            if (!(cyl.cb.loc.distance(t.ca.loc).abs() < TOLE)) {
                                t.reverse_my_points();
                            }
                            lastop = ops0.clone();
                        }
                        OpElem::TOR(_) => {}
                        OpElem::Nothing => {}
                    }
                }
                OpElem::Nothing => {}
            }
        }
    }
    pub fn calculate_extra_len(&self, extra_len_pts: &Vec<Point3>) -> CncOps {
        let mut new_ops = self.ops.clone();
        let last_index = new_ops.len() - 1;
        let first = self.ops.first().unwrap();
        let last = self.ops.last().unwrap();
        let mut new_first: Option<MainCylinder> = None;
        let mut new_last: Option<MainCylinder> = None;
        match first {
            OpElem::CYL(c) => {
                let mut new_p: Point3 = Point3::new(0.0, 0.0, 0.0);
                let mut curr_d = 0.0;
                let dir = c.ca.loc.sub(c.cb.loc).normalize();
                let find_r = c.r * EXTRA_R_CALC;
                let d = c.r * EXTRA_LEN_CALC;
                let sp = c.ca.loc.clone();
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
                    let mut new_c = c.clone();
                    new_c.ca.loc = new_p;
                    let h = new_c.ca.loc.distance(new_c.cb.loc);
                    new_c.h = h;
                    new_first = Some(new_c);
                }
            }
            OpElem::TOR(_) => {}
            OpElem::Nothing => {}
        }
        match last {
            OpElem::CYL(c) => {
                let mut new_p: Point3 = Point3::new(0.0, 0.0, 0.0);
                let mut curr_d = 0.0;
                let dir = c.cb.loc.sub(c.ca.loc).normalize();
                let find_r = c.r * EXTRA_R_CALC;
                let d = c.r * EXTRA_LEN_CALC;
                let sp = c.cb.loc.clone();
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
                    let mut new_c = c.clone();
                    new_c.cb.loc = new_p;
                    let h = new_c.ca.loc.distance(new_c.cb.loc);
                    //new_c.cb.to_pts_file();
                    new_c.h = h;
                    new_last = Some(new_c);
                }
            }
            OpElem::TOR(_) => {}
            OpElem::Nothing => {}
        }
        match new_first {
            None => {}
            Some(mc) => {
                new_ops[0] = OpElem::CYL(mc);
            }
        }
        match new_last {
            None => {}
            Some(mc) => {
                new_ops[last_index] = OpElem::CYL(mc);
            }
        }
        let mut ret = CncOps {
            ops: new_ops,
            current_step: 0,
        };
        ret
    }
    pub fn calculate_lraclr(&mut self) -> Vec<LRACLR> {
        let mut cncs: Vec<LRACLR> = vec![];
        if (self.ops.len() > 1 && self.ops.len().is_odd()) {
            let last_op: OpElem = self.ops.last().unwrap().clone();
            let mut pair_arr: Vec<OpElem> = vec![];
            for i in 0..self.ops.len() - 1 {
                pair_arr.push(self.ops[i].clone());
            }
            let mut it_first = true;
            let mut outd: f64 = 0.0;
            let mut counter = 0;
            let mut prev: Vector3 = P_UP;
            let mut prev_fwd: Vector3 = P_FORWARD;
            pair_arr.chunks(2).for_each(|ops| {
                let op1 = &ops[0];
                let op2 = &ops[1];
                let mut cnc: LRACLR = LRACLR::default();
                match op1 {
                    OpElem::CYL(c) => {
                        cnc.id1 = counter;
                        counter = counter + 1;
                        cnc.l = c.h;
                        outd = c.r;
                        cnc.pipe_radius = outd;

                        match op2 {
                            OpElem::TOR(t) => {
                                let r: Rad<f64> = {
                                    if (it_first) {
                                        prev = t.bend_plane_norm;
                                        prev_fwd = t.ca.dir;
                                        it_first = false;
                                        Rad(0.0)
                                    } else {
                                        let a =
                                            {
                                                //https://stackoverflow.com/questions/14066933/direct-way-of-computing-the-clockwise-angle-between-two-vectors
                                                //Plane embedded in 3D
                                                let dot = prev.dot(t.bend_plane_norm);
                                                //let det = prev.x*t.bend_plane_norm.y*t.ca.dir.z + t.bend_plane_norm.x*t.ca.dir.y*prev.z + t.ca.dir.x*prev.y*t.bend_plane_norm.z - prev.z*t.bend_plane_norm.y*t.ca.dir.x - t.bend_plane_norm.z*t.ca.dir.y*prev.x - t.ca.dir.z*prev.y*t.bend_plane_norm.x;
                                                let det = Matrix3::from_cols(t.bend_plane_norm, t.ca.dir, prev).determinant();
                                                let angle = det.atan2(dot);
                                                let d = prev.dot(t.bend_plane_norm);
                                                let d_frwd = prev_fwd.dot(t.cb.dir);
                                                //warn!("dot {:?}  {:?} {:?}",Deg::from(Rad(angle)),Deg::from(prev.angle(t.bend_plane_norm)),d_frwd);
                                                if (d == -1.0 && d_frwd < 0.0) {
                                                    Rad(0.0)
                                                } else {
                                                    Rad(angle)
                                                }
                                            };

                                        //warn!("a {:?} {:?}",Deg::from(a), prev.angle(t.bend_plane_norm));
                                        prev = t.bend_plane_norm;
                                        prev_fwd = t.ca.dir;
                                        a
                                    }
                                };
                                cnc.r = Deg::from(r).0;
                                cnc.clr = t.bend_radius;
                                cnc.id2 = counter;
                                counter = counter + 1;
                                let ba = t.ca.loc.sub(t.bend_center_point);
                                let bb = t.cb.loc.sub(t.bend_center_point);
                                let mut bend_angle = ba.angle(bb);
                                cnc.lt = abs(bend_angle.0) * (t.bend_radius);
                                cnc.a = Deg::from(bend_angle).0;
                            }
                            _ => {}
                        }

                        cnc.pipe_radius = outd;
                        cncs.push(cnc);
                    }
                    _ => {}
                }
            });
            match last_op {
                OpElem::CYL(c) => {
                    let mut cnc: LRACLR = LRACLR::default();
                    cnc.id1 = counter;
                    cnc.id2 = counter + 1;
                    cnc.l = c.h;
                    cnc.pipe_radius = outd;
                    cncs.push(cnc);
                }
                OpElem::TOR(_) => {}
                OpElem::Nothing => {}
            }
        }

        optimize_lraclr(&mut cncs);
        cncs
    }
    pub fn generate_cyl(h: f64, radius: f64) -> MainCylinder {
        let mut mc: MainCylinder = MainCylinder {
            id: 0,
            ca: MainCircle {
                id: random(),
                radius: radius,
                loc: Point3::new(0.0, 0.0, 0.0),
                dir: P_FORWARD,
                radius_dir: P_UP,
            },
            cb: MainCircle {
                id: random(),
                radius: radius,
                loc: Point3::new(h, 0.0, 0.0),
                dir: P_FORWARD,
                radius_dir: P_UP,
            },
            h: h,
            r: radius,
            r_gr_id: 0,
            ca_tor: 0,
            cb_tor: 0,
            step_vertex_buffer: StepVertexBuffer::default(),
            bbx: BoundingBox::default(),
        };
        mc.triangulate();
        mc
    }
}

fn generate_dummy_cyl(id: u64, stright_len: f64, radius: f64) -> MainCylinder {
    generate_cyl_by_2pts(
        id,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(-stright_len, 0.0, 0.0),
        radius,
        P_FORWARD_REVERSE,
        P_UP,
    )
}
fn generate_cyl_by_2pts(id: u64, sp: Point3, ep: Point3, radius: f64, fwd_dir: Vector3, up_dir: Vector3) -> MainCylinder {
    let h: f64 = sp.distance(ep);
    let mut mc: MainCylinder = MainCylinder {
        id: id,
        ca: MainCircle {
            id: random(),
            radius: radius,
            loc: sp,
            dir: fwd_dir,
            radius_dir: up_dir,
        },
        cb: MainCircle {
            id: random(),
            radius: radius,
            loc: ep,
            dir: fwd_dir,
            radius_dir: up_dir,
        },
        h: h,
        r: radius,
        r_gr_id: 0,
        ca_tor: 0,
        cb_tor: 0,
        step_vertex_buffer: StepVertexBuffer::default(),
        bbx: BoundingBox::default(),
    };
    mc.triangulate();
    mc
}

fn tot_pipe_len(lraclr_arr: &Vec<LRACLR>) -> f64 {
    let mut ret = 0.0;
    lraclr_arr.iter().for_each(|lracl| {
        ret = ret + lracl.l + lracl.lt;
    });
    ret
}

pub fn reverse_lraclr(lraclr: &Vec<LRACLR>) -> Vec<LRACLR> {
    let mut ret: Vec<LRACLR> = vec![];
    let mut ops: Vec<LRACLR> = lraclr.clone();
    ops.reverse();
    let mut is_first = false;
    let mut last_l: f64 = 0.0;
    let mut last_r: f64 = 0.0;

    let mut counter = 0;
    ops.iter().for_each(|op| {
        if (!is_first) {
            last_l = op.l;
            //last_r = op.r;
            is_first = true;
        } else {
            let new_l = op.l;
            let new_r = op.r;
            let id1 = counter;
            counter = counter + 1;
            let id2 = counter;
            counter = counter + 1;
            let mut new_lraclr = op.clone();
            new_lraclr.l = last_l;
            new_lraclr.r =
                if (last_r == 0.0) {
                    0.0
                } else {
                    -(360.0 - last_r)
                };

            new_lraclr.id1 = id1;
            new_lraclr.id2 = id2;
            last_l = new_l;
            last_r = new_r;
            ret.push(new_lraclr);
        }
    });
    let id1 = counter;
    counter = counter + 1;
    let id2 = counter;
    let last: LRACLR = LRACLR {
        id1: id1,
        id2: id2,
        l: ops.last().unwrap().l,
        lt: 0.0,
        r: 0.0,
        a: 0.0,
        clr: 0.0,
        pipe_radius: ops.last().unwrap().pipe_radius,
    };
    ret.push(last);
    optimize_lraclr(&mut ret);
    ret
}

pub fn cnc_to_poly_v(lraclr_arr: &Vec<LRACLR>, anim_state: &AnimState, v_up_orign: &Vector3, dt: f64, bend_params: &BendParameters) -> (Vec<MainCylinder>, Vec<BendToro>, AnimState) {
    let mut out_cyls: Vec<MainCylinder> = vec![];
    let mut out_tors: Vec<BendToro> = vec![];

    let incr_l = bend_params.stright_speed * dt;
    let incr_r = bend_params.rotate_speed * dt;
    let incr_a = bend_params.angle_speed * dt;
    //let totlen = tot_pipe_len(&lraclr_arr);

    let last_index = lraclr_arr.len() - 1;
    let id = anim_state.id;
    let stright_len: f64 = {
        if (id == 0 && anim_state.stright_len == 0.0) {
            tot_pipe_len(&lraclr_arr)
        } else {
            anim_state.stright_len
        }
    };


    let mut anim_lra: Vec<LRACLR> = vec![];
    if (!id.is_odd()) {
        let indx = (id / 2) as usize;
        if (indx > last_index) {
            (out_cyls, out_tors, AnimState::new(0, 4, 0.0, 0.0, LRACLR::default()))
        } else {
            let curr: &LRACLR = &lraclr_arr[indx as usize];
            let curr_l = curr.l;
            let curr_r = curr.r;
            //warn!("curr_l {:?} curr_r {:?}",curr_l,curr_r);
            match anim_state.opcode {
                0 => {
                    let next_val = anim_state.value + incr_l;
                    if (next_val >= curr_l) {
                        let next_stage: AnimState = AnimState::new(id, 1, 0.0, stright_len - (next_val - curr_l), curr.clone());
                        lraclr_arr.iter().take(indx + 1).for_each(|lr| {
                            anim_lra.push(lr.clone());
                        });
                        if (anim_lra.len() == 1) {
                            let cyl = CncOps::generate_cyl(anim_lra[0].l, anim_lra[0].pipe_radius);
                            out_cyls.push(cyl);
                        } else {
                            let reversed: Vec<LRACLR> = reverse_lraclr(&anim_lra);
                            let (cyls, tors): (Vec<MainCylinder>, Vec<BendToro>) = cnc_to_poly(&reversed, &v_up_orign);
                            out_cyls = cyls;
                            out_tors = tors;
                        }
                        //warn!("last len stage id {:?}  {:?} of {:?}",id, next_stage,curr_l);
                        if (next_stage.stright_len > 0.0) {
                            out_cyls.push(generate_dummy_cyl(out_cyls.last().unwrap().id + 1, next_stage.stright_len, curr.pipe_radius));
                        }
                        (out_cyls, out_tors, next_stage)
                    } else {
                        let mut anim_lra: Vec<LRACLR> = vec![];
                        let next_stage: AnimState = AnimState::new(id, 0, next_val, stright_len - incr_l, curr.clone());
                        lraclr_arr.iter().take(indx + 1).for_each(|lr| {
                            anim_lra.push(lr.clone());
                        });
                        anim_lra[indx].l = next_stage.value;


                        if (anim_lra.len() == 1) {
                            let cyl = CncOps::generate_cyl(anim_lra[0].l, anim_lra[0].pipe_radius);
                            out_cyls.push(cyl);
                        } else {
                            let reversed: Vec<LRACLR> = reverse_lraclr(&anim_lra);
                            let (cyls, tors): (Vec<MainCylinder>, Vec<BendToro>) = cnc_to_poly(&reversed, &v_up_orign);
                            out_cyls = cyls;
                            out_tors = tors;
                        }
                        //warn!("len stage id {:?}  {:?} of {:?}",id,next_stage,curr_l);
                        if (next_stage.stright_len > 0.0) {
                            out_cyls.push(generate_dummy_cyl(out_cyls.last().unwrap().id + 1, next_stage.stright_len, curr.pipe_radius));
                        }
                        (out_cyls, out_tors, next_stage)
                    }
                }
                1 => {
                    let next_val = anim_state.value + incr_r * signum(curr_r);
                    if (abs(next_val) >= abs(curr_r)) {
                        let next_stage: AnimState = AnimState::new(id + 1, 2, 0.0, stright_len, curr.clone());
                        lraclr_arr.iter().take(indx + 1).for_each(|lr| {
                            anim_lra.push(lr.clone());
                        });


                        let mut reversed: Vec<LRACLR> = reverse_lraclr(&anim_lra);
                        reversed[0].r = curr_r;
                        let (cyls, tors): (Vec<MainCylinder>, Vec<BendToro>) = cnc_to_poly(&reversed, &v_up_orign);
                        out_cyls = cyls;
                        out_tors = tors;
                        //warn!("last rot stage id {:?}  {:?} of {:?}",id,next_stage,curr_r);
                        if (next_stage.stright_len > 0.0) {
                            out_cyls.push(generate_dummy_cyl(out_cyls.last().unwrap().id + 1, next_stage.stright_len, curr.pipe_radius));
                        }
                        (out_cyls, out_tors, next_stage)
                    } else {
                        let next_stage: AnimState = AnimState::new(id, 1, next_val, stright_len, curr.clone());
                        lraclr_arr.iter().take(indx + 1).for_each(|lr| {
                            anim_lra.push(lr.clone());
                        });

                        let mut reversed: Vec<LRACLR> = reverse_lraclr(&anim_lra);
                        reversed[0].r = next_stage.value;
                        let (cyls, tors): (Vec<MainCylinder>, Vec<BendToro>) = cnc_to_poly(&reversed, &v_up_orign);
                        out_cyls = cyls;
                        out_tors = tors;

                        //warn!("rot stage id {:?}  {:?} of {:?}",id,next_stage,curr_r);
                        if (next_stage.stright_len > 0.0) {
                            out_cyls.push(generate_dummy_cyl(out_cyls.last().unwrap().id + 1, next_stage.stright_len, curr.pipe_radius));
                        }
                        (out_cyls, out_tors, next_stage)
                    }
                }
                _ => {
                    (out_cyls, out_tors, AnimState::new(anim_state.id, anim_state.opcode, anim_state.value, anim_state.stright_len, curr.clone()))
                }
            }
        }
    } else {
        let indx = ((id - 1) / 2) as usize;

        let curr = &lraclr_arr[indx];
        let curr_a = curr.a;
        let next_val = anim_state.value + incr_a;

        if (next_val >= curr_a) {
            let len_angle = Rad::from(Deg(next_val - curr_a)).0 * curr.clr;
            let next_stage: AnimState = AnimState::new(id + 1, 0, 0.0, stright_len - len_angle, curr.clone());
            //warn!("last bend stage id {:?}  {:?} of {:?}",id,next_stage,curr_a);
            lraclr_arr.iter().take(indx + 1).for_each(|lr| {
                anim_lra.push(lr.clone());
            });
            let mut dumb: LRACLR = lraclr_arr.last().unwrap().clone();
            dumb.l = 0.5;
            anim_lra.push(dumb);

            let reversed: Vec<LRACLR> = reverse_lraclr(&anim_lra);

            let (cyls, tors): (Vec<MainCylinder>, Vec<BendToro>) = cnc_to_poly(&reversed, &v_up_orign);
            out_cyls = cyls;
            out_tors = tors;

            if (next_stage.stright_len > 0.0) {
                out_cyls.push(generate_dummy_cyl(out_cyls.last().unwrap().id + 1, next_stage.stright_len, curr.pipe_radius));
            }

            (out_cyls, out_tors, next_stage)
        } else {
            let len_angle = Rad::from(Deg(incr_a)).0 * curr.clr;
            let next_stage: AnimState = AnimState::new(id, 2, next_val, stright_len - len_angle, curr.clone());
            //warn!("len_angle {:?} incr_a {:?} ",len_angle,incr_a);

            lraclr_arr.iter().take(indx + 1).for_each(|lr| {
                anim_lra.push(lr.clone());
            });


            let mut dumb: LRACLR = lraclr_arr.last().unwrap().clone();
            dumb.l = 0.5;
            anim_lra.push(dumb);
            let mut reversed: Vec<LRACLR> = reverse_lraclr(&anim_lra);
            reversed[0].a = next_stage.value;

            let (cyls, tors): (Vec<MainCylinder>, Vec<BendToro>) = cnc_to_poly(&reversed, &v_up_orign);
            out_cyls = cyls;
            out_tors = tors;
            if (next_stage.stright_len > 0.0) {
                out_cyls.push(generate_dummy_cyl(out_cyls.last().unwrap().id + 1, next_stage.stright_len, curr.pipe_radius));
            }
            (out_cyls, out_tors, next_stage)
        }
    }
}

pub fn cnc_to_poly(lraclr_arr: &Vec<LRACLR>, v_up_orign: &Vector3) -> (Vec<MainCylinder>, Vec<BendToro>) {
    let mut current_step = 0;
    let mut sp: Point3 = Point3::new(0.0, 0.0, 0.0);
    let mut v_up: Vector3 = v_up_orign.clone();
    let mut v_frw = P_FORWARD;
    let mut v_right = P_RIGHT;
    let mut cyls: Vec<MainCylinder> = vec![];
    let mut tors: Vec<BendToro> = vec![];
    lraclr_arr.iter().for_each(|lracl| {
        let pipe_r = lracl.pipe_radius;
        let bend_r = lracl.clr;
        let frw_move_dist = lracl.l;
        let ep = sp + v_frw.mul(frw_move_dist);
        let cyl = generate_cyl_by_2pts(current_step, sp, ep, pipe_r, v_frw, v_up);
        current_step = current_step + 1;
        //cyl.to_obj();
        cyls.push(cyl);
        //CYL FINISH
        sp = ep;
        if (lracl.clr > 0.0) {
            let x_rotation = Rad::from(Deg(lracl.r));
            let bend_angle = Rad::from(Deg(lracl.a));
            let bend_angle_half = bend_angle / 2.0;
            let rotation_x: Basis3<f64> = Rotation3::from_axis_angle(v_frw, x_rotation);
            v_right = rotation_x.rotate_vector(v_right);
            v_up = rotation_x.rotate_vector(v_up);

            let rotation_bend: Basis3<f64> = Rotation3::from_axis_angle(v_up, bend_angle);
            let v_frw_s = v_frw.clone();
            v_frw = rotation_bend.rotate_vector(v_frw);
            let dist_x = {
                let p0 = Point3::new(0.0, 0.0, 0.0) + v_right.mul(bend_r);
                let v_rotated_right = rotation_bend.rotate_vector(v_right);
                let p1 = Point3::new(0.0, 0.0, 0.0) + v_rotated_right.mul(bend_r);
                let dist_x = p0.distance(p1);
                //warn!("dist_x {:?} {:?}",dist_x,bend_r);
                dist_x
            };
            let rotation_x_half: Basis3<f64> = Rotation3::from_axis_angle(v_up, bend_angle_half);
            let v_frw_half = rotation_x_half.rotate_vector(v_frw_s);
            let ep = sp + v_frw_half.mul(dist_x);
            let tor: BendToro = generate_tor_by_2pts(current_step, sp, ep, pipe_r, v_frw_s, v_frw, v_up, bend_r);
            current_step = current_step + 1;
            sp = ep;
            //tor.to_obj();
            tors.push(tor);
        }
    });
    //all_to_one_obj_file(&cyls, &tors);
    //let fp = &cyls.first().unwrap().ca.loc;
    //let lp = &cyls.last().unwrap().cb.loc;
    //warn!("CHECK DIST 530 {:?}",fp.distance(*lp));
    (cyls, tors)
}

fn generate_tor_by_2pts(id: u64, sp: Point3, ep: Point3, radius: f64, fwd_dir_s: Vector3, fwd_dir_e: Vector3, up_dir: Vector3, bend_r: f64) -> BendToro {
    let bend_center_point = sp + up_dir.cross(fwd_dir_s).mul(bend_r);


    let mut tor: BendToro = BendToro {
        id: id,
        r: radius,
        bend_radius: bend_r,
        bend_center_point: bend_center_point,
        bend_plane_norm: up_dir,
        radius_dir: up_dir,
        ca: MainCircle {
            id: random(),
            radius: radius,
            loc: sp,
            dir: fwd_dir_s,
            radius_dir: up_dir,
        },
        cb: MainCircle {
            id: random(),
            radius: radius,
            loc: ep,
            dir: fwd_dir_e,
            radius_dir: up_dir,
        },
        r_gr_id: 0,
        step_vertex_buffer: StepVertexBuffer::default(),
        bbx: BoundingBox::default(),
    };
    tor.triangulate(&fwd_dir_s);
    tor
}

pub fn optimize_lraclr(lraclrs: &mut Vec<LRACLR>) {
    lraclrs.iter_mut().for_each(|lracl| {
        if (abs(lracl.r) >= 360.0) {
            let rounds = abs(lracl.r as i64 / 360);
            lracl.r = (abs(lracl.r) - 360.0 * rounds as f64) * signum(lracl.r);
        }
        if (abs(lracl.r) > 180.0) {
            lracl.r = -((360.0 - abs(lracl.r)) * signum(lracl.r));
        }
    });
}



