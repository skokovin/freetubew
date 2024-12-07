use crate::algo::{export_to_pt_str, perpendicular_rand_dir, project_point_to_vec, round_by_dec, BendToro, MainCircle, MainCylinder, DIVIDER, EXTRA_LEN_CALC, EXTRA_R_CALC, P_FORWARD, P_FORWARD_REVERSE, P_RIGHT, P_UP, ROT_DIR_CCW, TOLE};
use crate::device::graphics::{AnimState, BendParameters};
use crate::device::{MeshVertex, StepVertexBuffer};
use cgmath::num_traits::{abs, signum};
use cgmath::{
    Basis3, Deg, InnerSpace, Matrix, Matrix3, MetricSpace, Rad, Rotation, Rotation3, SquareMatrix,
};
use is_odd::IsOdd;
use itertools::Itertools;
use rand::{random, Rng};
use std::collections::{HashMap, HashSet};
use std::f64::consts::PI;
use std::fmt::{Display, Formatter};
use std::ops::{Mul, Sub};
use log::warn;
use truck_base::bounding_box::BoundingBox;
use truck_base::cgmath64::{Point3, Vector3};
use truck_stepio::out;
use web_sys::console::warn;

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

fn generate_cyl(id:u64,h: f64, radius: f64) -> MainCylinder {
    let mut mc: MainCylinder = MainCylinder {
        id: id,
        ca: MainCircle {
            id: random(),
            radius: radius,
            loc: Point3::new(0.0, 0.0, 0.0),
            dir: P_FORWARD,
            radius_dir: P_UP,
            r_gr_id: (round_by_dec(radius, 5) * DIVIDER) as u64,
        },
        cb: MainCircle {
            id: random(),
            radius: radius,
            loc: Point3::new(h, 0.0, 0.0),
            dir: P_FORWARD,
            radius_dir: P_UP,
            r_gr_id: (round_by_dec(radius, 5) * DIVIDER) as u64,
        },
        h: h,
        r: radius,
        r_gr_id: (round_by_dec(radius, 5) * DIVIDER) as u64,
        ca_tor: u64::MAX,
        cb_tor: u64::MAX,
        step_vertex_buffer: StepVertexBuffer::default(),
        bbx: BoundingBox::default(),
    };
    mc.triangulate();
    mc
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
            r_gr_id: (round_by_dec(radius, 5) * DIVIDER) as u64,
        },
        cb: MainCircle {
            id: random(),
            radius: radius,
            loc: ep,
            dir: fwd_dir,
            radius_dir: up_dir,
            r_gr_id: (round_by_dec(radius, 5) * DIVIDER) as u64,
        },
        h: h,
        r: radius,
        r_gr_id: (round_by_dec(radius, 5) * DIVIDER) as u64,
        ca_tor: u64::MAX,
        cb_tor: u64::MAX,
        step_vertex_buffer: StepVertexBuffer::default(),
        bbx: BoundingBox::default(),
    };
    mc.triangulate();
    mc
}

pub fn gen_cyl(sp: Point3, ep: Point3, radius: f64) -> MainCylinder {
    let dir=ep.sub(sp).normalize();
    let radius_dir=perpendicular_rand_dir(&dir).normalize();
    generate_cyl_by_2pts(random(),sp, ep,radius,dir,radius_dir)
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
            new_lraclr.r = if (last_r == 0.0) {
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
pub fn cnc_to_poly(lraclr_arr: &Vec<LRACLR>, v_up_orign: &Vector3, ) -> (Vec<MainCylinder>, Vec<BendToro>) {
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
            let x_rotation = Rad::from(Deg(lracl.r * ROT_DIR_CCW));
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
    all_to_one(&cyls, &tors);
    (cyls, tors)
}
fn generate_tor_by_2pts(id: u64, sp: Point3, ep: Point3, radius: f64, fwd_dir_s: Vector3, fwd_dir_e: Vector3, up_dir: Vector3, bend_r: f64, ) -> BendToro
{
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
            r_gr_id: (round_by_dec(radius, 5) * DIVIDER) as u64,
        },
        cb: MainCircle {
            id: random(),
            radius: radius,
            loc: ep,
            dir: fwd_dir_e,
            radius_dir: up_dir,
            r_gr_id: (round_by_dec(radius, 5) * DIVIDER) as u64,
        },
        r_gr_id: (round_by_dec(radius, 5) * DIVIDER) as u64,
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
pub fn cnc_to_poly_animate(lraclr_arr: &Vec<LRACLR>, anim_state: &AnimState, v_up_orign: &Vector3, dt: f64, bend_params: &BendParameters, ) -> (Vec<MainCylinder>, Vec<BendToro>, AnimState) {
    let mut out_cyls: Vec<MainCylinder> = vec![];
    let mut out_tors: Vec<BendToro> = vec![];

    let incr_l = bend_params.stright_speed * dt;
    let incr_r = bend_params.rotate_speed * dt;
    let incr_a = bend_params.angle_speed * dt;
    let last_index = lraclr_arr.len() - 1;
    let id = anim_state.id;
    let op_counter = anim_state.op_counter;
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
        if (indx > last_index) { (out_cyls, out_tors, AnimState::new(0, 4, 0.0, 0.0, LRACLR::default(), op_counter),) }
        else {
            let curr: &LRACLR = &lraclr_arr[indx as usize];
            let curr_l = curr.l;
            let curr_r = curr.r;
            //warn!("curr_l {:?} curr_r {:?}",curr_l,curr_r);
            match anim_state.opcode {
                0 => {
                    let next_val = anim_state.value + incr_l;
                    if (next_val >= curr_l) {
                        let mut next_stage: AnimState = AnimState::new(id, 1, 0.0, stright_len - (next_val - curr_l), curr.clone(), op_counter + 1, );
                        lraclr_arr.iter().take(indx + 1).for_each(|lr| { anim_lra.push(lr.clone()); });
                        if (anim_lra.len() == 1) {
                            let cyl = generate_cyl(0,anim_lra[0].l, anim_lra[0].pipe_radius);
                            out_cyls.push(cyl);
                        } else {
                            let reversed: Vec<LRACLR> = reverse_lraclr(&anim_lra);
                            let (cyls, tors): (Vec<MainCylinder>, Vec<BendToro>) = cnc_to_poly(&reversed, &v_up_orign);
                            out_cyls = cyls;
                            out_tors = tors;
                        }
                        //warn!("last len stage id {:?}  {:?} of {:?}",id, next_stage,curr_l);
                     if (next_stage.stright_len > 0.0) {
                            out_cyls.push(generate_dummy_cyl(
                                out_cyls.last().unwrap().id + 1,
                                next_stage.stright_len,
                                curr.pipe_radius,
                            ));
                        }
                        (out_cyls, out_tors, next_stage)
                    }
                    else {
                        let mut anim_lra: Vec<LRACLR> = vec![];
                        let next_stage: AnimState = AnimState::new(id, 0, next_val, stright_len - incr_l, curr.clone(), op_counter, );
                        lraclr_arr.iter().take(indx + 1).for_each(|lr| { anim_lra.push(lr.clone()); });
                        anim_lra[indx].l = next_stage.value;

                        if (anim_lra.len() == 1) {
                            let cyl = generate_cyl(0,anim_lra[0].l, anim_lra[0].pipe_radius);
                            out_cyls.push(cyl);
                        } else {
                            let reversed: Vec<LRACLR> = reverse_lraclr(&anim_lra);
                            let (cyls, tors): (Vec<MainCylinder>, Vec<BendToro>) = cnc_to_poly(&reversed, &v_up_orign);
                            out_cyls = cyls;
                            out_tors = tors;
                        }
                        //warn!("len stage id {:?}  {:?} of {:?}",id,next_stage,curr_l);
                        if (next_stage.stright_len > 0.0) {
                            out_cyls.push(generate_dummy_cyl(
                                out_cyls.last().unwrap().id + 1,
                                next_stage.stright_len,
                                curr.pipe_radius,
                            ));
                        }
                        (out_cyls, out_tors, next_stage)
                    }
                }
                1 => {
                    let next_val = anim_state.value + incr_r * signum(curr_r);
                    if (abs(next_val) >= abs(curr_r)) {
                        let mut next_stage: AnimState = AnimState::new(
                            id + 1,
                            2,
                            0.0,
                            stright_len,
                            curr.clone(),
                            op_counter + 1,
                        );
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
                            out_cyls.push(generate_dummy_cyl(
                                out_cyls.last().unwrap().id + 1,
                                next_stage.stright_len,
                                curr.pipe_radius,
                            ));
                        }
                        (out_cyls, out_tors, next_stage)
                    } else {
                        let next_stage: AnimState = AnimState::new(id, 1, next_val, stright_len, curr.clone(), op_counter);
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
                            out_cyls.push(generate_dummy_cyl(
                                out_cyls.last().unwrap().id + 1,
                                next_stage.stright_len,
                                curr.pipe_radius,
                            ));
                        }
                        (out_cyls, out_tors, next_stage)
                    }
                }
                _ => (
                    out_cyls,
                    out_tors,
                    AnimState::new(
                        anim_state.id,
                        anim_state.opcode,
                        anim_state.value,
                        anim_state.stright_len,
                        curr.clone(),
                        op_counter,
                    ),
                ),
            }
        }
    }

    else {
        let indx = ((id - 1) / 2) as usize;

        let curr = &lraclr_arr[indx];
        let curr_a = curr.a;
        let next_val = anim_state.value + incr_a;

        if (next_val >= curr_a) {
            let len_angle = Rad::from(Deg(next_val - curr_a)).0 * curr.clr;
            let mut next_stage: AnimState = AnimState::new(
                id + 1,
                0,
                0.0,
                stright_len - len_angle,
                curr.clone(),
                op_counter + 1,
            );
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
                out_cyls.push(generate_dummy_cyl(
                    out_cyls.last().unwrap().id + 1,
                    next_stage.stright_len,
                    curr.pipe_radius,
                ));
            }

            (out_cyls, out_tors, next_stage)
        } else {
            let len_angle = Rad::from(Deg(incr_a)).0 * curr.clr;
            let next_stage: AnimState = AnimState::new(
                id,
                2,
                next_val,
                stright_len - len_angle,
                curr.clone(),
                op_counter,
            );
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
                out_cyls.push(generate_dummy_cyl(
                    out_cyls.last().unwrap().id + 1,
                    next_stage.stright_len,
                    curr.pipe_radius,
                ));
            }
            (out_cyls, out_tors, next_stage)
        }
    }
}



pub fn all_to_one(c: &Vec<MainCylinder>, t: &Vec<BendToro>) -> (Vec<MeshVertex>, Vec<i32>) {
    let mut v: Vec<MeshVertex> = vec![];
    c.iter().for_each(|c| {
        v.extend_from_slice(c.step_vertex_buffer.buffer.as_slice());
    });
    t.iter().for_each(|t| {
        v.extend_from_slice(t.step_vertex_buffer.buffer.as_slice());
    });
    let i: Vec<i32> = (0..v.len() as i32).collect();
    (v, i)
}

pub fn all_to_stp(cyls: &Vec<MainCylinder>, tors: &Vec<BendToro>) -> Vec<u8> {
    use truck_modeling::*;
    let mut shells: Vec<truck_topology::Shell<Point3, Curve, Surface>> = vec![];
    tors.iter().for_each(|t| {
        let p = t.ca.loc + t.ca.radius_dir * t.r;
        let vertex1 = builder::vertex(p);
        let circle = builder::rsweep(&vertex1, t.ca.loc, t.ca.dir, Rad(7.0));
        let disk = builder::try_attach_plane(&[circle]).unwrap();
        let solid: truck_topology::Solid<Point3, Curve, Surface> = builder::rsweep(&disk, t.bend_center_point, t.bend_plane_norm, t.angle());
        let shells_loc: Vec<truck_topology::Shell<Point3, Curve, Surface>> = solid.into_boundaries();
        shells.extend(shells_loc);
    });
    cyls.iter().for_each(|c| {
        let p = c.ca.loc + c.ca.radius_dir * c.r;
        let vertex1 = builder::vertex(p);
        let circle = builder::rsweep(&vertex1, c.ca.loc, c.ca.dir, Rad(7.0));
        let disk = builder::try_attach_plane(&[circle]).unwrap();
        let v = c.ca.dir.mul(c.h);
        let solid = builder::tsweep(&disk, v);
        let shells_loc: Vec<truck_topology::Shell<Point3, Curve, Surface>> = solid.into_boundaries();
        shells.extend(shells_loc);
    });
    let solid = Solid::new(shells);
    let step_string = out::CompleteStepDisplay::new(
        out::StepModel::from(&solid.compress()),
        out::StepHeaderDescriptor {
            organization_system: "shape-to-step".to_owned(),
            ..Default::default()
        },
    ).to_string();
    let mut step_file: Vec<u8> = Vec::new();
    std::io::Write::write_all(&mut step_file, step_string.as_ref()).unwrap();
    let _ = ruststep::parser::parse(&step_string).unwrap();
    step_file
}
