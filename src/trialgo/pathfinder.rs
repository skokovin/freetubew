use std::collections::{HashMap, HashSet};
use std::f64::consts::PI;
use std::fmt::{Display, Formatter};
use std::ops::{Mul, Sub};
use cgmath::{Deg, InnerSpace, MetricSpace, Rad};
use is_odd::IsOdd;
use itertools::Itertools;
use rand::random;
use truck_base::bounding_box::BoundingBox;
use truck_base::cgmath64::{Point3, Vector3};
use truck_polymesh::{Faces, obj, PolygonMesh, StandardAttributes, StandardVertex};

use crate::trialgo::bendpipe::{BendToro, MainCircle, MainCylinder};
use crate::trialgo::analyzepl::{EXTRA_LEN_CALC, EXTRA_R_CALC, TOLE};
use crate::trialgo::{export_to_pt, project_point_to_vec, round_by_dec};
use truck_geometry::prelude::Plane;
use crate::device::{MeshVertex, RawMesh, Triangle, Z_FIGHTING_FACTOR};

const L: i32 = 0;
const R: i32 = 1;
const A: i32 = 2;
const K: i32 = 3;
#[derive(Debug)]
pub struct LRACLR {
    pub id1:i32,
    pub id2:i32,
    pub l: f64,
    pub lt: f64,
    pub r: f64,
    pub a: f64,
    pub clr: f64,
    pub outd: f64,
}
impl LRACLR {
    pub fn default() -> Self {
        Self {
            id1:0,
            id2:0,
            l: 0.0,
            lt: 0.0,
            r: 0.0,
            a: 0.0,
            clr: 0.0,
            outd: 0.0,
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
            let rounded5: i32 = (round_by_dec(cmd.outd, 3) * 1000.0) as i32;

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
        let mut tl:f64=0.0;
        let mut outd:f64=0.0;
        cmnd.iter().for_each(|cmd| {
            tl=tl+cmd.l+cmd.lt;
            outd=cmd.outd;
        });
        (tl,outd)
    }
}

impl Display for LRACLR{
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "L {} R {} A {} CLR {}", self.l, self.r, self.a, self.clr)
    }
}

pub struct LRACMD {
    pub id: i32,
    pub op_code: i32,
    pub value0: f64,
    pub value1: f64,
}


impl LRACMD {
    pub fn to_array(cmnd: &Vec<LRACMD>) -> Vec<i32> {
        let mut arr: Vec<i32> = vec![];
        cmnd.iter().for_each(|cmd| {
            let rounded0: i32 = (round_by_dec(cmd.value0, 3) * 1000.0) as i32;
            let rounded1: i32 = (round_by_dec(cmd.value1, 3) * 1000.0) as i32;
            arr.push(cmd.id);
            arr.push(cmd.op_code);
            arr.push(rounded0);
            arr.push(rounded1);
        });
        arr
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
    pub fn flush_to_pt_file(&self) {
        let mut counter = 0;
        self.ops.iter().for_each(|op| {
            match op {
                OpElem::CYL(c) => {
                    export_to_pt(&c.gen_points(), counter);
                    counter = counter + 1;
                }
                OpElem::TOR(tor) => {
                    export_to_pt(&tor.gen_points(), counter);
                    counter = counter + 1;
                }
                OpElem::Nothing => {}
            }
        });
    }
    pub fn triangulate_all(&mut self) {
        let mut prev_dir: Vector3 = Vector3::new(0.0, 0.0, 0.0);
        self.ops.iter_mut().for_each(|op| {
            match op {
                OpElem::CYL(c) => {
                    prev_dir = c.cb.loc.sub(c.ca.loc);
                    c.triangulate()
                }
                OpElem::TOR(t) => {
                    t.triangulate(&prev_dir)
                }
                OpElem::Nothing => {}
            }
        })
    }
    pub fn to_obj(&self) {
        self.ops.iter().for_each(|op| {
            match op {
                OpElem::CYL(c) => { c.to_obj() }
                OpElem::TOR(t) => { t.to_obj() }
                OpElem::Nothing => {}
            }
        })
    }
    pub fn all_to_one_obj(&self) {
        let mut triangles: Vec<Triangle> = vec![];
        self.ops.iter().for_each(|op| {
            match op {
                OpElem::CYL(c) => { triangles.extend_from_slice(c.triangles.as_slice()) }
                OpElem::TOR(t) => { triangles.extend_from_slice(t.triangles.as_slice()) }
                OpElem::Nothing => {}
            }
        });

        let mut positions: Vec<Point3> = vec![];
        let mut indx: Vec<[StandardVertex; 3]> = vec![];
        triangles.iter().for_each(|tri| {
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
        let id: i32 = random();
        let path = format!("d:\\pipe_project\\cyl_{}.obj", id);
        let mut obj_file = std::fs::File::create(path).unwrap();
        obj::write(&polymesh, obj_file).unwrap();
    }

    pub fn all_to_one_obj_bin(&self) -> Vec<u8> {
        let mut triangles: Vec<Triangle> = vec![];
        self.ops.iter().for_each(|op| {
            match op {
                OpElem::CYL(c) => { triangles.extend_from_slice(c.triangles.as_slice()) }
                OpElem::TOR(t) => { triangles.extend_from_slice(t.triangles.as_slice()) }
                OpElem::Nothing => {}
            }
        });

        let mut positions: Vec<Point3> = vec![];
        let mut indx: Vec<[StandardVertex; 3]> = vec![];
        triangles.iter().for_each(|tri| {
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
        let id: i32 = random();
        //let path = format!("d:\\pipe_project\\pipe.obj");
        //let mut obj_file = std::fs::File::create(path).unwrap();
        let mut file: Vec<u8> = Vec::new();
        obj::write(&polymesh, &mut file).unwrap();
        file
    }
    pub fn convert_polymesh(mesh: &PolygonMesh) -> (Vec<f32>, Vec<i32>, BoundingBox<cgmath::Point3<f64>>, Vec<Triangle>) {
        let mut vertsu8: Vec<f32> = vec![];
        let mut indxu8: Vec<i32> = vec![];
        let mut triangles: Vec<Triangle> = vec![];
        let mut bin_bbxu8: BoundingBox<Point3> = BoundingBox::default();
        let mut is_valid = true;
        let vrtx: &Vec<Point3> = mesh.positions();
        let norms: &Vec<Vector3> = mesh.normals();
        let mut vs: Vec<cgmath::Vector3<f32>> = vec![];
        let mut nms: Vec<cgmath::Vector3<f32>> = vec![];
        let mut is: Vec<i32> = vec![];
        let mut counter = 0;
        mesh.tri_faces().iter().for_each(|f| {
            f.iter().for_each(|sv| {
                let v = vrtx[sv.pos];
                let n = norms[sv.nor.unwrap()];
                is.push(counter);
                vs.push(cgmath::Vector3::new(v.x as f32, v.y as f32, v.z as f32));
                nms.push(cgmath::Vector3::new(n.x as f32, n.y as f32, n.z as f32));
                counter = counter + 1;
            });
        });
        bin_bbxu8 = mesh.bounding_box();

        let bbx: BoundingBox<Point3> =
            BoundingBox::from_iter([bin_bbxu8.min().mul(Z_FIGHTING_FACTOR as f64), bin_bbxu8.max().mul(Z_FIGHTING_FACTOR as f64)]);
        if (is_valid) {
            is.chunks(3).for_each(|tri| {
                let i0 = tri[0];
                let v0 = vs[i0 as usize];
                let n0 = nms[i0 as usize];

                let i1 = tri[1];
                let v1 = vs[i1 as usize];
                let n1 = nms[i1 as usize];

                let i2 = tri[2];
                let v2 = vs[i2 as usize];
                let n2 = nms[i2 as usize];
                let triangle: Triangle = Triangle::new(
                    cgmath::Point3::<f32>::new((v0[0] * Z_FIGHTING_FACTOR) as f32, (v0[1] * Z_FIGHTING_FACTOR) as f32, (v0[2] * Z_FIGHTING_FACTOR) as f32),
                    cgmath::Point3::<f32>::new((v1[0] * Z_FIGHTING_FACTOR) as f32, (v1[1] * Z_FIGHTING_FACTOR) as f32, (v1[2] * Z_FIGHTING_FACTOR) as f32),
                    cgmath::Point3::<f32>::new((v2[0] * Z_FIGHTING_FACTOR) as f32, (v2[1] * Z_FIGHTING_FACTOR) as f32, (v2[2] * Z_FIGHTING_FACTOR) as f32),
                );
                triangles.push(triangle);
                vertsu8.push(v0.x * Z_FIGHTING_FACTOR);
                vertsu8.push(v0.y * Z_FIGHTING_FACTOR);
                vertsu8.push(v0.z * Z_FIGHTING_FACTOR);
                vertsu8.push(n0.x);
                vertsu8.push(n0.y);
                vertsu8.push(n0.z);
                indxu8.push(i0);

                vertsu8.push(v1.x * Z_FIGHTING_FACTOR);
                vertsu8.push(v1.y * Z_FIGHTING_FACTOR);
                vertsu8.push(v1.z * Z_FIGHTING_FACTOR);
                vertsu8.push(n1.x);
                vertsu8.push(n1.y);
                vertsu8.push(n1.z);
                indxu8.push(i1);

                vertsu8.push(v2.x * Z_FIGHTING_FACTOR);
                vertsu8.push(v2.y * Z_FIGHTING_FACTOR);
                vertsu8.push(v2.z * Z_FIGHTING_FACTOR);
                vertsu8.push(n2.x);
                vertsu8.push(n2.y);
                vertsu8.push(n2.z);
                indxu8.push(i2);
            });
        } else {
            println!("NOT VALID V={:?}  N={:?}  I={:?}", vs.len(), nms.len(), is.len());
        }
        (vertsu8, indxu8, bbx, triangles)
    }
    pub fn to_render_data(&self) -> (Vec<MeshVertex>, Vec<u32>, Vec<f32>, Vec<u32>,f64) {
        let mut cyl_color_id: i32 = 74;
        let mut toro_color_id: i32 = 84;
        let mut outer_diam:f64=0.0;
        let mut id_count: u32 = 1;
        let mut meshes_all: Vec<RawMesh> = vec![];
        self.ops.iter().for_each(|op| {
            match op {
                OpElem::CYL(c) => {
                    outer_diam= c.r;
                    let pm = c.to_polygon_mesh();
                    let (verts, indx, bbx, triangles) = CncOps::convert_polymesh(&pm);
                    if (verts.len() > 0 && indx.len() > 0) {
                        let rm = RawMesh {
                            id: c.id.clone() as i32,
                            vertex_normal: verts,
                            indx: indx,
                            bbx: bbx,
                            triangles: triangles,
                        };
                        meshes_all.push(rm);
                    }
                }
                OpElem::TOR(t) => {
                    let pm = t.to_polygon_mesh();
                    let (verts, indx, bbx, triangles) = CncOps::convert_polymesh(&pm);
                    if (verts.len() > 0 && indx.len() > 0) {
                        let rm = RawMesh {
                            id: t.id.clone() as i32,
                            vertex_normal: verts,
                            indx: indx,
                            bbx: bbx,
                            triangles: triangles,
                        };
                        meshes_all.push(rm);
                    }
                }
                OpElem::Nothing => {}
            }
        });

        let mut bbxs: Vec<f32> = vec![];
        let mut buffer: Vec<MeshVertex> = vec![];
        let mut indxes: Vec<u32> = vec![];
        let mut id_hash: Vec<u32> = vec![];
        let mut curid: u32 = 1;
        let mut currmat: i32 = 0;
        let mut index: u32 = 0;
        meshes_all.iter().for_each(|mesh| {
            //curid = mesh.id as u32;
            curid = id_count;
            id_hash.push(curid.clone() as u32);
            id_hash.push(index.clone());
            let bbx: &BoundingBox<Point3> = &mesh.bbx;
            bbxs.push(bbx.min().x as f32);
            bbxs.push(bbx.min().y as f32);
            bbxs.push(bbx.min().z as f32);
            bbxs.push(bbx.max().x as f32);
            bbxs.push(bbx.max().y as f32);
            bbxs.push(bbx.max().z as f32);

            mesh.vertex_normal.chunks(6).for_each(|vn| {
                let mv = MeshVertex {
                    position: [vn[0], vn[1], vn[2], 1.0],
                    normal: [vn[3], vn[4], vn[5], 1.0],
                    id: curid as i32,
                };
                buffer.push(mv);
                indxes.push(index);
                index = index + 1;
            });
            id_hash.push(index.clone() - 1);
            id_count = id_count + 1;
        });
        (buffer, indxes, bbxs, id_hash,outer_diam)
    }

    pub fn to_render_data_unbend(&self,ops:Vec<OpElem>) -> (Vec<MeshVertex>, Vec<u32>, Vec<f32>, Vec<u32>,f64) {
        let mut cyl_color_id: i32 = 74;
        let mut toro_color_id: i32 = 84;
        let mut outer_diam:f64=0.0;
        let mut id_count: u32 = 1;
        let mut meshes_all: Vec<RawMesh> = vec![];
        ops.iter().for_each(|op| {
            match op {
                OpElem::CYL(c) => {
                    outer_diam= c.r;
                    let pm = c.to_polygon_mesh();
                    let (verts, indx, bbx, triangles) = CncOps::convert_polymesh(&pm);
                    if (verts.len() > 0 && indx.len() > 0) {
                        let rm = RawMesh {
                            id: c.id.clone() as i32,
                            vertex_normal: verts,
                            indx: indx,
                            bbx: bbx,
                            triangles: triangles,
                        };
                        meshes_all.push(rm);
                    }
                }
                OpElem::TOR(t) => {
                    let pm = t.to_polygon_mesh();
                    let (verts, indx, bbx, triangles) = CncOps::convert_polymesh(&pm);
                    if (verts.len() > 0 && indx.len() > 0) {
                        let rm = RawMesh {
                            id: t.id.clone() as i32,
                            vertex_normal: verts,
                            indx: indx,
                            bbx: bbx,
                            triangles: triangles,
                        };
                        meshes_all.push(rm);
                    }
                }
                OpElem::Nothing => {}
            }
        });
        let mut bbxs: Vec<f32> = vec![];
        let mut buffer: Vec<MeshVertex> = vec![];
        let mut indxes: Vec<u32> = vec![];
        let mut id_hash: Vec<u32> = vec![];
        let mut curid: u32 = 0;
        let mut currmat: i32 = 0;
        let mut index: u32 = 0;
        meshes_all.iter().for_each(|mesh| {
            //curid = mesh.id as u32;
            curid = id_count;
            id_hash.push(curid.clone() as u32);
            id_hash.push(index.clone());
            let bbx: &BoundingBox<Point3> = &mesh.bbx;
            bbxs.push(bbx.min().x as f32);
            bbxs.push(bbx.min().y as f32);
            bbxs.push(bbx.min().z as f32);
            bbxs.push(bbx.max().x as f32);
            bbxs.push(bbx.max().y as f32);
            bbxs.push(bbx.max().z as f32);

            mesh.vertex_normal.chunks(6).for_each(|vn| {
                let mv = MeshVertex {
                    position: [vn[0], vn[1], vn[2], 1.0],
                    normal: [vn[3], vn[4], vn[5], 1.0],
                    id: curid as i32,
                };
                buffer.push(mv);
                indxes.push(index);
                index = index + 1;
            });
            id_hash.push(index.clone() - 1);
            id_count = id_count + 1;
        });
        (buffer, indxes, bbxs, id_hash,outer_diam)
    }

    pub fn calculate_lraclr(&self) -> Vec<LRACLR> {

        let mut cncs:Vec<LRACLR>=vec![];
        if (self.ops.len() > 1 && self.ops.len().is_odd()) {
            let mut plane = Plane::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0));
            let last_op: OpElem =self.ops.last().unwrap().clone();
            let mut pair_arr:Vec<OpElem>=vec![];
            for i in 0..self.ops.len()-1 {
                pair_arr.push(self.ops[i].clone());
            }
            let mut it_first=true;
            let mut outd:f64=0.0;
            let mut counter=1;
            pair_arr.chunks(2).for_each(|ops|{
                let op1=&ops[0];
                let op2=&ops[1];
                let mut cnc: LRACLR =LRACLR::default();
                match op1 {
                    OpElem::CYL(c) => {
                        cnc.id1= counter;
                        counter=counter+1;
                        cnc.l=c.h;
                        outd=c.r*2.0;
                        cnc.outd=outd;
                    }
                    OpElem::TOR(t) => {}
                    OpElem::Nothing => {}
                }
                match op2 {
                    OpElem::CYL(c) => {}
                    OpElem::TOR(t) => {
                        let mut r_rad=0.0;
                        match op1 {
                            OpElem::CYL(c) => {
                                let r: Rad<f64> = if (it_first) {
                                    let new_plane = Plane::new(t.bend_center_point, t.ca.loc, t.cb.loc);
                                    plane = new_plane;
                                    it_first=false;
                                    Rad(0.0)
                                }
                                else
                                {
                                    let new_plane = Plane::new(t.bend_center_point, t.ca.loc, t.cb.loc);
                                    let rot_axe: Vector3 = c.cb.loc.sub(c.ca.loc);
                                    let prev_vec = plane.normal();
                                    let prev_right = rot_axe.cross(prev_vec);
                                    let curr_vec = new_plane.normal();
                                    let cp = c.cb.loc.clone();
                                    let prev_p = cp.clone() + prev_vec * c.r;
                                    let curr_p = cp.clone() + curr_vec * c.r;
                                    let ccw_vec = curr_p.sub(prev_p);
                                    let rot_angle = prev_vec.angle(curr_vec);
                                    let k: f64 = {
                                        if (rot_angle == Rad(PI) || rot_angle == Rad(2.0 * PI)) {
                                            -1.0
                                        } else if (prev_right.dot(ccw_vec) < 0.0) {
                                            1.0
                                        } else {
                                            1.0
                                        }
                                    };
                                    plane = new_plane;
                                    rot_angle * k
                                };
                                r_rad=r.0;
                                cnc.r=Deg::from(r).0;
                            }
                            OpElem::TOR(t) => {}
                            OpElem::Nothing => {}
                        }

                        cnc.clr=t.bend_radius;
                        cnc.id2= counter;
                        counter=counter+1;
                        let ba = t.ca.loc.sub(t.bend_center_point);
                        let bb = t.cb.loc.sub(t.bend_center_point);
                        let bend_angle = ba.angle(bb);
                        cnc.lt=r_rad*t.bend_radius;
                        cnc.a= Deg::from(bend_angle).0;
                    }
                    OpElem::Nothing => {}
                }
                cnc.outd=outd;
                cncs.push(cnc);
            });
            match last_op {
                OpElem::CYL(c) => {
                    let mut cnc: LRACLR =LRACLR::default();
                    cnc.id1= counter;
                    cnc.l=c.h;
                    cnc.outd=outd;
                    cncs.push(cnc);
                }
                OpElem::TOR(_) => {}
                OpElem::Nothing => {}
            }
        }
        cncs
    }

    pub fn calculate_lra(&self) -> Vec<i32> {
        let mut cmnds: Vec<LRACMD> = vec![];
        let mut plane = Plane::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0));
        if (self.ops.len() > 0) {
            for i in 0..self.ops.len() - 1 {
                match &self.ops[i] {
                    OpElem::CYL(c) => {
                        match &self.ops[i + 1] {
                            OpElem::CYL(cn) => {}
                            OpElem::TOR(t) => {
                                let l = c.h;
                                let r: Rad<f64> = if (i == 0) {
                                    let new_plane = Plane::new(t.bend_center_point, t.ca.loc, t.cb.loc);
                                    plane = new_plane;
                                    Rad(0.0)
                                } else {
                                    let new_plane = Plane::new(t.bend_center_point, t.ca.loc, t.cb.loc);
                                    let rot_axe: Vector3 = c.cb.loc.sub(c.ca.loc);
                                    let prev_vec = plane.normal();
                                    let prev_right = rot_axe.cross(prev_vec);
                                    let curr_vec = new_plane.normal();
                                    let cp = c.cb.loc.clone();
                                    let prev_p = cp.clone() + prev_vec * c.r;
                                    let curr_p = cp.clone() + curr_vec * c.r;
                                    let ccw_vec = curr_p.sub(prev_p);
                                    let rot_angle = prev_vec.angle(curr_vec);
                                    let k: f64 = {
                                        if (rot_angle == Rad(PI) || rot_angle == Rad(2.0 * PI)) {
                                            -1.0
                                        } else if (prev_right.dot(ccw_vec) < 0.0) {
                                            1.0
                                        } else {
                                            1.0
                                        }
                                    };
                                    plane = new_plane;
                                    rot_angle * k
                                };
                                let br = t.bend_radius;
                                let ba = t.ca.loc.sub(t.bend_center_point);
                                let bb = t.cb.loc.sub(t.bend_center_point);
                                let bend_angle = ba.angle(bb);
                                //println! {"CID={:?} TID={:?}  L={:?} R={:?} BR={:?} A={:?}", c.id, t.id, l, Deg::from(r), br, Deg::from(bend_angle)}
                                //LRAK
                                let l_op = LRACMD {
                                    id: c.id as i32,
                                    op_code: L,
                                    value0: l,
                                    value1: 0.0,
                                };
                                let r_op = LRACMD {
                                    id: c.id as i32,
                                    op_code: R,
                                    value0: Deg::from(r).0,
                                    value1: 0.0,
                                };
                                let k_op = LRACMD {
                                    id: t.id as i32,
                                    op_code: K,
                                    value0: br,
                                    value1: 0.0,
                                };
                                let a_op = LRACMD {
                                    id: t.id as i32,
                                    op_code: A,
                                    value0: Deg::from(bend_angle).0,
                                    value1: t.bend_radius,
                                };
                                cmnds.push(l_op);
                                cmnds.push(r_op);
                                cmnds.push(k_op);
                                cmnds.push(a_op);
                            }
                            OpElem::Nothing => {}
                        }
                    }
                    OpElem::TOR(t) => {}
                    OpElem::Nothing => {}
                }
            }
            match &self.ops.last().unwrap() {
                OpElem::CYL(c) => {
                    //println! {"CID={:?}  L={:?} ", c.id, c.h}
                    let l_op = LRACMD {
                        id: c.id as i32,
                        op_code: L,
                        value0: c.h,
                        value1: 0.0,
                    };
                    cmnds.push(l_op);
                }
                OpElem::TOR(_) => {}
                OpElem::Nothing => {}
            }
        }

        LRACMD::to_array(&cmnds)
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
        let ret = CncOps {
            ops: new_ops
        };
        ret
    }
    pub fn generate_unbend_model_from_cl(&self) -> (Vec<MeshVertex>, Vec<u32>, Vec<f32>, Vec<u32>, f64) {
        let mut unbend_cncs:Vec<OpElem>=Vec::new();
        let mut offset:f64 = 0.0;
        let mut id:u64=0;
        self.ops.iter().for_each(|op|{

            match op {
                OpElem::CYL(c) => {
                    let end_x=offset-c.h;
                    let mut mc:MainCylinder=MainCylinder{
                        id: id,
                        ca: MainCircle {
                            id:random(),
                            radius: c.r,
                            loc: Point3::new(offset,0.0,0.0),
                            dir: Vector3::new(-1.0,0.0,0.0),
                            radius_dir: Vector3::new(0.0,0.0,1.0),
                        },
                        cb: MainCircle {
                            id:random(),
                            radius: c.r,
                            loc:Point3::new(end_x,0.0,0.0),
                            dir:  Vector3::new(-1.0,0.0,0.0),
                            radius_dir: Vector3::new(0.0,0.0,1.0),
                        },
                        h:c.h,
                        r: c.r,
                        r_gr_id: 0,
                        ca_tor: 0,
                        cb_tor: 0,
                        triangles: vec![],
                    };
                    mc.triangulate();
                    unbend_cncs.push(OpElem::CYL(mc));
                    offset=offset-c.h;
                    id=id+1;
                }
                OpElem::TOR(t) => {
                    let bend_r=t.r+t.bend_radius;
                    let h =bend_r*t.angle().0;
                    let end_x=offset-h;
                    let mut mc:MainCylinder=MainCylinder{
                        id: id,
                        ca: MainCircle {
                            id:random(),
                            radius: t.r,
                            loc: Point3::new(offset,0.0,0.0),
                            dir: Vector3::new(-1.0,0.0,0.0),
                            radius_dir: Vector3::new(0.0,0.0,1.0),
                        },
                        cb: MainCircle {
                            id:random(),
                            radius: t.r,
                            loc:Point3::new(end_x,0.0,0.0),
                            dir:  Vector3::new(-1.0,0.0,0.0),
                            radius_dir: Vector3::new(0.0,0.0,1.0),
                        },
                        h: h,
                        r: t.r,
                        r_gr_id: 0,
                        ca_tor: 0,
                        cb_tor: 0,
                        triangles: vec![],
                    };
                    mc.triangulate();
                    unbend_cncs.push(OpElem::CYL(mc));
                    offset=offset-h;
                    id=id+1;
                }
                OpElem::Nothing => {}
            }
        });
        self.to_render_data_unbend(unbend_cncs)
    }

    pub fn generate_one_cyl() -> (Vec<MeshVertex>, Vec<u32>, Vec<f32>, Vec<u32>, f64) {
        let mut mc:MainCylinder=MainCylinder{
            id: 0,
            ca: MainCircle {
                id: random(),
                radius: 15.0,
                loc: Point3 ::new(0.0,0.0,0.0),
                dir: Vector3 ::new(-1.0,0.0,0.0),
                radius_dir:  Vector3 ::new(0.0,0.0,1.0),
            },
            cb: MainCircle {
                id: random(),
                radius: 15.0,
                loc:  Point3 ::new(-30.0,0.0,0.0),
                dir: Vector3 ::new(-1.0,0.0,0.0),
                radius_dir:  Vector3 ::new(0.0,0.0,1.0),
            },
            h: 30.0,
            r: 0.0,
            r_gr_id: 0,
            ca_tor: 0,
            cb_tor: 0,
            triangles: vec![],
        };
        mc.triangulate();
        let oe=OpElem::CYL(mc);
        let mut unbend_cncs:Vec<OpElem>=Vec::new();
        unbend_cncs.push(oe);

        let mut cyl_color_id: i32 = 74;
        let mut toro_color_id: i32 = 84;
        let mut outer_diam:f64=0.0;
        let mut id_count: u32 = 1;
        let mut meshes_all: Vec<RawMesh> = vec![];
        unbend_cncs.iter().for_each(|op| {
            match op {
                OpElem::CYL(c) => {
                    outer_diam= c.r;
                    let pm = c.to_polygon_mesh();
                    let (verts, indx, bbx, triangles) = CncOps::convert_polymesh(&pm);
                    if (verts.len() > 0 && indx.len() > 0) {
                        let rm = RawMesh {
                            id: c.id.clone() as i32,
                            vertex_normal: verts,
                            indx: indx,
                            bbx: bbx,
                            triangles: triangles,
                        };
                        meshes_all.push(rm);
                    }
                }
                OpElem::TOR(t) => {
                    let pm = t.to_polygon_mesh();
                    let (verts, indx, bbx, triangles) = CncOps::convert_polymesh(&pm);
                    if (verts.len() > 0 && indx.len() > 0) {
                        let rm = RawMesh {
                            id: t.id.clone() as i32,
                            vertex_normal: verts,
                            indx: indx,
                            bbx: bbx,
                            triangles: triangles,
                        };
                        meshes_all.push(rm);
                    }
                }
                OpElem::Nothing => {}
            }
        });
        let mut bbxs: Vec<f32> = vec![];
        let mut buffer: Vec<MeshVertex> = vec![];
        let mut indxes: Vec<u32> = vec![];
        let mut id_hash: Vec<u32> = vec![];
        let mut curid: u32 = 0;
        let mut currmat: i32 = 0;
        let mut index: u32 = 0;
        meshes_all.iter().for_each(|mesh| {
            //curid = mesh.id as u32;
            curid = id_count;
            id_hash.push(curid.clone() as u32);
            id_hash.push(index.clone());
            let bbx: &BoundingBox<Point3> = &mesh.bbx;
            bbxs.push(bbx.min().x as f32);
            bbxs.push(bbx.min().y as f32);
            bbxs.push(bbx.min().z as f32);
            bbxs.push(bbx.max().x as f32);
            bbxs.push(bbx.max().y as f32);
            bbxs.push(bbx.max().z as f32);

            mesh.vertex_normal.chunks(6).for_each(|vn| {
                let mv = MeshVertex {
                    position: [vn[0], vn[1], vn[2], 1.0],
                    normal: [vn[3], vn[4], vn[5], 1.0],
                    id: curid as i32,
                };
                buffer.push(mv);
                indxes.push(index);
                index = index + 1;
            });
            id_hash.push(index.clone() - 1);
            id_count = id_count + 1;
        });
        (buffer, indxes, bbxs, id_hash,outer_diam)
    }
}