use std::fmt;
use cgmath::{Deg, InnerSpace, MetricSpace, Point3, Rad, Vector3, Zero};
use cgmath::num_traits::{abs, Float};
use ruststep::ast::{Exchange, Name};
use ruststep::tables::PlaceHolder;
use truck_stepio::r#in::Table;
//use crate::{extract_circles_profiles, extract_toroidal};

const TOL: f64 = 0.00000000001;

#[derive(PartialEq, Debug, Clone)]
pub enum BendState {
    Bend(PipeToroidal),
    PreBend(PipeToroidal),
    PostBend(PipeToroidal),
    Stright,
}

#[derive(Clone)]
pub struct PipeCircleProfile {
    pub id: u64,
    pub radius: f64,
    pub direction: Vector3<f64>,
    pub direction_ref: Vector3<f64>,
    pub center_point: Point3<f64>,
}

impl PipeCircleProfile {
    pub fn new(id: u64, radius: f64, direction: Vector3<f64>, direction_ref: Vector3<f64>, center_point: Point3<f64>) -> Self {
        Self {
            id: id,
            radius: radius,
            direction: direction,
            direction_ref: direction_ref,
            center_point: center_point,
        }
    }
}

impl fmt::Debug for PipeCircleProfile {
    fn fmt(&self, fmt: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt.debug_struct("PipeCircleProfile")
            .field("id: ", &self.id)
            .field("radius: ", &self.radius)
            .field("center_point", &format_args!("{:?}", self.center_point))
            .field("direction", &format_args!("{:?}", self.direction))
            .field("direction_ref", &format_args!("{:?}", self.direction_ref))
            .finish()
    }
}

#[derive(Clone, PartialEq)]
pub struct PipeToroidal {
    pub id: u64,
    pub minor_radius: f64,
    pub major_radius: f64,
    pub direction: Vector3<f64>,
    pub direction_ref: Vector3<f64>,
    pub center_point: Point3<f64>,
}

impl PipeToroidal {
    pub fn new(id: u64, minor_radius: f64, major_radius: f64, direction: Vector3<f64>, direction_ref: Vector3<f64>, center_point: Point3<f64>) -> Self {
        Self {
            id: id,
            minor_radius: minor_radius,
            major_radius: major_radius,
            direction: direction,
            direction_ref: direction_ref,
            center_point: center_point,
        }
    }
}

impl fmt::Debug for PipeToroidal {
    fn fmt(&self, fmt: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt.debug_struct("PipeToroidal")
            .field("id: ", &self.id)
            .field("minor_radius: ", &self.minor_radius)
            .field("major_radius: ", &self.major_radius)
            .field("center_point", &format_args!("{:?}", self.center_point))
            .field("direction", &format_args!("{:?}", self.direction))
            .field("direction_ref", &format_args!("{:?}", self.direction_ref))
            .finish()
    }
}

#[derive(Clone)]
pub struct BendOp {
    pub id: u64,
    pub start_position: PipeCircleProfile,
    pub end_position: PipeCircleProfile,
    pub bend_state: BendState,
}

impl BendOp {
    pub fn new(id: u64, start_position: PipeCircleProfile, end_position: PipeCircleProfile) -> Self {
        Self {
            id: id,
            start_position: start_position,
            end_position: end_position,
            bend_state: BendState::Stright,
        }
    }
}


pub struct BendPipe {
    pub tol: f64,
    pub n_radius: f64,
    prof_circles_raw: Vec<PipeCircleProfile>,
    prof_bend_raw: Vec<PipeToroidal>,
    pub prof_circles: Vec<PipeCircleProfile>,
    pub prof_bends: Vec<PipeToroidal>,
    pub bend_ops: Vec<BendOp>,
    pub ops_names: Vec<i32>,
    pub ops_values: Vec<f32>,
}

impl BendPipe {
    pub fn new() -> Self {
        //let step_file = std::fs::read_to_string("d:\\pipe_project\\pipe1.stp").unwrap();
        let step_file = DEMO_PIPE_STEP;
        let exchange: Exchange = ruststep::parser::parse(step_file).unwrap();
        let table: Table = Table::from_data_section(&exchange.data[0]);
        let prof_circles_raw: Vec<PipeCircleProfile> = BendPipe::extract_circles_profiles(table.clone());
        let prof_bend_raw: Vec<PipeToroidal> = BendPipe::extract_toroidal(table.clone());
        let n_radius = BendPipe::calculate_pipe_radius(&prof_circles_raw);
        let mut prof_circles: Vec<PipeCircleProfile> = prof_circles_raw.iter().filter(|&x| x.radius == n_radius).cloned().collect();
        prof_circles.sort_by(|a, b| a.id.cmp(&b.id));
        let mut prof_bends: Vec<PipeToroidal> = prof_bend_raw.iter().filter(|&x| x.minor_radius == n_radius).cloned().collect();
        prof_bends.sort_by(|a, b| a.id.cmp(&b.id));

        let mut bend_ops: Vec<BendOp> = vec![];
        let mut counter = 0;
        prof_circles.iter().for_each(|pc| {
            if (counter != prof_circles.len() - 1) {
                let a = prof_circles[counter].clone();
                let b = prof_circles[counter + 1].clone();
                let mut bo = BendOp::new(counter as u64, a, b);
                let state = BendPipe::find_bending_parameters(&bo, &prof_bends);
                bo.bend_state = state;
                bend_ops.push(bo);
            }
            counter = counter + 1;
        });

        Self {
            tol: 0.00000000001,
            n_radius: n_radius,
            prof_circles_raw: prof_circles_raw,
            prof_bend_raw: prof_bend_raw,
            prof_circles: prof_circles,
            prof_bends: prof_bends,
            bend_ops: bend_ops,
            ops_names: vec![],
            ops_values: vec![],
        }
    }
    fn find_bending_parameters(bo: &BendOp, prof_bends: &Vec<PipeToroidal>) -> BendState {
        let mut a = false;
        let mut b = false;
        let mut state: BendState = BendState::Stright;
        prof_bends.iter().for_each(|pb| {
            let a_dist = pb.center_point.distance(bo.start_position.center_point);
            let b_dist = pb.center_point.distance(bo.end_position.center_point);
            if (!a && !b) {
                a = BendPipe::check(a_dist, pb.major_radius, TOL);
                b = BendPipe::check(b_dist, pb.major_radius, TOL);
                match (a, b) {
                    (false, false) => { state = BendState::Stright }
                    (true, true) => { state = BendState::Bend(pb.clone()) }
                    (false, true) => { state = BendState::PreBend(pb.clone()) }
                    (true, false) => { state = BendState::PostBend(pb.clone()) }
                }
            }
        });

        state
    }
    fn calculate_pipe_radius(prof_circles_raw: &Vec<PipeCircleProfile>) -> f64 {
        let mut v: Vec<f64> = vec![];
        prof_circles_raw.iter().for_each(|pc| {
            v.push(pc.radius);
        });
        v.iter().fold(f64::NEG_INFINITY, |prev, curr| prev.max(*curr))
    }
    fn check(a: f64, b: f64, tol: f64) -> bool {
        if (abs(a - b) < tol) {
            true
        } else { false }
    }
    fn set_rot_angle(&self)->Vector3<f64>{
        let mut v:Vector3<f64>=Vector3::new(0.0,1.0,0.0);
        let mut is_set=false;
        self.bend_ops.iter().for_each(|op|{
            if(!is_set){
               match  &op.bend_state{
                   BendState::Bend(b) => {
                       v=b.direction.clone();
                       is_set=true;
                   }
                   BendState::PreBend(_) => {}
                   BendState::PostBend(_) => {}
                   BendState::Stright => {}
               }
            }
        });
        v
    }

    pub fn generate_abstarct_cnc(&mut self) {
        let mut rot_circle=self.set_rot_angle();
        self.bend_ops.iter().for_each(|op| {
            match &op.bend_state {
                BendState::Bend(p) => {
                    let bend_dir=p.direction;
                    let p1 = op.start_position.center_point;
                    let p2 = op.end_position.center_point;
                    let d1=op.start_position.direction;
                    let d2=op.end_position.direction;
                    let b_angle: Rad<f64> =d1.angle(d2);
                    let c_angle: Rad<f64> =rot_circle.angle(bend_dir);
                    let d = p1.distance(p2);
                    //println!("MOVE {:?} mm",d);
                    println!("Rot ANGLE {:?} mm",Deg::from(c_angle));
                    self.ops_names.push(0);
                    self.ops_values.push(Deg::from(c_angle).0 as f32);
                    println!("Radius {:?} mm", p.major_radius);
                    self.ops_names.push(1);
                    self.ops_values.push(p.major_radius as f32);
                    println!("Bend ANGLE {:?} mm", Deg::from(b_angle));
                    self.ops_names.push(2);
                    self.ops_values.push( Deg::from(b_angle).0 as f32);
                    rot_circle=bend_dir;
                }
                BendState::PreBend(p) => {
                    let p1 = op.start_position.center_point;
                    let p2 = op.end_position.center_point;
                    let d1=op.start_position.direction;
                    let d2=op.end_position.direction;
                    let angle: Rad<f64> =d1.angle(d2);
                    let d = p1.distance(p2);
                    println!("MOVE {:?} mm",d);
                    self.ops_names.push(3);
                    self.ops_values.push( d as f32);
                    //println!("PreBend ANGLE {:?} mm", Deg::from(angle));
                }
                BendState::PostBend(p) => {
                    let bend_dir=p.direction;
                    let p1 = op.start_position.center_point;
                    let p2 = op.end_position.center_point;
                    let d = p1.distance(p2);
                    let d1=op.start_position.direction;
                    let d2=op.end_position.direction;
                    let angle: Rad<f64> =d1.angle(d2);
                    let angle_rot: Rad<f64> =d1.angle(bend_dir);
                    println!("PostBend MOVE {:?} mm",d);
                    self.ops_names.push(3);
                    self.ops_values.push( d as f32);
                    //println!("PostBend ANGLE {:?} mm", Deg::from(angle));
                    //println!("PostBend ANGLE_ROT {:?} mm", Deg::from(angle_rot));
                }
                BendState::Stright => {
                    let p1 = op.start_position.center_point;
                    let p2 = op.end_position.center_point;
                    let d = p1.distance(p2);
                    self.ops_names.push(3);
                    self.ops_values.push( d as f32);
                    println!("MOVE {:?} mm",d);
                }
            }
        })
    }

    fn extract_circles_profiles(table: Table) -> Vec<PipeCircleProfile> {
        let mut ret: Vec<PipeCircleProfile> = vec![];
        table.circle.iter().for_each(|(k, v)| {
            let id=k.clone();
            let radius=v.radius;
            let mut direction: cgmath::Vector3<f64>= truck_geometry::base::Vector3::zero();
            let mut direction_ref: cgmath::Vector3<f64>= truck_geometry::base::Vector3::zero();
            let mut center_point: cgmath::Point3<f64>= truck_geometry::base::Point3::new(0.0, 0.0, 0.0);
            match &v.position {
                PlaceHolder::Ref(name) => {
                    match name {
                        Name::Entity(key) => {
                            match table.axis2_placement_3d.get(key) {
                                None => {}
                                Some(ax) => {
                                    match &ax.axis {
                                        None => {}
                                        Some(a) => {
                                            match a {
                                                PlaceHolder::Ref(name) => {
                                                    match name {
                                                        Name::Entity(key) => {
                                                            match table.direction.get(key) {
                                                                None => {}
                                                                Some(dir) => {
                                                                    let mut count=0;
                                                                    dir.direction_ratios.iter().for_each(|d| {
                                                                        match count{
                                                                            0 => {direction.x=d.clone()}
                                                                            1 => {direction.y=d.clone()}
                                                                            2 => {direction.z=d.clone()}
                                                                            _ => {}
                                                                        }
                                                                        count=count+1;
                                                                    })
                                                                }
                                                            }
                                                        }
                                                        Name::Value(_) => {}
                                                        Name::ConstantEntity(_) => {}
                                                        Name::ConstantValue(_) => {}
                                                    }
                                                }
                                                PlaceHolder::Owned(v) => {}
                                            }
                                            //println!(" CIRCLE DIR {:?}",a)
                                        }
                                    }

                                    match &ax.location {
                                        PlaceHolder::Ref(name) => {
                                            match name {
                                                Name::Entity(key) => {
                                                    match table.cartesian_point.get(key) {
                                                        None => {}
                                                        Some(dir) => {
                                                            let mut count=0;
                                                            dir.coordinates.iter().for_each(|d| {
                                                                match count{
                                                                    0 => {center_point.x=d.clone()}
                                                                    1 => {center_point.y=d.clone()}
                                                                    2 => {center_point.z=d.clone()}
                                                                    _ => {}
                                                                }
                                                                count=count+1;
                                                            })
                                                        }
                                                    }
                                                }
                                                Name::Value(_) => {}
                                                Name::ConstantEntity(_) => {}
                                                Name::ConstantValue(_) => {}
                                            }
                                        }
                                        PlaceHolder::Owned(_) => {}
                                    }

                                    match &ax.ref_direction {
                                        None => {}
                                        Some(rd) => {
                                            match rd {
                                                PlaceHolder::Ref(name) => {
                                                    match name {
                                                        Name::Entity(key) => {
                                                            match table.direction.get(key) {
                                                                None => {}
                                                                Some(dir) => {
                                                                    let count=0;
                                                                    dir.direction_ratios.iter().for_each(|d| {
                                                                        match count{
                                                                            0 => {direction_ref.x=d.clone()}
                                                                            1 => {direction_ref.y=d.clone()}
                                                                            2 => {direction_ref.z=d.clone()}
                                                                            _ => {}
                                                                        }
                                                                    })
                                                                }
                                                            }
                                                        }
                                                        Name::Value(_) => {}
                                                        Name::ConstantEntity(_) => {}
                                                        Name::ConstantValue(_) => {}
                                                    }
                                                }
                                                PlaceHolder::Owned(v) => {}
                                            }
                                            //println!(" A {:?}",rd)
                                        }
                                    }
                                }
                            }
                        }
                        Name::Value(_) => {}
                        Name::ConstantEntity(_) => {}
                        Name::ConstantValue(_) => {}
                    }
                }
                PlaceHolder::Owned(_) => {}
            }
            let circle: PipeCircleProfile = PipeCircleProfile::new(id, radius, direction, direction_ref, center_point);
            ret.push(circle);
        });
        ret
    }
    fn extract_toroidal(table: Table)-> Vec<PipeToroidal> {
        let mut ret: Vec<PipeToroidal> = vec![];
        table.toroidal_surface.iter().for_each(|(k, v)|{
            let id=k.clone();
            let minor_radius=v.minor_radius.clone();
            let major_radius=v.major_radius.clone();
            let mut direction: cgmath::Vector3<f64>= truck_geometry::base::Vector3::zero();
            let mut direction_ref: cgmath::Vector3<f64>= truck_geometry::base::Vector3::zero();
            let mut center_point: cgmath::Point3<f64>= truck_geometry::base::Point3::new(0.0, 0.0, 0.0);
            match &v.position {
                PlaceHolder::Ref(name) => {
                    match name {
                        Name::Entity(key) => {
                            match table.axis2_placement_3d.get(key) {
                                None => {}
                                Some(ax) => {
                                    match &ax.axis {
                                        None => {}
                                        Some(a) => {
                                            match a {
                                                PlaceHolder::Ref(name) => {
                                                    match name {
                                                        Name::Entity(key) => {
                                                            match table.direction.get(key) {
                                                                None => {}
                                                                Some(dir) => {
                                                                    let mut count=0;
                                                                    dir.direction_ratios.iter().for_each(|d| {
                                                                        match count{
                                                                            0 => {direction.x=d.clone()}
                                                                            1 => {direction.y=d.clone()}
                                                                            2 => {direction.z=d.clone()}
                                                                            _ => {}
                                                                        }
                                                                        count=count+1;
                                                                    })
                                                                }
                                                            }
                                                        }
                                                        Name::Value(_) => {}
                                                        Name::ConstantEntity(_) => {}
                                                        Name::ConstantValue(_) => {}
                                                    }
                                                }
                                                PlaceHolder::Owned(v) => {}
                                            }
                                            //println!(" CIRCLE DIR {:?}",a)
                                        }
                                    }

                                    match &ax.location {
                                        PlaceHolder::Ref(name) => {
                                            match name {
                                                Name::Entity(key) => {
                                                    match table.cartesian_point.get(key) {
                                                        None => {}
                                                        Some(dir) => {
                                                            let mut count=0;
                                                            dir.coordinates.iter().for_each(|d| {
                                                                match count{
                                                                    0 => {center_point.x=d.clone()}
                                                                    1 => {center_point.y=d.clone()}
                                                                    2 => {center_point.z=d.clone()}
                                                                    _ => {}
                                                                }
                                                                count=count+1;
                                                            })
                                                        }
                                                    }
                                                }
                                                Name::Value(_) => {}
                                                Name::ConstantEntity(_) => {}
                                                Name::ConstantValue(_) => {}
                                            }
                                        }
                                        PlaceHolder::Owned(_) => {}
                                    }

                                    match &ax.ref_direction {
                                        None => {}
                                        Some(rd) => {
                                            match rd {
                                                PlaceHolder::Ref(name) => {
                                                    match name {
                                                        Name::Entity(key) => {
                                                            match table.direction.get(key) {
                                                                None => {}
                                                                Some(dir) => {
                                                                    let count=0;
                                                                    dir.direction_ratios.iter().for_each(|d| {
                                                                        match count{
                                                                            0 => {direction_ref.x=d.clone()}
                                                                            1 => {direction_ref.y=d.clone()}
                                                                            2 => {direction_ref.z=d.clone()}
                                                                            _ => {}
                                                                        }
                                                                    })
                                                                }
                                                            }
                                                        }
                                                        Name::Value(_) => {}
                                                        Name::ConstantEntity(_) => {}
                                                        Name::ConstantValue(_) => {}
                                                    }
                                                }
                                                PlaceHolder::Owned(v) => {}
                                            }
                                            //println!(" A {:?}",rd)
                                        }
                                    }
                                }
                            }
                        }
                        Name::Value(_) => {}
                        Name::ConstantEntity(_) => {}
                        Name::ConstantValue(_) => {}
                    }
                }
                PlaceHolder::Owned(_) => {}
            }
            let toro: PipeToroidal = PipeToroidal::new(id, minor_radius,major_radius, direction, direction_ref, center_point);
            ret.push(toro);
        });
        ret
    }
}

const DEMO_PIPE_STEP: &str = "ISO-10303-21;
HEADER;
/* Generated by software containing ST-Developer
 * from STEP Tools, Inc. (www.steptools.com)
 */

FILE_DESCRIPTION(
/* description */ (''),
/* implementation_level */ '2;1');

FILE_NAME(
/* name */ 'C:\\Users\\Beladril\\Desktop\\ROTASYON',
/* time_stamp */ '2020-10-20T17:50:39+03:00',
/* author */ ('Beladril'),
/* organization */ (''),
/* preprocessor_version */ 'ST-DEVELOPER v16.13',
/* originating_system */ 'Autodesk Inventor 2018',
/* authorisation */ '');

FILE_SCHEMA (('AUTOMOTIVE_DESIGN { 1 0 10303 214 3 1 1 }'));
ENDSEC;

DATA;
#10=MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION('',(#13),#361);
#11=SHAPE_REPRESENTATION_RELATIONSHIP('SRR','None',#368,#12);
#12=ADVANCED_BREP_SHAPE_REPRESENTATION('',(#14),#360);
#13=STYLED_ITEM('',(#377),#14);
#14=MANIFOLD_SOLID_BREP('Solid1',#191);
#15=PLANE('',#241);
#16=PLANE('',#242);
#17=TOROIDAL_SURFACE('',#214,77.9999999999999,6.5);
#18=TOROIDAL_SURFACE('',#218,78.,6.5);
#19=TOROIDAL_SURFACE('',#222,78.,6.5);
#20=TOROIDAL_SURFACE('',#229,77.9999999999999,8.);
#21=TOROIDAL_SURFACE('',#233,78.,8.);
#22=TOROIDAL_SURFACE('',#237,78.,8.);
#23=FACE_BOUND('',#56,.T.);
#24=FACE_BOUND('',#58,.T.);
#25=FACE_BOUND('',#60,.T.);
#26=FACE_BOUND('',#62,.T.);
#27=FACE_BOUND('',#64,.T.);
#28=FACE_BOUND('',#66,.T.);
#29=FACE_BOUND('',#68,.T.);
#30=FACE_BOUND('',#70,.T.);
#31=FACE_BOUND('',#72,.T.);
#32=FACE_BOUND('',#74,.T.);
#33=FACE_BOUND('',#76,.T.);
#34=FACE_BOUND('',#78,.T.);
#35=FACE_BOUND('',#80,.T.);
#36=FACE_BOUND('',#82,.T.);
#37=FACE_BOUND('',#84,.T.);
#38=FACE_BOUND('',#86,.T.);
#39=FACE_OUTER_BOUND('',#55,.T.);
#40=FACE_OUTER_BOUND('',#57,.T.);
#41=FACE_OUTER_BOUND('',#59,.T.);
#42=FACE_OUTER_BOUND('',#61,.T.);
#43=FACE_OUTER_BOUND('',#63,.T.);
#44=FACE_OUTER_BOUND('',#65,.T.);
#45=FACE_OUTER_BOUND('',#67,.T.);
#46=FACE_OUTER_BOUND('',#69,.T.);
#47=FACE_OUTER_BOUND('',#71,.T.);
#48=FACE_OUTER_BOUND('',#73,.T.);
#49=FACE_OUTER_BOUND('',#75,.T.);
#50=FACE_OUTER_BOUND('',#77,.T.);
#51=FACE_OUTER_BOUND('',#79,.T.);
#52=FACE_OUTER_BOUND('',#81,.T.);
#53=FACE_OUTER_BOUND('',#83,.T.);
#54=FACE_OUTER_BOUND('',#85,.T.);
#55=EDGE_LOOP('',(#135));
#56=EDGE_LOOP('',(#136));
#57=EDGE_LOOP('',(#137));
#58=EDGE_LOOP('',(#138));
#59=EDGE_LOOP('',(#139));
#60=EDGE_LOOP('',(#140));
#61=EDGE_LOOP('',(#141));
#62=EDGE_LOOP('',(#142));
#63=EDGE_LOOP('',(#143));
#64=EDGE_LOOP('',(#144));
#65=EDGE_LOOP('',(#145));
#66=EDGE_LOOP('',(#146));
#67=EDGE_LOOP('',(#147));
#68=EDGE_LOOP('',(#148));
#69=EDGE_LOOP('',(#149));
#70=EDGE_LOOP('',(#150));
#71=EDGE_LOOP('',(#151));
#72=EDGE_LOOP('',(#152));
#73=EDGE_LOOP('',(#153));
#74=EDGE_LOOP('',(#154));
#75=EDGE_LOOP('',(#155));
#76=EDGE_LOOP('',(#156));
#77=EDGE_LOOP('',(#157));
#78=EDGE_LOOP('',(#158));
#79=EDGE_LOOP('',(#159));
#80=EDGE_LOOP('',(#160));
#81=EDGE_LOOP('',(#161));
#82=EDGE_LOOP('',(#162));
#83=EDGE_LOOP('',(#163));
#84=EDGE_LOOP('',(#164));
#85=EDGE_LOOP('',(#165));
#86=EDGE_LOOP('',(#166));
#87=CIRCLE('',#212,6.5);
#88=CIRCLE('',#213,6.5);
#89=CIRCLE('',#215,6.5);
#90=CIRCLE('',#217,6.5);
#91=CIRCLE('',#219,6.5);
#92=CIRCLE('',#221,6.5);
#93=CIRCLE('',#223,6.5);
#94=CIRCLE('',#225,6.5);
#95=CIRCLE('',#227,8.);
#96=CIRCLE('',#228,8.);
#97=CIRCLE('',#230,8.);
#98=CIRCLE('',#232,8.);
#99=CIRCLE('',#234,8.);
#100=CIRCLE('',#236,8.);
#101=CIRCLE('',#238,8.);
#102=CIRCLE('',#240,8.);
#103=VERTEX_POINT('',#311);
#104=VERTEX_POINT('',#313);
#105=VERTEX_POINT('',#316);
#106=VERTEX_POINT('',#319);
#107=VERTEX_POINT('',#322);
#108=VERTEX_POINT('',#325);
#109=VERTEX_POINT('',#328);
#110=VERTEX_POINT('',#331);
#111=VERTEX_POINT('',#334);
#112=VERTEX_POINT('',#336);
#113=VERTEX_POINT('',#339);
#114=VERTEX_POINT('',#342);
#115=VERTEX_POINT('',#345);
#116=VERTEX_POINT('',#348);
#117=VERTEX_POINT('',#351);
#118=VERTEX_POINT('',#354);
#119=EDGE_CURVE('',#103,#103,#87,.T.);
#120=EDGE_CURVE('',#104,#104,#88,.T.);
#121=EDGE_CURVE('',#105,#105,#89,.T.);
#122=EDGE_CURVE('',#106,#106,#90,.T.);
#123=EDGE_CURVE('',#107,#107,#91,.T.);
#124=EDGE_CURVE('',#108,#108,#92,.T.);
#125=EDGE_CURVE('',#109,#109,#93,.T.);
#126=EDGE_CURVE('',#110,#110,#94,.T.);
#127=EDGE_CURVE('',#111,#111,#95,.T.);
#128=EDGE_CURVE('',#112,#112,#96,.T.);
#129=EDGE_CURVE('',#113,#113,#97,.T.);
#130=EDGE_CURVE('',#114,#114,#98,.T.);
#131=EDGE_CURVE('',#115,#115,#99,.T.);
#132=EDGE_CURVE('',#116,#116,#100,.T.);
#133=EDGE_CURVE('',#117,#117,#101,.T.);
#134=EDGE_CURVE('',#118,#118,#102,.T.);
#135=ORIENTED_EDGE('',*,*,#119,.F.);
#136=ORIENTED_EDGE('',*,*,#120,.F.);
#137=ORIENTED_EDGE('',*,*,#120,.T.);
#138=ORIENTED_EDGE('',*,*,#121,.F.);
#139=ORIENTED_EDGE('',*,*,#121,.T.);
#140=ORIENTED_EDGE('',*,*,#122,.F.);
#141=ORIENTED_EDGE('',*,*,#122,.T.);
#142=ORIENTED_EDGE('',*,*,#123,.F.);
#143=ORIENTED_EDGE('',*,*,#123,.T.);
#144=ORIENTED_EDGE('',*,*,#124,.F.);
#145=ORIENTED_EDGE('',*,*,#124,.T.);
#146=ORIENTED_EDGE('',*,*,#125,.F.);
#147=ORIENTED_EDGE('',*,*,#125,.T.);
#148=ORIENTED_EDGE('',*,*,#126,.F.);
#149=ORIENTED_EDGE('',*,*,#127,.F.);
#150=ORIENTED_EDGE('',*,*,#128,.T.);
#151=ORIENTED_EDGE('',*,*,#128,.F.);
#152=ORIENTED_EDGE('',*,*,#129,.T.);
#153=ORIENTED_EDGE('',*,*,#129,.F.);
#154=ORIENTED_EDGE('',*,*,#130,.T.);
#155=ORIENTED_EDGE('',*,*,#130,.F.);
#156=ORIENTED_EDGE('',*,*,#131,.T.);
#157=ORIENTED_EDGE('',*,*,#131,.F.);
#158=ORIENTED_EDGE('',*,*,#132,.T.);
#159=ORIENTED_EDGE('',*,*,#132,.F.);
#160=ORIENTED_EDGE('',*,*,#133,.T.);
#161=ORIENTED_EDGE('',*,*,#133,.F.);
#162=ORIENTED_EDGE('',*,*,#134,.T.);
#163=ORIENTED_EDGE('',*,*,#127,.T.);
#164=ORIENTED_EDGE('',*,*,#119,.T.);
#165=ORIENTED_EDGE('',*,*,#134,.F.);
#166=ORIENTED_EDGE('',*,*,#126,.T.);
#167=CYLINDRICAL_SURFACE('',#211,6.5);
#168=CYLINDRICAL_SURFACE('',#216,6.5);
#169=CYLINDRICAL_SURFACE('',#220,6.5);
#170=CYLINDRICAL_SURFACE('',#224,6.5);
#171=CYLINDRICAL_SURFACE('',#226,8.);
#172=CYLINDRICAL_SURFACE('',#231,8.);
#173=CYLINDRICAL_SURFACE('',#235,8.);
#174=CYLINDRICAL_SURFACE('',#239,8.);
#175=ADVANCED_FACE('',(#39,#23),#167,.F.);
#176=ADVANCED_FACE('',(#40,#24),#17,.F.);
#177=ADVANCED_FACE('',(#41,#25),#168,.F.);
#178=ADVANCED_FACE('',(#42,#26),#18,.F.);
#179=ADVANCED_FACE('',(#43,#27),#169,.F.);
#180=ADVANCED_FACE('',(#44,#28),#19,.F.);
#181=ADVANCED_FACE('',(#45,#29),#170,.F.);
#182=ADVANCED_FACE('',(#46,#30),#171,.T.);
#183=ADVANCED_FACE('',(#47,#31),#20,.T.);
#184=ADVANCED_FACE('',(#48,#32),#172,.T.);
#185=ADVANCED_FACE('',(#49,#33),#21,.T.);
#186=ADVANCED_FACE('',(#50,#34),#173,.T.);
#187=ADVANCED_FACE('',(#51,#35),#22,.T.);
#188=ADVANCED_FACE('',(#52,#36),#174,.T.);
#189=ADVANCED_FACE('',(#53,#37),#15,.T.);
#190=ADVANCED_FACE('',(#54,#38),#16,.F.);
#191=CLOSED_SHELL('',(#175,#176,#177,#178,#179,#180,#181,#182,#183,#184,
#185,#186,#187,#188,#189,#190));
#192=DERIVED_UNIT_ELEMENT(#194,0.);
#193=DERIVED_UNIT_ELEMENT(#363,0.);
#194=(
MASS_UNIT()
NAMED_UNIT(*)
SI_UNIT($,.GRAM.)
);
#195=DERIVED_UNIT((#192,#193));
#196=MEASURE_REPRESENTATION_ITEM('density measure',
POSITIVE_RATIO_MEASURE(1.),#195);
#197=PROPERTY_DEFINITION_REPRESENTATION(#202,#199);
#198=PROPERTY_DEFINITION_REPRESENTATION(#203,#200);
#199=REPRESENTATION('material name',(#201),#360);
#200=REPRESENTATION('density',(#196),#360);
#201=DESCRIPTIVE_REPRESENTATION_ITEM('Generic','Generic');
#202=PROPERTY_DEFINITION('material property','material name',#370);
#203=PROPERTY_DEFINITION('material property','density of part',#370);
#204=DATE_TIME_ROLE('creation_date');
#205=APPLIED_DATE_AND_TIME_ASSIGNMENT(#206,#204,(#370));
#206=DATE_AND_TIME(#207,#208);
#207=CALENDAR_DATE(2020,20,10);
#208=LOCAL_TIME(14,46,46.,#209);
#209=COORDINATED_UNIVERSAL_TIME_OFFSET(0,0,.BEHIND.);
#210=AXIS2_PLACEMENT_3D('placement',#309,#243,#244);
#211=AXIS2_PLACEMENT_3D('',#310,#245,#246);
#212=AXIS2_PLACEMENT_3D('',#312,#247,#248);
#213=AXIS2_PLACEMENT_3D('',#314,#249,#250);
#214=AXIS2_PLACEMENT_3D('',#315,#251,#252);
#215=AXIS2_PLACEMENT_3D('',#317,#253,#254);
#216=AXIS2_PLACEMENT_3D('',#318,#255,#256);
#217=AXIS2_PLACEMENT_3D('',#320,#257,#258);
#218=AXIS2_PLACEMENT_3D('',#321,#259,#260);
#219=AXIS2_PLACEMENT_3D('',#323,#261,#262);
#220=AXIS2_PLACEMENT_3D('',#324,#263,#264);
#221=AXIS2_PLACEMENT_3D('',#326,#265,#266);
#222=AXIS2_PLACEMENT_3D('',#327,#267,#268);
#223=AXIS2_PLACEMENT_3D('',#329,#269,#270);
#224=AXIS2_PLACEMENT_3D('',#330,#271,#272);
#225=AXIS2_PLACEMENT_3D('',#332,#273,#274);
#226=AXIS2_PLACEMENT_3D('',#333,#275,#276);
#227=AXIS2_PLACEMENT_3D('',#335,#277,#278);
#228=AXIS2_PLACEMENT_3D('',#337,#279,#280);
#229=AXIS2_PLACEMENT_3D('',#338,#281,#282);
#230=AXIS2_PLACEMENT_3D('',#340,#283,#284);
#231=AXIS2_PLACEMENT_3D('',#341,#285,#286);
#232=AXIS2_PLACEMENT_3D('',#343,#287,#288);
#233=AXIS2_PLACEMENT_3D('',#344,#289,#290);
#234=AXIS2_PLACEMENT_3D('',#346,#291,#292);
#235=AXIS2_PLACEMENT_3D('',#347,#293,#294);
#236=AXIS2_PLACEMENT_3D('',#349,#295,#296);
#237=AXIS2_PLACEMENT_3D('',#350,#297,#298);
#238=AXIS2_PLACEMENT_3D('',#352,#299,#300);
#239=AXIS2_PLACEMENT_3D('',#353,#301,#302);
#240=AXIS2_PLACEMENT_3D('',#355,#303,#304);
#241=AXIS2_PLACEMENT_3D('',#356,#305,#306);
#242=AXIS2_PLACEMENT_3D('',#357,#307,#308);
#243=DIRECTION('axis',(0.,0.,1.));
#244=DIRECTION('refdir',(1.,0.,0.));
#245=DIRECTION('center_axis',(-5.55111512312578E-17,0.866025403784439,0.5));
#246=DIRECTION('ref_axis',(0.866025403784439,0.25,-0.433012701892219));
#247=DIRECTION('center_axis',(-5.55111512312578E-17,-0.866025403784438,
-0.500000000000001));
#248=DIRECTION('ref_axis',(0.866025403784439,0.25,-0.433012701892219));
#249=DIRECTION('center_axis',(-3.63373587047937E-17,0.866025403784438,0.5));
#250=DIRECTION('ref_axis',(0.866025403784439,0.25,-0.433012701892219));
#251=DIRECTION('center_axis',(0.,0.5,-0.866025403784439));
#252=DIRECTION('ref_axis',(-1.,0.,0.));
#253=DIRECTION('center_axis',(-1.,0.,4.9789962505148E-17));
#254=DIRECTION('ref_axis',(0.,1.,0.));
#255=DIRECTION('center_axis',(-1.,0.,0.));
#256=DIRECTION('ref_axis',(0.,1.,0.));
#257=DIRECTION('center_axis',(-1.,0.,4.9789962505148E-17));
#258=DIRECTION('ref_axis',(0.,1.,0.));
#259=DIRECTION('center_axis',(0.,1.,0.));
#260=DIRECTION('ref_axis',(0.,0.,1.));
#261=DIRECTION('center_axis',(-1.11022302462516E-16,0.,-1.));
#262=DIRECTION('ref_axis',(0.,1.,0.));
#263=DIRECTION('center_axis',(0.,0.,-1.));
#264=DIRECTION('ref_axis',(0.,1.,0.));
#265=DIRECTION('center_axis',(-1.11022302462516E-16,0.,-1.));
#266=DIRECTION('ref_axis',(0.,1.,0.));
#267=DIRECTION('center_axis',(0.,1.,0.));
#268=DIRECTION('ref_axis',(0.,0.,1.));
#269=DIRECTION('center_axis',(0.707106781186547,0.,-0.707106781186548));
#270=DIRECTION('ref_axis',(0.,1.,0.));
#271=DIRECTION('center_axis',(0.707106781186547,0.,-0.707106781186548));
#272=DIRECTION('ref_axis',(0.,1.,0.));
#273=DIRECTION('center_axis',(0.707106781186547,0.,-0.707106781186548));
#274=DIRECTION('ref_axis',(0.,1.,0.));
#275=DIRECTION('center_axis',(-5.55111512312578E-17,0.866025403784439,0.5));
#276=DIRECTION('ref_axis',(0.866025403784439,0.25,-0.43301270189222));
#277=DIRECTION('center_axis',(-3.63373587047937E-17,0.866025403784438,0.5));
#278=DIRECTION('ref_axis',(0.866025403784438,0.25,-0.433012701892219));
#279=DIRECTION('center_axis',(-3.63373587047937E-17,0.866025403784438,0.5));
#280=DIRECTION('ref_axis',(0.866025403784439,0.25,-0.433012701892219));
#281=DIRECTION('center_axis',(0.,0.5,-0.866025403784439));
#282=DIRECTION('ref_axis',(-1.,0.,0.));
#283=DIRECTION('center_axis',(-1.,0.,4.9789962505148E-17));
#284=DIRECTION('ref_axis',(0.,1.,0.));
#285=DIRECTION('center_axis',(-1.,0.,0.));
#286=DIRECTION('ref_axis',(0.,1.,0.));
#287=DIRECTION('center_axis',(-1.,0.,4.9789962505148E-17));
#288=DIRECTION('ref_axis',(0.,1.,0.));
#289=DIRECTION('center_axis',(0.,1.,0.));
#290=DIRECTION('ref_axis',(0.,0.,1.));
#291=DIRECTION('center_axis',(-1.11022302462516E-16,0.,-1.));
#292=DIRECTION('ref_axis',(0.,1.,0.));
#293=DIRECTION('center_axis',(0.,0.,-1.));
#294=DIRECTION('ref_axis',(0.,1.,0.));
#295=DIRECTION('center_axis',(-1.11022302462516E-16,0.,-1.));
#296=DIRECTION('ref_axis',(0.,1.,0.));
#297=DIRECTION('center_axis',(0.,1.,0.));
#298=DIRECTION('ref_axis',(0.,0.,1.));
#299=DIRECTION('center_axis',(0.707106781186547,0.,-0.707106781186548));
#300=DIRECTION('ref_axis',(0.,1.,0.));
#301=DIRECTION('center_axis',(0.707106781186547,0.,-0.707106781186548));
#302=DIRECTION('ref_axis',(0.,1.,0.));
#303=DIRECTION('center_axis',(0.707106781186547,0.,-0.707106781186548));
#304=DIRECTION('ref_axis',(0.,1.,0.));
#305=DIRECTION('center_axis',(5.55111512312578E-17,0.866025403784438,0.500000000000001));
#306=DIRECTION('ref_axis',(0.5,-0.433012701892219,0.75));
#307=DIRECTION('center_axis',(0.707106781186547,0.,-0.707106781186548));
#308=DIRECTION('ref_axis',(-0.707106781186548,0.,-0.707106781186547));
#309=CARTESIAN_POINT('',(0.,0.,0.));
#310=CARTESIAN_POINT('Origin',(-111.927830462912,67.5499814951862,-217.));
#311=CARTESIAN_POINT('',(-117.556995587511,133.418138285303,-175.218225196469));
#312=CARTESIAN_POINT('Origin',(-111.927830462912,135.043138285303,-178.032807758769));
#313=CARTESIAN_POINT('',(-117.556995587511,65.9249814951862,-214.1854174377));
#314=CARTESIAN_POINT('Origin',(-111.927830462912,67.5499814951862,-217.));
#315=CARTESIAN_POINT('Origin',(-33.9278304629119,67.5499814951862,-217.));
#316=CARTESIAN_POINT('',(-33.927830462912,-6.5,-256.));
#317=CARTESIAN_POINT('Origin',(-33.927830462912,0.,-256.));
#318=CARTESIAN_POINT('Origin',(71.9999999999999,0.,-256.));
#319=CARTESIAN_POINT('',(71.9999999999999,-6.5,-256.));
#320=CARTESIAN_POINT('Origin',(71.9999999999999,0.,-256.));
#321=CARTESIAN_POINT('Origin',(72.,0.,-178.));
#322=CARTESIAN_POINT('',(150.,-6.5,-178.));
#323=CARTESIAN_POINT('Origin',(150.,0.,-178.));
#324=CARTESIAN_POINT('Origin',(150.,0.,-32.3086578651014));
#325=CARTESIAN_POINT('',(150.,-6.5,-32.3086578651014));
#326=CARTESIAN_POINT('Origin',(150.,0.,-32.3086578651014));
#327=CARTESIAN_POINT('Origin',(72.,0.,-32.3086578651014));
#328=CARTESIAN_POINT('',(127.154328932551,-6.5,22.8456710674493));
#329=CARTESIAN_POINT('Origin',(127.154328932551,0.,22.8456710674493));
#330=CARTESIAN_POINT('Origin',(24.1349929487945,0.,125.865007051205));
#331=CARTESIAN_POINT('',(24.1349929487945,-6.5,125.865007051205));
#332=CARTESIAN_POINT('Origin',(24.1349929487945,0.,125.865007051205));
#333=CARTESIAN_POINT('Origin',(-111.927830462912,67.5499814951862,-217.));
#334=CARTESIAN_POINT('',(-118.856033693187,133.043138285303,-174.568706143631));
#335=CARTESIAN_POINT('Origin',(-111.927830462912,135.043138285303,-178.032807758769));
#336=CARTESIAN_POINT('',(-118.856033693187,65.5499814951862,-213.535898384862));
#337=CARTESIAN_POINT('Origin',(-111.927830462912,67.5499814951862,-217.));
#338=CARTESIAN_POINT('Origin',(-33.9278304629119,67.5499814951862,-217.));
#339=CARTESIAN_POINT('',(-33.927830462912,-8.,-256.));
#340=CARTESIAN_POINT('Origin',(-33.927830462912,0.,-256.));
#341=CARTESIAN_POINT('Origin',(71.9999999999999,0.,-256.));
#342=CARTESIAN_POINT('',(71.9999999999999,-8.,-256.));
#343=CARTESIAN_POINT('Origin',(71.9999999999999,0.,-256.));
#344=CARTESIAN_POINT('Origin',(72.,0.,-178.));
#345=CARTESIAN_POINT('',(150.,-8.,-178.));
#346=CARTESIAN_POINT('Origin',(150.,0.,-178.));
#347=CARTESIAN_POINT('Origin',(150.,0.,-32.3086578651014));
#348=CARTESIAN_POINT('',(150.,-8.,-32.3086578651014));
#349=CARTESIAN_POINT('Origin',(150.,0.,-32.3086578651014));
#350=CARTESIAN_POINT('Origin',(72.,0.,-32.3086578651014));
#351=CARTESIAN_POINT('',(127.154328932551,-8.,22.8456710674493));
#352=CARTESIAN_POINT('Origin',(127.154328932551,0.,22.8456710674493));
#353=CARTESIAN_POINT('Origin',(24.1349929487945,0.,125.865007051205));
#354=CARTESIAN_POINT('',(24.1349929487945,-8.,125.865007051205));
#355=CARTESIAN_POINT('Origin',(24.1349929487945,0.,125.865007051205));
#356=CARTESIAN_POINT('Origin',(-111.927830462912,135.043138285303,-178.032807758769));
#357=CARTESIAN_POINT('Origin',(24.1349929487945,5.44236046870305E-16,125.865007051205));
#358=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(0.01),#362,
'DISTANCE_ACCURACY_VALUE',
'Maximum model space distance between geometric entities at asserted c
onnectivities');
#359=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(0.01),#362,
'DISTANCE_ACCURACY_VALUE',
'Maximum model space distance between geometric entities at asserted c
onnectivities');
#360=(
GEOMETRIC_REPRESENTATION_CONTEXT(3)
GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#358))
GLOBAL_UNIT_ASSIGNED_CONTEXT((#362,#365,#364))
REPRESENTATION_CONTEXT('','3D')
);
#361=(
GEOMETRIC_REPRESENTATION_CONTEXT(3)
GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#359))
GLOBAL_UNIT_ASSIGNED_CONTEXT((#362,#365,#364))
REPRESENTATION_CONTEXT('','3D')
);
#362=(
LENGTH_UNIT()
NAMED_UNIT(*)
SI_UNIT(.MILLI.,.METRE.)
);
#363=(
LENGTH_UNIT()
NAMED_UNIT(*)
SI_UNIT(.CENTI.,.METRE.)
);
#364=(
NAMED_UNIT(*)
SI_UNIT($,.STERADIAN.)
SOLID_ANGLE_UNIT()
);
#365=(
NAMED_UNIT(*)
PLANE_ANGLE_UNIT()
SI_UNIT($,.RADIAN.)
);
#366=SHAPE_DEFINITION_REPRESENTATION(#367,#368);
#367=PRODUCT_DEFINITION_SHAPE('',$,#370);
#368=SHAPE_REPRESENTATION('',(#210),#360);
#369=PRODUCT_DEFINITION_CONTEXT('part definition',#374,'design');
#370=PRODUCT_DEFINITION('Part1','Part1',#371,#369);
#371=PRODUCT_DEFINITION_FORMATION('',$,#376);
#372=PRODUCT_RELATED_PRODUCT_CATEGORY('Part1','Part1',(#376));
#373=APPLICATION_PROTOCOL_DEFINITION('international standard',
'automotive_design',2009,#374);
#374=APPLICATION_CONTEXT(
'Core Data for Automotive Mechanical Design Process');
#375=PRODUCT_CONTEXT('part definition',#374,'mechanical');
#376=PRODUCT('Part1','Part1',$,(#375));
#377=PRESENTATION_STYLE_ASSIGNMENT((#378));
#378=SURFACE_STYLE_USAGE(.BOTH.,#379);
#379=SURFACE_SIDE_STYLE('',(#380));
#380=SURFACE_STYLE_FILL_AREA(#381);
#381=FILL_AREA_STYLE('',(#382));
#382=FILL_AREA_STYLE_COLOUR('',#383);
#383=COLOUR_RGB('',0.749019607843137,0.749019607843137,0.749019607843137);
ENDSEC;
END-ISO-10303-21;
";