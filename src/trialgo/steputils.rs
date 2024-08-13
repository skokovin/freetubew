use std::fs::File;
use std::io::BufWriter;
use std::ops::{Mul, Sub};
use cgmath::{InnerSpace, MetricSpace};
use rand::random;
use ruststep::tables::PlaceHolder;
use ruststep::tables::PlaceHolder::Ref;
use truck_base::cgmath64::{Point3, Vector3};
use truck_geometry::nurbs::BSplineCurve;
use truck_geometry::prelude::Plane;
use truck_geotrait::ParametricCurve;
use truck_stepio::r#in::{Axis2Placement3dHolder, Axis2PlacementHolder, BSplineCurveWithKnots, CartesianPoint, CartesianPointHolder, CurveAnyHolder, DirectionHolder, FaceBoundHolder, Table, VectorHolder, VertexPointHolder};
use crate::trialgo::bendpipe::{BendToro, MainCircle, MainCylinder};
use crate::trialgo::analyzepl::{DIVIDER, STPSCALE, TOLE};
use crate::trialgo::{name_to_id, project_point_to_vec, round_by_dec};

pub fn extract_position(t: &Table, pos: &PlaceHolder<Axis2PlacementHolder>) -> (Option<Point3>, Option<Vector3>, Option<Vector3>) {
    match pos {
        Ref(name) => {
            match t.axis2_placement_3d.get(&name_to_id(name.clone())) {
                None => { (None, None, None) }
                Some(p) => {
                    let dir_ref: Option<Vector3> = extract_direction(t, &p.ref_direction);
                    let dir: Option<Vector3> = extract_direction(t, &p.axis);
                    let loc: Option<Point3> = extract_cartesian_point(t, &p.location);
                    (loc, dir, dir_ref)
                }
            }
        }
        PlaceHolder::Owned(_) => { (None, None, None) }
    }
}
pub fn extract_position3d(t: &Table, pos: &PlaceHolder<Axis2Placement3dHolder>) -> (Option<Point3>, Option<Vector3>, Option<Vector3>) {
    match pos {
        Ref(name) => {
            match t.axis2_placement_3d.get(&name_to_id(name.clone())) {
                None => { (None, None, None) }
                Some(p) => {
                    let dir_ref: Option<Vector3> = extract_direction(t, &p.ref_direction);
                    let dir: Option<Vector3> = extract_direction(t, &p.axis);
                    let loc: Option<Point3> = extract_cartesian_point(t, &p.location);
                    (loc, dir, dir_ref)
                }
            }
        }
        PlaceHolder::Owned(_) => { (None, None, None) }
    }
}
pub fn extract_direction(t: &Table, _pos: &Option<PlaceHolder<DirectionHolder>>) -> Option<Vector3> {
    match _pos {
        None => { None }
        Some(pos) => {
            match pos {
                Ref(name) => {
                    match t.direction.get(&name_to_id(name.clone())) {
                        None => { None }
                        Some(p) => {
                            let x = p.direction_ratios[0];
                            let y = p.direction_ratios[1];
                            let z = p.direction_ratios[2];
                            Some(unsafe { Vector3::new(x, y, z).mul(STPSCALE).normalize() })
                        }
                    }
                }
                PlaceHolder::Owned(_) => { None }
            }
        }
    }
}
pub fn extract_vector(t: &Table, pos: &PlaceHolder<VectorHolder>) -> Vector3 {
    match pos {
        Ref(name) => {
            match t.vector.get(&name_to_id(name.clone())) {
                None => { Vector3::new(f64::MIN, f64::MIN, f64::MIN) }
                Some(p) => {
                    let magnitude = unsafe { p.magnitude * STPSCALE };
                    match extract_direction(t, &Some(p.orientation.clone())) {
                        None => { Vector3::new(f64::MIN, f64::MIN, f64::MIN) }
                        Some(v) => {
                            let vect: Vector3 = v.mul(magnitude);
                            vect
                        }
                    }
                }
            }
        }
        PlaceHolder::Owned(_) => { Vector3::new(f64::MIN, f64::MIN, f64::MIN) }
    }
}
pub fn extract_vertex(t: &Table, vertex: &PlaceHolder<VertexPointHolder>) -> Option<Point3> {
    match vertex {
        Ref(name) => {
            match t.vertex_point.get(&name_to_id(name.clone())) {
                None => { None }
                Some(vtx) => {
                    extract_cartesian_point(t, &vtx.vertex_geometry)
                }
            }
        }
        PlaceHolder::Owned(_) => { None }
    }
}
pub fn extract_cartesian_point(t: &Table, point: &PlaceHolder<CartesianPointHolder>) -> Option<Point3> {
    match point {
        Ref(name) => {
            match t.cartesian_point.get(&name_to_id(name.clone())) {
                None => { None }
                Some(p) => {
                    let x = unsafe { p.coordinates[0] * STPSCALE };
                    let y = unsafe { p.coordinates[1] * STPSCALE };
                    let z = unsafe { p.coordinates[2] * STPSCALE };
                    Some(Point3::new(x, y, z))
                }
            }
        }
        PlaceHolder::Owned(_) => { None }
    }
}
pub fn extract_circles_bounds(table: &Table, bounds: &Vec<PlaceHolder<FaceBoundHolder>>, cyl_sp: &Point3, cyl_dir: &Vector3, cyl_id: u64, surf_radius: f64) -> Option<MainCylinder> {
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
                                                                                            let circle_r = circle.radius * unsafe { STPSCALE };
                                                                                            let (loc, dir, dir_rad) = extract_position(table, &circle.position);
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
                                                                                            &spline.control_points_list.iter().for_each(|cp| {
                                                                                                let pnt = extract_cartesian_point(&table, &cp).unwrap();
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
                                                                                            let (loc, dir, dir_rad) = extract_position(table, &ellipse.position);
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

                                                                                    if (!found) {

                                                                                    }

                                                                                    //println!("name ID {:?} ", &name_to_id(name.clone()))
                                                                                }
                                                                                PlaceHolder::Owned(_) => {}
                                                                            }
                                                                            let sp = extract_vertex(&table, &c.edge_start).unwrap();
                                                                            let ep = extract_vertex(&table, &c.edge_end).unwrap();
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
            triangles: vec![],
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
    pipe_radius: f64) -> Option<BendToro>
{
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
                                                                                            let circle_r = circle.radius * unsafe { STPSCALE };
                                                                                            let (loc, dir, dir_rad) = extract_position(table, &circle.position);
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
                                                                                            &spline.control_points_list.iter().for_each(|cp| {
                                                                                                let pnt = extract_cartesian_point(&table, &cp).unwrap();
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
                                                                            let sp = extract_vertex(&table, &c.edge_start).unwrap();
                                                                            let ep = extract_vertex(&table, &c.edge_end).unwrap();
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
        let dist = candidates_remove_dubs[0].loc.distance(candidates_remove_dubs[1].loc);
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
                triangles: vec![],
            };
            Some(bend)
        } else {
            None
        }
    } else {
        None
    }
}

pub fn extract_plane_points(table: &Table) -> Vec<Point3> {
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
                                                                                                                    let sp = extract_vertex(&table, &c.edge_start).unwrap();
                                                                                                                    let ep = extract_vertex(&table, &c.edge_end).unwrap();
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

