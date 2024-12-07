use crate::device::graphics::{GlobalState, Graphics};
use cgmath::num_traits::abs;
use cgmath::{
    perspective, InnerSpace, Matrix, Matrix4, MetricSpace, Point3, Quaternion, Rad, Rotation,
    Rotation3, SquareMatrix, Vector3,
};
use log::warn;
use shipyard::{Unique, UniqueViewMut};
use std::f32::consts::PI;
use std::ops::{Add, Sub};
use truck_base::bounding_box::BoundingBox;
use truck_geometry::prelude::Plane;

pub const SHIP_FORWARD: Vector3<f32> = Vector3::new(1.0, 0.0, 0.0);
pub const SHIP_RIGHT: Vector3<f32> = Vector3::new(0.0, -1.0, 0.0);
pub const SHIP_UP: Vector3<f32> = Vector3::new(0.0, 0.0, 1.0);
pub const SHIP_RIGHT64: Vector3<f64> = Vector3::new(0.0, -1.0, 0.0);

const MOUSE_SENSITIVITY_HORIZONTAL: f32 = 0.01;
const MOUSE_SENSITIVITY_VERTICAL: f32 = 0.01;
const ZOOM_SENSITIVITY: f32 = 10.0;
const CAMERA_FOCUS: f32 = 3000.0;
const OFFSET_MULTIPLIER: f32 = 10.0;
#[derive(Unique)]
pub struct Camera {
    pub is_dirty: bool,
    pub eye: Point3<f32>,
    pub head_up: Vector3<f32>,
    pub head_forward: Vector3<f32>,
    pub head_right: Vector3<f32>,
    pub vp_matrix: Matrix4<f32>,
    view: Matrix4<f32>,
    proj: Matrix4<f32>,
    pub n_matrix: Matrix4<f32>,
    pub near: f32,
    pub far: f32,
    pub fovy: Rad<f32>,
    pub aspect: f32,
    pub dx: f32,
    pub dy: f32,
    yaw: f32,
    pitch: f32,
    pub tot_bbx: BoundingBox<Point3<f64>>,
    center_p: Point3<f64>,
}

impl Camera {
    pub fn new(fovy: Rad<f32>, aspect: f32, near: f32, far: f32) -> Self {
        let eye = Point3::new(0.0, 0.0, 0.0);
        Self {
            is_dirty: true,
            eye: eye,
            head_up: SHIP_UP,
            head_forward: SHIP_FORWARD,
            head_right: SHIP_RIGHT,
            vp_matrix: Matrix4::identity(),
            view: Matrix4::identity(),
            proj: Matrix4::identity(),
            n_matrix: Matrix4::identity(),
            near: near,
            far: far ,
            fovy: fovy,
            aspect: aspect,
            dx: 0.0,
            dy: 0.0,
            yaw: 0.0,
            pitch: 0.0,
            tot_bbx: Default::default(),
            center_p: Point3::new(0.0, 0.0, 0.0),
        }
    }
    pub fn default() -> Self {
        let cam = Camera::new(Rad(0.1), 800.0 / 600.0, 0.001, 200000.0);
        cam
    }
    pub fn resize(&mut self, w: u32, h: u32) {
        if w > 0 && h > 0 {
            self.aspect = w as f32 / h as f32;
            self.update();
        }
    }
    pub fn set_tot_bbx(&mut self, bbxs_in: BoundingBox<truck_base::cgmath64::Point3>) {
        self.tot_bbx = Default::default();
        self.tot_bbx += (bbxs_in.clone());
        self.center_p= bbxs_in.center();

        let center: Point3<f32> = Point3::new(
            self.center_p.x as f32,
            self.center_p.y as f32,
            self.center_p.z as f32,
        );
        let dist = center.distance(self.eye);
        warn!("BBX dist {:?}", dist);
        self.eye = center - self.head_forward * dist;
        //self.calculate_focus();
        //self.update();
    }
    pub fn set_up_dir(&mut self, up_dir: &Vector3<f64>) {
        self.head_up.z = self.head_up.z * up_dir.z as f32;
        //gs.v_up_orign
    }
    pub fn move_camera_to_bbx_limits(&mut self) {
        //self.center_p = self.tot_bbx.center();
        //self.center_p = Point3::new(0.0, 0.0, 0.0);

        self.reset_pos();
        let cp = self.center_p;
        let ep = self.tot_bbx.max();

        let head_forward = cp.sub(ep).normalize();
        let head_forward32: Vector3<f32> = Vector3::new(
            head_forward.x as f32,
            head_forward.y as f32,
            head_forward.z as f32,
        );

        let right_point = ep.clone() + SHIP_RIGHT64 * 100.0;
        let plane = Plane::new(ep, cp, right_point);
        let new_up = plane.normal().normalize();
        let new_right = new_up.cross(head_forward).normalize();

        let new_up32: Vector3<f32> =
            Vector3::new(new_up.x as f32, new_up.y as f32, new_up.z as f32);
        let new_right32: Vector3<f32> =
            Vector3::new(new_right.x as f32, new_right.y as f32, new_right.z as f32);
        //let p: Point3<f32> = Point3::new(ep.x as f32, ep.y as f32, ep.z as f32);

        let new_eye: Point3<f64> =
            self.center_p - head_forward * self.tot_bbx.diameter() * OFFSET_MULTIPLIER as f64;
        self.eye = Point3::new(new_eye.x as f32, new_eye.y as f32, new_eye.z as f32);
        //self.eye.clone_from(&p);
        self.head_forward.clone_from(&head_forward32);
        self.head_up.clone().clone_from(&new_up32);
        self.head_right.clone().clone_from(&new_right32);

        self.is_dirty = true;
        self.update();
    }


    pub fn reset_pos(&mut self) {
        /*  self.yaw = 0.0;
        self.pitch = 0.0;
        self.update();*/
    }
    pub fn get_mvp_buffer(&mut self) -> &[f32; 16] {
        self.is_dirty = false;
        let view_projection_ref: &[f32; 16] = self.vp_matrix.as_ref();
        view_projection_ref
    }
    pub fn get_norm_buffer(&self) -> &[f32; 16] {
        let view_projection_ref: &[f32; 16] = self.n_matrix.as_ref();
        view_projection_ref
    }
    pub fn get_forward_dir_buffer(&self) -> &[f32; 3] {
        self.head_forward.as_ref()
    }
    pub fn update_mouse(&mut self, dx_in: f32, dy_in: f32) {
        self.yaw += -dx_in * MOUSE_SENSITIVITY_HORIZONTAL;
        self.pitch += -dy_in * MOUSE_SENSITIVITY_VERTICAL;
        if abs(self.yaw) > PI * 2.0 {
            self.yaw = 0.0
        }
        if abs(self.pitch) > PI * 2.0 {
            self.pitch = 0.0
        }
        self.rotate();
    }
/*    pub fn reset_cp_to_bbx_center(&mut self) {
        self.center_p = self.tot_bbx.center();
        self.update();
    }*/
/*    pub fn move_to_anim_pos(&mut self, pipe_len: f64, up_dir: &Vector3<f64>) {
        self.center_p = Point3::new(0.0, 0.0, 0.0);
        let center: Point3<f32> = Point3::new(
            self.center_p.x as f32,
            self.center_p.y as f32,
            self.center_p.z as f32,
        );
        self.eye.x= -pipe_len as f32;
        self.eye.y= pipe_len as f32;
        self.eye.z= pipe_len as f32;

        self.head_forward= center.sub(self.eye);
        
        self.update();
    }*/

    
    pub fn zoom(&mut self, delta: f32) {
        let incr = self.tot_bbx.diameter() as f32 * 0.3 * delta;
        self.eye = self.eye + self.head_forward * incr;
        self.update();
    }
    fn update(&mut self) {
        self.view = Matrix4::look_to_rh(self.eye, self.head_forward, self.head_up);
        self.proj = perspective(self.fovy, self.aspect, self.near, self.far);
        self.vp_matrix = self.proj * self.view;
        self.n_matrix = self.view.transpose();
    }
    fn rotate(&mut self) {
        let _up: Vector3<f32> = self.head_up.clone();
        let _right: Vector3<f32> = self.head_right.clone();
        let new_forward_rot = Quaternion::from_axis_angle(SHIP_UP, Rad(self.yaw)).normalize();
        let new_forward_lr: Vector3<f32> = new_forward_rot.rotate_vector(SHIP_FORWARD);
        let new_right: Vector3<f32> = new_forward_lr.cross(SHIP_UP);

        let new_right_rot = Quaternion::from_axis_angle(new_right, Rad(self.pitch)).normalize();
        let new_forward: Vector3<f32> = new_right_rot.rotate_vector(new_forward_lr);
        let new_up: Vector3<f32> = new_right.cross(new_forward);

        let center: Point3<f32> = Point3::new(
            self.center_p.x as f32,
            self.center_p.y as f32,
            self.center_p.z as f32,
        );
        let dist = center.distance(self.eye);
        self.eye = center - new_forward * dist;

        self.head_forward = new_forward;
        self.head_right = new_right;
        self.head_up = new_up;

        self.update();
    }
}

pub fn update_camera_by_mouse(
    delta: (f64, f64),
    mut graphics: UniqueViewMut<Graphics>,
    gs: UniqueViewMut<GlobalState>,
) {
    if (gs.is_right_mouse_pressed) {
        graphics.camera.yaw += -delta.0 as f32 * MOUSE_SENSITIVITY_HORIZONTAL;
        graphics.camera.pitch += -delta.1 as f32 * MOUSE_SENSITIVITY_VERTICAL;
        if abs(graphics.camera.yaw) > PI * 2.0 {
            graphics.camera.yaw = 0.0
        }
        if abs(graphics.camera.pitch) > PI * 2.0 {
            graphics.camera.pitch = 0.0
        }
        graphics.camera.rotate();
    }
}

pub fn camera_zoom(
    delta: f32,
    mut graphics: UniqueViewMut<Graphics>,
    gs: UniqueViewMut<GlobalState>,
) {
    graphics.camera.zoom(delta);
}
