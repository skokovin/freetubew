use std::f32::consts::PI;
use std::ops::Sub;
use std::rc::Rc;
use cgmath::{InnerSpace, Matrix4, MetricSpace, Point3, Quaternion, Rad, Rotation, Rotation3, SquareMatrix, Vector3};
use cgmath::num_traits::abs;
use parking_lot::RwLock;
use truck_base::bounding_box::BoundingBox;
use truck_geometry::prelude::Plane;
use crate::device::Z_FIGHTING_FACTOR;

pub const SHIP_FORWARD: Vector3<f32> = Vector3::new(1.0, 0.0, 0.0);
pub const SHIP_RIGHT: Vector3<f32> = Vector3::new(0.0, -1.0, 0.0);
pub const SHIP_UP: Vector3<f32> = Vector3::new(0.0, 0.0, 1.0);

pub const SHIP_FORWARD64: Vector3<f64> = Vector3::new(1.0, 0.0, 0.0);
pub const SHIP_RIGHT64: Vector3<f64> = Vector3::new(0.0, -1.0, 0.0);
pub const SHIP_UP64: Vector3<f64> = Vector3::new(0.0, 0.0, 1.0);

const MOUSE_SENSITIVITY_HORIZONTAL: f32 = 0.01;
const MOUSE_SENSITIVITY_VERTICAL: f32 = 0.01;

const ZOOM_SENSITIVITY: f32 = 10.0;

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
    focus: f32,
    pub tot_bbx: BoundingBox<Point3<f64>>,
}

impl Camera {
    pub fn new(fovy: Rad<f32>, aspect: f32, near: f32, far: f32) -> Self {
        let eye = Point3::new(0.0, 0.0, 0.0);
        let head_forward = SHIP_FORWARD.clone();
        let head_up = Rc::new(RwLock::new(SHIP_UP.clone()));
        let head_right = Rc::new(RwLock::new(SHIP_RIGHT.clone()));
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
            far: far * Z_FIGHTING_FACTOR,
            fovy: fovy,
            aspect: aspect,
            dx: 0.0,
            dy: 0.0,
            yaw: 0.0,
            pitch: 0.0,
            focus: 700.0,
            tot_bbx: Default::default(),
        }
    }

    pub fn default() -> Self {
        let cam = Camera::new(
            Rad(std::f32::consts::PI / 2.0),
            1.0,
            0.001,
            200000.0,
        );
        cam
    }

    pub fn resize(&mut self, w: u32, h: u32) {
        if w > 0 && h > 0 {
            self.aspect = w as f32 / h as f32;
        }
    }

    pub fn update_mouse(&mut self, dx_in: f32, dy_in: f32) {
        self.yaw += dx_in * MOUSE_SENSITIVITY_HORIZONTAL;
        self.pitch += dy_in * MOUSE_SENSITIVITY_VERTICAL;
        if abs(self.yaw) > PI * 2.0 { self.yaw = 0.0 }
        if abs(self.pitch) > PI * 2.0 { self.pitch = 0.0 }
        self.rotate();
    }

    fn rotate(&mut self) {
        let _up: Vector3<f32> = self.head_up.clone();
        let forward: Vector3<f32> = self.head_forward.clone();
        let _right: Vector3<f32> = self.head_right.clone();
        let eye: Point3<f32> = self.eye.clone();

        let new_forward_rot = Quaternion::from_axis_angle(SHIP_UP, Rad(self.yaw)).normalize();
        let new_forward_lr: Vector3<f32> = new_forward_rot.rotate_vector(SHIP_FORWARD);
        let new_right: Vector3<f32> = new_forward_lr.cross(SHIP_UP);

        let new_right_rot = Quaternion::from_axis_angle(new_right, Rad(self.pitch)).normalize();
        let new_forward: Vector3<f32> = new_right_rot.rotate_vector(new_forward_lr);
        let new_up: Vector3<f32> = new_right.cross(new_forward);

        let center: Point3<f32> = eye.clone() + forward * self.focus;
        let new_eye: Point3<f32> = center - new_forward * self.focus;

        self.head_forward = new_forward;
        self.head_right = new_right;
        self.head_up = new_up;
        self.eye = new_eye;
    }

    pub fn move_and_look_at(&mut self, new_eye_pos: Point3<f32>, look_at_point: Point3<f32>) {
        let dir_raw = look_at_point.sub(new_eye_pos);
        let d = dir_raw.magnitude();
        self.yaw = 0.0;
        self.pitch = 0.0;
        self.head_forward = SHIP_FORWARD;
        self.head_right = SHIP_RIGHT;
        self.head_up = SHIP_UP;
        self.eye = new_eye_pos;
        self.focus = d;
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

    pub fn calculate_tot_bbx(&mut self, bbxs: Vec<f32>) {
        let mut out_bbx = {
            let pmin: Point3<f64> = Point3::new(-100.0, -100.0, -100.0);
            let pmax: Point3<f64> = Point3::new(100.0, 100.0, 100.0);
            let bbx = BoundingBox::from_iter([pmin, pmax]);
            bbx
        };
        let mut bbxes: Vec<BoundingBox<Point3<f64>>> = vec![];
        let mut is_first_step = true;
        bbxs.chunks(6).for_each(|b| {
            let bbx: BoundingBox<Point3<f64>> = {
                let pmin: Point3<f64> = Point3::new(b[0] as f64, b[1] as f64, b[2] as f64);
                let pmax: Point3<f64> = Point3::new(b[3] as f64, b[4] as f64, b[5] as f64);
                let bbx = BoundingBox::from_iter([pmin, pmax]);
                bbx
            };
            if (is_first_step) {
                self.tot_bbx = bbx.clone();
                is_first_step = false;
            } else {
                self.tot_bbx += bbx.clone();
            }
            out_bbx += bbx.clone();
            bbxes.push(bbx);
        });
        self.focus=(self.tot_bbx.diagonal().magnitude() / ZOOM_SENSITIVITY as f64) as f32;
    }

    pub fn move_camera_to_bbx_limits(&mut self) {
        let cp = self.tot_bbx.center();
        let ep = self.tot_bbx.max();

        let focus = cp.distance(ep);
        let head_forward = cp.sub(ep).normalize();
        let head_forward32: Vector3<f32> = Vector3::new(head_forward.x as f32, head_forward.y as f32, head_forward.z as f32);

        let right_point = ep.clone() + SHIP_RIGHT64 * 100.0;
        let plane = Plane::new(ep, cp, right_point);
        let new_up = plane.normal().normalize();
        let new_right = new_up.cross(head_forward).normalize();

        let new_up32: Vector3<f32> = Vector3::new(new_up.x as f32, new_up.y as f32, new_up.z as f32);
        let new_right32: Vector3<f32> = Vector3::new(new_right.x as f32, new_right.y as f32, new_right.z as f32);
        let p: Point3<f32> = Point3::new(ep.x as f32, ep.y as f32, ep.z as f32);
        self.eye.clone_from(&p);
        self.head_forward.clone_from(&head_forward32);
        self.head_up.clone().clone_from(&new_up32);
        self.head_right.clone().clone_from(&new_right32);
        self.set_start_pos();
        self.is_dirty=true;
    }

    pub fn set_start_pos(&mut self) {
        self.yaw = 0.0;
        self.pitch = 0.0;
    }
}