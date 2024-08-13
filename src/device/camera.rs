use std::f32::consts::PI;
use std::ops::Sub;
use std::rc::Rc;
use cgmath::{InnerSpace, Matrix4, Point3, Quaternion, Rad, Rotation, Rotation3, SquareMatrix, Vector3};
use cgmath::num_traits::abs;
use parking_lot::RwLock;
use truck_base::bounding_box::BoundingBox;
use crate::device::Z_FIGHTING_FACTOR;

pub const SHIP_FORWARD: Vector3<f32> = Vector3::new(1.0, 0.0, 0.0);
pub const SHIP_RIGHT: Vector3<f32> = Vector3::new(0.0, -1.0, 0.0);
pub const SHIP_UP: Vector3<f32> = Vector3::new(0.0, 0.0, 1.0);

pub const SHIP_FORWARD64: Vector3<f64> = Vector3::new(1.0, 0.0, 0.0);
pub const SHIP_RIGHT64: Vector3<f64> = Vector3::new(0.0, -1.0, 0.0);
pub const SHIP_UP64: Vector3<f64> = Vector3::new(0.0, 0.0, 1.0);

const MOUSE_SENSITIVITY_HORIZONTAL: f32 = 0.01;
const MOUSE_SENSITIVITY_VERTICAL: f32 = 0.01;

pub struct Camera {
    pub is_dirty:bool,
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
}

impl Camera {
    pub fn new(fovy: Rad<f32>, aspect: f32, near: f32, far: f32) -> Self {
        let eye = Point3::new(0.0, 0.0, 0.0);
        let head_forward = SHIP_FORWARD.clone();
        let head_up = Rc::new(RwLock::new(SHIP_UP.clone()));
        let head_right = Rc::new(RwLock::new(SHIP_RIGHT.clone()));
        Self {
            is_dirty:true,
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
       self.is_dirty=false;
        let view_projection_ref: &[f32; 16] = self.vp_matrix.as_ref();
        view_projection_ref
    }

}