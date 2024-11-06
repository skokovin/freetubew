// vertex shader

//THIS IS FOR LOGARITHMIC Z FIGHTING THROBLES
   //gl_Position.z = log(gl_Position.w*C + 1)/log(far*C + 1);
    //gl_Position.z *= gl_Position.w;
const C:f32=0.001;
const FAR:f32=20000.0;

const PI:f32= 3.14159265358979323846;

const vx:vec3<f32>=vec3<f32>(1.0,0.0,0.0);
const vy:vec3<f32>=vec3<f32>(0.0,1.0,0.0);
const vz:vec3<f32>=vec3<f32>(0.0,0.0,1.0);
const def_color =vec4<f32>(0.1,0.1,0.1,1.0);

const dorn_id:i32=200;
const r_arrow_x_id:i32=201;

struct VertexInput {
    @location(0) position: vec4<f32>,
    @location(1) normal: vec4<f32>,
    @location(2) id: i32,
    @location(3) tex_coords: vec2<f32>,
};

struct Camera {
    mvp : mat4x4<f32>,
    n_matrix : mat4x4<f32>,
    forward_dir:vec3<f32>,
};
@binding(0) @group(0) var<uniform> camera : Camera;

struct CameraUniforms {
    light_position : vec4<f32>,
    eye_position : vec4<f32>,
    resolution : vec4<f32>
};
@binding(1) @group(0) var<uniform> camera_uniforms : CameraUniforms;

struct LightUniforms {
    color : vec4<f32>,
    specular_color : vec4<f32>,
    ambient_intensity: f32,
    diffuse_intensity :f32,
    specular_intensity: f32,
    specular_shininess: f32
};
@binding(2) @group(0) var<uniform> light_uniformsArray: array<LightUniforms, 140>;

@binding(3) @group(0) var<uniform> vertex_meta_data0 : array<vec4<i32>, 256>;

@binding(4) @group(0) var<uniform> dorn_scale : mat4x4<f32>;

@binding(5) @group(0) var<uniform> dorn_translate : mat4x4<f32>;

@binding(6) @group(0)
var t_diffuse: texture_2d<f32>;

@binding(7) @group(0)
var s_diffuse: sampler;


struct Output {
    @builtin(position) position : vec4<f32>,
    @location(0) world_normal : vec4<f32>,
    @location(1) world_position : vec4<f32>,
    @location(2) @interpolate(flat)  mat_id: i32,
    @location(3) originalpos : vec4<f32>,
    @location(4) tex_coords: vec2<f32>,
};

@vertex
fn vs_main(@builtin(vertex_index) vertex_index : u32,in:VertexInput) -> Output {
    //let feed_translation : mat4x4<f32>=feed_translations[in.id];
    //let feed_pos=feed_translation*in.position;


    var hull_meta_data=37;
    var output: Output;
    output.originalpos= in.position;
    output.mat_id=in.id;

    switch in.id {
        case dorn_id: {
            output.position = camera.mvp *dorn_translate * dorn_scale * in.position;
        }
   
        default: {
            output.position = camera.mvp  * in.position;
        }
      }
    output.world_normal = in.normal;

    //
    output.position.z = log(output.position.w*C + 1)/log(FAR*C + 1);
    output.position.z *= output.position.w;

    output.world_position = in.position;

    output.tex_coords = in.tex_coords;

    //

    return output;
}


@fragment
fn fs_main(in:Output) ->  @location(0) vec4<f32> {
   var v:vec4<f32>=textureSample(t_diffuse, s_diffuse, in.tex_coords);
   //var v1:vec4<f32>=vec4<f32>(v.x,v.y,v.z,v.w);
   return v;
    //FOR WASM
   //return vec4<f32>(1.0,1.0,1.0,0.0);
}


