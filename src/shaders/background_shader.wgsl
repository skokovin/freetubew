struct AddData {
    resolution:vec4<i32>,
};
@binding(0) @group(0) var<uniform> add_data : AddData;

@vertex
fn vs_main(@builtin(vertex_index) in_vertex_index: u32) -> @builtin(position) vec4<f32> {
    //let x = f32(i32(in_vertex_index) - 1);
    //let y = f32(i32(in_vertex_index & 1u) * 2 - 1);

       // let x = f32(i32(in_vertex_index) - 1);
       // let y = f32(i32(in_vertex_index & 1u) * 2 - 1);

    var x:f32=0.0;
    var y:f32=0.0;

   if(in_vertex_index==0){
         x=-1.0;
         y=-1.0;
    }

    if(in_vertex_index==1){
        x=1.0;
        y=1.0;
    }
    if(in_vertex_index==2){
        x=-1.0;
        y=1.0;
    }

     if(in_vertex_index==3){
             x=-1.0;
             y=-1.0;
        }

        if(in_vertex_index==4){
            x=1.0;
            y=-1.0;
        }
        if(in_vertex_index==5){
            x=1.0;
            y=1.0;
        }
    return vec4<f32>(x, y, 0.0, 1.0);
}
@fragment
fn fs_main(@builtin(position) p: vec4<f32>) -> @location(0) vec4<f32> {
        let yy0:f32=min(0.95,(p.y)/ f32(add_data.resolution.y));
        let yy:f32=max(0.3,yy0);
        let col =vec3(yy,yy,yy);
        return vec4<f32>(col, 1.0);
}


