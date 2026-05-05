struct Uniforms {
    view_proj: mat4x4<f32>,
    color: vec4<f32>,
    light_dir: vec4<f32>,
    edge_color: vec4<f32>,
};

@group(0) @binding(0) var<uniform> u: Uniforms;

struct VsIn {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) color: vec4<f32>,
};

struct VsOut {
    @builtin(position) clip_pos: vec4<f32>,
    @location(0) world_normal: vec3<f32>,
    @location(1) vert_color: vec4<f32>,
};

@vertex
fn vs_main(in: VsIn) -> VsOut {
    var out: VsOut;
    out.clip_pos = u.view_proj * vec4<f32>(in.position, 1.0);
    out.world_normal = in.normal;
    // per-vertex color: alpha > 0 means use vertex color, otherwise uniform
    out.vert_color = in.color;
    return out;
}

@fragment
fn fs_main(in: VsOut) -> @location(0) vec4<f32> {
    let n = normalize(in.world_normal);
    let l = normalize(-u.light_dir.xyz);
    let diffuse = max(dot(n, l), 0.0);
    let ambient = 0.2;
    let intensity = ambient + diffuse * 0.8;

    var base_color: vec4<f32>;
    if (in.vert_color.a > 0.0) {
        base_color = in.vert_color;
    } else {
        base_color = u.color;
    }

    return vec4<f32>(base_color.rgb * intensity, base_color.a);
}

@vertex
fn vs_edge(in: VsIn) -> @builtin(position) vec4<f32> {
    return u.view_proj * vec4<f32>(in.position, 1.0);
}

@fragment
fn fs_edge() -> @location(0) vec4<f32> {
    return u.edge_color;
}
