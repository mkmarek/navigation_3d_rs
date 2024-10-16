struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) tex_coords: vec2<f32>,
}

@group(0) @binding(0) var<uniform> inverse_proj: mat4x4<f32>;
@group(0) @binding(1) var<uniform> inverse_view: mat4x4<f32>;

@vertex
fn vert(
    @builtin(vertex_index) in_vertex_index: u32,
) -> VertexOutput {
    var out: VertexOutput;
    let square_point = vec3<f32>(f32(in_vertex_index & 1u) - 0.5, f32((in_vertex_index >> 1u) & 1u) - 0.5, 0.0);

    out.tex_coords = vec2<f32>(square_point.x + 0.5, square_point.y + 0.5);
    out.clip_position = vec4<f32>(square_point.xy * 2.0, 0.0, 1.0);
    return out;
}

// https://sibaku.github.io/computer-graphics/2017/01/10/Camera-Ray-Generation.html
fn createRay(px: vec2<f32>, PInv: mat4x4<f32>, VInv: mat4x4<f32>) -> vec3<f32> {
    // convert pixel to NDS
    // [0,1] -> [-1,1]
    let pxNDS = px * 2.0 - 1.0;

    // choose an arbitrary point in the viewing volume
    // z = -1 equals a point on the near plane, i.e. the screen
    let pointNDS = vec3<f32>(pxNDS * 2.0, -1.);

    // as this is in homogenous space, add the last homogenous coordinate
    let pointNDSH = vec4<f32>(pointNDS, 1.0);
    // transform by inverse projection to get the point in view space
    let dirEye = vec4<f32>((PInv * pointNDSH).xyz, 0.0);

    // since the camera is at the origin in view space by definition,
    // the current point is already the correct direction 
    // (dir(0,P) = P - 0 = P as a direction, an infinite point,
    // the homogenous component becomes 0 the scaling done by the 
    // w-division is not of interest, as the direction in xyz will 
    // stay the same and we can just normalize it later
    //dirEye.w = 0.;

    // compute world ray direction by multiplying the inverse view matrix
    let dirWorld = (VInv * dirEye).xyz;

    // now normalize direction
    return normalize(dirWorld);
}

fn normalize_scale(scale: f32) -> f32 {
    let log_2 = floor(log2(scale));

    if abs(log_2) > 0.001 {
        return scale / pow(2.0, log_2);
    } else {
        return scale;
    }
}

fn smoothstep(low: f32, high: f32, value: f32) -> f32 {
    let x = clamp((value - low) / (high - low), 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
}

fn calculate_line(y: f32, spacing: f32, line_width: f32) -> f32 {
    var center_distance = abs(fract(y / spacing));

    if center_distance > 0.5 {
        center_distance = 1.0 - center_distance;
    }

    return 1.0 - smoothstep(0.0, line_width / spacing, center_distance * 2.0);
}

const PI = 3.14159265359;

fn directionToCubeUV(direction: vec3<f32>) -> vec2<f32> {
    var absDir = abs(direction);
    let majorAxis = max(max(absDir.x, absDir.y), absDir.z);

    var u: f32;
    var v: f32;

    if absDir.x >= absDir.y && absDir.x >= absDir.z {
        // X axis is dominant
        if direction.x > 0.0 {
            // +X face
            u = -direction.z / absDir.x;
            v = -direction.y / absDir.x;
        } else {
            // -X face
            u = direction.z / absDir.x;
            v = -direction.y / absDir.x;
        }
    } else if absDir.y >= absDir.x && absDir.y >= absDir.z {
        // Y axis is dominant
        if direction.y > 0.0 {
            // +Y face
            u = direction.x / absDir.y;
            v = direction.z / absDir.y;
        } else {
            // -Y face
            u = direction.x / absDir.y;
            v = -direction.z / absDir.y;
        }
    } else {
        // Z axis is dominant
        if direction.z > 0.0 {
            // +Z face
            u = direction.x / absDir.z;
            v = -direction.y / absDir.z;
        } else {
            // -Z face
            u = -direction.x / absDir.z;
            v = -direction.y / absDir.z;
        }
    }

    // Transform from [-1, 1] to [0, 1]
    u = (u + 1.0) * 0.5;
    v = (v + 1.0) * 0.5;

    return vec2<f32>(u, v);
}

@fragment
fn frag(in: VertexOutput) -> @location(0) vec4<f32> {
    let ray = createRay(in.tex_coords, inverse_proj, inverse_view);
    let uv = directionToCubeUV(ray);

    let scale = vec2<f32>(1.0, 1.0);

    let horizontal_lines = 10.0;
    let normalized = normalize_scale(scale.x);
    let size = vec2<f32>(normalized, scale.y / scale.x * normalized) / scale;
    let pos = ((uv * scale)) * size;

    let background_color = vec3<f32>(0.169, 0.173, 0.184) / 10.0;

    let major_line_color = background_color * 2.0;
    let minor_line_color = background_color * 1.5;
    let tiny_line_color = background_color * 1.2;

    let line_width = 0.0015;

    let base_spacing = (1.0) / horizontal_lines;
    let half_spacing1 = base_spacing / 2.0;
    let half_spacing2 = half_spacing1 / 2.0;

    let major_line = clamp(calculate_line(pos.y, base_spacing, line_width) + calculate_line(pos.x, base_spacing, line_width), 0.0, 1.0);
    let minor_line = clamp(calculate_line(pos.y, half_spacing1, line_width) + calculate_line(pos.x, half_spacing1, line_width), 0.0, 1.0);
    let tiny_line = clamp(calculate_line(pos.y, half_spacing2, line_width) + calculate_line(pos.x, half_spacing2, line_width), 0.0, 1.0);

    if major_line > 0.0 {
        return vec4<f32>(major_line_color, 1.0);
    } else if tiny_line > 0.0 {
        return vec4<f32>(tiny_line_color, 1.0);
    }

    return vec4<f32>(background_color, 1.0);
}
