#import bevy_pbr::{
    mesh_view_bindings::globals,
    prepass_utils,
    forward_io::VertexOutput,
}

@group(1) @binding(0) var<uniform> color: vec4<f32>;
@group(1) @binding(1) var<uniform> horizontal_lines: f32;

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

@fragment
fn fragment(mesh: VertexOutput) -> @location(0) vec4<f32> {
  let uv = mesh.uv * vec2<f32>(1.0, -1.0) + vec2<f32>(0.0, 1.0);
  let pos = uv;

  let background_color = vec3<f32>(0.169, 0.173, 0.184) * color.xyz;

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
