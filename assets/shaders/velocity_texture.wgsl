#import bevy_sprite::mesh2d_vertex_output::VertexOutput
#import "shaders/custom_material_import.wgsl"::COLOR_MULTIPLIER

@group(1) @binding(0) var<uniform> in_collision_color: vec4<f32>;
@group(1) @binding(1) var<uniform> out_of_collision_color: vec4<f32>;
@group(1) @binding(2) var<uniform> radius_sum: f32;
@group(1) @binding(3) var<uniform> first_agent_position: vec2<f32>;
@group(1) @binding(4) var<uniform> second_agent_position: vec2<f32>;
@group(1) @binding(5) var<uniform> second_agent_velocity: vec2<f32>;
@group(1) @binding(6) var<uniform> look_ahead_time: f32;
@group(1) @binding(7) var<uniform> resolution: vec2<f32>;

@fragment
fn fragment(mesh: VertexOutput) -> @location(0) vec4<f32> {
  let adjusted_uv = (mesh.uv * vec2<f32>(1.0, -1.0) + vec2<f32>(0.0, 1.0)) * resolution;
  let first_agent_velocity = adjusted_uv - first_agent_position;

  if velocity_intersection(
    first_agent_position,
    second_agent_position,
    first_agent_velocity,
    second_agent_velocity,
    radius_sum,
    look_ahead_time) {
    return in_collision_color;
  } else {
    return out_of_collision_color;
  }
}

fn length_squared(v: vec2<f32>) -> f32 {
    return dot(v, v);
}

fn velocity_intersection(
    agent_a_pos: vec2<f32>,
    agent_b_pos: vec2<f32>,
    agent_a_velocity: vec2<f32>,
    agent_b_velocity: vec2<f32>,
    combined_radius: f32,
    look_ahead_time: f32,
) -> bool {
    let relative_velocity = agent_a_velocity - agent_b_velocity;
    if length_squared(relative_velocity) < 0.00001 {
        return false;
    }

    let relative_position = agent_b_pos - agent_a_pos;

    let t = clamp(dot(relative_velocity, relative_position) / length_squared(relative_velocity), 0.0, look_ahead_time);

    let distance_squared = length_squared(relative_velocity * t - relative_position);

    return distance_squared < combined_radius * combined_radius;
}
