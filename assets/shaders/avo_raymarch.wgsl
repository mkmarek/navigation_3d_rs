#import bevy_core_pipeline::fullscreen_vertex_shader::FullscreenVertexOutput

@group(0) @binding(0) var screen_texture: texture_2d<f32>;
@group(0) @binding(1) var texture_sampler: sampler;
@group(0) @binding(2) var<uniform> data: RayMarchData;

struct RayMarchData {
  projection: mat4x4<f32>,
  projection_inverse: mat4x4<f32>,
  view: mat4x4<f32>,
  offset: vec3<f32>,

  acceleration_ctrl_param: f32,
  e: f32,
  lookahead: f32,
  velocity_ab: vec3<f32>,
  velocity_b: vec3<f32>,
  position_ab: vec3<f32>,
  radius_ab: f32
}

struct Ray {
  origin: vec3<f32>,
  direction: vec3<f32>
}

struct Lighting {
  diffuse: vec3<f32>,
  specular: vec3<f32>,
}

struct PointLight {
  position: vec3<f32>,
  diffuse_color: vec3<f32>,
  diffuse_power: f32,
  specular_color: vec3<f32>,
  specular_power: f32,
};

const EPSILON: f32 = 0.01;
const RAY_MARCHING_STEPS: u32 = 300u;
const MAX_DISTANCE: f32 = 10000.0;
const AVO_SAMPLES: u32 = 25u;

fn create_ray_for_coord(coord: vec2<f32>) -> Ray {
    let ndc_to_world = data.view * data.projection_inverse;

    let near_4d = ndc_to_world * vec4<f32>(coord.x, coord.y, 1.0, 1.0);
    let far_4d = ndc_to_world * vec4<f32>(coord.x, coord.y, EPSILON, 1.0);

    let near = near_4d / near_4d.w;
    let far = far_4d / far_4d.w;

    return Ray(near.xyz, normalize(far.xyz - near.xyz));
}

fn sdf_sphere(p: vec3<f32>, radius: f32) -> f32 {
    return length(p) - radius;
}

fn lerp(a: f32, b: f32, t: f32) -> f32 {
    return a + (b - a) * t;
}

fn sdf_scene(p: vec3<f32>) -> f32 {
    let pt = p - data.offset;

    var distance = MAX_DISTANCE;

    for (var i: u32 = 0u; i < AVO_SAMPLES; i = i + 1u) {
        let t = lerp(0.001, data.lookahead, f32(i) / f32(AVO_SAMPLES));

        let param = data.acceleration_ctrl_param * (pow(data.e, -t / data.acceleration_ctrl_param) - 1.0);

        let sphere_center = (param * data.velocity_ab - data.position_ab) / (t + param);
        let sphere_radius = data.radius_ab / (t + param);

        let sphere_distance = sdf_sphere(pt - (sphere_center + data.velocity_b), sphere_radius);
        distance = min(distance, sphere_distance);
    }

    return distance;
}

fn estimateNormal(p: vec3<f32>) -> vec3<f32> {
    return normalize(vec3<f32>(
        sdf_scene(vec3<f32>(p.x + EPSILON, p.y, p.z)) - sdf_scene(vec3(p.x - EPSILON, p.y, p.z)),
        sdf_scene(vec3<f32>(p.x, p.y + EPSILON, p.z)) - sdf_scene(vec3(p.x, p.y - EPSILON, p.z)),
        sdf_scene(vec3<f32>(p.x, p.y, p.z + EPSILON)) - sdf_scene(vec3(p.x, p.y, p.z - EPSILON))
    ));
}

fn calculateLighting(origin: vec3<f32>, p: vec3<f32>, normal: vec3<f32>) -> Lighting {
    let light = PointLight(
        origin,
        vec3<f32>(1.0, 1.0, 1.0),
        0.5,
        vec3<f32>(1.0, 1.0, 1.0),
        1.0
    );

    let light_dir = normalize(light.position - p);
    let light_reflect = reflect(-light_dir, normal);
    let view_dir = normalize(p);

    let diffuse = max(dot(normal, light_dir), 0.0) * light.diffuse_color * light.diffuse_power;
    let specular = pow(max(dot(view_dir, light_reflect), 0.0), 32.0) * light.specular_color * light.specular_power;

    return Lighting(diffuse, specular);
}

fn ray_march(ray: Ray) -> vec3<f32> {
    var t = 0.0;
    var sign = 1.0;

    if sdf_scene(ray.origin) < 0.0 {
        sign = -1.0;
    }

    for (var i: u32 = 0u; i < RAY_MARCHING_STEPS; i = i + 1u) {
        let pt = ray.origin + ray.direction * t;
        let dist = sign * sdf_scene(pt);
        if dist < EPSILON {
            return pt;
        }

        t += dist;

        if t >= MAX_DISTANCE {
      break;
        }
    }
    return ray.origin + ray.direction * MAX_DISTANCE;
}

@fragment
fn fragment(in: FullscreenVertexOutput) -> @location(0) vec4<f32> {

    var coord = in.uv * 2.0 - 1.0;
    coord.y = -coord.y;

    let ray = create_ray_for_coord(coord);
    let pt = ray_march(ray);

    var color = textureSample(screen_texture, texture_sampler, in.uv);

    if length(pt - ray.origin) < MAX_DISTANCE - EPSILON {
        let lighting = calculateLighting(ray.origin, pt, estimateNormal(pt));
        color += vec4<f32>(lighting.diffuse + lighting.specular, 0.5);
    }

    return color;
}
