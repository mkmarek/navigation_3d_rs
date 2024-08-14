#import bevy_core_pipeline::fullscreen_vertex_shader::FullscreenVertexOutput

@group(0) @binding(0) var screen_texture: texture_2d<f32>;
@group(0) @binding(1) var texture_sampler: sampler;
@group(0) @binding(2) var<uniform> data: RayMarchData;

struct Hyperplane {
  normal: vec4<f32>,
  origin: vec4<f32>,
  u_direction: vec4<f32>,
  v_direction: vec4<f32>,
  w_direction: vec4<f32>,
}

struct Plane {
  normal: vec3<f32>,  
  origin: vec3<f32>,
  u_direction: vec3<f32>,
  v_direction: vec3<f32>,
}

fn new_hyperplane_basis(normal: vec4<f32>) -> array<vec4<f32>, 3> {
    let x_dot = abs(dot(vec4<f32>(1.0, 0.0, 0.0, 0.0), normal));
    let y_dot = abs(dot(vec4<f32>(0.0, 1.0, 0.0, 0.0), normal));
    let z_dot = abs(dot(vec4<f32>(0.0, 0.0, 1.0, 0.0), normal));
    let w_dot = abs(dot(vec4<f32>(0.0, 0.0, 0.0, 1.0), normal));

    if x_dot > y_dot && x_dot > z_dot && x_dot > w_dot {
        return array<vec4<f32>, 3>(vec4<f32>(0.0, 1.0, 0.0, 0.0), vec4<f32>(0.0, 0.0, 1.0, 0.0), vec4<f32>(0.0, 0.0, 0.0, 1.0));
    } else if y_dot > z_dot && y_dot > w_dot {
        return array<vec4<f32>, 3>(vec4<f32>(1.0, 0.0, 0.0, 0.0), vec4<f32>(0.0, 0.0, 1.0, 0.0), vec4<f32>(0.0, 0.0, 0.0, 1.0));
    } else if z_dot > w_dot {
        return array<vec4<f32>, 3>(vec4<f32>(1.0, 0.0, 0.0, 0.0), vec4<f32>(0.0, 1.0, 0.0, 0.0), vec4<f32>(0.0, 0.0, 0.0, 1.0));
    } else {
        return array<vec4<f32>, 3>(vec4<f32>(1.0, 0.0, 0.0, 0.0), vec4<f32>(0.0, 1.0, 0.0, 0.0), vec4<f32>(0.0, 0.0, 1.0, 0.0));
    }
}

fn new_hyperplane(origin: vec4<f32>, n: vec4<f32>) -> Hyperplane {
    let normal = normalize(n);
    let basis = new_hyperplane_basis(normal);

  
  // Use Gram-Schmidt orthogonalization to get the u, v and w vectors, which are orthogonal to the normal and each other
    let u_direction = normalize(basis[0] - normal * dot(normal, basis[0]));
    let v_direction = normalize(basis[1] - normal * dot(normal, basis[1]) - u_direction * dot(u_direction, basis[1]));
    let w_direction = normalize(basis[2] - normal * dot(normal, basis[2]) - u_direction * dot(u_direction, basis[2]) - v_direction * dot(v_direction, basis[2]));

    return Hyperplane(normal, origin, u_direction, v_direction, w_direction);
}

fn new_plane_basis(normal: vec3<f32>) -> vec3<f32> {
    let x_dot = abs(dot(vec3<f32>(1.0, 0.0, 0.0), normal));
    let y_dot = abs(dot(vec3<f32>(0.0, 1.0, 0.0), normal));
    let z_dot = abs(dot(vec3<f32>(0.0, 0.0, 1.0), normal));

    if x_dot < y_dot && x_dot < z_dot {
        return vec3<f32>(1.0, 0.0, 0.0);
    } else if y_dot < z_dot {
        return vec3<f32>(0.0, 1.0, 0.0);
    } else {
        return vec3<f32>(0.0, 0.0, 1.0);
    }
}

fn new_plane(origin: vec3<f32>, n: vec3<f32>) -> Plane {
    let normal = normalize(n);

    let basis = new_plane_basis(normal);

    let u_direction = normalize(cross(normal, basis));
    let v_direction = normalize(cross(normal, u_direction));

    return Plane(normal, origin, u_direction, v_direction);
}

fn plane_from_hyperplane_intersection(plane: Hyperplane, other: Hyperplane) -> Plane {
    let normal_x = dot(plane.u_direction, other.normal);
    let normal_y = dot(plane.v_direction, other.normal);
    let normal_z = dot(plane.w_direction, other.normal);

    let d2 = dot(other.origin, other.normal);
    let d_result = d2 - dot(plane.origin, other.normal);

    let normal = normalize(vec3<f32>(normal_x, normal_y, normal_z));
    let origin = -d_result * normal;

    return new_plane(origin, normal);
}

struct RayMarchData {
  projection: mat4x4<f32>,
  projection_inverse: mat4x4<f32>,
  view: mat4x4<f32>,
  w: f32,
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
const SCALE_FACTOR: f32 = 1.2;

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

fn sdf_torus(p: vec3<f32>, t: vec2<f32>) -> f32 {
    let q = vec2<f32>(length(p.xz) - t.x, p.y);
    return length(q) - t.y;
}

fn sdf_plane(point: vec3<f32>, plane: Plane) -> f32 {
  // Calculate the vector from the plane origin to the point
    let vector_to_point = point - plane.origin;

  // Calculate the dot product of this vector with the plane normal
    let distance = dot(vector_to_point, plane.normal);

    return distance;
}

fn sdf_spherinder_hyperplane_intersection(pt: vec3<f32>, radius: f32, hyperplane: Hyperplane) -> f32 {
    let a = (pt.x * hyperplane.u_direction.x + pt.y * hyperplane.v_direction.x + pt.z * hyperplane.w_direction.x + hyperplane.origin.x);

    let b = (pt.x * hyperplane.u_direction.y + pt.y * hyperplane.v_direction.y + pt.z * hyperplane.w_direction.y + hyperplane.origin.y);

    let c = (pt.x * hyperplane.u_direction.z + pt.y * hyperplane.v_direction.z + pt.z * hyperplane.w_direction.z + hyperplane.origin.z);

    return sqrt(a * a + b * b + c * c) - radius;
}

fn sdf_scene(p: vec3<f32>) -> f32 {
    let pt = p;

    let hyperplane_c = new_hyperplane(vec4<f32>(-43.733166, 138.09503, -105.5708, 0.0), vec4<f32>(-0.7777588, -0.12597124, -0.42334685, 0.44721353));
    let spherinder_distance = sdf_spherinder_hyperplane_intersection(pt, 100.0, hyperplane_c);
    let intersecting_plane_1 = new_plane(vec3<f32>(3.4217021, -3.7875133, 5.4648843), vec3<f32>(0.45757827, -0.50649756, 0.73080945));

    let intersecting_plane_2 = new_plane(vec3<f32>(-44.757824, 15.044995, -5.303109), vec3<f32>(0.9419595, -0.3166324, 0.11160762));

    return max(max(spherinder_distance, -sdf_plane(pt, intersecting_plane_1)), sdf_plane(pt, intersecting_plane_2));
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
