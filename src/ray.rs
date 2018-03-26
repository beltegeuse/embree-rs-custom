use std::f32;
use cgmath::{Vector3,Point3,Vector2,InnerSpace};
use scene::*;

/// Value for an invalid ID
pub const INVALID_GEOMETRY_ID: u32 = !0;

/// Structure for embree to represent a ray
#[repr(C,align(16))]
pub struct Ray {
    /// Ray origin
    pub org: [f32; 4usize],
    pub dir: [f32; 4usize],

    /// Start of ray segment
    pub tnear: f32,
    /// End of ray segment
    pub tfar: f32,

    /// Time of this ray for motion blur
    pub time: f32,
    /// Used to mask objets during ray traversal
    pub mask: u32,

    /// Unnormalized geometric normal
    pub n_g: [f32; 4usize],

    /// Barycentric u coordinate at hit
    pub u: f32,
    /// Barycentric v coordinate at hit
    pub v: f32,

    /// Geometry ID
    pub geom_id: u32,
    /// Primitive ID
    pub prim_id: u32,
    /// Instance ID
    pub inst_id: u32,
}

#[test]
fn align_ray() {
    assert_eq!(::std::mem::align_of::<Ray>(), 16usize);
}

#[test]
fn memsize_ray() {
    assert_eq!(::std::mem::size_of::<Ray>() , 96usize , concat ! (
               "Size of: " , stringify ! ( Ray ) ));
}

/// Holds the data representing a successful ray intersection
pub struct Intersection {
    /// Intersection distance
    pub t: f32,
    /// Geometry normal
    pub n_g: Vector3<f32>,
    /// Shading normal
    pub n_s: Vector3<f32>,
    /// Intersection point
    pub p: Point3<f32>,
    /// Textures coordinates
    pub uv: Option<Vector2<f32>>,
    /// Geometry ID
    pub geom_id: u32,
    /// Primitive ID
    pub prim_id: u32,
    /// Instance ID
    pub inst_id: u32,
}

impl Ray {
    pub fn new(origin: &Point3<f32>,
               direction: &Vector3<f32>,
               tnear: f32,
               tfar: f32) -> Ray {
        Ray {
            org: [origin.x, origin.y, origin.z, 0.0],
            dir: [direction.x, direction.y, direction.z, 0.0],
            tnear,
            tfar,
            time: 0.0,
            mask: 0xFFFFFFFF,
            n_g: [0.0,0.0,0.0,0.0],
            u: 0.0,
            v: 0.0,
            geom_id: INVALID_GEOMETRY_ID,
            prim_id: INVALID_GEOMETRY_ID,
            inst_id: INVALID_GEOMETRY_ID,
        }
    }

    pub fn hit(&self) -> bool {
        self.geom_id != INVALID_GEOMETRY_ID
    }
}

impl Intersection {
    pub fn from_ray(scene: &Scene, ray: Ray) -> Option<Intersection> {
        if ray.hit() {
            let mesh = scene.mesh(ray.geom_id as usize);
            let i0 = mesh.indices[(ray.prim_id * 3) as usize] as usize;
            let i1 = mesh.indices[(ray.prim_id * 3 + 1) as usize] as usize;
            let i2 = mesh.indices[(ray.prim_id * 3 + 2) as usize] as usize;

            // Normal interpolation
            let d0 = &mesh.normals[i0];
            let d1 = &mesh.normals[i1];
            let d2 = &mesh.normals[i2];
            let n_s = d0 * ( 1.0 - ray.u - ray.v) + d1 * ray.u + d2 * ray.v;

            // UV interpolation
            let uv: Option<Vector2<f32>>;
            if mesh.uv.is_empty() {
                uv = None;
            } else {
                let d0 = &mesh.uv[i0];
                let d1 = &mesh.uv[i1];
                let d2 = &mesh.uv[i2];
                uv = Some(d0 * ( 1.0 - ray.u - ray.v) + d1 * ray.u + d2 * ray.v);
            }

            let mut n_g = Vector3::new(ray.n_g[0], ray.n_g[1], ray.n_g[2]).normalize();
            if n_g.dot(n_s) < 0.0 {
                n_g = -n_g;
            }

            // Construct the intersection
            return Some(Intersection{
                t: ray.tfar,
                n_g,
                n_s,
                p : Point3::new(
                    ray.org[0] + ray.tfar * ray.dir[0],
                    ray.org[1] + ray.tfar * ray.dir[1],
                    ray.org[2] + ray.tfar * ray.dir[2],
                ),
                uv,
                geom_id: ray.geom_id,
                prim_id: ray.prim_id,
                inst_id: ray.inst_id,
            });
        } else {
            return None;
        }
    }
}