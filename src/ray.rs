use std::f32;
use cgmath::{Vector3,Point3,Vector2,InnerSpace};
use scene::*;
use super::root;

/// Value for an invalid ID
pub const INVALID_GEOMETRY_ID: u32 = !0;

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

impl root::RTCRay {
    pub fn hit(&self) -> bool {
        self.geomID != INVALID_GEOMETRY_ID
    }
}

impl Intersection {
    pub fn from_ray(scene: &Scene, ray: root::RTCRay) -> Option<Intersection> {
        if ray.hit() {
            let mesh = scene.mesh(ray.geomID as usize);
            let i0 = mesh.indices[(ray.primID * 3) as usize] as usize;
            let i1 = mesh.indices[(ray.primID * 3 + 1) as usize] as usize;
            let i2 = mesh.indices[(ray.primID * 3 + 2) as usize] as usize;

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

            let mut n_g = Vector3::new(ray.Ng[0], ray.Ng[1], ray.Ng[2]).normalize();
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
                geom_id: ray.geomID,
                prim_id: ray.primID,
                inst_id: ray.instID,
            });
        } else {
            return None;
        }
    }
}