#![cfg_attr(feature = "clippy", feature(plugin))]
#![cfg_attr(feature = "clippy", plugin(clippy))]
#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

extern crate cgmath;
extern crate libc;

use cgmath::InnerSpace;
use cgmath::{Point3, Vector2, Vector3};
use std::ffi::CString;
use std::ptr;
use std::sync::Arc;

// Include the binding
include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
#[test]
fn test_ray_align() {
    assert_eq!(::std::mem::align_of::<root::RTCRay>(), 16usize);
}

pub const INVALID_GEOMETRY_ID: u32 = !0;
impl root::RTCRay {
    pub fn hit(&self) -> bool {
        self.id != INVALID_GEOMETRY_ID
    }
}

/// Wrapper around the RTCDevice opaque struct.
/// Embree supports a device concept which allows different components of the
/// application to use the API without interfering with one another.
/// An application typically creates a single device only and should not create
/// many of them.
unsafe impl Send for Device {}
unsafe impl Sync for Device {}
pub struct Device {
    handle: root::RTCDevice,
}

impl Device {
    /// Creates a new Device
    pub fn new() -> Device {
        let p = unsafe { root::rtcNewDevice(ptr::null()) };
        // If the RTCDevice is null then something very bad has happened and we
        // can't continue
        assert!(!p.is_null());
        Device { handle: p }
    }

    pub fn debug() -> Device {
        let cfg = CString::new("verbose=1").unwrap();
        Device {
            handle: unsafe { root::rtcNewDevice(cfg.as_ptr()) },
        }
    }
}

impl Drop for Device {
    fn drop(&mut self) {
        unsafe { root::rtcReleaseDevice(self.handle) };
    }
}

/// Internal vertex representation
#[repr(C)]
struct EmbreeVertex {
    x: f32,
    y: f32,
    z: f32,
    w: f32,
}

/// Internal triangle presentation
#[repr(C)]
struct EmbreeTriangle {
    i0: i32,
    i1: i32,
    i2: i32,
}

/// The object used to pass data to Embree.
/// The commit function need to be called in order to get Scene object.
/// Only Scene object provides intersection routines.
pub struct SceneConstruct {
    handle: root::RTCScene,
    geometry: Vec<Arc<TriangleMesh>>,
}

impl SceneConstruct {
    pub fn new(device: &Device) -> SceneConstruct {
        let p = unsafe { root::rtcNewScene(device.handle) };

        SceneConstruct {
            handle: p,
            geometry: Vec::<Arc<TriangleMesh>>::new(),
        }
    }

    /// Add a new TriangleMesh with the supplied vertex and indices vectors.
    pub fn add_triangle_mesh(
        &mut self,
        device: &Device,
        vertices: Vec<Vector3<f32>>,
        normals: Vec<Vector3<f32>>,
        uv: Vec<Vector2<f32>>,
        indices: Vec<u32>,
    ) -> Arc<TriangleMesh> {
        assert_eq!(
            indices.len() % 3,
            0,
            "Indices must be a multiple of 3 for a triangle mesh but \
             indices Vec has length {}",
            indices.len()
        );
        let num_triangles = indices.len() / 3;
        let geom_handle =
            unsafe { root::rtcNewGeometry(device.handle, root::RTCGeometryType::TRIANGLE) };

        // Set vertex buffer
        unsafe {
            let buffer_handle = {
                root::rtcNewBuffer(
                    device.handle,
                    vertices.len() * std::mem::size_of::<Vector3<EmbreeVertex>>(),
                )
            };
            let e_vertices = { root::rtcGetBufferData(buffer_handle) as *mut EmbreeVertex };
            for (i, v) in vertices.iter().enumerate() {
                (*e_vertices.offset(i as isize)).x = v.x;
                (*e_vertices.offset(i as isize)).y = v.y;
                (*e_vertices.offset(i as isize)).z = v.z;
                (*e_vertices.offset(i as isize)).w = 1.0;
            }
            root::rtcSetGeometryBuffer(
                geom_handle,
                root::RTCBufferType::VERTEX,
                0,
                root::RTCFormat::FLOAT3,
                buffer_handle,
                0,
                16,
                vertices.len(),
            );
        }

        // Set index buffer
        unsafe {
            let buffer_handle = {
                root::rtcNewBuffer(
                    device.handle,
                    num_triangles * std::mem::size_of::<Vector3<EmbreeTriangle>>(),
                )
            };
            let e_indicies = { root::rtcGetBufferData(buffer_handle) as *mut EmbreeTriangle };
            {
                let mut i: isize = 0;
                let mut indice_iter = indices.iter();
                while let Some(i0) = indice_iter.next() {
                    let i1 = indice_iter.next().unwrap();
                    let i2 = indice_iter.next().unwrap();
                    (*e_indicies.offset(i as isize)).i0 = *i0 as i32;
                    (*e_indicies.offset(i as isize)).i1 = *i1 as i32;
                    (*e_indicies.offset(i as isize)).i2 = *i2 as i32;
                    i += 1;
                }
            }
            root::rtcSetGeometryBuffer(
                geom_handle,
                root::RTCBufferType::INDEX,
                0,
                root::RTCFormat::UINT3,
                buffer_handle,
                0,
                12,
                num_triangles,
            );
        }
        unsafe {
            root::rtcCommitGeometry(geom_handle);
        }

        let geom_id = unsafe { root::rtcAttachGeometry(self.handle, geom_handle) };

        // Insert the new mesh into the geometry vector
        self.geometry.push(Arc::new(TriangleMesh {
            // handle: geom_handle,
            geom_id,
            vertices,
            normals,
            uv,
            indices,
        }));

        return Arc::clone(self.geometry.last().unwrap());
    }

    pub fn commit(self) -> Result<Scene, String> {
        unsafe { root::rtcCommitScene(self.handle) }
        Ok(Scene {
            handle: self.handle,
            geometry: self.geometry,
        })
    }
}

unsafe impl Send for Scene {}
unsafe impl Sync for Scene {}
pub struct Scene {
    handle: root::RTCScene,
    geometry: Vec<Arc<TriangleMesh>>,
}

impl Drop for Scene {
    fn drop(&mut self) {
        unsafe { root::rtcReleaseScene(self.handle) };
    }
}

impl Scene {
    pub fn mesh(&self, i: usize) -> &Arc<TriangleMesh> {
        &self.geometry[i]
    }

    /// Intersects a single ray with the scene.
    /// This function can only be called for scenes with the rtcIntersect1 flag
    /// set.
    /// TODO: Express this constraint through the type system so that a user
    /// can't call the wrong type of function
    pub fn intersect(&self, ray: Ray) -> Option<Intersection> {
        let mut rayhit = root::RTCRayHit {
            ray: ray.embree(),
            hit: root::RTCHit {
                Ng_x: 0.0,
                Ng_y: 0.0,
                Ng_z: 0.0,
                u: 0.0,
                v: 0.0,
                primID: std::u32::MAX,
                geomID: std::u32::MAX,
                instID: [std::u32::MAX; 1],
            },
        };
        let mut context = root::RTCIntersectContext {
            flags: root::RTCIntersectContextFlags::INCOHERENT,
            filter: None,
            instID: [std::u32::MAX; 1],
        };
        unsafe { root::rtcIntersect1(self.handle, &mut context, &mut rayhit) };
        Intersection::from_ray(&self, rayhit)
    }

    /// Tests if a single ray is occluded by the scene.
    /// This function can only be called for scenes with the rtcIntersect1 flag
    /// set.
    /// TODO: Express this constraint through the type system so that a user
    /// can't call the wrong type of function
    pub fn occluded(&self, ray: Ray) -> bool {
        let mut internal_ray = ray.embree();
        let mut context = root::RTCIntersectContext {
            flags: root::RTCIntersectContextFlags::INCOHERENT,
            filter: None,
            instID: [std::u32::MAX; 1],
        };
        unsafe { root::rtcOccluded1(self.handle, &mut context, &mut internal_ray) }
        internal_ray.tfar == std::f32::NEG_INFINITY
    }
}

/// Structure that represent a triangle mesh (vertices, normal, ...).
/// This structure need to be kept inside the rendering engine
/// to enable more advance sampling strategy
pub struct TriangleMesh {
    /// Geometry id from embree
    /// This id will be the same as the intersection/ray geometry ID
    // pub(crate) handle: &'a root::RTCGeometry,
    pub geom_id: u32,
    /// The list of all vertices
    pub vertices: Vec<Vector3<f32>>,
    /// The list of all normals
    pub normals: Vec<Vector3<f32>>,
    /// The list of all uv.
    /// This list can be empty
    pub uv: Vec<Vector2<f32>>,
    /// The list of all indices to defining the triangles.
    pub indices: Vec<u32>,
}

pub struct Ray {
    pub o: Point3<f32>,
    pub d: Vector3<f32>,
    pub tnear: f32,
    pub tfar: f32,
    pub time: f32,
}

mod constants {
    pub const EPSILON: f32 = 0.0001;
}

impl Ray {
    pub fn new(o: Point3<f32>, d: Vector3<f32>) -> Ray {
        Ray {
            o,
            d,
            tnear: constants::EPSILON,
            tfar: std::f32::MAX,
            time: 0.0,
        }
    }

    pub fn near(mut self, n: f32) -> Ray {
        self.tnear = n;
        self
    }

    pub fn far(mut self, f: f32) -> Ray {
        self.tfar = f;
        self
    }

    pub fn time(mut self, t: f32) -> Ray {
        self.time = t;
        self
    }

    pub fn embree(self) -> root::RTCRay {
        root::RTCRay {
            org_x: self.o.x,
            org_y: self.o.y,
            org_z: self.o.z,
            tnear: self.tnear,
            dir_x: self.d.x,
            dir_y: self.d.y,
            dir_z: self.d.z,
            time: self.time,
            tfar: self.tfar,
            mask: std::u32::MAX,
            id: 0,
            flags: 0,
            __bindgen_align: [],
        }
    }
}

/// Holds the data representing a successful ray intersection
pub struct Intersection {
    /// Intersection distance
    pub t: f32,
    /// Geometry normal
    pub n_g: Vector3<f32>,
    /// Shading normal
    pub n_s: Option<Vector3<f32>>,
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

impl Intersection {
    pub fn from_ray(scene: &Scene, rh: root::RTCRayHit) -> Option<Intersection> {
        if rh.hit.geomID != std::u32::MAX {
            let mesh = scene.mesh(rh.hit.geomID as usize);
            let i0 = mesh.indices[(rh.hit.primID * 3) as usize] as usize;
            let i1 = mesh.indices[(rh.hit.primID * 3 + 1) as usize] as usize;
            let i2 = mesh.indices[(rh.hit.primID * 3 + 2) as usize] as usize;

            // Normal interpolation
            // use the face normal to make the face shading normal consistent
            let n_g = Vector3::new(rh.hit.Ng_x, rh.hit.Ng_y, rh.hit.Ng_z).normalize();
            let n_s = if mesh.normals.is_empty() {
                None
            } else {
                let d0 = &mesh.normals[i0];
                let d1 = &mesh.normals[i1];
                let d2 = &mesh.normals[i2];
                let mut n_s = d0 * (1.0 - rh.hit.u - rh.hit.v) + d1 * rh.hit.u + d2 * rh.hit.v;
                if n_g.dot(n_s) < 0.0 {
                    n_s = -n_s;
                }
                Some(n_s)
            };

            // UV interpolation
            let uv = if mesh.uv.is_empty() {
                None
            } else {
                let d0 = &mesh.uv[i0];
                let d1 = &mesh.uv[i1];
                let d2 = &mesh.uv[i2];
                Some(d0 * (1.0 - rh.hit.u - rh.hit.v) + d1 * rh.hit.u + d2 * rh.hit.v)
            };

            // Construct the intersection
            return Some(Intersection {
                t: rh.ray.tfar,
                n_g,
                n_s,
                p: Point3::new(
                    rh.ray.org_x + rh.ray.tfar * rh.ray.dir_x,
                    rh.ray.org_y + rh.ray.tfar * rh.ray.dir_y,
                    rh.ray.org_z + rh.ray.tfar * rh.ray.dir_z,
                ),
                uv,
                geom_id: rh.hit.geomID,
                prim_id: rh.hit.primID,
                inst_id: rh.hit.instID[0],
            });
        } else {
            return None;
        }
    }
}
