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
        let cfg = CString::new("verbose=4").unwrap();
        Device {
            handle: unsafe { root::rtcNewDevice(cfg.as_ptr()) },
        }
    }

    pub fn get_error(&mut self) -> Option<root::RTCError> {
        let error = unsafe { root::rtcDeviceGetError(self.handle) };
        match error {
            root::RTCError::NONE => None,
            _x => Some(_x),
        }
    }

    pub fn commit(&mut self, scene: SceneConstruct) -> Result<Scene, String> {
        unsafe { root::rtcCommit(scene.handle) }

        match self.get_error() {
            None => Ok(Scene {
                handle: scene.handle,
                geometry: scene.geometry,
            }),
            Some(x) => Err(format!("Embree raised an error: {:?}", x)),
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
    pub fn new(device: &mut Device) -> SceneConstruct {
        let p = unsafe { root::rtcNewScene(device.handle) };

        SceneConstruct {
            handle: p,
            geometry: Vec::<Arc<TriangleMesh>>::new(),
        }
    }

    /// Add a new TriangleMesh with the supplied vertex and indices vectors.
    pub fn add_triangle_mesh(
        &mut self,
        device: &mut Device,
        geom_flags: GeometryFlags,
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

        let geom_id =
            unsafe { root::rtcNewGeometry(device.handle, root::RTCGeometryType::TRIANGLE) };

        // Set vertex buffer
        unsafe {
            let e_vertices = root::rtcMapBuffer(self.handle, geom_id, root::RTCBufferType::VERTEX)
                as *mut EmbreeVertex;

            for (i, v) in vertices.iter().enumerate() {
                (*e_vertices.offset(i as isize)).x = v.x;
                (*e_vertices.offset(i as isize)).y = v.y;
                (*e_vertices.offset(i as isize)).z = v.z;
                (*e_vertices.offset(i as isize)).w = 1.0;
            }
            root::rtcUnmapBuffer(self.handle, geom_id, root::RTCBufferType::VERTEX);
        }

        // Set index buffer
        unsafe {
            let e_indicies = root::rtcMapBuffer(self.handle, geom_id, root::RTCBufferType::INDEX)
                as *mut EmbreeTriangle;
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
            root::rtcUnmapBuffer(self.handle, geom_id, root::RTCBufferType::INDEX);
        }

        // Insert the new mesh into the geometry vector
        self.geometry.push(Arc::new(TriangleMesh {
            geom_id,
            vertices,
            normals,
            uv,
            indices,
        }));

        return Arc::clone(self.geometry.last().unwrap());
    }
}

pub struct Scene {
    handle: root::RTCScene,
    geometry: Vec<Arc<TriangleMesh>>,
}

impl Drop for Scene {
    fn drop(&mut self) {
        unsafe { root::rtcDeleteScene(self.handle) };
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
        let mut internal_ray = ray.embree();
        unsafe { root::rtcIntersect(self.handle, &mut internal_ray) };
        Intersection::from_ray(&self, internal_ray)
    }

    /// Tests if a single ray is occluded by the scene.
    /// This function can only be called for scenes with the rtcIntersect1 flag
    /// set.
    /// TODO: Express this constraint through the type system so that a user
    /// can't call the wrong type of function
    pub fn occluded(&self, ray: Ray) -> bool {
        let mut internal_ray = ray.embree();
        unsafe { root::rtcOccluded(self.handle, &mut internal_ray) }
        internal_ray.hit()
    }
}

/// Structure that represent a triangle mesh (vertices, normal, ...).
/// This structure need to be kept inside the rendering engine
/// to enable more advance sampling strategy
pub struct TriangleMesh {
    /// Geometry id from embree
    /// This id will be the same as the intersection/ray geometry ID
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
            mask: 0xFFFFFFFF,
            Ng: [0.0, 0.0, 0.0],
            align2: 0.0,
            u: 0.0,
            v: 0.0,
            geomID: INVALID_GEOMETRY_ID,
            primID: INVALID_GEOMETRY_ID,
            instID: INVALID_GEOMETRY_ID,
            __bindgen_padding_0: [0; 3usize],
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

impl Intersection {
    pub fn from_ray(scene: &Scene, ray: root::RTCRay) -> Option<Intersection> {
        if ray.hit() {
            let mesh = scene.mesh(ray.id as usize);
            let i0 = mesh.indices[(ray.primID * 3) as usize] as usize;
            let i1 = mesh.indices[(ray.primID * 3 + 1) as usize] as usize;
            let i2 = mesh.indices[(ray.primID * 3 + 2) as usize] as usize;

            // Normal interpolation
            let d0 = &mesh.normals[i0];
            let d1 = &mesh.normals[i1];
            let d2 = &mesh.normals[i2];
            let n_s = d0 * (1.0 - ray.u - ray.v) + d1 * ray.u + d2 * ray.v;

            // UV interpolation
            let uv: Option<Vector2<f32>>;
            if mesh.uv.is_empty() {
                uv = None;
            } else {
                let d0 = &mesh.uv[i0];
                let d1 = &mesh.uv[i1];
                let d2 = &mesh.uv[i2];
                uv = Some(d0 * (1.0 - ray.u - ray.v) + d1 * ray.u + d2 * ray.v);
            }

            let mut n_g = Vector3::new(ray.Ng[0], ray.Ng[1], ray.Ng[2]).normalize();
            if n_g.dot(n_s) < 0.0 {
                n_g = -n_g;
            }

            // Construct the intersection
            return Some(Intersection {
                t: ray.tfar,
                n_g,
                n_s,
                p: Point3::new(
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
