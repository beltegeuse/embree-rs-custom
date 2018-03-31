extern crate cgmath;
use cgmath::{Vector3,Vector2};

pub use ray::*;
use super::root;

use std::ptr;
use std::sync::Arc;



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

    pub fn get_error(&mut self) -> Option<root::RTCError> {
        let error = unsafe { root::rtcDeviceGetError(self.handle) };
        match error {
            root::RTCError_RTC_NO_ERROR => None,
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
        unsafe { root::rtcDeleteDevice(self.handle) };
    }
}

/// Internal vertex representation
#[repr(C)]
struct EmbreeVertex {
    x : f32,
    y : f32,
    z : f32,
    w : f32,
}

/// Internal triangle presentation
#[repr(C)]
struct EmbreeTriangle {
    i0 : i32,
    i1 : i32,
    i2 : i32
}

/// The object used to pass data to Embree.
/// The commit function need to be called in order to get Scene object.
/// Only Scene object provides intersection routines.
pub struct SceneConstruct {
    handle: root::RTCScene,
    geometry: Vec<Arc<TriangleMesh>>,
}

impl SceneConstruct {
    pub fn new(device: &mut Device, sflags: root::RTCSceneFlags, aflags: root::RTCAlgorithmFlags) -> SceneConstruct {
        let p = unsafe {
            root::rtcDeviceNewScene(device.handle, sflags, aflags)
        };

        SceneConstruct {
            handle: p,
            geometry: Vec::<Arc<TriangleMesh>>::new(),
        }
    }

    /// Add a new TriangleMesh with the supplied vertex and indices vectors.
    pub fn add_triangle_mesh(&mut self,
                             geom_flags: root::RTCGeometryFlags,
                             vertices: Vec<Vector3<f32>>,
                             normals: Vec<Vector3<f32>>,
                             uv: Vec<Vector2<f32>>,
                             indices: Vec<u32>) -> Arc<TriangleMesh> {
        assert_eq!(indices.len() % 3, 0, "Indices must be a multiple of 3 for a triangle mesh but \
         indices Vec has length {}", indices.len());
        let num_triangles = indices.len() / 3;

        let geom_id = unsafe { root::rtcNewTriangleMesh(self.handle,
                                                  geom_flags,
                                                  num_triangles,
                                                  vertices.len(),
                                                  1) };

        // Set vertex buffer
        unsafe {
            let e_vertices =  root::rtcMapBuffer(
                self.handle,
                geom_id,
                root::RTCBufferType_RTC_VERTEX_BUFFER0) as *mut EmbreeVertex ;

            for (i,v) in vertices.iter().enumerate() {
                (*e_vertices.offset(i as isize)).x = v.x;
                (*e_vertices.offset(i as isize)).y = v.y;
                (*e_vertices.offset(i as isize)).z = v.z;
                (*e_vertices.offset(i as isize)).w = 1.0;

            }
            root::rtcUnmapBuffer(self.handle,
                                 geom_id,
                                 root::RTCBufferType_RTC_VERTEX_BUFFER);
        }

        // Set index buffer
        unsafe {
            let e_indicies = root::rtcMapBuffer(self.handle, geom_id,
                                                root::RTCBufferType_RTC_INDEX_BUFFER) as *mut EmbreeTriangle;
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
            root::rtcUnmapBuffer(self.handle, geom_id,
                                 root::RTCBufferType_RTC_INDEX_BUFFER);
        }

        // Insert the new mesh into the geometry vector
        self.geometry.push(Arc::new(TriangleMesh {
            geom_id,
            vertices,
            normals,
            uv,
            indices,
        }));

        return Arc::clone(self.geometry.last().unwrap())
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
    pub fn mesh(&self, i: usize) -> &Arc<TriangleMesh>{
        &self.geometry[i]
    }

    /// Intersects a single ray with the scene.
    /// This function can only be called for scenes with the rtcIntersect1 flag
    /// set.
    /// TODO: Express this constraint through the type system so that a user
    /// can't call the wrong type of function
    pub fn intersect(&self, mut ray: root::RTCRay) -> Option<Intersection> {
        unsafe { root::rtcIntersect(self.handle, &mut ray) };
        Intersection::from_ray(&self, ray)
    }

    /// Tests if a single ray is occluded by the scene.
    /// This function can only be called for scenes with the rtcIntersect1 flag
    /// set.
    /// TODO: Express this constraint through the type system so that a user
    /// can't call the wrong type of function
    pub fn occluded(&self, mut ray: root::RTCRay) -> bool {
        unsafe { root::rtcOccluded(self.handle, &mut ray) }
        ray.hit()
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