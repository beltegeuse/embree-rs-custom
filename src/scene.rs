extern crate cgmath;
use cgmath::{Vector3,Vector2};

use ffi::*;
pub use ray::*;

use std::ptr;
use std::sync::Arc;

/// Wrapper around the RTCDevice opaque struct.
/// Embree supports a device concept which allows different components of the
/// application to use the API without interfering with one another.
/// An application typically creates a single device only and should not create
/// many of them.
pub struct Device<'device> {
    handle: &'device mut RTCDevice,
}

impl<'device> Device<'device> {
    /// Creates a new Device
    pub fn new() -> Device<'device> {
        let p = unsafe { rtcNewDevice(ptr::null()) };
        // If the RTCDevice is null then something very bad has happened and we
        // can't continue
        assert!(!p.is_null());
        Device { handle:unsafe {&mut * p} }
    }

    pub fn get_error(&mut self) -> Option<RTCError> {
        let error = unsafe { rtcDeviceGetError(self.handle) };
        match error {
            RTCError::RtcNoError => None,
            _x => Some(_x),
        }
    }

    pub fn commit(&mut self, scene: SceneConstruct<'device>) -> Result<Scene<'device>, String> {
        unsafe { rtcCommit(scene.handle) }

        match self.get_error() {
            None => Ok(Scene {
                handle: scene.handle,
                geometry: scene.geometry,
            }),
            Some(x) => Err(format!("Embree raised an error: {:?}", x)),
        }
    }
}

impl<'device> Drop for Device<'device> {
    fn drop(&mut self) {
        unsafe { rtcDeleteDevice(self.handle) };
    }
}

bitflags! {
    /// Flags to configure the scene
    pub struct SceneFlags: i32 {
        const STATIC       = (0 << 0); /// Static spatial data structure
        const DYNAMIC      = (0 << 1); /// Dynamic spatial data structure

        const COMPACT      = (1 << 8); /// Prefer to have compact spatial data structure
        const COHERENT     = (1 << 9); /// Prefer high-performance spatial data structure for coherent rays
        const INCOHERENT   = (1 << 10); /// Prefer high-performance spatial data structure for incoherent rays
        const HIGH_QUALITY = (1 << 11); /// Prefer high-quality spatial data structure (more costly to construct)

        const ROBUST       = (1 << 16);
    }
}

bitflags! {
    /// Flags to configure which ray query we will request
    /// # NOTES
    /// Currently only a single intersection is supported.
    pub struct AlgorithmFlags: i32 {
        const INTERSECT1  = 0b00000001;
        const INTERSECT4  = 0b00000010;
        const INTERSECT8  = 0b00000100;
        const INTERSECT16 = 0b00001000;
        const INTERPOLATE = 0b00010000;

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
pub struct SceneConstruct<'device> {
    handle: &'device mut RTCScene,
    geometry: Vec<Arc<TriangleMesh>>,
}

impl<'device> SceneConstruct<'device> {
    pub fn new(device: &mut Device<'device>, sflags: SceneFlags, aflags: AlgorithmFlags) -> SceneConstruct<'device> {
        let p = unsafe {
            rtcDeviceNewScene(device.handle, sflags.bits as i32, aflags.bits as i32)
        };

        SceneConstruct {
            handle: unsafe {&mut * p},
            geometry: Vec::<Arc<TriangleMesh>>::new(),
        }
    }

    /// Add a new TriangleMesh with the supplied vertex and indices vectors.
    pub fn add_triangle_mesh(&mut self,
                             geom_flags: GeometryFlags,
                             vertices: Vec<Vector3<f32>>,
                             normals: Vec<Vector3<f32>>,
                             uv: Vec<Vector2<f32>>,
                             indices: Vec<u32>) -> Arc<TriangleMesh> {
        assert_eq!(indices.len() % 3, 0, "Indices must be a multiple of 3 for a triangle mesh but \
         indices Vec has length {}", indices.len());
        let num_triangles = indices.len() / 3;

        let geom_id = unsafe { rtcNewTriangleMesh(self.handle,
                                                  geom_flags as i32,
                                                  num_triangles,
                                                  vertices.len(),
                                                  1) };

        // Set vertex buffer
        unsafe {
            let e_vertices =  rtcMapBuffer(self.handle, geom_id,
                                           BufferType::VertexBuffer0 as i32) as *mut EmbreeVertex ;

            for (i,v) in vertices.iter().enumerate() {
                (*e_vertices.offset(i as isize)).x = v.x;
                (*e_vertices.offset(i as isize)).y = v.y;
                (*e_vertices.offset(i as isize)).z = v.z;
                (*e_vertices.offset(i as isize)).w = 1.0;

            }
            rtcUnmapBuffer(self.handle, geom_id, BufferType::VertexBuffer0 as i32);
        }

        // Set index buffer
        unsafe {
            let e_indicies = rtcMapBuffer(self.handle, geom_id,
                                          BufferType::IndexBuffer as i32) as *mut EmbreeTriangle;
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
            rtcUnmapBuffer(self.handle, geom_id, BufferType::IndexBuffer as i32);
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

pub struct Scene<'device> {
    handle: &'device mut RTCScene,
    geometry: Vec<Arc<TriangleMesh>>,
}

impl<'device> Drop for Scene<'device> {
    fn drop(&mut self) {
        unsafe { rtcDeleteScene(self.handle) };
    }
}

impl<'device> Scene<'device> {
    pub fn mesh(&self, i: usize) -> &Arc<TriangleMesh>{
        &self.geometry[i]
    }

    /// Intersects a single ray with the scene.
    /// This function can only be called for scenes with the rtcIntersect1 flag
    /// set.
    /// TODO: Express this constraint through the type system so that a user
    /// can't call the wrong type of function
    pub fn intersect(&self, mut ray: Ray) -> Option<Intersection> {
        unsafe { rtcIntersect(self.handle, &mut ray) };
        Intersection::from_ray(&self, ray)
    }

    /// Tests if a single ray is occluded by the scene.
    /// This function can only be called for scenes with the rtcIntersect1 flag
    /// set.
    /// TODO: Express this constraint through the type system so that a user
    /// can't call the wrong type of function
    pub fn occluded(&self, mut ray: Ray) -> bool {
        unsafe { rtcOccluded(self.handle, &mut ray) }
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

enum_from_primitive! {
/// Enum describing the type of buffer to map.
/// Note that the original flags from embree contain duplicates so in Rust
/// VertexBuffer becomes VertexBuffer0 etc.
        enum BufferType {
            IndexBuffer              = 0x01000000,

            VertexBuffer0            = 0x02000000,
            VertexBuffer1            = 0x02000001,

            UserVertexBuffer0        = 0x02100000,
            UserVertexBuffer1        = 0x02100001,

            FaceBuffer               = 0x03000000,
            LevelBuffer              = 0x04000001,

            EdgeCreaseIndexBuffer    = 0x05000000,
            EdgeCreaseWeightBuffer   = 0x06000000,

            VertexCreaseIndexBuffer  = 0x07000000,
            VertexCreaseWeightBuffer = 0x08000000,

            HoleBuffer               = 0x09000001,
        }
    }

enum_from_primitive! {
        /// For requesting a particular behavior for a given triangle mesh
        pub enum GeometryFlags {
            Static     = 0,
            Deformable = 1,
            Dynamic    = 2,
        }
    }
