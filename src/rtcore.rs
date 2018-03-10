extern crate cgmath;
use cgmath::{Vector3,Vector2};

use rtcore_ffi::*;
pub use rtcore_ray::*;
use std::ptr;
use std::sync::Arc;

/// Wrapper around the RTCDevice opaque struct.
/// Embree supports a device concept which allows different components of the
/// application to use the API without interfering with one another.
/// An application typically creates a single device only and should not create
/// many of them.
pub struct Device<'a> {
    ptr: &'a mut RTCDevice,
}

impl<'a> Device<'a> {
    /// Creates a new Device
    pub fn new() -> Device<'a> {
        let p = unsafe { rtcNewDevice(ptr::null()) };
        // If the RTCDevice is null then something very bad has happened and we
        // can't continue
        assert!(!p.is_null());
        Device { ptr: unsafe { &mut *p } }
    }

    /// Creates a new Scene attached to this Device
    pub fn new_scene(&mut self,
                     sflags: SceneFlags,
                     aflags: AlgorithmFlags)
                     -> Scene<'a> {
        let p = unsafe {
            rtcDeviceNewScene(self.ptr, sflags.bits as i32, aflags.bits as i32)
        };
        // If the RTCScene is null then something very bad has happened and we
        // can't continue
        assert!(!p.is_null());
        Scene::new(p)
    }
}

impl<'a> Drop for Device<'a> {
    fn drop(&mut self) {
        unsafe { rtcDeleteDevice(self.ptr) };
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

/// Wrapper struct representing a scene, i.e. a collection of geometry to be
/// traced.
pub struct Scene<'a> {
    ptr: &'a mut RTCScene,
    geometry: Vec<Arc<TriangleMesh>>,
}

impl<'a> Drop for Scene<'a> {
    fn drop(&mut self) {
        unsafe { rtcDeleteScene(self.ptr) };
    }
}

impl<'a> Scene<'a> {
    fn new(p: *mut RTCScene) -> Scene<'a> {
        Scene {
            ptr: unsafe { &mut *p },
            geometry: Vec::<Arc<TriangleMesh>>::new(),
        }
    }

    pub fn mesh(&self, i: usize) -> &Arc<TriangleMesh>{
        &self.geometry[i]
    }

    /// Must be called after creating geometry and before tracing any rays
    /// TODO: Express this constraint through the type system so the user
    /// can't call intersect on an uncommited scene, and can't specify new
    /// geometry once the scene is committed.
    pub fn commit(&mut self) {
        unsafe { rtcCommit(self.ptr) }
    }

    /// Intersects a single ray with the scene.
    /// This function can only be called for scenes with the rtcIntersect1 flag
    /// set.
    /// TODO: Express this constraint through the type system so that a user
    /// can't call the wrong type of function
    pub fn intersect(&self, mut ray: Ray) -> Option<Intersection> {
        unsafe { rtcIntersect(self.ptr, &mut ray) };
        Intersection::from_ray(&self, ray)
    }

    /// Tests if a single ray is occluded by the scene.
    /// This function can only be called for scenes with the rtcIntersect1 flag
    /// set.
    /// TODO: Express this constraint through the type system so that a user
    /// can't call the wrong type of function
    pub fn occluded(&self, ray: &mut Ray) {
        unsafe { rtcIntersect(self.ptr, ray) }
    }

    /// Create a new TriangleMesh with the supplied vertex and indices vectors.
    pub fn new_triangle_mesh(&mut self,
                             geom_flags: GeometryFlags,
                             vertices: Vec<Vector3<f32>>,
                             normals: Vec<Vector3<f32>>,
                             uv: Vec<Vector2<f32>>,
                             indices: Vec<u32>) -> Arc<TriangleMesh> {
        assert!(indices.len() % 3 == 0,
                "Indices must be a multiple of 3 for a triangle mesh but \
                 indices Vec has length {}",
                indices.len());
        let num_triangles = indices.len() / 3;

        let geom_id = unsafe { rtcNewTriangleMesh(self.ptr,
                                         geom_flags as i32,
                                         num_triangles,
                                         vertices.len(),
                                         1) };

        // Set vertex buffer
        unsafe {
            let e_vertices =  rtcMapBuffer(self.ptr, geom_id,
                         BufferType::VertexBuffer0 as i32) as *mut EmbreeVertex ;

            for (i,v) in vertices.iter().enumerate() {
                (*e_vertices.offset(i as isize)).x = v.x;
                (*e_vertices.offset(i as isize)).y = v.y;
                (*e_vertices.offset(i as isize)).z = v.z;
                (*e_vertices.offset(i as isize)).w = 1.0;

            }
            rtcUnmapBuffer(self.ptr, geom_id, BufferType::VertexBuffer0 as i32);
        }

        // Set index buffer
        unsafe {
            let e_indicies = rtcMapBuffer(self.ptr, geom_id,
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
            rtcUnmapBuffer(self.ptr, geom_id, BufferType::IndexBuffer as i32);
        }

        // Insert the new mesh into the geometry vector
        self.geometry.push(Arc::new(TriangleMesh {
            geom_id: geom_id,
            vertices: vertices,
            normals: normals,
            uv: uv,
            indices: indices,
        }));

        return Arc::clone(self.geometry.last().unwrap())
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
        enum MatrixType {
            RowMajor             = 0,
            ColumnMajor          = 1,
            ColumnMajorAligned16 = 2,
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

enum_from_primitive! {
        enum BoundaryMode {
            None          = 0,
            EdgeOnly      = 1,
            EdgeAndCorner = 2,
        }
    }
