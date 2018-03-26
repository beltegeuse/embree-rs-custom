use libc::{c_char, c_int, uint32_t, c_void, size_t};
use ray::Ray;

pub enum RTCDevice {}
pub enum RTCScene {}

#[repr(u32)]
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum RTCError {
    RtcNoError = 0,
    RtcUnknownError = 1,
    RtcInvalidArgument = 2,
    RtcInvalidOperation = 3,
    RtcOutOfMemory = 4,
    RtcUnsupportedCpu = 5,
    RtcCancelled = 6,
}

#[allow(dead_code)]
#[link(name="embree")]
extern "C" {
    pub fn rtcNewDevice(cfg: *const c_char) -> *mut RTCDevice;
    pub fn rtcDeleteDevice(d: *mut RTCDevice);

    pub fn rtcDeviceNewScene(d: *mut RTCDevice,
                             sflags: c_int,
                             aflags: c_int)
                             -> *mut RTCScene;
    pub fn rtcDeleteScene(s: *mut RTCScene);

    pub fn rtcCommit(s: *mut RTCScene);

    pub fn rtcIntersect(s: *const RTCScene, ray: *mut Ray);
    pub fn rtcOccluded(s: *const RTCScene, ray: *mut Ray);

    pub fn rtcNewTriangleMesh(s: *mut RTCScene,
                              geom_flags: c_int,
                              num_triangles: size_t,
                              num_vertices: size_t,
                              num_time_steps: size_t)
                              -> uint32_t;

    pub fn rtcDeleteGeometry(s: *mut RTCScene, geom_id: uint32_t);

    pub fn rtcSetBuffer(s: *mut RTCScene,
                        geom_id: uint32_t,
                        buffer_type: c_int,
                        ptr: *const c_void,
                        offset: size_t,
                        stride: size_t);

    pub fn rtcSetUserData(s: *mut RTCScene, geom_id: uint32_t, ptr: *const c_void);
    pub fn rtcGetUserData(s: *const RTCScene, geom_id: uint32_t) -> *mut c_void;

    pub fn rtcMapBuffer(s: *mut RTCScene, geom_id: uint32_t,
                        buffer_type: c_int) -> *mut c_void;
    pub fn rtcUnmapBuffer(s: *mut RTCScene, geom_id: uint32_t,
                          buffer_type: c_int);

    pub fn rtcDeviceGetError(device: *mut RTCDevice) -> RTCError;
}
