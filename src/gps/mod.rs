use num_derive::FromPrimitive;

pub mod ublox;

#[derive(FromPrimitive, Clone, Copy, Debug, Default)]
pub enum GpsFixType {
    #[default]
    NoFix = 0,
    DeadReckoningOnly = 1,
    Fix2D = 2,
    Fix3D = 3,
    DeadReckoningAndGNSS = 4,
    TimeOnly = 5,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct GPSPayload {
    pub itow: u32,
    pub year: u16,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
    pub nanoseconds: i32,
    pub time_accuracy: u32,
    pub fix_type: GpsFixType,
    pub valid_fix: bool,
    pub valid_date: bool,
    pub valid_time: bool,
    pub sat_num: u8,
    pub longitude: f64,
    pub latitude: f64,
    // in meters
    pub ellipsoid_height: f32,
    pub msl_height: f32,
    pub horizontal_accuracy_estimate: f32,
    pub vertical_accuracy_estimate: f32,
    // in m/s
    pub north_vel: f32,
    pub east_vel: f32,
    pub down_vel: f32,
    pub ground_speed: f32,
    pub ground_speed_accuracy: f32,
    pub motion_heading: f32,
    pub heading_accuracy: f32,
    // dilution of position
    pub dop: f32,
}
