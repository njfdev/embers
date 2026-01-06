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
