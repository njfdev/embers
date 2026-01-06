use embedded_io_async::{Read, Write};
use num_traits::FromPrimitive;

use crate::gps::GpsFixType;

pub const SUGGESTED_UBLOX_BUFFER_SIZE: usize = 8192;

pub struct UBlox<R: Read, T: Write, const N: usize> {
    rx: R,
    tx: T,

    buf: [u8; N],
    buf_len: usize,

    latest_gps: GPSPayload,
    latest_received: bool,
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

impl<R: Read, T: Write, const N: usize> UBlox<R, T, N> {
    pub fn new(rx: R, tx: T) -> Self {
        Self {
            rx,
            tx,

            buf: [0; N],
            buf_len: 0,

            latest_gps: GPSPayload::default(),
            latest_received: true,
        }
    }

    pub async fn update(&mut self) -> Result<(), String> {
        let mut rx_buf = [0; N];
        match self.rx.read(&mut rx_buf).await {
            Ok(0) => return Ok(()),
            Ok(n) => {
                if n + self.buf_len <= N {
                    self.buf[self.buf_len..(self.buf_len + n)].copy_from_slice(&rx_buf[..n]);
                    self.buf_len += n;
                } else {
                    self.buf_len = 0;
                    return Err(
                        "GPS UART Buffer is too full... Resetting buffer and dropping bytes."
                            .to_string(),
                    );
                }
            }
            Err(e) => {
                return Err(e.to_string());
            }
        }

        if self.buf_len < 2 {
            return Ok(());
        }

        // If buffer doesn't start with expected packet framing, adjust to find framing bytes
        if self.buf[0] != 0xb5 || self.buf[1] != 0x62 {
            let new_starting;
            for (i, byte) in self.buf[..self.buf_len].iter().enumerate() {
                if *byte == 0xb5 {
                    new_starting = i;
                    self.buf.copy_within(new_starting.., 0);
                    self.buf_len -= new_starting;
                    break;
                }
            }
        }

        // In this case, no framing bytes have appeared yet
        if self.buf[0] != 0xb5 || self.buf[1] != 0x62 {
            return Ok(());
        }

        // the length bytes are at bytes 5-6, so continue if we don't have that much yet
        if self.buf_len < 6 {
            return Ok(());
        }

        // process packet if it is the correct length (otherwise continue to wait for more bytes)
        let len = self.buf[4] as usize + ((self.buf[5] as usize) << 8) + 8; // 8 bytes not included in payload length
        if self.buf_len >= len {
            let result = self.handle_packet();

            // now remove it
            self.buf.copy_within(len.., 0);
            self.buf_len -= len;

            return result;
        }

        return Ok(());
    }

    fn handle_packet(&mut self) -> Result<(), String> {
        let message_class = self.buf[2];
        let message_id = self.buf[3];
        let payload = &self.buf[6..(self.buf_len - 2)];
        let payload_len = self.buf_len - 8;

        match message_class {
            // RC Channels Packed Payload
            0x01 => match message_id {
                0x07 => {
                    let itow = payload[0] as u32
                        + ((payload[1] as u32) << 8)
                        + ((payload[2] as u32) << 16)
                        + ((payload[3] as u32) << 24);

                    let year = payload[4] as u16 + ((payload[5] as u16) << 8);

                    let month = payload[6];

                    let day = payload[7];

                    let hour = payload[8];

                    let min = payload[9];

                    let sec = payload[10];

                    let nano = payload[16] as i32
                        + ((payload[17] as i32) << 8)
                        + ((payload[18] as i32) << 16)
                        + ((payload[19] as i32) << 24);

                    let time_accuracy = payload[12] as u32
                        + ((payload[13] as u32) << 8)
                        + ((payload[14] as u32) << 16)
                        + ((payload[15] as u32) << 24);

                    let fix_type = GpsFixType::from_u8(payload[20]).unwrap_or(GpsFixType::NoFix);

                    let is_fix_valid = payload[21] & 1 == 1;

                    let is_date_valid = payload[22] & 0b1000000 == 1;

                    let is_time_valid = payload[22] & 0b10000000 == 1;

                    let num_sats = payload[23];

                    let lon = ((payload[24] as i32
                        + ((payload[25] as i32) << 8)
                        + ((payload[26] as i32) << 16)
                        + ((payload[27] as i32) << 24)) as f64)
                        / 10_000_000.0;

                    let lat = ((payload[28] as i32
                        + ((payload[29] as i32) << 8)
                        + ((payload[30] as i32) << 16)
                        + ((payload[31] as i32) << 24)) as f64)
                        / 10_000_000.0;

                    let height_ellipsoid = ((payload[32] as i32
                        + ((payload[33] as i32) << 8)
                        + ((payload[34] as i32) << 16)
                        + ((payload[35] as i32) << 24))
                        as f32)
                        / 1_000.0;

                    let height_msl = ((payload[36] as i32
                        + ((payload[37] as i32) << 8)
                        + ((payload[38] as i32) << 16)
                        + ((payload[39] as i32) << 24))
                        as f32)
                        / 1_000.0;

                    let h_acc = ((payload[40] as u32
                        + ((payload[41] as u32) << 8)
                        + ((payload[42] as u32) << 16)
                        + ((payload[43] as u32) << 24)) as f32)
                        / 1_000.0;

                    let v_acc = ((payload[44] as u32
                        + ((payload[45] as u32) << 8)
                        + ((payload[46] as u32) << 16)
                        + ((payload[47] as u32) << 24)) as f32)
                        / 1_000.0;

                    let vel_n = ((payload[48] as i32
                        + ((payload[49] as i32) << 8)
                        + ((payload[50] as i32) << 16)
                        + ((payload[51] as i32) << 24)) as f32)
                        / 1_000.0;

                    let vel_e = ((payload[52] as i32
                        + ((payload[53] as i32) << 8)
                        + ((payload[54] as i32) << 16)
                        + ((payload[55] as i32) << 24)) as f32)
                        / 1_000.0;

                    let vel_d = ((payload[56] as i32
                        + ((payload[57] as i32) << 8)
                        + ((payload[58] as i32) << 16)
                        + ((payload[59] as i32) << 24)) as f32)
                        / 1_000.0;

                    let ground_speed = ((payload[60] as i32
                        + ((payload[61] as i32) << 8)
                        + ((payload[62] as i32) << 16)
                        + ((payload[63] as i32) << 24))
                        as f32)
                        / 1_000.0;

                    let motion_heading = ((payload[64] as i32
                        + ((payload[65] as i32) << 8)
                        + ((payload[66] as i32) << 16)
                        + ((payload[67] as i32) << 24))
                        as f32)
                        / 100_000.0;

                    let speed_acc = ((payload[68] as u32
                        + ((payload[69] as u32) << 8)
                        + ((payload[70] as u32) << 16)
                        + ((payload[71] as u32) << 24))
                        as f32)
                        / 1_000.0;

                    let heading_acc = ((payload[72] as u32
                        + ((payload[73] as u32) << 8)
                        + ((payload[74] as u32) << 16)
                        + ((payload[75] as u32) << 24))
                        as f32)
                        / 100_000.0;

                    let p_dop = ((payload[72] as u16 + ((payload[73] as u16) << 8)) as f32) / 100.0;

                    let gps_payload = GPSPayload {
                        itow,
                        year,
                        month,
                        day,
                        hour,
                        minute: min,
                        second: sec,
                        nanoseconds: nano,
                        time_accuracy,
                        fix_type,
                        valid_fix: is_fix_valid,
                        valid_date: is_date_valid,
                        valid_time: is_time_valid,
                        sat_num: num_sats,
                        longitude: lon,
                        latitude: lat,
                        ellipsoid_height: height_ellipsoid,
                        msl_height: height_msl,
                        horizontal_accuracy_estimate: h_acc,
                        vertical_accuracy_estimate: v_acc,
                        north_vel: vel_n,
                        east_vel: vel_e,
                        down_vel: vel_d,
                        ground_speed,
                        ground_speed_accuracy: speed_acc,
                        motion_heading,
                        heading_accuracy: heading_acc,
                        dop: p_dop,
                    };

                    self.latest_gps = gps_payload;
                    self.latest_received = false;

                    return Ok(());
                }
                _ => {
                    return Err("GPS Unhandled message id".to_string()); //: {:2x?}", message_id;
                }
            },
            _ => {
                return Err("GPS Unhandled message class".to_string()); //: {:2x?}", message_class);
            }
        }
    }
}
