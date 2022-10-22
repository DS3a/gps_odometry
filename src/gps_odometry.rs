use utm;
use ndarray::arr2;

/*
TODO decide if magnetometer can be used to get stuff done or if odometry has to be used
*/

pub fn get_heading(mag_x: f64, mag_y: f64) -> f64 {
    // angle with respect to north
    // CW = +ve ??
    // ACW = +ve
    mag_y.atan2(mag_x) * 180.0 / std::f64::consts::PI
}


// Easting = eastward measured distance
// Northing = nortward measured distance
//
pub fn map_to_utm_tf(map_x: f64, map_y: f64, utm_x: f64, utm_y: f64) -> f64 {
    0.0f64
}


// NO clue how to proceed
