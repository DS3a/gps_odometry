use utm;
use ndarray::arr2;
use nalgebra::{Vector2, Matrix2};
use std::sync::Mutex;

pub struct Odometry {
    datum_position: Vector2<f64>,
    datum_heading: f64, // in radians
    datum_lat: f64,
    datum_long: f64,
    datum_northing: f64,
    datum_easting: f64,
    position: Mutex<Vector2<f64>>,
    yaw: Mutex<f64>,
}

impl Odometry {
    pub fn new(datum_position: Vector2<f64>, datum_heading: f64, datum_lat: f64, datum_long: f64) -> Self {
        let (datum_northing, datum_easting, _initial_meridian_convergence)
            = utm::to_utm_wgs84_no_zone(datum_lat, datum_long);

        Self {
            datum_position,
            datum_heading,
            datum_lat,
            datum_long,
            datum_northing,
            datum_easting,
            position: Mutex::new(datum_position),
            yaw: Mutex::new(datum_heading), // can't get this
        }
    }

    pub fn transform_utm_to_local_frames(&self, relative_northing: f64, relative_easting: f64) -> Vector2<f64> {
        /*
         * returns (x, y)
         * X-axis => North axis
         * Y-axis => East axis
         */

        let inv_heading = -self.datum_heading;
        let cosine = inv_heading.cos();
        let sine = inv_heading.sin();
        Matrix2::new(cosine, -sine, sine, cosine) * Vector2::new(relative_northing, relative_easting)
    }

    pub fn update_odom(&self, lat: f64, long: f64) {
        let (northing, easting, _meridian_convergence) = utm::to_utm_wgs84_no_zone(lat, long);
        let relative_northing = northing - self.datum_northing;
        let relative_easting = easting - self.datum_easting;
        let transformation = self.transform_utm_to_local_frames(relative_northing, relative_easting);
        *self.position.lock().unwrap() += transformation;
    }
}

/*
TODO decide if magnetometer can be used to get stuff done or if odometry has to be used
*/
pub fn get_heading(mag_x: f64, mag_y: f64) -> f64 {
    // angle with respect to north
    // CW = +ve ??
    // ACW = +ve
    // N = 0 degrees
    // W = 90 degrees
    // S = 180 degrees
    // E = -90 degrees


    // TODO add magnetic declination based on current lat long etc.
    // output in radians
    let heading = -mag_y.atan2(mag_x);

    static utm_to_map: Matrix2<f64> = Matrix2::new(1f64, 1f64, 1f64, 1f64);

    return heading;
}


// Easting = eastward measured distance
// Northing = nortward measured distance
//
pub fn map_to_utm_tf(map_x: f64, map_y: f64, utm_n: f64, utm_e: f64) -> f64 {
    0.0f64
}

// TODO function to get the x and y in map frame given northing and easting


// NO clue how to proceed
