use anyhow::{Error, Result};
use std::env;
use gps_odometry;
use rclrs;
use nav_msgs::msg::Odometry as OdometryMsg;
use sensor_msgs::msg::NavSatFix as NavSatFixMsg;
use sensor_msgs::msg::Imu as ImuMsg;
use sensor_msgs::msg::MagneticField as MagneticFieldMsg;
use std::sync::{Arc, Mutex};
use std::sync::mpsc;
use std::sync::mpsc::{Sender, Receiver};
use std::thread;


fn main() -> Result<(), Error> {
    let context = rclrs::Context::new(env::args())?;

    let gnss_msg_global_handler = Arc::new(Mutex::new(Option::<NavSatFixMsg>::None));
    let imu_msg_global_handler = Arc::new(Mutex::new(ImuMsg::default()));
    let mag_msg_global_handler = Arc::new(Mutex::new(Option::<MagneticFieldMsg>::None));
    let odom_msg_global_handler = Arc::new(Mutex::new(Option::<OdometryMsg>::None));


    // this channel will be used to wake up the thread that will be used to unfreeze the odom publisher thread whenever a gps ping is recieved
    let (gps_msg_tx, gps_msg_rx): (Sender<bool>, Receiver<bool>) = mpsc::channel();
    let mut node = rclrs::create_node(&context, "gps_odometry_node").unwrap();

    let odom_publisher = node.create_publisher::<OdometryMsg>("/odometry/gps", rclrs::QOS_PROFILE_DEFAULT).unwrap();

    let gnss_msg_subscriber_handler = Arc::clone(&gnss_msg_global_handler);
    let gps_msg_subscriber_tx = gps_msg_tx.clone();
    let _gnss_subscriber = node.create_subscription::<NavSatFixMsg,  _>("/gnss", rclrs::QOS_PROFILE_DEFAULT, move |msg: NavSatFixMsg| {
        *gnss_msg_subscriber_handler.lock().unwrap() = Some(msg);
        gps_msg_tx.send(true).unwrap();
    });

    let imu_msg_subscriber_handler = Arc::clone(&imu_msg_global_handler);
    let _imu_subscriber = node.create_subscription::<ImuMsg, _>("/imu/filtered", rclrs::QOS_PROFILE_DEFAULT, move |msg: ImuMsg| {
        *imu_msg_subscriber_handler.lock().unwrap() = msg;
    });

    let mag_msg_subscriber_handler = Arc::clone(&mag_msg_global_handler);
    let _mag_subscriber = node.create_subscription::<MagneticFieldMsg, _>("/imu/mag", rclrs::QOS_PROFILE_DEFAULT, move |msg: MagneticFieldMsg| {
        *mag_msg_subscriber_handler.lock().unwrap() = Some(msg);
    });

    thread::spawn(move || {
        rclrs::spin(&node).unwrap();
    });

    let mut gps_odometry_opt = Option::<gps_odometry::Odometry>::None;

    loop {
        if let Some(gnss_msg) = &*gnss_msg_global_handler.lock().unwrap() {
            if let Some(mag_msg) = &*mag_msg_global_handler.lock().unwrap() {
                // TODO get yaw heading from `gps_odometry::get_heading`
                gps_odometry_opt = Option::<gps_odometry::Odometry>::None;
                // TODO replace this       ^^^^ with the init method
                break;
            }
        }
        println!("did not get gps or magnetometer lock, waiting 500 ms");
        thread::sleep(std::time::Duration::from_millis(500));
    }

    while context.ok() {
        // locks the thread till a new gps ping is received
        gps_msg_rx.recv().unwrap();

        // blocks the thread until a message is received by gps callback
        if let Some(odom_msg) = &mut *odom_msg_global_handler.lock().unwrap() {
            if let Some(gnss_msg) = &*gnss_msg_global_handler.lock().unwrap() {
                odom_msg.header.stamp.sec = gnss_msg.header.stamp.sec;
                odom_msg.header.stamp.nanosec = gnss_msg.header.stamp.nanosec;
                // TODO call function that modifies odom based on the gnss msg
                // TODO update odom_msg based on what is given out by gps_odometry_opt
            }
            odom_publisher.publish(&*odom_msg).unwrap();
            println!("{:?}", &odom_msg.header.stamp.sec);
        }
    }
    Ok(())
}
