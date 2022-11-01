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
    let mag_msg_global_handler = Arc::new(Mutex::new(Option::<MagneticFieldMsg>::None));
    let odom_msg_global_handler = Arc::new(Mutex::new(Some(OdometryMsg::default())));


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
            println!("got datum lat long");
            if let Some(mag_msg) = &*mag_msg_global_handler.lock().unwrap() {
                println!("got datum yaw");
                // done TODO get yaw heading from `gps_odometry::get_heading`
                // done
                let datum_heading = gps_odometry::get_heading(mag_msg.magnetic_field.x,
                                                              mag_msg.magnetic_field.y);
                let (lat, long) = (gnss_msg.latitude, gnss_msg.longitude);
                // gps_odometry_opt = Option::<gps_odometry::Odometry>::None;
                // done TODO replace this       ^^^^ with the init method
                gps_odometry_opt = Some(gps_odometry::Odometry::new(datum_heading, lat, long));
                break;
            }
        }
        println!("did not get gps or magnetometer lock, waiting 500 ms");
        thread::sleep(std::time::Duration::from_millis(500));
    }

    println!("ready to start publishing odom");
    while context.ok() {
        // locks the thread till a new gps ping is received
        println!("waiting for gps ping");
        gps_msg_rx.recv().unwrap();
        println!("got gps ping, calculating new odom");
        // blocks the thread until a message is received by gps callback
        if let Some(odom_msg) = &mut *odom_msg_global_handler.lock().unwrap() {
            println!("got odom msg, now waiting for gnss msg");
            if let Some(gnss_msg) = &*gnss_msg_global_handler.lock().unwrap() {
                println!("got gnss msg, now waiting for odom instance");
                odom_msg.header.stamp.sec = gnss_msg.header.stamp.sec;
                odom_msg.header.stamp.nanosec = gnss_msg.header.stamp.nanosec;
               if let Some(gps_odometry_instance) = &gps_odometry_opt {
                   // TODO call function that modifies odom based on the gnss msg
                   // done
                   gps_odometry_instance.update_odom(gnss_msg.latitude, gnss_msg.longitude);
                   // TODO update odom_msg based on what is given out by gps_odometry_opt
                   // done
                   let position = *gps_odometry_instance.position.lock().unwrap();
                   odom_msg.pose.pose.position.x = position.x;
                   odom_msg.pose.pose.position.y = position.y;
                   println!("sending odom");
                   println!("Current position: {}, {}", position.x, position.y);
                   odom_publisher.publish(&*odom_msg).unwrap();
               }
            }
        }
    }
    Ok(())
}
