use anyhow::{Error, Result};
use std::env;
use gps_odometry;
use rclrs;
use nav_msgs::msg::Odometry as OdometryMsg;
use sensor_msgs::msg::NavSatFix as NavSatFixMsg;
use sensor_msgs::msg::Imu as ImuMsg;
use std::sync::{Arc, Mutex};
use std::thread;


fn main() -> Result<(), Error> {
    let context = rclrs::Context::new(env::args())?;


    let gnss_msg_global_handler = Arc::new(Mutex::new(NavSatFixMsg::default()));
    let imu_msg_global_handler = Arc::new(Mutex::new(ImuMsg::default()));

    let mut node = rclrs::create_node(&context, "gps_odometry_node").unwrap();

    let publisher = node.create_publisher::<std_msgs::msg::String>("topic", rclrs::QOS_PROFILE_DEFAULT).unwrap();

    let odom_publisher = node.create_publisher::<OdometryMsg>("/odometry/gps", rclrs::QOS_PROFILE_DEFAULT).unwrap();

    let gnss_msg_subscriber_handler = Arc::clone(&gnss_msg_global_handler);
    let _gnss_subscriber = node.create_subscription::<NavSatFixMsg,  _>("/gnss", rclrs::QOS_PROFILE_DEFAULT, move |msg: NavSatFixMsg| {
        *gnss_msg_subscriber_handler.lock().unwrap() = msg;
    });

    let imu_msg_subscriber_handler = Arc::clone(&imu_msg_global_handler);
    let _imu_subscriber = node.create_subscription::<ImuMsg, _>("/imu/filtered", rclrs::QOS_PROFILE_DEFAULT, move |msg: ImuMsg| {
        *imu_msg_subscriber_handler.lock().unwrap() = msg;
    });

    let mut message = std_msgs::msg::String::default();

    let mut publish_count: u32 = 1;

    thread::spawn(move || {
        rclrs::spin(&node).unwrap();
    });

    while context.ok() {
        message.data = format!("Hello, world! {}", publish_count);
        println!("Publishing: [{}]", message.data);
        publisher.publish(&message);
        publish_count += 1;
        std::thread::sleep(std::time::Duration::from_millis(500));
    }
    Ok(())
}
