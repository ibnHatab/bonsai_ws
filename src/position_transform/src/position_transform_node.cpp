#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <nav_msgs/Odometry.h>


using namespace geometry_msgs;
using namespace message_filters;
using namespace nav_msgs;

//! @brief Publisher of Nav relative to odom (datum) frame
ros::Publisher nav_odom_pub_;

void callback(const Vector3StampedConstPtr& lla, const QuaternionStampedConstPtr& orientation)
{
	// Solve all of perception here...
	Odometry nav_in_odom {};

	// Publish Nav odometry in odom frame - note frames are set in ::run()
	nav_in_odom.header.stamp = lla->header.stamp;
	nav_in_odom.header.seq = lla->header.seq;
	nav_in_odom.header.frame_id = lla->header.frame_id;

	nav_in_odom.pose.pose.position.x = lla->vector.y;
	nav_in_odom.pose.pose.position.y = lla->vector.x;
	nav_in_odom.pose.pose.position.z = lla->vector.z;

	nav_in_odom.pose.pose.orientation = orientation->quaternion;

	// Ignore twist
	nav_odom_pub_.publish(nav_in_odom);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "position_transform_node");

  ros::NodeHandle nh;
  message_filters::Subscriber<Vector3Stamped> pose_sub(nh, "/filter/positionlla", 1);
  message_filters::Subscriber<QuaternionStamped> twist_sub(nh, "/filter/quaternion", 1);
  typedef sync_policies::ApproximateTime<Vector3Stamped, QuaternionStamped> SyncPolicy;

  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<SyncPolicy> sync(SyncPolicy(10), pose_sub, twist_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  nav_odom_pub_ = nh.advertise<Odometry>("nav_odom", 10);

  ros::spin();
  return 0;
}
