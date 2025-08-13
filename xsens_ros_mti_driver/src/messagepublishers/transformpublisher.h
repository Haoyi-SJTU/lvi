#ifndef TRANSFORMPUBLISHER_H
#define TRANSFORMPUBLISHER_H

#include "packetcallback.h"
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

struct TransformPublisher : public PacketCallback
{
    tf2_ros::TransformBroadcaster tf_broadcaster;
    std::string frame_id = DEFAULT_FRAME_ID;


    TransformPublisher(ros::NodeHandle &node) : tf_broadcaster()
    {
        ros::param::get("~frame_id", frame_id);
    }

    void operator()(const XsDataPacket &packet, ros::Time timestamp)
    {
        // if (packet.containsOrientation())
        // {
        //     geometry_msgs::TransformStamped tf;

        //     XsQuaternion q = packet.orientationQuaternion();
        //     // 在这里发布TF变换

        //     tf.header.stamp = timestamp;
        //     tf.header.frame_id = "world";
        //     tf.child_frame_id = "sensor_link";
        //     tf.transform.translation.x = 0.0;
        //     tf.transform.translation.y = 0.0;
        //     tf.transform.translation.z = 0.0;
        //     tf.transform.rotation.x = q.x();
        //     tf.transform.rotation.y = q.y();
        //     tf.transform.rotation.z = q.z();
        //     tf.transform.rotation.w = q.w();

        //     tf_broadcaster.sendTransform(tf);
        // }
    }
};

#endif
