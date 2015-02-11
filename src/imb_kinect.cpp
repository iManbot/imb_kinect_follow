/*------------------------------------------------------------------------------
 *  Title:        cse_kinect.cpp
 *  Description:  ROS node for subscribing to Kinect skeleton transforms and
 *                finding gestures.
 *----------------------------------------------------------------------------*/

/*
 *
 *    Copyright (c) 2010 <iBotics -- www.sdibotics.org>
 *    All rights reserved.
 *
 *    Redistribution and use in source and binary forms, with or without
 *    modification, are permitted provided that the following conditions are
 *    met:
 *
 *    * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer
 *    in the documentation and/or other materials provided with the
 *    distribution.
 *    * Neither the name of the Stingray, iBotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *    A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *    OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "imb_kinect.h"


/*------------------------------------------------------------------------------
 * CseKinect()
 * Constructor.
 *----------------------------------------------------------------------------*/

ImbKinect::ImbKinect()
{
} // end CseKinect()


/*------------------------------------------------------------------------------
 * ~CseKinect()
 * Destructor.
 *----------------------------------------------------------------------------*/

ImbKinect::~ImbKinect()
{
    header.stamp.sec=-1;
} // end ~CseKinect()


/*---------------------------------------------------------------------
* publishPoseData()
* Publishes poses that have been found.
* -------------------------------------------------------------------*/

void ImbKinect::publishPoseData(ros::Publisher *pub_pose_data)
{
    imb_kinect::PoseData msg;
  
    // Send out the pose data.
    msg.pose1  = pose1;
    msg.pose2  = pose2;
    msg.lVoila = lVoila;
    msg.rVoila = rVoila;
    msg.flat   = flat;
    msg.leftArm= leftArm;
    msg.rightArm=rightArm;

    msg.header = header;
    msg.pos = pos;
    msg.vel = vel;


    pub_pose_data->publish(msg);

    // Reset whether we have found the poses.
    pose1  = false;
    pose2  = false;
    lVoila = false;
    rVoila = false;
    flat   = false;
    leftArm=false;
    rightArm=false;
} // end publishPoseData()


/*---------------------------------------------------------------------
* lookForPoses()
* Looks for poses in Kinect data.
* -------------------------------------------------------------------*/

void ImbKinect::lookForPoses()
{
    try
    {
        // Right shoulder.
        listener.lookupTransform("/openni_depth_frame", "/right_shoulder_1", ros::Time(0), tf_right_shoulder);
        x_right_shoulder = tf_right_shoulder.getOrigin().x();
        y_right_shoulder = tf_right_shoulder.getOrigin().y();
  
        // Right elbow.
        listener.lookupTransform("/openni_depth_frame", "/right_elbow_1", ros::Time(0), tf_right_elbow);
        x_right_elbow = tf_right_elbow.getOrigin().x();
        y_right_elbow = tf_right_elbow.getOrigin().y();
  
        // Right hand.
        listener.lookupTransform("/openni_depth_frame", "/right_hand_1", ros::Time(0), tf_right_hand);
        x_right_hand = tf_right_hand.getOrigin().x();
        y_right_hand = tf_right_hand.getOrigin().y();
  
        // Left shoulder.
        listener.lookupTransform("/openni_depth_frame", "/left_shoulder_1", ros::Time(0), tf_left_shoulder);
        x_left_shoulder = tf_left_shoulder.getOrigin().x();
        y_left_shoulder = tf_left_shoulder.getOrigin().y();
  
        // Left elbow.
        listener.lookupTransform("/openni_depth_frame", "/left_elbow_1", ros::Time(0), tf_left_elbow);
        x_left_elbow = tf_left_elbow.getOrigin().x();
        y_left_elbow = tf_left_elbow.getOrigin().y();
        
        // Left hand.
        listener.lookupTransform("/openni_depth_frame", "/left_hand_1", ros::Time(0), tf_left_hand);
        x_left_hand = tf_left_hand.getOrigin().x();
        y_left_hand = tf_left_hand.getOrigin().y();
  
        // Neck.
        listener.lookupTransform("/openni_depth_frame", "/neck_1", ros::Time(0), tf_neck);
        x_neck = tf_neck.getOrigin().x();
        y_neck = tf_neck.getOrigin().y();
  
        // Right hip.
        listener.lookupTransform("/openni_depth_frame", "/right_hip_1", ros::Time(0), tf_right_hip);
        x_right_hip = tf_right_hip.getOrigin().x();
        y_right_hip = tf_right_hip.getOrigin().y();
  
        // Left hip.
        listener.lookupTransform("/openni_depth_frame", "/left_hip_1", ros::Time(0), tf_left_hip);
        x_left_hip = tf_left_hip.getOrigin().x();
        y_left_hip = tf_left_hip.getOrigin().y();
  
        // Right foot.
        listener.lookupTransform("/openni_depth_frame", "/right_foot_1", ros::Time(0), tf_right_foot);
        x_right_foot = tf_right_foot.getOrigin().x();
        y_right_foot = tf_right_foot.getOrigin().y();
  
        // Left foot.
        listener.lookupTransform("/openni_depth_frame", "/left_foot_1", ros::Time(0), tf_left_foot);
        x_left_foot = tf_left_foot.getOrigin().x();
        y_left_foot = tf_left_foot.getOrigin().y();
  
        // Right knee.
        listener.lookupTransform("/openni_depth_frame", "/right_knee_1", ros::Time(0), tf_right_knee);
        x_right_knee = tf_right_knee.getOrigin().x();
        y_right_knee = tf_right_knee.getOrigin().y();
  
        // Left knee.
        listener.lookupTransform("/openni_depth_frame", "/left_knee_1", ros::Time(0), tf_left_knee);
        x_left_knee = tf_left_knee.getOrigin().x();
        y_left_knee = tf_left_knee.getOrigin().y();
  
        // Head.
        listener.lookupTransform("/openni_depth_frame", "/head_1", ros::Time(0), tf_head);
        x_head = tf_head.getOrigin().x();
        y_head = tf_head.getOrigin().y();
  
        // Start looking for poses now that we have positions of body parts.
        float a = 0.0, b = 0.0, c = 0.0, d = 0.0, z = 0.0;
        float diff = 0.0, diff2 = 0.0, diff4 = 0.0, diff5 = 0.0, diffHRH = 0.0;
        float machoAngle = 0.0, diagonal = 0.0;
        a = x_right_elbow - x_right_shoulder;
        b = y_right_elbow - y_right_shoulder;
        c = x_right_hand - x_right_elbow;
        d = y_right_hand - y_right_elbow;
        z = ((a*c + b*d) / (sqrt(a*a + b*b) * sqrt(c*c + d*d)));
        machoAngle = acos(z) * 180.0 / M_PI;
        
        diff = (x_left_elbow) - (x_left_hand);
        diff2 =   x_left_shoulder - x_left_elbow;
        diff4 =   x_right_elbow - x_right_hand;
        diff5 =   x_right_shoulder - x_right_elbow;
        diffHRH = y_head - y_right_hand;
        
        // If angle is between this and .
        if (machoAngle > 75.0 && machoAngle < 100.0 && diff < 0.1 && diff2 < 0.1 && diffHRH < 0.2)
        {
            pose1 = true;
            //ROS_WARN("Macho Pose Detected!");
        }
  
        // Right upper arm.
        float y, libertyAngle = 0;
        diagonal = sqrt(((x_right_hand-x_right_shoulder) * (x_right_hand-x_right_shoulder)) + (y_right_hand*y_right_hand));
        y = (x_right_hand-x_right_shoulder) / diagonal;
        libertyAngle = acos(y) * 180.0 / M_PI;
  
        // Left lower arm.
        float x, leftAngle, leftDiagonal = 0;
        leftDiagonal = sqrt(((x_left_shoulder-x_left_hand) * (x_left_shoulder-x_left_hand)) + (y_left_hand*y_left_hand));
        x = (x_left_hand-x_left_shoulder) / leftDiagonal;
        leftAngle = acos(x) * 180.0 / M_PI;
        leftAngle = 180.0 - leftAngle;
  
        // Voila pose, right arm up.
        float voilaAngle;
        voilaAngle = libertyAngle - leftAngle;
        
        if (voilaAngle < 10.0 && y_right_hand > y_head && y_left_hand < y_head)
        {
            //ROS_WARN("Right Voila Pose Detected!");
            rVoila = true;
        }
  
        if (voilaAngle < 20.0 && y_left_hand > y_head && y_right_hand < y_head)
        {
            //ROS_WARN("Left Voila Pose Detected!");
            lVoila = true;
        }
  
        // Left hip.
        if(voilaAngle < 10.0 && y_right_hand > 0.0 && y_left_hand > 0.0)
        {
          //ROS_WARN("High V Pose Detected!");
        }
  
        if(voilaAngle < 10.0 && y_right_hand < y_right_hip && y_left_hand < y_left_hip)
        {
          //ROS_WARN("Low V Pose Detected!");
        }
  
        float diff6 = 0.0, diff7 = 0.0, diff13 = 0.0, diff14 = 0.0;
        float flatAngle1 = 0.0, flatAngle2 = 0.0, flatAngle3 = 0.0;
        diff6  = y_right_hand - y_right_elbow;
        diff13 = y_right_elbow - y_right_shoulder;
        diff7  = y_left_shoulder - y_left_elbow;
        diff14 = y_left_elbow - y_left_hand;
  
        flatAngle1 = acos(y_head/x_right_hand) * 180.0 / M_PI;
        flatAngle2 = acos(y_head/x_left_hand) * 180.0 / M_PI;
        flatAngle3 = flatAngle1 - flatAngle2;
        if(diff6 < 0.1 && diff7 < 0.1 && diff13 < 0.1 && diff14 < 0.1 && y_right_hand < y_head  && y_left_hand < y_head)
        {
          //ROS_WARN("Flat Pose Detected!");
          flat = true;
        }
    
        if (((x_right_shoulder - x_right_elbow) < .1) &&
            ((x_right_elbow - x_left_hand) < .2) &&
            ((x_left_shoulder - x_left_elbow) < .1) &&
             (x_left_elbow > x_head) && 
             (x_right_elbow > x_head) && 
             (x_right_hand > x_head) && 
             (x_left_hand > x_head))
        {
            //ROS_WARN("Stop Pose Detected!");
        }

        // Right arm.
        tf::Vector3 pos_right_arm = tf_right_hand.getOrigin();
        tf::Vector3 pos_right_shoulder = tf_right_shoulder.getOrigin();
        tf::Vector3 rel_pos_right_arm=pos_right_arm-pos_right_shoulder;

        // Left arm
        tf::Vector3 pos_left_arm = tf_left_hand.getOrigin();
        tf::Vector3 pos_left_shoulder = tf_left_shoulder.getOrigin();
        tf::Vector3 rel_pos_left_arm=pos_left_arm-pos_left_shoulder;

        float left_arm_angle=asin(rel_pos_left_arm.z()/rel_pos_left_arm.length()) * 180.0 / M_PI;
        float right_arm_angle=asin(rel_pos_right_arm.z()/rel_pos_right_arm.length()) * 180.0 / M_PI;

        if (left_arm_angle>20)
        {
            leftArm=true;
            ROS_WARN("Left arm up!");
        }else
        {
            leftArm=false;
            ROS_WARN("Left arm down!");
        }

        if (right_arm_angle>20)
        {
            rightArm=true;
            ROS_WARN("Right arm up!");
        }else
        {
            rightArm=false;
            ROS_WARN("Right arm down!");
        }

        listener.lookupTransform("/openni_depth_frame", "/torso_1", ros::Time(0), tf_waist);
        x_waist = tf_waist.getOrigin().x();
        y_waist = tf_waist.getOrigin().y();
        tf::Vector3 tmp=tf_waist.getOrigin();

        if (header.stamp.sec<0)
        {
            vel.x=0;
            vel.y=0;
            vel.z=0;
            header.stamp=tf_waist.stamp_;
            pre_pos.x=tmp.x();
            pre_pos.y=tmp.y();
            pre_pos.z=tmp.z();
            pos=pre_pos;
        }
        else
        {
            float dt=(header.stamp.toSec()-tf_right_shoulder.stamp_.toSec());

            vel.x=vel.x*0.9+0.1*(tmp.x()-pre_pos.x)/dt;
            vel.y=vel.x*0.9+0.1*(tmp.y()-pre_pos.y)/dt;
            vel.z=vel.x*0.9+0.1*(tmp.z()-pre_pos.z)/dt;
            header.stamp=tf_waist.stamp_;
            pre_pos.x=tmp.x();
            pre_pos.y=tmp.y();
            pre_pos.z=tmp.z();
            pos=pre_pos;
        }



    }
  
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }

} // end lookForPoses()


/*---------------------------------------------------------------------
* main()
* Main function for ROS node.
* -------------------------------------------------------------------*/

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "imb_kinect");
    ros::NodeHandle n;
  
    // Set up a CseKinect object.
    ImbKinect *imb_kinect = new ImbKinect();
  
    // Set up parameter server variables.
    ros::NodeHandle pnh("~");
    int rate = 20;
    std::string pub_name_poses;
    pnh.param("pub_name_poses", pub_name_poses, std::string("imb_pose_data"));
    pnh.param("rate",           rate,           int(20));
    
    // Set up a publisher.
    ros::Publisher pub_pose_data = n.advertise<imb_kinect::PoseData>(pub_name_poses.c_str(), 1000);
  
    // Tell ROS to run this node at the desired rate.
    ros::Rate r(rate);
  
    // Main loop.
    while (n.ok())
    {
        imb_kinect->lookForPoses();
        imb_kinect->publishPoseData(&pub_pose_data);
        ros::spinOnce();
        r.sleep();
    }
  
    return 0;
} // end main()
