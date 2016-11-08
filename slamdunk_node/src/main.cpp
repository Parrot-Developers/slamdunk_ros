/*
 * Copyright (c) 2016 Parrot S.A.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Parrot Company nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE PARROT COMPANY BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <endian.h>
#include <time.h>

#include <chrono>
#include <cinttypes>
#include <memory>

#include <opencv2/core/core.hpp>

#include "cvutils.hpp"

#include "kalamos_context.hpp"

// ROS
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <slamdunk_msgs/BoolStamped.h>
#include <slamdunk_msgs/QualityStamped.h>
#include <slamdunk_node/SLAMDunkNodeConfig.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <nodelet/loader.h>
#include <octomap_msgs/Octomap.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/FluidPressure.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Range.h>
#include <stereo_msgs/DisparityImage.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

namespace
{
    class Context
    {
    public:
        Context(ros::NodeHandle& n)
            : m_rosNode(n)
            , m_privateNode("~")
            , m_depthmapPublisher(image_transport::ImageTransport(n).advertiseCamera("depth_map/image", 1))
            , m_dispmapPublisher(image_transport::ImageTransport(n).advertiseCamera("disp_map/image", 1))
            , m_stereoPublisher(n.advertise<stereo_msgs::DisparityImage>("stereo/disparity", 1))
            , m_leftRgbPublisher(image_transport::ImageTransport(n).advertiseCamera("left_rgb/image", 1))
            , m_leftGrayscalePublisher(image_transport::ImageTransport(n).advertiseCamera("left_grayscale/image", 1))
            , m_leftRgbRectPublisher(image_transport::ImageTransport(n).advertiseCamera("left_rgb_rect/image_rect_color", 1))
            , m_rightRgbPublisher(image_transport::ImageTransport(n).advertiseCamera("right_rgb/image", 1))
            , m_rightGrayscalePublisher(image_transport::ImageTransport(n).advertiseCamera("right_grayscale/image", 1))
            , m_pointCloud2Publisher(n.advertise<sensor_msgs::PointCloud2>("pcl_xyz", 1))
            , m_imuPublisher(n.advertise<sensor_msgs::Imu>("imu", 1))
            , m_posePublisher(n.advertise<geometry_msgs::PoseStamped>("pose", 1))
            , m_occupancyPublisher(n.advertise<octomap_msgs::Octomap>("octomap", 1))
            , m_ultrasoundPublisher(n.advertise<sensor_msgs::Range>("ultrasound", 1))
            , m_barometerPublisher(n.advertise<sensor_msgs::FluidPressure>("barometer", 1))
            , m_magnetometerPublisher(n.advertise<sensor_msgs::MagneticField>("magnetometer", 1))
            , m_keyframePublisher(n.advertise<slamdunk_msgs::BoolStamped>("keyframe", 1))
            , m_qualityPublisher(n.advertise<slamdunk_msgs::QualityStamped>("quality", 1))

        {
            // get the clock_sync_period parameter, defaulting to a period of 10 seconds
            double clockSyncPeriod = 0.0;
            m_privateNode.param("clock_sync_period", clockSyncPeriod, 10.0);

            if (clockSyncPeriod < 0)
            {
                ROS_ERROR("invalid clock_sync_period: %f", clockSyncPeriod);
            }

            ROS_INFO("clock_sync_period: %f", clockSyncPeriod);
            m_clockOffsetTimer = n.createTimer(ros::Duration(clockSyncPeriod),
                                               boost::bind(&Context::onClockOffsetTimerCallback, this, _1));
            // initialize the clock offset
            onClockOffsetTimerCallback(ros::TimerEvent{});

            m_restartCaptureService = n.advertiseService("restart_capture", &Context::restartCaptureServiceCb, this);
            m_restartSlamService = n.advertiseService("restart_slam", &Context::restartSlamServiceCb, this);
            m_restartOccupancyService = n.advertiseService("restart_occupancy", &Context::restartOccupancyServiceCb, this);

            m_dynamicReconfigureServer.setCallback(boost::bind(&Context::dynamicReconfigureCb, this, _1, _2));

            menuInit();
        }

        void setKalamosContext(kalamos::Context* c);
        void tick();

        void depthmapPublish(kalamos::DepthmapData const& dm);
        void dispmapPublish(kalamos::DispmapData const& dm);
        void imuPublish(kalamos::ImuData const& imuData);
        void posePublish(kalamos::PoseData const& poseData);
        void occupancyPublish(kalamos::OccupancyData const& occupancyData);
        void ultrasoundPublish(kalamos::UltrasoundData const& ultrasoundData);
        void barometerPublish(kalamos::BarometerData const& barometerData);
        void magnetometerPublish(kalamos::MagnetometerData const& magnetometerData);

        void onStereoYuvData(kalamos::StereoYuvData const& stereoYuvData);

    private:
        void leftRgbPublish(cv::Mat const& rgbData, std::uint64_t ts);
        void leftGrayscalePublish(cv::Mat const& grayscaleData, std::uint64_t ts);
        void leftRgbRectPublish(cv::Mat const& rgbData, std::uint64_t ts);
        void rightRgbPublish(cv::Mat const& rgbData, std::uint64_t ts);
        void rightGrayscalePublish(cv::Mat const& grayscaleData, std::uint64_t ts);

    private:
        void menuInit();

        void onNewDepthFrame(const sensor_msgs::ImageConstPtr& depthMap, const sensor_msgs::CameraInfoConstPtr& camInfo);

        void onClockOffsetTimerCallback(const ros::TimerEvent& e);

        /// Convert kalamos timestamp to ROS time
        ros::Time convertStamp(std::uint64_t kstamp) const;

    private:
        bool restartCapture();
        bool restartSlam();
        bool restartOccupancy();
        // Services
        typedef std_srvs::Empty RestartCaptureSrv;
        typedef std_srvs::Empty RestartSlamSrv;
        typedef std_srvs::Empty RestartOccupancySrv;
        bool restartCaptureServiceCb(RestartCaptureSrv::Request& req, RestartCaptureSrv::Response& resp);
        bool restartSlamServiceCb(RestartSlamSrv::Request& req, RestartSlamSrv::Response& resp);
        bool restartOccupancyServiceCb(RestartOccupancySrv::Request& req, RestartOccupancySrv::Response& resp);
        ros::ServiceServer m_restartCaptureService;
        ros::ServiceServer m_restartSlamService;
        ros::ServiceServer m_restartOccupancyService;

    private:
        ros::NodeHandle& m_rosNode;
        ros::NodeHandle m_privateNode;

        // Offset to get ROS time from kalamos time
        std::chrono::nanoseconds m_kalamosToROSOffset;
        ros::Timer m_clockOffsetTimer;

        kalamos::Context* m_kalamosContext = nullptr;

        std::unique_ptr<kalamos::ServiceHandle> m_captureHandle;
        std::unique_ptr<kalamos::ServiceHandle> m_slamHandle;
        std::unique_ptr<kalamos::ServiceHandle> m_occupancyHandle;

        image_transport::CameraPublisher m_depthmapPublisher;
        image_transport::CameraPublisher m_dispmapPublisher;
        ros::Publisher m_stereoPublisher;

        image_transport::CameraPublisher m_leftRgbPublisher;
        image_transport::CameraPublisher m_leftGrayscalePublisher;

        image_transport::CameraPublisher m_leftRgbRectPublisher;

        image_transport::CameraPublisher m_rightRgbPublisher;
        image_transport::CameraPublisher m_rightGrayscalePublisher;

        ros::Publisher m_pointCloud2Publisher;
        image_transport::CameraSubscriber m_pointCloud2DepthSubscriber;

        ros::Publisher m_imuPublisher;

        ros::Publisher m_posePublisher;
        tf2_ros::TransformBroadcaster m_transformBroadcaster;

        ros::Publisher m_occupancyPublisher;
        octomap::OcTree m_octomapOctree = {kalamos::OccupancyData::RESOLUTION};

        ros::Publisher m_ultrasoundPublisher;
        ros::Publisher m_barometerPublisher;
        ros::Publisher m_magnetometerPublisher;

        ros::Publisher m_keyframePublisher;
        ros::Publisher m_qualityPublisher;

        // Dynamic reconfigure
        dynamic_reconfigure::Server<slamdunk_node::SLAMDunkNodeConfig> m_dynamicReconfigureServer;
        void dynamicReconfigureCb(slamdunk_node::SLAMDunkNodeConfig& config, uint32_t level);
        slamdunk_node::SLAMDunkNodeConfig m_config;

        // menu
        std::shared_ptr<interactive_markers::InteractiveMarkerServer> m_menuServer;
        interactive_markers::MenuHandler m_menuHandle;
        interactive_markers::MenuHandler::EntryHandle m_menuRestartCaptureHandle;
        interactive_markers::MenuHandler::EntryHandle m_menuRestartSlamHandle;
        interactive_markers::MenuHandler::EntryHandle m_menuRestartOccupancyHandle;
        void menuRestartCaptureCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
        void menuRestartSlamCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
        void menuRestartOccupancyCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    };

    void Context::setKalamosContext(kalamos::Context* c)
    {
        m_kalamosContext = c;
        if (m_kalamosContext)
        {
            std::pair<const char*, const char*> propertiesDefaultValuePairs[] = {
                {"ro.parrot.build.uid", ""},
                {"ro.parrot.build.version", "0.0.0"},
                {"ro.factory.serial", ""},
                {"ro.revision", ""},
            };
            // export some properties using the following mapping:
            //     ro.revision -> properties/ro_revision
            for (const auto& pair : propertiesDefaultValuePairs)
            {
                std::string topicName = pair.first;
                // in place replacement of '.' by '_'
                std::replace(topicName.begin(), topicName.end(), '.', '_');

                m_rosNode.setParam("properties/" + topicName, m_kalamosContext->getProperty(pair.first, pair.second));
            }
        }
    }

    void Context::depthmapPublish(kalamos::DepthmapData const& dm)
    {
        cv_bridge::CvImage img_bridge;
        img_bridge.image = dm.depth;
        img_bridge.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        img_bridge.header.frame_id = "cam0_rect_optical";
        img_bridge.header.stamp = convertStamp(dm.ts);

        auto msg = img_bridge.toImageMsg();

        // make sure depth is in a ROS friendly format
        // REP 117 requires unknown depth to be NaN, kalamos uses 0.0f
        auto data = reinterpret_cast<float*>(msg->data.data());
        const int step = msg->step / sizeof(*data);
        for (int r = 0; r < msg->height; ++r)
        {
            for (int c = 0; c < msg->width; ++c)
            {
                float& depth = data[r * step + c];

                // replace kalamos unknown value (0.0f) with NaN
                if (depth == 0.0f)
                {
                    depth = std::numeric_limits<float>::quiet_NaN();
                }
            }
        }

        sensor_msgs::CameraInfoPtr camInfo(new sensor_msgs::CameraInfo());
        camInfo->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
        camInfo->D.resize(5);

        kalamos::RectifiedFrameStaticProperties staticProps = m_kalamosContext->getDepthmapStaticProperties();

        float fx = staticProps.fx;
        float fy = staticProps.fy;
        float cx = dm.depth.cols / 2.0;
        float cy = dm.depth.rows / 2.0;

        camInfo->P.fill(0);
        camInfo->P[0] = fx;
        camInfo->P[2] = cx;
        camInfo->P[5] = fy;
        camInfo->P[6] = cy;
        camInfo->P[10] = 1.0;

        camInfo->width = dm.depth.cols;
        camInfo->height = dm.depth.rows;
        camInfo->header = img_bridge.header;

        m_depthmapPublisher.publish(msg, camInfo);
    }

    void Context::dispmapPublish(kalamos::DispmapData const& dm)
    {
        // disparity sensors_msgs/image
        cv_bridge::CvImage img_bridge;
        img_bridge.image = dm.disp;
        img_bridge.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        img_bridge.header.frame_id = "cam0_rect_optical";
        img_bridge.header.stamp = convertStamp(dm.ts);

        auto dispImg = img_bridge.toImageMsg();

        sensor_msgs::CameraInfoPtr camInfo(new sensor_msgs::CameraInfo());
        camInfo->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
        camInfo->D.resize(5);

        kalamos::RectifiedFrameStaticProperties staticProps = m_kalamosContext->getDispmapStaticProperties();

        float fx = staticProps.fx;
        float fy = staticProps.fy;
        float cx = dm.disp.cols / 2.0;
        float cy = dm.disp.rows / 2.0;

        camInfo->P.fill(0);
        camInfo->P[0] = fx;
        camInfo->P[2] = cx;
        camInfo->P[5] = fy;
        camInfo->P[6] = cy;
        camInfo->P[10] = 1.0;

        camInfo->width = dm.disp.cols;
        camInfo->height = dm.disp.rows;
        camInfo->header = img_bridge.header;

        m_dispmapPublisher.publish(dispImg, camInfo);

        // now, stereo_msg/Disparity
        stereo_msgs::DisparityImagePtr stereo(new stereo_msgs::DisparityImage());

        stereo->header = dispImg->header;
        stereo->image = *dispImg;
        // ROI: already describe in the header.frame_id => full frame
        stereo->valid_window.x_offset = 0;
        stereo->valid_window.y_offset = 0;
        stereo->valid_window.width = dispImg->width;
        stereo->valid_window.height = dispImg->height;
        stereo->valid_window.do_rectify = 0;
        // disp->depth params
        stereo->f = dm.f;
        stereo->T = dm.T;
        // range & precision
        stereo->min_disparity = dm.min_disparity;
        stereo->max_disparity = dm.max_disparity;
        stereo->delta_d = 0.25;

        m_stereoPublisher.publish(stereo);
    }

    void Context::leftRgbPublish(cv::Mat const& rgbData, std::uint64_t ts)
    {
        cv_bridge::CvImage img_bridge;
        img_bridge.image = rgbData;
        img_bridge.encoding = sensor_msgs::image_encodings::RGB8;
        // optical suffix, REP 103, suffix frames
        img_bridge.header.frame_id = "cam0_optical";
        img_bridge.header.stamp = convertStamp(ts);

        // REP 104 - Calibration Parameters
        // > Empty D and distortion_model indicate that the CameraInfo cannot be
        // > used to rectify points or images, either because the camera is not
        // > calibrated or because the rectified image was produced using an
        // > unsupported distortion model
        // -- http://www.ros.org/reps/rep-0104.html#calibration-parameters
        //
        // Let's not specify anything for now as we prefer to favor kalamos context
        // way of rectifying the image, the internal distortion model is better.
        sensor_msgs::CameraInfoPtr camInfo(new sensor_msgs::CameraInfo());
        camInfo->width = rgbData.cols;
        camInfo->height = rgbData.rows;
        camInfo->header = img_bridge.header;

        m_leftRgbPublisher.publish(img_bridge.toImageMsg(), camInfo);
    }

    void Context::leftGrayscalePublish(cv::Mat const& grayscaleData, std::uint64_t ts)
    {
        cv_bridge::CvImage img_bridge;
        img_bridge.image = grayscaleData;
        img_bridge.encoding = sensor_msgs::image_encodings::MONO8;
        img_bridge.header.frame_id = "cam0_optical";
        img_bridge.header.stamp = convertStamp(ts);

        sensor_msgs::CameraInfoPtr camInfo(new sensor_msgs::CameraInfo());
        camInfo->width = grayscaleData.cols;
        camInfo->height = grayscaleData.rows;
        camInfo->header = img_bridge.header;

        m_leftGrayscalePublisher.publish(img_bridge.toImageMsg(), camInfo);
    }

    void Context::leftRgbRectPublish(cv::Mat const& rgbData, std::uint64_t ts)
    {
        cv_bridge::CvImage img_bridge;
        img_bridge.image = rgbData;
        img_bridge.encoding = sensor_msgs::image_encodings::RGB8;
        // optical suffix, REP 103, suffix frames
        img_bridge.header.frame_id = "cam0_rect_optical";
        img_bridge.header.stamp = convertStamp(ts);

        sensor_msgs::CameraInfoPtr camInfo(new sensor_msgs::CameraInfo());
        camInfo->header.frame_id = "cam0_rect_optical";
        camInfo->header.stamp = img_bridge.header.stamp;
        camInfo->width = rgbData.cols;
        camInfo->height = rgbData.rows;

        camInfo->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
        camInfo->D.resize(5);

        kalamos::RectifiedFrameStaticProperties staticProps = m_kalamosContext->getLeftRectStaticProperties();

        float fx = staticProps.fx;
        float fy = staticProps.fy;
        float cx = rgbData.cols / 2.0;
        float cy = rgbData.rows / 2.0;

        camInfo->P.fill(0);
        camInfo->P[0] = fx;
        camInfo->P[2] = cx;
        camInfo->P[5] = fy;
        camInfo->P[6] = cy;
        camInfo->P[10] = 1.0;

        m_leftRgbRectPublisher.publish(img_bridge.toImageMsg(), camInfo);
    }

    void Context::rightRgbPublish(cv::Mat const& rgbData, std::uint64_t ts)
    {
        cv_bridge::CvImage img_bridge;
        img_bridge.image = rgbData;
        img_bridge.encoding = sensor_msgs::image_encodings::RGB8;
        img_bridge.header.frame_id = "cam1_optical";
        img_bridge.header.stamp = convertStamp(ts);

        sensor_msgs::CameraInfoPtr camInfo(new sensor_msgs::CameraInfo());
        camInfo->width = rgbData.cols;
        camInfo->height = rgbData.rows;
        camInfo->header = img_bridge.header;

        m_rightRgbPublisher.publish(img_bridge.toImageMsg(), camInfo);
    }

    void Context::rightGrayscalePublish(cv::Mat const& grayscaleData, std::uint64_t ts)
    {
        cv_bridge::CvImage img_bridge;
        img_bridge.image = grayscaleData;
        img_bridge.encoding = sensor_msgs::image_encodings::MONO8;
        img_bridge.header.frame_id = "cam1_opctical";
        img_bridge.header.stamp = convertStamp(ts);

        sensor_msgs::CameraInfoPtr camInfo(new sensor_msgs::CameraInfo());
        camInfo->width = grayscaleData.cols;
        camInfo->height = grayscaleData.rows;
        camInfo->header = img_bridge.header;

        m_rightGrayscalePublisher.publish(img_bridge.toImageMsg(), camInfo);
    }

    void Context::imuPublish(kalamos::ImuData const& imuData)
    {
        sensor_msgs::ImuPtr imu(new sensor_msgs::Imu());

        // frame is the map name
        imu->header.frame_id = "imu";
        imu->header.stamp = convertStamp(imuData.ts);

        // We don't know orientation -> set covariance[0] to -1 to signal this
        imu->orientation_covariance[0] = -1;

        // kalamos provides rad/sec
        // ROS wants rad/sec
        imu->angular_velocity.x = imuData.gyro[0];
        imu->angular_velocity.y = imuData.gyro[1];
        imu->angular_velocity.z = imuData.gyro[2];
        // We don't know covariance -> set all fields to 0
        imu->angular_velocity_covariance.fill(0);

        // kalamos provides m/s^2
        // ROS wants m/s^2
        imu->linear_acceleration.x = imuData.accel[0];
        imu->linear_acceleration.y = imuData.accel[1];
        imu->linear_acceleration.z = imuData.accel[2];
        imu->linear_acceleration_covariance.fill(0); // covariance unknown

        m_imuPublisher.publish(imu);
    }

    void Context::posePublish(kalamos::PoseData const& poseData)
    {
        geometry_msgs::PoseStampedPtr pose(new geometry_msgs::PoseStamped());

        // frame is the map name
        pose->header.frame_id = "map";
        pose->header.stamp = convertStamp(poseData.ts);

        pose->pose.position.x = poseData.translation[0];
        pose->pose.position.y = poseData.translation[1];
        pose->pose.position.z = poseData.translation[2];
        pose->pose.orientation.w = poseData.quaternion[0];
        pose->pose.orientation.x = poseData.quaternion[1];
        pose->pose.orientation.y = poseData.quaternion[2];
        pose->pose.orientation.z = poseData.quaternion[3];

        m_posePublisher.publish(pose);

        {
            geometry_msgs::TransformStamped transform;
            transform.header = pose->header;
            transform.header.frame_id = "map";
            transform.child_frame_id = "cam0";
            transform.transform.translation.x = pose->pose.position.x;
            transform.transform.translation.y = pose->pose.position.y;
            transform.transform.translation.z = pose->pose.position.z;
            transform.transform.rotation = pose->pose.orientation;
            m_transformBroadcaster.sendTransform(transform);
        }
        {
            geometry_msgs::TransformStamped transform;
            transform.header = pose->header;
            transform.header.frame_id = "cam0";
            transform.child_frame_id = "cam1";
            transform.transform.translation.x = 0;
            transform.transform.translation.y = -0.2; // arbitrary 20cm between leftcam and rightcam
            transform.transform.translation.z = 0;
            transform.transform.rotation.w = 1;
            transform.transform.rotation.x = 0;
            transform.transform.rotation.y = 0;
            transform.transform.rotation.z = 0;
            m_transformBroadcaster.sendTransform(transform);
        }
        {
            geometry_msgs::TransformStamped transform;
            transform.header = pose->header;
            transform.header.frame_id = "cam0_optical";
            transform.child_frame_id = "cam0_rect_optical";
            transform.transform.translation.x = 0;
            transform.transform.translation.y = 0;
            transform.transform.translation.z = 0;
            transform.transform.rotation.w = 1;
            transform.transform.rotation.x = 0;
            transform.transform.rotation.y = 0;
            transform.transform.rotation.z = 0;
            m_transformBroadcaster.sendTransform(transform);
        }
        {
            geometry_msgs::TransformStamped transform;
            transform.header = pose->header;
            transform.header.frame_id = "cam0";
            transform.child_frame_id = "cam0_optical";
            transform.transform.translation.x = 0;
            transform.transform.translation.y = 0;
            transform.transform.translation.z = 0;

            tf2::Quaternion quat;
            double yaw = -M_PI / 2.0;
            double pitch = 0.0;
            double roll = -M_PI / 2.0;
            quat.setRPY(roll, pitch, yaw);
            transform.transform.rotation.x = quat.x();
            transform.transform.rotation.y = quat.y();
            transform.transform.rotation.z = quat.z();
            transform.transform.rotation.w = quat.w();

            m_transformBroadcaster.sendTransform(transform);
        }
        {
            geometry_msgs::TransformStamped transform;
            transform.header = pose->header;
            transform.header.frame_id = "cam1";
            transform.child_frame_id = "cam1_optical";
            transform.transform.translation.x = 0;
            transform.transform.translation.y = 0;
            transform.transform.translation.z = 0;
            transform.transform.rotation.w = 1;
            transform.transform.rotation.x = 0;
            transform.transform.rotation.y = 0;
            transform.transform.rotation.z = 0;
            m_transformBroadcaster.sendTransform(transform);
        }
        {
            kalamos::Transform imu2cam(m_kalamosContext->getCam2ImuTransform());
            geometry_msgs::TransformStamped transform;
            transform.header = pose->header;
            transform.header.frame_id = "cam0";
            transform.child_frame_id = "imu";
            transform.transform.translation.x = imu2cam.translation[0];
            transform.transform.translation.y = imu2cam.translation[1];
            transform.transform.translation.z = imu2cam.translation[2];
            transform.transform.rotation.w = imu2cam.rotation[0];
            transform.transform.rotation.x = imu2cam.rotation[1];
            transform.transform.rotation.y = imu2cam.rotation[2];
            transform.transform.rotation.z = imu2cam.rotation[3];
            m_transformBroadcaster.sendTransform(transform);
        }

        {
            slamdunk_msgs::BoolStampedPtr keyframeMsg(new slamdunk_msgs::BoolStamped());
            keyframeMsg->header = pose->header;
            keyframeMsg->value = poseData.keyframe;
            m_keyframePublisher.publish(keyframeMsg);
        }

        {
            slamdunk_msgs::QualityStampedPtr qualityMsg(new slamdunk_msgs::QualityStamped());
            qualityMsg->header = pose->header;
            qualityMsg->quality.value = (uint8_t)poseData.quality;
            m_qualityPublisher.publish(qualityMsg);
        }
    }

    void Context::occupancyPublish(kalamos::OccupancyData const& occupancyData)
    {
        for (kalamos::OccupancyDataPoint const& occupancyDataPoint : occupancyData.points)
        {
            octomap::point3d point(
                    occupancyDataPoint.position[0],
                    occupancyDataPoint.position[1],
                    occupancyDataPoint.position[2]);
            m_octomapOctree.updateNode(point, occupancyDataPoint.occupied, true);
        }
        m_octomapOctree.updateInnerOccupancy();

        octomap_msgs::OctomapPtr octomap(new octomap_msgs::Octomap());

        octomap->header.frame_id = "map";
        octomap->header.stamp = ros::Time::now();

        octomap_msgs::binaryMapToMsg(m_octomapOctree, *octomap);

        m_occupancyPublisher.publish(octomap);
    }

    void Context::ultrasoundPublish(kalamos::UltrasoundData const& ultrasoundData)
    {
        sensor_msgs::RangePtr range(new sensor_msgs::Range());

        range->header.frame_id = "cam0";
        range->header.stamp = convertStamp(ultrasoundData.ts);

        range->radiation_type = sensor_msgs::Range::ULTRASOUND;
        // the size of the arc that the distance reading is
        // valid for [rad]
        // the object causing the range reading may have
        // been anywhere within -field_of_view/2 and
        // field_of_view/2 at the measured range->
        // 0 angle corresponds to the x-axis of the sensor.
        range->field_of_view = 0.3f; // TODO get value from kalamos

        // minimum range value [m]
        range->min_range = 0.0f; // TODO get value from kalamos

        // maximum range value [m]
        range->max_range = 10.0f; // TODO get value from kalamos

        // range data [m]
        // (Note: values < range_min or > range_max should be discarded)
        range->range = ultrasoundData.distance;

        m_ultrasoundPublisher.publish(range);
    }

    void Context::barometerPublish(kalamos::BarometerData const& barometerData)
    {
        sensor_msgs::FluidPressurePtr message(new sensor_msgs::FluidPressure());

        message->header.frame_id = "cam0";
        message->header.stamp = convertStamp(barometerData.ts);

        // fluid_pressure in Pascal
        message->fluid_pressure = barometerData.pressure;

        // 0 for variance unknown
        message->variance = 0;

        m_barometerPublisher.publish(message);
    }

    void Context::magnetometerPublish(kalamos::MagnetometerData const& magnetometerData)
    {
        sensor_msgs::MagneticFieldPtr message(new sensor_msgs::MagneticField());

        message->header.frame_id = "cam0";
        message->header.stamp = convertStamp(magnetometerData.ts);

        // kalamos provides uTesla
        // ROS wants Tesla
        // NaN for unknown component
        message->magnetic_field.x = magnetometerData.field[0] / 1000000.0f;
        message->magnetic_field.y = magnetometerData.field[1] / 1000000.0f;
        message->magnetic_field.z = magnetometerData.field[2] / 1000000.0f;

        // 0 for covariance unknown
        message->magnetic_field_covariance.fill(0);

        m_magnetometerPublisher.publish(message);
    }

    void Context::onStereoYuvData(kalamos::StereoYuvData const& stereoYuvData)
    {
        cv::Size rectCropSize(m_kalamosContext->getCropSize());
        cv::Mat leftRgbMat;
        if (m_leftRgbRectPublisher.getNumSubscribers() > 0)
        {
            // note: we are not using a CameraInfo here, instead we favor kalamos context
            // which has better knowledge of the fisheye camera model and will use the
            // same set of parameters it used to generate the depth frame

            // XXX: REP 104 seems to indicate that rectified image are supposed to
            // be the same size of original image?
            // http://www.ros.org/reps/rep-0104.html#different-dimensions-for-rectified-image
            //
            // > Finally, if truly needed, this behavior could be implemented as a
            // > node publishing the enlarged and rectified image and a tweaked
            // > CameraInfo to a separate namespace. This solution loses the ability
            // > to unrectify points back to the original image resolution, but this
            // > is a minor drawback.

            leftRgbMat = yuvToRgb(stereoYuvData.leftYuv, rectCropSize.width, rectCropSize.height);

            cv::Mat rectFrame;
            m_kalamosContext->rectifyFrame(leftRgbMat, rectFrame);
            leftRgbRectPublish(rectFrame, stereoYuvData.ts);
        }

        cv::Size cropSize{m_config.crop_width, m_config.crop_height};

        if (m_leftRgbPublisher.getNumSubscribers() > 0)
        {
            // Try to crop inside previously calculated RGB frame
            if (leftRgbMat.empty() || rectCropSize.width < cropSize.width || rectCropSize.height < cropSize.height)
            {
                leftRgbPublish(yuvToRgb(stereoYuvData.leftYuv, cropSize.width, cropSize.height), stereoYuvData.ts);
            }
            else
            {
                cv::Rect roi(
                    cv::Point((rectCropSize.width - cropSize.width) / 2, (rectCropSize.height - cropSize.height) / 2),
                    cropSize);
                leftRgbPublish(leftRgbMat(roi), stereoYuvData.ts);
            }
        }
        if (m_leftGrayscalePublisher.getNumSubscribers() > 0)
            leftGrayscalePublish(*yuvToGrayscale(stereoYuvData.leftYuv, cropSize.width, cropSize.height), stereoYuvData.ts);
        if (m_rightRgbPublisher.getNumSubscribers() > 0)
            rightRgbPublish(yuvToRgb(stereoYuvData.rightYuv, cropSize.width, cropSize.height), stereoYuvData.ts);
        if (m_rightGrayscalePublisher.getNumSubscribers() > 0)
            rightGrayscalePublish(*yuvToGrayscale(stereoYuvData.rightYuv, cropSize.width, cropSize.height), stereoYuvData.ts);
    }

    void fillMarkerControl(visualization_msgs::InteractiveMarkerControl* control)
    {
        visualization_msgs::Marker marker;
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        // The STL model crashes on ARM (Bus error),
        // the mesh file does not seem to produce this issue
        // marker.mesh_resource = "package://slamdunk_visualization/models/slamdunk.stl";
        marker.mesh_resource = "package://slamdunk_visualization/models/slamdunk.mesh";
        marker.mesh_use_embedded_materials = false;

        marker.color.r = 0.1;
        marker.color.g = 0.1;
        marker.color.b = 0.1;
        marker.color.a = 1;

        tf2::Quaternion quat;
        double yaw = -M_PI / 2;
        double pitch = 0;
        double roll = M_PI;
        quat.setRPY(roll, pitch, yaw);
        marker.pose.orientation.w = quat.w();
        marker.pose.orientation.x = quat.x();
        marker.pose.orientation.y = quat.y();
        marker.pose.orientation.z = quat.z();

        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;

        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;

        control->markers.push_back(marker);
    }

    void Context::menuInit()
    {
        m_menuServer.reset(new interactive_markers::InteractiveMarkerServer("menu_topic_ns", "menu_server_id", false));

        m_menuRestartCaptureHandle =
            m_menuHandle.insert("Restart capture", boost::bind(&Context::menuRestartCaptureCb, this, _1));
        m_menuRestartSlamHandle =
            m_menuHandle.insert("Restart slam", boost::bind(&Context::menuRestartSlamCb, this, _1));
        m_menuRestartOccupancyHandle =
            m_menuHandle.insert("Restart occupancy", boost::bind(&Context::menuRestartOccupancyCb, this, _1));

        visualization_msgs::InteractiveMarker int_marker;
        int_marker.header.frame_id = "cam0";
        int_marker.scale = 1;
        int_marker.name = "marker0";

        visualization_msgs::InteractiveMarkerControl control;
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
        control.always_visible = true;
        control.name = "Reset control";
        fillMarkerControl(&control);
        int_marker.controls.push_back(control);

        m_menuServer->insert( int_marker );

        m_menuHandle.apply(*m_menuServer, "marker0");
        m_menuServer->applyChanges();
    }

    void Context::onNewDepthFrame(const sensor_msgs::ImageConstPtr& depthMap, const sensor_msgs::CameraInfoConstPtr& camInfo)
    {
        sensor_msgs::PointCloud2Ptr pc(new sensor_msgs::PointCloud2());

        pc->header.frame_id = depthMap->header.frame_id;
        pc->header.stamp = depthMap->header.stamp;

        pc->width = depthMap->width;
        pc->height = depthMap->height;
        pc->is_bigendian = (__BYTE_ORDER == __BIG_ENDIAN);
        pc->is_dense = false;

        // Set the point fields to xyzrgb and resize the vector with the following command
        // 4 is for the number of added fields. Each come in triplet: the name of the PointField,
        // the number of occurences of the type in the PointField, the type of the PointField
        sensor_msgs::PointCloud2Modifier modifier(*pc);
        // You have to be aware that the following function does add extra padding
        // for backward compatibility though so it is definitely the solution of
        // choice for PointXYZ and PointXYZRGB
        modifier.setPointCloud2FieldsByString(1, "xyz");

        sensor_msgs::PointCloud2Iterator<float> iter_x(*pc, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*pc, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*pc, "z");

        image_geometry::PinholeCameraModel model;
        model.fromCameraInfo(camInfo);

        const float fx = model.fx();
        const float fy = model.fy();
        const float cx = model.cx();
        const float cy = model.cy();

        assert(depthMap->encoding == sensor_msgs::image_encodings::TYPE_32FC1);

        auto data = reinterpret_cast<const float*>(depthMap->data.data());
        int step = depthMap->step / sizeof(float);
        for (int r = 0; r < depthMap->height; ++r)
        {
            for (int c = 0; c < depthMap->width; ++c, ++iter_x, ++iter_y, ++iter_z)
            {
                float depth = data[r * step + c];

                // Missing points denoted by quiet NaNs as per REP 117
                if (depth == 0.0f)
                {
                    *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
                    continue;
                }

                *iter_x = ((c - cx) / fx) * depth;
                *iter_y = ((r - cy) / fy) * depth;
                *iter_z = depth;
            }
        }

        m_pointCloud2Publisher.publish(pc);
    }

    bool Context::restartCapture()
    {
        ROS_INFO("restart capture requested...");
        if (!m_captureHandle)
        {
            ROS_INFO("... but capture was not started to begin with");
            return false;
        }
        // octomap is going to be invalid, clear it
        m_octomapOctree.clear();
        return m_captureHandle->restart();
    }

    bool Context::restartSlam()
    {
        ROS_INFO("restart slam requested...");
        if (!m_slamHandle)
        {
            ROS_INFO("... but slam was not started to begin with");
            return false;
        }
        // octomap is going to be invalid, clear it
        m_octomapOctree.clear();
        return m_slamHandle->restart();
    }

    bool Context::restartOccupancy()
    {
        ROS_INFO("restart occupancy requested...");
        if (!m_occupancyHandle)
        {
            ROS_INFO("... but occupancy was not started to begin with");
            return false;
        }
        // octomap is going to be invalid, clear it
        m_octomapOctree.clear();
        return m_occupancyHandle->restart();
    }

    bool Context::restartCaptureServiceCb(RestartCaptureSrv::Request& req, RestartCaptureSrv::Response& resp)
    {
        return restartCapture();
    }

    bool Context::restartSlamServiceCb(RestartSlamSrv::Request& req, RestartSlamSrv::Response& resp)
    {
        return restartSlam();
    }

    bool Context::restartOccupancyServiceCb(RestartOccupancySrv::Request& req, RestartOccupancySrv::Response& resp)
    {
        return restartOccupancy();
    }

    void Context::menuRestartCaptureCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
    {
        restartCapture();
    }

    void Context::menuRestartSlamCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
    {
        restartSlam();
    }

    void Context::menuRestartOccupancyCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
    {
        restartOccupancy();
    }

    void Context::dynamicReconfigureCb(slamdunk_node::SLAMDunkNodeConfig& config, uint32_t level)
    {
        ROS_INFO("Reconfigure Request: %d %d %d",
                 level,
                 config.crop_width,
                 config.crop_height);

        m_config = config;
    }

    void Context::onClockOffsetTimerCallback(const ros::TimerEvent& e)
    {
        timespec kTime;
        if (clock_gettime(CLOCK_MONOTONIC, &kTime) == -1)
        {
            ROS_ERROR("fail to get kalamos time: %s", strerror(errno));
            return;
        }
        ros::Time rTime = ros::Time::now();

        m_kalamosToROSOffset = std::chrono::nanoseconds(rTime.toNSec()) -
                               (std::chrono::seconds(kTime.tv_sec) + std::chrono::nanoseconds(kTime.tv_nsec));

        ROS_DEBUG("time offset: %" PRId64 ", kalamos: %" PRId64 ", ros: %" PRId64, m_kalamosToROSOffset.count(),
                  (std::chrono::seconds(kTime.tv_sec) + std::chrono::nanoseconds(kTime.tv_nsec)).count(),
                  std::chrono::nanoseconds(rTime.toNSec()).count());
    }

    ros::Time Context::convertStamp(std::uint64_t kstamp) const
    {
        return ros::Time().fromNSec((std::chrono::nanoseconds(kstamp) + m_kalamosToROSOffset).count());
    }

    void Context::tick()
    {
        ros::spinOnce();
        if (!ros::ok())
        {
            // issue with ROS: finish periodic cb
            m_kalamosContext->setPeriodicCallback(0, nullptr);
        }

        bool needOccupancy =
            !!m_occupancyPublisher.getNumSubscribers();

        bool needPointCloud =
            !!m_pointCloud2Publisher.getNumSubscribers();

        bool needSlam =
            needOccupancy ||
            needPointCloud ||
            !!m_posePublisher.getNumSubscribers();

        bool needCapture =
            needSlam ||
            !!m_depthmapPublisher.getNumSubscribers() ||
            !!m_dispmapPublisher.getNumSubscribers() ||
            !!m_stereoPublisher.getNumSubscribers() ||
            !!m_leftRgbPublisher.getNumSubscribers() ||
            !!m_leftGrayscalePublisher.getNumSubscribers() ||
            !!m_leftRgbRectPublisher.getNumSubscribers() ||
            !!m_rightRgbPublisher.getNumSubscribers() ||
            !!m_rightGrayscalePublisher.getNumSubscribers() ||
            !!m_imuPublisher.getNumSubscribers() ||
            !!m_ultrasoundPublisher.getNumSubscribers() ||
            !!m_barometerPublisher.getNumSubscribers() ||
            !!m_magnetometerPublisher.getNumSubscribers();

        if (needCapture && !m_captureHandle)
        {
            ROS_INFO("start capture");
            m_captureHandle = m_kalamosContext->startService(kalamos::ServiceType::CAPTURE);
        }
        if (needSlam && !m_slamHandle)
        {
            ROS_INFO("start slam");
            m_slamHandle = m_kalamosContext->startService(kalamos::ServiceType::SLAM);
        }
        if (needOccupancy && !m_occupancyHandle)
        {
            ROS_INFO("start occupancy calculation");
            m_occupancyHandle = m_kalamosContext->startService(kalamos::ServiceType::OCCUPANCY);
        }

        if (needPointCloud && !m_pointCloud2DepthSubscriber)
        {
            m_pointCloud2DepthSubscriber =
                image_transport::ImageTransport(m_rosNode).subscribeCamera(
                        "depth_map/image", 1, &Context::onNewDepthFrame, this);
        }

        if (!needPointCloud && m_pointCloud2DepthSubscriber)
            m_pointCloud2DepthSubscriber = {};

        if (!needOccupancy && m_occupancyHandle)
        {
            ROS_INFO("stop occupancy");
            m_occupancyHandle.reset();
        }
        if (!needSlam && m_slamHandle)
        {
            ROS_INFO("stop slam");
            m_slamHandle.reset();
        }
        if (!needCapture && m_captureHandle)
        {
            ROS_INFO("stop capture");
            m_captureHandle.reset();
        }
    }

}

int main(int ac, char** av)
{
    // A version of ros::init takes (ac,av), another takes parsed args
    ros::init(ac, av, "slamdunk_node");

    ros::NodeHandle n;

    Context context(n);

    nodelet::Loader nodelet(n);

    {
        nodelet::M_string remappings(ros::names::getRemappings());
        nodelet::V_string nargv;

        // keyframe_publisher expects the following topics:
        // - input: Image camera_in, Bool keyframe_in
        // - output: Image camera_out
        remappings["image_in"] = ros::names::resolve("depth_map/image");
        remappings["keyframe_in"] = ros::names::resolve("keyframe");
        remappings["image_out"] = ros::names::resolve("depth_map/image_kf");
        nodelet.load(ros::this_node::getName() + "_depth_map_kf", "slamdunk_nodelets/KeyframePublisherNodelet", remappings, nargv);
    }
    {
        nodelet::M_string remappings(ros::names::getRemappings());
        nodelet::V_string nargv;

        // quality_publisher expects the following topics:
        // - input: Image camera_in, Bool quality_in
        // - output: Image camera_out
        remappings["image_in"] = ros::names::resolve("depth_map/image");
        remappings["quality_in"] = ros::names::resolve("quality");
        remappings["image_out"] = ros::names::resolve("depth_map/image_good");
        nodelet.load(ros::this_node::getName() + "_depth_map_good", "slamdunk_nodelets/GoodframePublisherNodelet", remappings, nargv);
    }
    {
        nodelet::M_string remappings(ros::names::getRemappings());
        nodelet::V_string nargv;

        // quality_publisher expects the following topics:
        // - input: Image camera_in, Bool quality_in
        // - output: Image camera_out
        remappings["image_in"] = ros::names::resolve("depth_map/image_kf");
        remappings["quality_in"] = ros::names::resolve("quality");
        remappings["image_out"] = ros::names::resolve("depth_map/image_kf_good");
        nodelet.load(ros::this_node::getName() + "_depth_map_kf_good", "slamdunk_nodelets/GoodframePublisherNodelet", remappings, nargv);
    }

    {
        nodelet::M_string remappings(ros::names::getRemappings());
        nodelet::V_string nargv;

        // keyframe_publisher expects the following topics:
        // - input: Image camera_in, Bool keyframe_in
        // - output: Image camera_out
        remappings["image_in"] = ros::names::resolve("left_rgb/image");
        remappings["keyframe_in"] = ros::names::resolve("keyframe");
        remappings["image_out"] = ros::names::resolve("left_rgb/image_kf");
        nodelet.load(ros::this_node::getName() + "_left_rgb_kf", "slamdunk_nodelets/KeyframePublisherNodelet", remappings, nargv);
    }
    {
        nodelet::M_string remappings(ros::names::getRemappings());
        nodelet::V_string nargv;

        // quality_publisher expects the following topics:
        // - input: Image camera_in, Bool quality_in
        // - output: Image camera_out
        remappings["image_in"] = ros::names::resolve("left_rgb/image");
        remappings["quality_in"] = ros::names::resolve("quality");
        remappings["image_out"] = ros::names::resolve("left_rgb/image_good");
        nodelet.load(ros::this_node::getName() + "_left_rgb_good", "slamdunk_nodelets/GoodframePublisherNodelet", remappings, nargv);
    }
    {
        nodelet::M_string remappings(ros::names::getRemappings());
        nodelet::V_string nargv;

        // quality_publisher expects the following topics:
        // - input: Image camera_in, Bool quality_in
        // - output: Image camera_out
        remappings["image_in"] = ros::names::resolve("left_rgb/image_kf");
        remappings["quality_in"] = ros::names::resolve("quality");
        remappings["image_out"] = ros::names::resolve("left_rgb/image_kf_good");
        nodelet.load(ros::this_node::getName() + "_left_rgb_kf_good", "slamdunk_nodelets/GoodframePublisherNodelet", remappings, nargv);
    }

    {
        nodelet::M_string remappings(ros::names::getRemappings());
        nodelet::V_string nargv;

        // keyframe_publisher expects the following topics:
        // - input: Image image_in, Bool keyframe_in
        // - output: Image image_out
        remappings["image_in"] = ros::names::resolve("left_rgb_rect/image_rect_color");
        remappings["keyframe_in"] = ros::names::resolve("keyframe");
        remappings["image_out"] = ros::names::resolve("left_rgb_rect/image_rect_color_kf");
        nodelet.load(ros::this_node::getName() + "_left_rgb_rect_kf", "slamdunk_nodelets/KeyframePublisherNodelet", remappings, nargv);
    }
    {
        nodelet::M_string remappings(ros::names::getRemappings());
        nodelet::V_string nargv;

        // quality_publisher expects the following topics:
        // - input: Image image_in, Bool quality_in
        // - output: Image image_out
        remappings["image_in"] = ros::names::resolve("left_rgb_rect/image_rect_color");
        remappings["quality_in"] = ros::names::resolve("quality");
        remappings["image_out"] = ros::names::resolve("left_rgb_rect/image_rect_color_good");
        nodelet.load(ros::this_node::getName() + "_left_rgb_rect_good", "slamdunk_nodelets/GoodframePublisherNodelet", remappings, nargv);
    }
    {
        nodelet::M_string remappings(ros::names::getRemappings());
        nodelet::V_string nargv;

        // keyframe_publisher expects the following topics:
        // - input: Image image_in, Bool keyframe_in
        // - output: Image image_out
        remappings["image_in"] = ros::names::resolve("left_rgb_rect/image_rect_color_kf");
        remappings["quality_in"] = ros::names::resolve("quality");
        remappings["image_out"] = ros::names::resolve("left_rgb_rect/image_rect_color_kf_good");
        nodelet.load(ros::this_node::getName() + "_left_rgb_rect_kf_good", "slamdunk_nodelets/GoodframePublisherNodelet", remappings, nargv);
    }

    {
        nodelet::M_string remappings(ros::names::getRemappings());
        nodelet::V_string nargv;

        // dephmap_image_proc/point_cloud_xyzrgb expects the following topics:
        // - input: depth_registered/image_rect, rgb/image_rect_color and rgb/camera_info
        // - output: depth_registered/points
        remappings["rgb/image_rect_color"] = ros::names::resolve("left_rgb_rect/image_rect_color");
        remappings["rgb/camera_info"] = ros::names::resolve("left_rgb_rect/camera_info");
        remappings["depth_registered/image_rect"] = ros::names::resolve("depth_map/image");
        remappings["depth_registered/points"] = ros::names::resolve("pcl_xyzrgb");
        nodelet.load(ros::this_node::getName() + "_xyzrgb", "depth_image_proc/point_cloud_xyzrgb", remappings, nargv);
    }
    {
        nodelet::M_string remappings(ros::names::getRemappings());
        nodelet::V_string nargv;

        // dephmap_image_proc/point_cloud_xyzrgb expects the following topics:
        // - input: depth_registered/image_rect, rgb/image_rect_color and rgb/camera_info
        // - output: depth_registered/points
        remappings["rgb/image_rect_color"] = ros::names::resolve("left_rgb_rect/image_rect_color_good");
        remappings["rgb/camera_info"] = ros::names::resolve("left_rgb_rect/camera_info");
        remappings["depth_registered/image_rect"] = ros::names::resolve("depth_map/image_good");
        remappings["depth_registered/points"] = ros::names::resolve("pcl_xyzrgb_good");
        nodelet.load(ros::this_node::getName() + "_xyzrgb_good", "depth_image_proc/point_cloud_xyzrgb", remappings, nargv);
    }
    {
        nodelet::M_string remappings(ros::names::getRemappings());
        nodelet::V_string nargv;

        // dephmap_image_proc/point_cloud_xyzrgb expects the following topics:
        // - input: depth_registered/image_rect, rgb/image_rect_color and rgb/camera_info
        // - output: depth_registered/points
        remappings["rgb/image_rect_color"] = ros::names::resolve("left_rgb_rect/image_rect_color_kf");
        remappings["rgb/camera_info"] = ros::names::resolve("left_rgb_rect/camera_info");
        remappings["depth_registered/image_rect"] = ros::names::resolve("depth_map/image_kf");
        remappings["depth_registered/points"] = ros::names::resolve("pcl_xyzrgb_kf");
        nodelet.load(ros::this_node::getName() + "_xyzrgb_kf", "depth_image_proc/point_cloud_xyzrgb", remappings, nargv);
    }
    {
        nodelet::M_string remappings(ros::names::getRemappings());
        nodelet::V_string nargv;

        // dephmap_image_proc/point_cloud_xyzrgb expects the following topics:
        // - input: depth_registered/image_rect, rgb/image_rect_color and rgb/camera_info
        // - output: depth_registered/points
        remappings["rgb/image_rect_color"] = ros::names::resolve("left_rgb_rect/image_rect_color_kf_good");
        remappings["rgb/camera_info"] = ros::names::resolve("left_rgb_rect/camera_info");
        remappings["depth_registered/image_rect"] = ros::names::resolve("depth_map/image_kf_good");
        remappings["depth_registered/points"] = ros::names::resolve("pcl_xyzrgb_kf_good");
        nodelet.load(ros::this_node::getName() + "_xyzrgb_kf_good", "depth_image_proc/point_cloud_xyzrgb", remappings, nargv);
    }

    {
        nodelet::M_string remappings(ros::names::getRemappings());
        nodelet::V_string nargv;

        // pcl_xyzrgb_concat expects the following topics:
        // - input: PointCloud2 pcl_xyzrgb
        // - output: PointCloud2 pcldepth_registered/points
        remappings["pcl_xyzrgb_in"] = ros::names::resolve("pcl_xyzrgb_kf_good");
        remappings["pcl_xyzrgb_out"] = ros::names::resolve("pcl_xyzrgb_concat");
        remappings["clear"] = ros::names::resolve("pcl_xyzrgb_concat_clear");
        remappings["start"] = ros::names::resolve("pcl_xyzrgb_concat_start");
        remappings["stop"] = ros::names::resolve("pcl_xyzrgb_concat_stop");
        remappings["save"] = ros::names::resolve("pcl_xyzrgb_concat_save");
        remappings["clear"] = ros::names::resolve("pcl_xyzrgb_concat_clear");
        nodelet.load(ros::this_node::getName() + "_xyzrgb_concat", "slamdunk_nodelets/PclXYZRGBConcatNodelet", remappings, nargv);
    }

    {
        nodelet::M_string remappings(ros::names::getRemappings());
        nodelet::V_string nargv;

        remappings["path"] = ros::names::resolve("path");
        remappings["path_keyframe"] = ros::names::resolve("path_keyframe");
        nodelet.load(ros::this_node::getName() + "_pose_to_path", "slamdunk_nodelets/PoseToPathNodelet", remappings, nargv);
    }

    {
        nodelet::M_string remappings(ros::names::getRemappings());
        nodelet::V_string nargv;

        // pcl_xyzrgb_record expects the following topics:
        // - input: PointCloud2 pcl_xyzrgb
        remappings["pcl_xyzrgb_in"] = ros::names::resolve("pcl_xyzrgb_kf_good");
        remappings["start"] = ros::names::resolve("pcl_xyzrgb_kf_record_start");
        remappings["stop"] = ros::names::resolve("pcl_xyzrgb_kf_record_stop");
        nodelet.load(ros::this_node::getName() + "_xyzrgb_kf_record", "slamdunk_nodelets/PclXYZRGBRecordNodelet", remappings, nargv);
    }

    {
        kalamos::Callbacks cbs;

        cbs.depthmapCallback = std::bind(&Context::depthmapPublish, &context, std::placeholders::_1);
        cbs.dispmapCallback = std::bind(&Context::dispmapPublish, &context, std::placeholders::_1);
        cbs.stereoYuvCallback = std::bind(&Context::onStereoYuvData, &context, std::placeholders::_1);
        cbs.imuCallback = std::bind(&Context::imuPublish, &context, std::placeholders::_1);
        cbs.poseCallback = std::bind(&Context::posePublish, &context, std::placeholders::_1);
        cbs.occupancyCallback = std::bind(&Context::occupancyPublish, &context, std::placeholders::_1);
        cbs.ultrasoundCallback = std::bind(&Context::ultrasoundPublish, &context, std::placeholders::_1);
        cbs.barometerCallback = std::bind(&Context::barometerPublish, &context, std::placeholders::_1);
        cbs.magnetometerCallback = std::bind(&Context::magnetometerPublish, &context, std::placeholders::_1);

        cbs.period = 30;
        cbs.periodicCallback = std::bind(&Context::tick, &context);

        std::unique_ptr<kalamos::Context> kalamosContext = kalamos::init(cbs);
        if (kalamosContext)
        {
            context.setKalamosContext(kalamosContext.get());
            kalamosContext->run();
        }
    }

    return 0;
}
