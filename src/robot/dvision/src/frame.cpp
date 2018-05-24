// Created on: May 20, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>

#include "dvision/frame.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

namespace dvision {
ros_h264_streamer::H264Encoder* Frame::m_encoder = nullptr;
ros_h264_streamer::H264Decoder* Frame::m_decoder = nullptr;

void
Frame::initEncoder()
{
    std_msgs::Header header;
    // FIXME(MWX): remove magic number
    cv::Mat m(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
    cv_bridge::CvImage cvmsg(header, "bgr8", m);
    sensor_msgs::ImagePtr msg = cvmsg.toImageMsg();
    m_encoder = new ros_h264_streamer::H264Encoder(msg->width, msg->height, 20, 30, 1, msg->encoding);
    m_decoder = new ros_h264_streamer::H264Decoder(msg->width, msg->height);
}

std::unique_ptr<uint8_t>
Frame::encode(int& length)
{
    if (!m_encoder) {
        ROS_ERROR("Encoder not initialised!");
        return std::unique_ptr<uint8_t>();
    }
    std_msgs::Header header;
    cv_bridge::CvImage cvmsg(header, "bgr8", getBGR());
    sensor_msgs::ImagePtr msg = cvmsg.toImageMsg();

    //    ros::Time t1 = ros::Time::now();
    auto res = m_encoder->encode(msg);
    //    ros::Time t2 = ros::Time::now();

    //    ros::Duration d = t2 -  t1;
    //    ROS_INFO("Image encoding: size: %d, time: %lf", res.frame_size, d.toSec());

    std::unique_ptr<uint8_t> buf(new uint8_t[res.frame_size + 4]);
    memcpy(buf.get(), &res.frame_size, sizeof(uint32_t));
    memcpy(&buf.get()[4], res.frame_data, sizeof(uint8_t) * res.frame_size);

    length = res.frame_size + sizeof(uint32_t);
    return buf;
}

std::unique_ptr<uint8_t>
Frame::encode(const cv::Mat& src, int& length)
{
    if (!m_encoder) {
        ROS_ERROR("Encoder not initialised!");
        return std::unique_ptr<uint8_t>();
    }
    std_msgs::Header header;
    cv_bridge::CvImage cvmsg(header, "bgr8", src.clone());
    sensor_msgs::ImagePtr msg = cvmsg.toImageMsg();

    ros::Time t1 = ros::Time::now();
    auto res = m_encoder->encode(msg);
    ros::Time t2 = ros::Time::now();

    ros::Duration d = t2 - t1;
    // ROS_INFO("Image encoding: size: %d, time: %lf", res.frame_size, d.toSec());
    std::unique_ptr<uint8_t> buf(new uint8_t[res.frame_size + 4]);
    memcpy(buf.get(), &res.frame_size, sizeof(uint32_t));
    memcpy(&buf.get()[4], res.frame_data, sizeof(uint8_t) * res.frame_size);

    length = res.frame_size + sizeof(uint32_t);
    return buf;
}

void
Frame::decode(void* buffer)
{
    if (!m_decoder) {
        ROS_WARN("Decoder not initialised!");
        return;
    }
    int size; // !? 32t
    memcpy(&size, buffer, sizeof(uint32_t));
    sensor_msgs::ImagePtr out(new sensor_msgs::Image);

    //    ros::Time t1 = ros::Time::now();
    m_decoder->decode(size, &static_cast<uint8_t*>(buffer)[4], out);
    //    ros::Time t2 = ros::Time::now();
    //    ROS_INFO("Image decoding: size: %d, time: %lf", size, (t2 - t1).toSec());

    cv_bridge::CvImagePtr cvout = cv_bridge::toCvCopy(out);
    m_bgr = cvout->image;
    m_converted = true;
}

void
Frame::save(std::string path)
{
    cvt();
    std::string filename = path + std::to_string(m_timeStamp.toNSec()) + ".png";
    cv::imwrite(filename, m_bgr);

    std::cout << "Saved as " << filename << std::endl;
}

void
Frame::show()
{
    cvt();
    cv::namedWindow("camera", CV_WINDOW_NORMAL);

    cv::imshow("camera", m_bgr);
}
} // namespace dvision
