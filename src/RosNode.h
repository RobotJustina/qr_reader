/* ---------------------------------------------------------------------------
** This software is in the public domain, furnished "as is", without technical
** support, and with no warranty, express or implied, as to its usefulness for
** any purpose.
**
** RosNode.h
** Class definition for a ROS node to be used with the QR code detector
** It creates the ROS node, configures the ROS envoronment, sets up publishers
** and subscribers, and manages the execution loop.
** 
** Author: Mauricio Matamoros
** -------------------------------------------------------------------------*/
#pragma once
#ifndef __ROS_NODE_H__
#define __ROS_NODE_H__

#include <cstdlib>
#include <string>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/thread.hpp>
#include <boost/signals2/signal.hpp>
#include <image_transport/image_transport.h>
#include "Types.h"

namespace qr_reader{

	class RosNode{
	public:
		RosNode(int argc, char** argv, const std::string& image_src, const std::string& recognized_text);

		void addImageReceivedHandler(const cvImgFunctionType& handler);
		void publishRecognizedText(const std::string& text);
		void runAsyncSpin();
		void spin();

	private:
		ros::NodeHandle nh;
		ros::Publisher text_publisher;
		image_transport::Subscriber image_subscriber;
		image_transport::ImageTransport it;
		cvImgFunction imageReceived;
		boost::thread *mainThread;

		void imageCallback(const sensor_msgs::ImageConstPtr& msg);
		void mainThreadTask();
	};

} /* namespace qr_reader */

// ros::isInitialized ()


#endif /* __ROS_NODE_H__ */