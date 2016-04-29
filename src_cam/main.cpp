#include <cstdlib>
#include <string>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/asio/signal_set.hpp>
#include <boost/asio.hpp>

// Topic for publishing captured video.
#define CAMERA_IMG "camera/image"

CvCapture *capture = NULL;

void ctrlC_handler(const boost::system::error_code& error, int signal_number){
	if (error){
		std::cerr<<"signalHandler() :"<<error.message();
		return;
	}

	if(signal_number == SIGTERM) std::cout<<"Recvd SIGTERM, exiting...";
	if(signal_number == SIGINT) std::cout<<"Recvd SIGINT, exiting...";
	cvDestroyAllWindows();
	cvReleaseCapture(&capture);
	exit(0);
}

int main(int argc,char *argv[])
{
	std::cout << "Initializing video capture device... " << std::endl;
	// start camera
	capture = cvCaptureFromCAM(0);
	if(!capture){
		std::cerr << "No camera detected!!!" << std::endl;
		return -1;
	}
	std::cout << "Done!" << std::endl;

	// Construct a signal set registered for process termination.
	boost::asio::io_service io;
	boost::asio::signal_set ctrlC(io, SIGINT, SIGTERM);
	// Start an asynchronous wait for one of the signals to occur.
	ctrlC.async_wait(ctrlC_handler);
	// io.run();

	ros::init(argc, argv, "camera");
	ros::NodeHandle nodeHandle;
	image_transport::ImageTransport it(nodeHandle);
	image_transport::Publisher pub;
	pub = it.advertise(CAMERA_IMG, 1);

	cvNamedWindow("src", 1);
	// cvStartWindowThread();

	std::cout << std::endl << "Publishing video in " << CAMERA_IMG << std::endl;

	while(true){
		IplImage *frame = cvQueryFrame(capture);
		if( !frame ){
			std::cerr << "Can't read from capture device." << std::endl;
			break;
		}
		cvShowImage("src", frame);

		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(
			std_msgs::Header(),
			"bgr8",
			frame).toImageMsg();
		
		pub.publish(msg);
		io.poll();
		ros::spinOnce();

		char key = cvWaitKey( 10 );
		if( key >= 27 )
			break;
	}

	std::cout << "Stopping capture device... ";
	cvDestroyAllWindows();
	cvReleaseCapture(&capture);
	std::cout << "Stopped." << std::endl;

	return 0;
}