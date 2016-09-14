// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.
#include <vector>
#include <sstream>
#include <iostream>
#include <algorithm>

#include <librealsense/rs.hpp>
#include <opencv2/opencv.hpp>

struct state {
	double yaw, pitch, lastX, lastY;
	bool ml;
	std::vector<rs::stream> tex_streams;
	float depth_scale;
	rs::extrinsics extrin;
	rs::intrinsics depth_intrin;
	rs::intrinsics tex_intrin;
	bool identical;
	int index;
	rs::device * dev;

	cv::SimpleBlobDetector::Params params;

	std::vector<cv::KeyPoint> keypoints;

	cv::Ptr<cv::SimpleBlobDetector> detector;
};

static rs::context ctx;
static state app_state;
static int frames = 0;

state *initialize_app_state()
{
	if (ctx.get_device_count() > 0)
	{
		static rs::device & dev = *ctx.get_device(0);

		int inputWidth = 320, inputHeight = 240, frameRate = 60;
		dev.enable_stream(rs::stream::color, inputWidth, inputHeight, rs::format::rgb8, frameRate);
		dev.enable_stream(rs::stream::depth, inputWidth, inputHeight, rs::format::z16, frameRate);
		dev.start();

		// some placeholders were added for intrinsics and extrinsics.
		state initState = { 0, 0, 0, 0, false,{ rs::stream::color, rs::stream::depth, rs::stream::infrared }, dev.get_depth_scale(),
			dev.get_extrinsics(rs::stream::depth, rs::stream::color), dev.get_stream_intrinsics(rs::stream::depth),
			dev.get_stream_intrinsics(rs::stream::depth), 0, 0, &dev };

		initState.params.minThreshold = 10;
		initState.params.maxThreshold = 200;
		// Filter by Area.
		initState.params.filterByArea = true;
		initState.params.minArea = 1500;

		// Filter by Circularity
		initState.params.filterByCircularity = true;
		initState.params.minCircularity = 0.1f;

		// Filter by Convexity
		initState.params.filterByConvexity = true;
		initState.params.minConvexity = 0.87f;

		// Filter by Inertia
		initState.params.filterByInertia = true;
		initState.params.minInertiaRatio = 0.01f;

		initState.detector = cv::SimpleBlobDetector::create(initState.params);

		app_state = initState;

		return &app_state;
	}
	return 0;
}

bool app_next_frame(int &xInOut, int &yInOut, int &zInOut)
{
	rs::device & dev = *app_state.dev;

	if (dev.is_streaming())
		dev.wait_for_frames();

	const rs::stream tex_stream = app_state.tex_streams[app_state.index];
	app_state.depth_scale = dev.get_depth_scale();
	app_state.extrin = dev.get_extrinsics(rs::stream::depth, tex_stream);
	app_state.depth_intrin = dev.get_stream_intrinsics(rs::stream::depth);
	app_state.tex_intrin = dev.get_stream_intrinsics(tex_stream);
	app_state.identical = app_state.depth_intrin == app_state.tex_intrin && app_state.extrin.is_identity();

	// setup the OpenCV Mat structures
	cv::Mat depth16(app_state.depth_intrin.height, app_state.depth_intrin.width, CV_16U, (uchar *)dev.get_frame_data(rs::stream::depth));

	rs::intrinsics color_intrin = dev.get_stream_intrinsics(rs::stream::color);
	cv::Mat rgb(color_intrin.height, color_intrin.width, CV_8UC3, (uchar *)dev.get_frame_data(rs::stream::color));

	cv::Mat depth8u = depth16;// < 800;

	depth8u.convertTo(depth8u, CV_8U, 255.0 / 1000);

	cv::Mat imPoints;
	cv::drawKeypoints(depth8u,
		app_state.keypoints,
		imPoints,
		cv::Scalar(0, 0, 255),
		cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

	imshow("depth8u", imPoints);
	cvWaitKey(1);

	cv::cvtColor(rgb, rgb, cv::COLOR_BGR2RGB);
	app_state.detector->detect(rgb, app_state.keypoints);



	imshow("rgb", rgb);
	cvWaitKey(1);

	// app_state.keypoints.clear();

	return true;
}

int main(int argc, char * argv[]) try
{
	rs::log_to_console(rs::log_severity::warn);

	state *app_state = initialize_app_state();
	if (app_state == 0)
	{
		std::cout << "Unable to locate a camera" << std::endl;
		rs::log_to_console(rs::log_severity::fatal);
		return EXIT_FAILURE;
	}
	rs::device & dev = *app_state->dev;

	while (true) {
		// wait for the next frames.
		cv::Point handPoint(0, 0);
		int z = 0;
		app_next_frame(handPoint.x, handPoint.y, z);
	}

	return EXIT_SUCCESS;

}
catch (const rs::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception & e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}
