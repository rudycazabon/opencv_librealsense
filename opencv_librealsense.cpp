// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#define CHRONO
#define GLFW
#define OPENCV

#ifdef CHRONO
#include <chrono>
#endif 

#include <vector>
#include <sstream>
#include <iostream>
#include <algorithm>

#include <librealsense/rs.hpp>
#include "example.hpp"
#ifdef OPENCV
#include <opencv2/opencv.hpp>
#endif

inline void glVertex(const rs::float3 & vertex) { glVertex3fv(&vertex.x); }
inline void glTexCoord(const rs::float2 & tex_coord) { glTexCoord2fv(&tex_coord.x); }

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
};

static rs::context ctx;
static state app_state;
static int frames = 0;

#ifdef CHRONO
static float nexttime = 0, fps = 0;
static std::chrono::steady_clock::time_point t0;
#endif

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
		app_state = initState;

#ifdef CHRONO
		auto t0 = std::chrono::high_resolution_clock::now();
#endif
		return &app_state;
	}
	return 0;
}

bool app_next_frame(int &xInOut, int &yInOut, int &zInOut)
{
	rs::device & dev = *app_state.dev;

	if (dev.is_streaming())
		dev.wait_for_frames();

#ifdef CHRONO
	auto t1 = std::chrono::high_resolution_clock::now();
	nexttime += std::chrono::duration<float>(t1 - t0).count();
	t0 = t1;
	++frames;
	if (nexttime > 0.5f)
	{
		fps = frames / nexttime;
		frames = 0;
		nexttime = 0;
	}
#endif

	const rs::stream tex_stream = app_state.tex_streams[app_state.index];
	app_state.depth_scale = dev.get_depth_scale();
	app_state.extrin = dev.get_extrinsics(rs::stream::depth, tex_stream);
	app_state.depth_intrin = dev.get_stream_intrinsics(rs::stream::depth);
	app_state.tex_intrin = dev.get_stream_intrinsics(tex_stream);
	app_state.identical = app_state.depth_intrin == app_state.tex_intrin && app_state.extrin.is_identity();

#ifdef OPENCV
	// setup the OpenCV Mat structures
	cv::Mat depth16(app_state.depth_intrin.height, app_state.depth_intrin.width, CV_16U, (uchar *)dev.get_frame_data(rs::stream::depth));

	rs::intrinsics color_intrin = dev.get_stream_intrinsics(rs::stream::color);
	cv::Mat rgb(color_intrin.height, color_intrin.width, CV_8UC3, (uchar *)dev.get_frame_data(rs::stream::color));

	// ignore depth greater than 800 mm's.
	depth16.setTo(10000, depth16 > 800);
	depth16.setTo(10000, depth16 == 0);
	cv::Mat depth8u = depth16 < 800;

	depth8u.convertTo(depth8u, CV_8UC1, 255.0/800);

#if false
	cv::Point handPoint(0, 0);
	for (int y = 0; y < depth8u.rows; ++y)
	{
		uchar *d = depth8u.row(y).ptr();
		for (int x = 0; x < depth8u.cols; ++x)
		{
			if (d[x])
			{
				int floodCount = cv::floodFill(depth8u, cv::Point(x, y), 255);

				// we have found the top most point
				if (floodCount > 100)
				{
					handPoint = cv::Point(x, y);
					break;
				}
			}
		}
		if (handPoint != cv::Point(0, 0)) break;
	}

	if (handPoint != cv::Point(0, 0))
		cv::circle(depth8u, handPoint, 10, 172, cv::FILLED);
#endif

	imshow("depth8u", depth8u);
	cvWaitKey(1);

	cv::cvtColor(rgb, rgb, cv::COLOR_BGR2RGB);
	imshow("rgb", rgb);
	cvWaitKey(1);

// xInOut = handPoint.x;
// yInOut = handPoint.y;
// zInOut = 0; // not using this yet.
// if (handPoint == cv::Point(0, 0))
// 	return false;
#endif

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

#ifdef GLFW
	glfwInit();
	std::ostringstream ss; ss << "CPP Point Cloud Example (" << dev.get_name() << ")";
	GLFWwindow * win = glfwCreateWindow(640, 480, ss.str().c_str(), 0, 0);
	glfwSetWindowUserPointer(win, app_state);

	glfwSetMouseButtonCallback(win, [](GLFWwindow * win, int button, int action, int mods)
	{
		auto s = (state *)glfwGetWindowUserPointer(win);
		if (button == GLFW_MOUSE_BUTTON_LEFT) s->ml = action == GLFW_PRESS;
		if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) s->index = (s->index + 1) % s->tex_streams.size();
	});

	glfwSetCursorPosCallback(win, [](GLFWwindow * win, double x, double y)
	{
		auto s = (state *)glfwGetWindowUserPointer(win);
		if (s->ml)
		{
			s->yaw -= (x - s->lastX);
			s->yaw = std::max(s->yaw, -120.0);
			s->yaw = std::min(s->yaw, +120.0);
			s->pitch += (y - s->lastY);
			s->pitch = std::max(s->pitch, -80.0);
			s->pitch = std::min(s->pitch, +80.0);
		}
		s->lastX = x;
		s->lastY = y;
	});

	glfwSetKeyCallback(win, [](GLFWwindow * win, int key, int scancode, int action, int mods)
	{
		auto s = (state *)glfwGetWindowUserPointer(win);
		if (action == GLFW_RELEASE)
		{
			if (key == GLFW_KEY_ESCAPE) glfwSetWindowShouldClose(win, 1);
			else if (key == GLFW_KEY_F1)
			{
				if (!s->dev->is_streaming()) s->dev->start();
			}
			else if (key == GLFW_KEY_F2)
			{
				if (s->dev->is_streaming()) s->dev->stop();
			}
		}
	});

	glfwMakeContextCurrent(win);
	texture_buffer tex;
#ifdef CHRONO
	int frames = 0; float time = 0, fps = 0;
	auto t0 = std::chrono::high_resolution_clock::now();
#endif
	while (!glfwWindowShouldClose(win))
	{
		glfwPollEvents();

		// wait for the next frames.
		//cv::Point handPoint(0, 0);
		// int z = 0;
		static int x, y, z;
		bool handPresent = app_next_frame(x, y, z);

		glPushAttrib(GL_ALL_ATTRIB_BITS);

		tex.upload(dev, app_state->tex_streams[app_state->index]);

		int width, height;
		glfwGetFramebufferSize(win, &width, &height);
		glViewport(0, 0, width, height);
		glClearColor(52.0f / 255, 72.f / 255, 94.0f / 255.0f, 1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		gluPerspective(60, (float)width / height, 0.01f, 20.0f);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);

		glTranslatef(0, 0, +0.5f);
		glRotated(app_state->pitch, 1, 0, 0);
		glRotated(app_state->yaw, 0, 1, 0);
		glTranslatef(0, 0, -0.5f);

		glPointSize((float)width / 640);
		glEnable(GL_DEPTH_TEST);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, tex.get_gl_handle());
		glBegin(GL_POINTS);

		auto points = reinterpret_cast<const rs::float3 *>(dev.get_frame_data(rs::stream::points));
		for (int y = 0; y< app_state->depth_intrin.height; ++y)
		{
			for (int x = 0; x< app_state->depth_intrin.width; ++x)
			{
				if (points->z) 
				{
					if (points->z < 0.8f) // if we are closer that x meters (units are meters).
					{
						glTexCoord(app_state->identical ? app_state->tex_intrin.pixel_to_texcoord({ static_cast<float>(x),static_cast<float>(y) }) :
									app_state->tex_intrin.project_to_texcoord(app_state->extrin.transform(*points)));
						glVertex(*points);
					}
				}
				++points;
			}
		}
		glEnd();
		glPopMatrix();
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glPopAttrib();

		glfwGetWindowSize(win, &width, &height);
		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glPushMatrix();
		glOrtho(0, width, height, 0, -1, +1);

		std::ostringstream ss; ss << dev.get_name() << " (" << app_state->tex_streams[app_state->index] << ")";
		draw_text((width - get_text_width(ss.str().c_str())) / 2, height - 20, ss.str().c_str());

#ifdef CHRONO
		ss.str(""); ss << fps << " FPS";
		draw_text(20, 40, ss.str().c_str());
#endif
		glPopMatrix();

		glfwSwapBuffers(win);
	}

	glfwDestroyWindow(win);
	glfwTerminate();

	return EXIT_SUCCESS;
#else
	while (true) {
		// wait for the next frames.
		cv::Point handPoint(0, 0);
		int z = 0;
		bool handPresent = app_next_frame(handPoint.x, handPoint.y, z);
	}

	return EXIT_SUCCESS;
#endif
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
