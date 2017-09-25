//
// Copyright (c) 2009, Markus Rickert
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#ifndef RL_HAL_DC1394CAMERA_H
#define RL_HAL_DC1394CAMERA_H

#if (LIBDC1394_VERSION_MAJOR > 10)
#include <dc1394/dc1394.h>
#else
#include <libdc1394/dc1394_control.h>
#include <libraw1394/raw1394.h>
#endif

#include <string>

#include "Camera.h"
#include "CyclicDevice.h"
#include "DeviceException.h"

namespace rl
{
	namespace hal
	{
		/**
		 * IEEE 1394 based cameras. 
		 */
		class Dc1394Camera : public Camera, public CyclicDevice
		{
			public:
				enum ColorCoding
				{
#if (LIBDC1394_VERSION_MAJOR > 10)
					  COLOR_CODING_MONO8 = ::DC1394_COLOR_CODING_MONO8,
#else
					  COLOR_CODING_MONO8 = ::COLOR_FORMAT7_MONO8,
#endif
					  COLOR_CODING_YUV411,
					  COLOR_CODING_YUV422,
					  COLOR_CODING_YUV444,
					  COLOR_CODING_RGB8,
					  COLOR_CODING_MONO16,
					  COLOR_CODING_RGB16,
					  COLOR_CODING_MONO16S,
					  COLOR_CODING_RGB16S,
					  COLOR_CODING_RAW8,
					  COLOR_CODING_RAW16
				};
				
				enum Feature
				{
#if (LIBDC1394_VERSION_MAJOR > 10)
					FEATURE_BRIGHTNESS = ::DC1394_FEATURE_BRIGHTNESS,
#else
					FEATURE_BRIGHTNESS = ::FEATURE_BRIGHTNESS,
#endif
					FEATURE_EXPOSURE,
					FEATURE_SHARPNESS,
					FEATURE_WHITE_BALANCE,
					FEATURE_HUE,
					FEATURE_SATURATION,
					FEATURE_GAMMA,
					FEATURE_SHUTTER,
					FEATURE_GAIN,
					FEATURE_IRIS,
					FEATURE_FOCUS,
					FEATURE_TEMPERATURE,
					FEATURE_TRIGGER,
					FEATURE_TRIGGER_DELAY,
					FEATURE_WHITE_SHADING,
					FEATURE_FRAME_RATE,
					FEATURE_ZOOM,
					FEATURE_PAN,
					FEATURE_TILT,
					FEATURE_OPTICAL_FILTER,
					FEATURE_CAPTURE_SIZE,
					FEATURE_CAPTURE_QUALITY
				};
				
				enum FeatureMode
				{
#if (LIBDC1394_VERSION_MAJOR > 10)
					FEATURE_MODE_MANUAL = ::DC1394_FEATURE_MODE_MANUAL,
#else
					FEATURE_MODE_MANUAL,
#endif
					FEATURE_MODE_AUTO,
					FEATURE_MODE_ONE_PUSH_AUTO
				};
				
				enum Framerate
				{
#if (LIBDC1394_VERSION_MAJOR > 10)
					FRAMERATE_1_875 = ::DC1394_FRAMERATE_1_875,
#else
					FRAMERATE_1_875 = ::FRAMERATE_1_875,
#endif
					FRAMERATE_3_75,
					FRAMERATE_7_5,
					FRAMERATE_15,
					FRAMERATE_30,
					FRAMERATE_60,
					FRAMERATE_120,
					FRAMERATE_240
				};
				
				enum IsoSpeed
				{
#if (LIBDC1394_VERSION_MAJOR > 10)
					ISO_SPEED_100 = ::DC1394_ISO_SPEED_100,
#else
					ISO_SPEED_100 = ::SPEED_100,
#endif
					ISO_SPEED_200,
					ISO_SPEED_400,
					ISO_SPEED_800,
					ISO_SPEED_1600,
					ISO_SPEED_3200
				};
				
				enum OperationMode
				{
#if (LIBDC1394_VERSION_MAJOR > 10)
					OPERATION_MODE_LEGACY = ::DC1394_OPERATION_MODE_LEGACY,
#else
					OPERATION_MODE_LEGACY = ::OPERATION_MODE_LEGACY,
#endif
					OPERATION_MODE_1394B
				};
				
				enum VideoMode
				{
#if (LIBDC1394_VERSION_MAJOR > 10)
					VIDEO_MODE_160x120_YUV444 = ::DC1394_VIDEO_MODE_160x120_YUV444,
#else
					VIDEO_MODE_160x120_YUV444 = ::MODE_160x120_YUV444,
#endif
					VIDEO_MODE_320x240_YUV422,
					VIDEO_MODE_640x480_YUV411,
					VIDEO_MODE_640x480_YUV422,
					VIDEO_MODE_640x480_RGB8,
					VIDEO_MODE_640x480_MONO8,
					VIDEO_MODE_640x480_MONO16,
#if (LIBDC1394_VERSION_MAJOR > 10)
					VIDEO_MODE_800x600_YUV422,
#else
					VIDEO_MODE_800x600_YUV422 = ::MODE_800x600_YUV422,
#endif
					VIDEO_MODE_800x600_RGB8,
					VIDEO_MODE_800x600_MONO8,
					VIDEO_MODE_1024x768_YUV422,
					VIDEO_MODE_1024x768_RGB8,
					VIDEO_MODE_1024x768_MONO8,
					VIDEO_MODE_800x600_MONO16,
					VIDEO_MODE_1024x768_MONO16,
#if (LIBDC1394_VERSION_MAJOR > 10)
					VIDEO_MODE_1280x960_YUV422,
#else
					VIDEO_MODE_1280x960_YUV422 = ::MODE_1280x960_YUV422,
#endif
					VIDEO_MODE_1280x960_RGB8,
					VIDEO_MODE_1280x960_MONO8,
					VIDEO_MODE_1600x1200_YUV422,
					VIDEO_MODE_1600x1200_RGB8,
					VIDEO_MODE_1600x1200_MONO8,
					VIDEO_MODE_1280x960_MONO16,
					VIDEO_MODE_1600x1200_MONO16,
#if (LIBDC1394_VERSION_MAJOR > 10)
					VIDEO_MODE_EXIF,
#else
					VIDEO_MODE_EXIF = ::MODE_EXIF,
#endif
#if (LIBDC1394_VERSION_MAJOR > 10)
					VIDEO_MODE_FORMAT7_0,
#else
					VIDEO_MODE_FORMAT7_0 = ::MODE_FORMAT7_0,
#endif
					VIDEO_MODE_FORMAT7_1,
					VIDEO_MODE_FORMAT7_2,
					VIDEO_MODE_FORMAT7_3,
					VIDEO_MODE_FORMAT7_4,
					VIDEO_MODE_FORMAT7_5,
					VIDEO_MODE_FORMAT7_6,
					VIDEO_MODE_FORMAT7_7
				};
				
				class Exception : public DeviceException
				{
				public:
#if (LIBDC1394_VERSION_MAJOR > 10)
					Exception(const ::dc1394error_t& error);
#else
					Exception(const int& error);
#endif
					
					virtual ~Exception() throw();
					
#if (LIBDC1394_VERSION_MAJOR > 10)
					::dc1394error_t getError() const;
#else
					int getError() const;
#endif
					
					virtual const char* what() const throw();
					
				protected:
					
				private:
#if (LIBDC1394_VERSION_MAJOR > 10)
					::dc1394error_t error;
#else
					int error;
#endif
				};
				
				Dc1394Camera(const ::std::string& filename = "", const unsigned int& node = 0);
				
				virtual ~Dc1394Camera();
				
				void close();
				
				unsigned int getBitsPerPixel() const;
				
				unsigned int getColorCodingDepth() const;
				
				unsigned int getHeight() const;
				
				bool getFeatureAbsoluteControl(const Feature& feature) const;
				
				void getFeatureBoundaries(const Feature& feature, unsigned int& min, unsigned int& max) const;
				
				void getFeatureBoundariesAbsolute(const Feature& feature, float& min, float& max) const;
				
				FeatureMode getFeatureMode(const Feature& feature) const;
				
				void getFeatureModes(const Feature& feature, bool& hasManual, bool& hasAuto, bool& hasOnePushAuto) const;
				
				unsigned int getFeatureValue(const Feature& feature) const;
				
				float getFeatureValueAbsolute(const Feature& feature) const;
				
				::std::string getFilename() const;
				
				void getFormat7(VideoMode& videoMode, ColorCoding& colorCoding, unsigned int& left, unsigned int& top, unsigned int& width, unsigned int& height) const;
				
				void getFormat7MaximumImageSize(const unsigned int& mode, unsigned int& width, unsigned& height) const;
				
				Framerate getFramerate() const;
				
				unsigned int getNode() const;
				
				int getNumCameras() const;
				
				OperationMode getOperationMode() const;
				
				unsigned int getPort() const;
				
				unsigned int getSize() const;
				
				IsoSpeed getSpeed() const;
				
				::std::chrono::nanoseconds getUpdateRate() const;
				
				VideoMode getVideoMode() const;
				
				unsigned int getWidth() const;
				
				void grab(unsigned char* image);
				
				bool hasFeatureAbsoluteControl(const Feature& feature) const;
				
				bool isFeatureEnabled(const Feature& feature) const;
				
				bool isFeaturePresent(const Feature& feature) const;
				
				bool isFeatureReadable(const Feature& feature) const;
				
				bool isFeatureSwitchable(const Feature& feature) const;
				
				void open();
				
				void reset();
				
				void setFeatureAbsoluteControl(const Feature& feature, const bool& doOn);
				
				void setFeatureEnabled(const Feature& feature, const bool& doOn);
				
				void setFeatureMode(const Feature& feature, const FeatureMode& mode);
				
				void setFeatureValue(const Feature& feature, const unsigned int& value);
				
				void setFeatureValueAbsolute(const Feature& feature, const float& value);
				
				void setFilename(const ::std::string& filename);
				
				void setFormat7(const VideoMode& videoMode, const ColorCoding& colorCoding, const unsigned int& left, const unsigned int& top, const unsigned int& width, const unsigned int& height);
				
				void setFramerate(const Framerate& framerate);
				
				void setNode(const unsigned int& node);
				
				void setOperationMode(const OperationMode& mode);
				
				void setPort(const unsigned int& port);
				
				void setSpeed(const IsoSpeed& speed);
				
				void setVideoMode(const VideoMode& mode);
				
				void start();
				
				void step();
				
				void stop();
				
			protected:
				
			private:
#if (LIBDC1394_VERSION_MAJOR > 10)
				unsigned int buffer;
				
				::dc1394camera_t* camera;
				
				int cameras;
				
				ColorCoding colorCoding;
				
				::dc1394_t* dc1394;
				
				::std::string filename;
				
				::dc1394video_frame_t* frame;
				
				Framerate framerate;
				
				unsigned int height;
				
				unsigned int left;
				
				unsigned int node;
				
				unsigned int speed;
				
				unsigned int top;
				
				VideoMode videoMode;
				
				unsigned int width;
#else
				unsigned int buffer;
				
				::dc1394_cameracapture camera;
				
				int cameras;
				
				unsigned int channel;
				
				ColorCoding colorCoding;
				
				unsigned int drop;
				
				::std::string filename;
				
				Framerate framerate;
				
				::raw1394handle_t handle;
				
				unsigned int height;
				
				::dc1394_camerainfo info;
				
				unsigned int left;
				
				unsigned int node;
				
				nodeid_t* nodes;
				
				unsigned int port;
				
				unsigned int speed;
				
				unsigned int top;
				
				VideoMode videoMode;
				
				unsigned int width;
#endif
		};
	}
}

#endif // RL_HAL_DC1394CAMERA_H
