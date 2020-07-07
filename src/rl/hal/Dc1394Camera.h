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

#include <string>
#include <dc1394/dc1394.h>

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
		class RL_HAL_EXPORT Dc1394Camera : public Camera, public CyclicDevice
		{
			public:
				enum class ColorCoding
				{
					  mono8 = ::DC1394_COLOR_CODING_MONO8,
					  yuv411,
					  yuv422,
					  yuv444,
					  rgb8,
					  mono16,
					  rgb16,
					  mono16s,
					  rgb16s,
					  raw8,
					  raw16
				};
				
				enum class Feature
				{
					brightness = ::DC1394_FEATURE_BRIGHTNESS,
					exposure,
					sharpness,
					whiteBalance,
					hue,
					saturation,
					gamma,
					shutter,
					gain,
					iris,
					focus,
					temperature,
					trigger,
					triggerDelay,
					whiteShading,
					frameRate,
					zoom,
					pan,
					tilt,
					opticalFilter,
					captureSize,
					captureQuality
				};
				
				enum class FeatureMode
				{
					manual = ::DC1394_FEATURE_MODE_MANUAL,
					automatic,
					onePushAuto
				};
				
				enum class Framerate
				{
					f1_875 = ::DC1394_FRAMERATE_1_875,
					f3_75,
					f7_5,
					f15,
					f30,
					f60,
					f120,
					f240
				};
				
				enum class IsoSpeed
				{
					i100 = ::DC1394_ISO_SPEED_100,
					i200,
					i400,
					i800,
					i1600,
					i3200
				};
				
				enum class OperationMode
				{
					legacy = ::DC1394_OPERATION_MODE_LEGACY,
					o1394b
				};
				
				enum class VideoMode
				{
					v160x120_yuv444 = ::DC1394_VIDEO_MODE_160x120_YUV444,
					v320x240_yuv422,
					v640x480_yuv411,
					v640x480_yuv422,
					v640x480_rgb8,
					v640x480_mono8,
					v640x480_mono16,
					v800x600_yuv422,
					v800x600_rgb8,
					v800x600_mono8,
					v1024x768_yuv422,
					v1024x768_rgb8,
					v1024x768_mono8,
					v800x600_mono16,
					v1024x768_mono16,
					v1280x960_yuv422,
					v1280x960_rgb8,
					v1280x960_mono8,
					v1600x1200_yuv422,
					v1600x1200_rgb8,
					v1600x1200_mono8,
					v1280x960_mono16,
					v1600x1200_mono16,
					exif,
					format7_0,
					format7_1,
					format7_2,
					format7_3,
					format7_4,
					format7_5,
					format7_6,
					format7_7
				};
				
				class Exception : public DeviceException
				{
				public:
					Exception(const ::dc1394error_t& error);
					
					virtual ~Exception() throw();
					
					::dc1394error_t getError() const;
					
					virtual const char* what() const throw();
					
				protected:
					
				private:
					::dc1394error_t error;
				};
				
				Dc1394Camera(const unsigned int& node = 0);
				
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
				unsigned int buffer;
				
				::dc1394camera_t* camera;
				
				int cameras;
				
				ColorCoding colorCoding;
				
				::dc1394_t* dc1394;
				
				::dc1394video_frame_t* frame;
				
				Framerate framerate;
				
				unsigned int height;
				
				unsigned int left;
				
				unsigned int node;
				
				IsoSpeed speed;
				
				unsigned int top;
				
				VideoMode videoMode;
				
				unsigned int width;
		};
	}
}

#endif // RL_HAL_DC1394CAMERA_H
