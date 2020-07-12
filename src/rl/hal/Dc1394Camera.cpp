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

#include <cstring>
#include <rl/math/Constants.h>

#include "Dc1394Camera.h"

namespace rl
{
	namespace hal
	{
		constexpr Dc1394Camera::ColorCoding Dc1394Camera::COLOR_CODING_MONO8;
		constexpr Dc1394Camera::ColorCoding Dc1394Camera::COLOR_CODING_YUV411;
		constexpr Dc1394Camera::ColorCoding Dc1394Camera::COLOR_CODING_YUV422;
		constexpr Dc1394Camera::ColorCoding Dc1394Camera::COLOR_CODING_YUV444;
		constexpr Dc1394Camera::ColorCoding Dc1394Camera::COLOR_CODING_RGB8;
		constexpr Dc1394Camera::ColorCoding Dc1394Camera::COLOR_CODING_MONO16;
		constexpr Dc1394Camera::ColorCoding Dc1394Camera::COLOR_CODING_RGB16;
		constexpr Dc1394Camera::ColorCoding Dc1394Camera::COLOR_CODING_MONO16S;
		constexpr Dc1394Camera::ColorCoding Dc1394Camera::COLOR_CODING_RGB16S;
		constexpr Dc1394Camera::ColorCoding Dc1394Camera::COLOR_CODING_RAW8;
		constexpr Dc1394Camera::ColorCoding Dc1394Camera::COLOR_CODING_RAW16;
		
		constexpr Dc1394Camera::Feature Dc1394Camera::FEATURE_BRIGHTNESS;
		constexpr Dc1394Camera::Feature Dc1394Camera::FEATURE_EXPOSURE;
		constexpr Dc1394Camera::Feature Dc1394Camera::FEATURE_SHARPNESS;
		constexpr Dc1394Camera::Feature Dc1394Camera::FEATURE_WHITE_BALANCE;
		constexpr Dc1394Camera::Feature Dc1394Camera::FEATURE_HUE;
		constexpr Dc1394Camera::Feature Dc1394Camera::FEATURE_SATURATION;
		constexpr Dc1394Camera::Feature Dc1394Camera::FEATURE_GAMMA;
		constexpr Dc1394Camera::Feature Dc1394Camera::FEATURE_SHUTTER;
		constexpr Dc1394Camera::Feature Dc1394Camera::FEATURE_GAIN;
		constexpr Dc1394Camera::Feature Dc1394Camera::FEATURE_IRIS;
		constexpr Dc1394Camera::Feature Dc1394Camera::FEATURE_FOCUS;
		constexpr Dc1394Camera::Feature Dc1394Camera::FEATURE_TEMPERATURE;
		constexpr Dc1394Camera::Feature Dc1394Camera::FEATURE_TRIGGER;
		constexpr Dc1394Camera::Feature Dc1394Camera::FEATURE_TRIGGER_DELAY;
		constexpr Dc1394Camera::Feature Dc1394Camera::FEATURE_WHITE_SHADING;
		constexpr Dc1394Camera::Feature Dc1394Camera::FEATURE_FRAME_RATE;
		constexpr Dc1394Camera::Feature Dc1394Camera::FEATURE_ZOOM;
		constexpr Dc1394Camera::Feature Dc1394Camera::FEATURE_PAN;
		constexpr Dc1394Camera::Feature Dc1394Camera::FEATURE_TILT;
		constexpr Dc1394Camera::Feature Dc1394Camera::FEATURE_OPTICAL_FILTER;
		constexpr Dc1394Camera::Feature Dc1394Camera::FEATURE_CAPTURE_SIZE;
		constexpr Dc1394Camera::Feature Dc1394Camera::FEATURE_CAPTURE_QUALITY;
		
		constexpr Dc1394Camera::FeatureMode Dc1394Camera::FEATURE_MODE_MANUAL;
		constexpr Dc1394Camera::FeatureMode Dc1394Camera::FEATURE_MODE_AUTO;
		constexpr Dc1394Camera::FeatureMode Dc1394Camera::FEATURE_MODE_ONE_PUSH_AUTO;
		
		constexpr Dc1394Camera::Framerate Dc1394Camera::FRAMERATE_1_875;
		constexpr Dc1394Camera::Framerate Dc1394Camera::FRAMERATE_3_75;
		constexpr Dc1394Camera::Framerate Dc1394Camera::FRAMERATE_7_5;
		constexpr Dc1394Camera::Framerate Dc1394Camera::FRAMERATE_15;
		constexpr Dc1394Camera::Framerate Dc1394Camera::FRAMERATE_30;
		constexpr Dc1394Camera::Framerate Dc1394Camera::FRAMERATE_60;
		constexpr Dc1394Camera::Framerate Dc1394Camera::FRAMERATE_120;
		constexpr Dc1394Camera::Framerate Dc1394Camera::FRAMERATE_240;
		
		constexpr Dc1394Camera::IsoSpeed Dc1394Camera::ISO_SPEED_100;
		constexpr Dc1394Camera::IsoSpeed Dc1394Camera::ISO_SPEED_200;
		constexpr Dc1394Camera::IsoSpeed Dc1394Camera::ISO_SPEED_400;
		constexpr Dc1394Camera::IsoSpeed Dc1394Camera::ISO_SPEED_800;
		constexpr Dc1394Camera::IsoSpeed Dc1394Camera::ISO_SPEED_1600;
		constexpr Dc1394Camera::IsoSpeed Dc1394Camera::ISO_SPEED_3200;
		
		constexpr Dc1394Camera::OperationMode Dc1394Camera::OPERATION_MODE_LEGACY;
		constexpr Dc1394Camera::OperationMode Dc1394Camera::OPERATION_MODE_1394B;
		
		constexpr Dc1394Camera::VideoMode Dc1394Camera::VIDEO_MODE_160x120_YUV444;
		constexpr Dc1394Camera::VideoMode Dc1394Camera::VIDEO_MODE_320x240_YUV422;
		constexpr Dc1394Camera::VideoMode Dc1394Camera::VIDEO_MODE_640x480_YUV411;
		constexpr Dc1394Camera::VideoMode Dc1394Camera::VIDEO_MODE_640x480_YUV422;
		constexpr Dc1394Camera::VideoMode Dc1394Camera::VIDEO_MODE_640x480_RGB8;
		constexpr Dc1394Camera::VideoMode Dc1394Camera::VIDEO_MODE_640x480_MONO8;
		constexpr Dc1394Camera::VideoMode Dc1394Camera::VIDEO_MODE_640x480_MONO16;
		constexpr Dc1394Camera::VideoMode Dc1394Camera::VIDEO_MODE_800x600_YUV422;
		constexpr Dc1394Camera::VideoMode Dc1394Camera::VIDEO_MODE_800x600_RGB8;
		constexpr Dc1394Camera::VideoMode Dc1394Camera::VIDEO_MODE_800x600_MONO8;
		constexpr Dc1394Camera::VideoMode Dc1394Camera::VIDEO_MODE_1024x768_YUV422;
		constexpr Dc1394Camera::VideoMode Dc1394Camera::VIDEO_MODE_1024x768_RGB8;
		constexpr Dc1394Camera::VideoMode Dc1394Camera::VIDEO_MODE_1024x768_MONO8;
		constexpr Dc1394Camera::VideoMode Dc1394Camera::VIDEO_MODE_800x600_MONO16;
		constexpr Dc1394Camera::VideoMode Dc1394Camera::VIDEO_MODE_1024x768_MONO16;
		constexpr Dc1394Camera::VideoMode Dc1394Camera::VIDEO_MODE_1280x960_YUV422;
		constexpr Dc1394Camera::VideoMode Dc1394Camera::VIDEO_MODE_1280x960_RGB8;
		constexpr Dc1394Camera::VideoMode Dc1394Camera::VIDEO_MODE_1280x960_MONO8;
		constexpr Dc1394Camera::VideoMode Dc1394Camera::VIDEO_MODE_1600x1200_YUV422;
		constexpr Dc1394Camera::VideoMode Dc1394Camera::VIDEO_MODE_1600x1200_RGB8;
		constexpr Dc1394Camera::VideoMode Dc1394Camera::VIDEO_MODE_1600x1200_MONO8;
		constexpr Dc1394Camera::VideoMode Dc1394Camera::VIDEO_MODE_1280x960_MONO16;
		constexpr Dc1394Camera::VideoMode Dc1394Camera::VIDEO_MODE_1600x1200_MONO16;
		constexpr Dc1394Camera::VideoMode Dc1394Camera::VIDEO_MODE_EXIF;
		constexpr Dc1394Camera::VideoMode Dc1394Camera::VIDEO_MODE_FORMAT7_0;
		constexpr Dc1394Camera::VideoMode Dc1394Camera::VIDEO_MODE_FORMAT7_1;
		constexpr Dc1394Camera::VideoMode Dc1394Camera::VIDEO_MODE_FORMAT7_2;
		constexpr Dc1394Camera::VideoMode Dc1394Camera::VIDEO_MODE_FORMAT7_3;
		constexpr Dc1394Camera::VideoMode Dc1394Camera::VIDEO_MODE_FORMAT7_4;
		constexpr Dc1394Camera::VideoMode Dc1394Camera::VIDEO_MODE_FORMAT7_5;
		constexpr Dc1394Camera::VideoMode Dc1394Camera::VIDEO_MODE_FORMAT7_6;
		constexpr Dc1394Camera::VideoMode Dc1394Camera::VIDEO_MODE_FORMAT7_7;
		
		Dc1394Camera::Dc1394Camera(const unsigned int& node) :
			Camera(),
			CyclicDevice(::std::chrono::nanoseconds::zero()),
			buffer(8),
			camera(nullptr),
			cameras(0),
			colorCoding(ColorCoding::raw8),
			dc1394(::dc1394_new()),
			frame(),
			framerate(static_cast<Framerate>(DC1394_FRAMERATE_MIN)),
			height(DC1394_USE_MAX_AVAIL),
			left(0),
			node(node),
			speed(IsoSpeed::i400),
			top(0),
			videoMode(VideoMode::v640x480_rgb8),
			width(DC1394_USE_MAX_AVAIL)
		{
		}
		
		Dc1394Camera::~Dc1394Camera()
		{
			if (nullptr != this->dc1394)
			{
				::dc1394_free(this->dc1394);
			}
		}
		
		void
		Dc1394Camera::close()
		{
			if (nullptr != this->camera)
			{
				::dc1394_camera_free(this->camera);
			}
		}
		
		unsigned int
		Dc1394Camera::getBitsPerPixel() const
		{
			switch (this->videoMode)
			{
			case VideoMode::v640x480_mono8:
			case VideoMode::v800x600_mono8:
			case VideoMode::v1024x768_mono8:
			case VideoMode::v1280x960_mono8:
			case VideoMode::v1600x1200_mono8:
				return 8;
				break;
			case VideoMode::v640x480_yuv411:
				return 12;
				break;
			case VideoMode::v640x480_mono16:
			case VideoMode::v800x600_mono16:
			case VideoMode::v1024x768_mono16:
			case VideoMode::v1280x960_mono16:
			case VideoMode::v1600x1200_mono16:
			case VideoMode::v320x240_yuv422:
			case VideoMode::v640x480_yuv422:
			case VideoMode::v800x600_yuv422:
			case VideoMode::v1024x768_yuv422:
			case VideoMode::v1280x960_yuv422:
			case VideoMode::v1600x1200_yuv422:
				return 16;
				break;
			case VideoMode::v640x480_rgb8:
			case VideoMode::v800x600_rgb8:
			case VideoMode::v1024x768_rgb8:
			case VideoMode::v1280x960_rgb8:
			case VideoMode::v1600x1200_rgb8:
			case VideoMode::v160x120_yuv444:
				return 24;
				break;
			case VideoMode::format7_0:
			case VideoMode::format7_1:
			case VideoMode::format7_2:
			case VideoMode::format7_3:
			case VideoMode::format7_4:
			case VideoMode::format7_5:
			case VideoMode::format7_6:
			case VideoMode::format7_7:
				switch (this->colorCoding)
				{
				case ColorCoding::mono8:
				case ColorCoding::raw8:
					return 8;
					break;
				case ColorCoding::yuv411:
					return 12;
					break;
				case ColorCoding::mono16:
				case ColorCoding::mono16s:
				case ColorCoding::raw16:
				case ColorCoding::yuv422:
					return 16;
					break;
				case ColorCoding::rgb8:
				case ColorCoding::yuv444:
					return 24;
					break;
				case ColorCoding::rgb16:
				case ColorCoding::rgb16s:
					return 48;
					break;
				default:
					break;
				}
				break;
			default:
				break;
			}
			
			return 0;
		}
		
		unsigned int
		Dc1394Camera::getColorCodingDepth() const
		{
			switch (this->videoMode)
			{
			case VideoMode::v640x480_mono8:
			case VideoMode::v800x600_mono8:
			case VideoMode::v1024x768_mono8:
			case VideoMode::v1280x960_mono8:
			case VideoMode::v1600x1200_mono8:
			case VideoMode::v640x480_rgb8:
			case VideoMode::v800x600_rgb8:
			case VideoMode::v1024x768_rgb8:
			case VideoMode::v1280x960_rgb8:
			case VideoMode::v1600x1200_rgb8:
			case VideoMode::v640x480_yuv411:
			case VideoMode::v320x240_yuv422:
			case VideoMode::v640x480_yuv422:
			case VideoMode::v800x600_yuv422:
			case VideoMode::v1024x768_yuv422:
			case VideoMode::v1280x960_yuv422:
			case VideoMode::v1600x1200_yuv422:
			case VideoMode::v160x120_yuv444:
				return 8;
				break;
			case VideoMode::v640x480_mono16:
			case VideoMode::v800x600_mono16:
			case VideoMode::v1024x768_mono16:
			case VideoMode::v1280x960_mono16:
			case VideoMode::v1600x1200_mono16:
				return 16;
				break;
			case VideoMode::format7_0:
			case VideoMode::format7_1:
			case VideoMode::format7_2:
			case VideoMode::format7_3:
			case VideoMode::format7_4:
			case VideoMode::format7_5:
			case VideoMode::format7_6:
			case VideoMode::format7_7:
				switch (this->colorCoding)
				{
				case ColorCoding::mono8:
				case ColorCoding::raw8:
				case ColorCoding::rgb8:
				case ColorCoding::yuv411:
				case ColorCoding::yuv422:
				case ColorCoding::yuv444:
					return 8;
					break;
				case ColorCoding::mono16:
				case ColorCoding::mono16s:
				case ColorCoding::raw16:
				case ColorCoding::rgb16:
				case ColorCoding::rgb16s:
					return 16;
					break;
				default:
					break;
				}
				break;
			default:
				break;
			}
			
			return 0;
		}
		
		unsigned int
		Dc1394Camera::getHeight() const
		{
			unsigned int width;
			unsigned int height;
			
			::dc1394error_t error = ::dc1394_get_image_size_from_video_mode(this->camera, static_cast<::dc1394video_mode_t>(this->videoMode), &width, &height);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			return height;
		}
		
		bool
		Dc1394Camera::getFeatureAbsoluteControl(const Feature& feature) const
		{
			::dc1394bool_t hasAbsoluteControl;
			
			::dc1394error_t error = ::dc1394_feature_has_absolute_control(this->camera, static_cast<::dc1394feature_t>(feature), &hasAbsoluteControl);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			return hasAbsoluteControl;
		}
		
		void
		Dc1394Camera::getFeatureBoundaries(const Feature& feature, unsigned int& min, unsigned int& max) const
		{
			::dc1394error_t error = ::dc1394_feature_get_boundaries(this->camera, static_cast<::dc1394feature_t>(feature), &min, &max);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
		}
		
		void
		Dc1394Camera::getFeatureBoundariesAbsolute(const Feature& feature, float& min, float& max) const
		{
			::dc1394error_t error = ::dc1394_feature_get_absolute_boundaries(this->camera, static_cast<::dc1394feature_t>(feature), &min, &max);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
		}
		
		Dc1394Camera::FeatureMode
		Dc1394Camera::getFeatureMode(const Feature& feature) const
		{
			::dc1394feature_mode_t mode;
			
			::dc1394error_t error = ::dc1394_feature_get_mode(this->camera, static_cast<::dc1394feature_t>(feature), &mode);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			return static_cast<FeatureMode>(mode);
		}
		
		void
		Dc1394Camera::getFeatureModes(const Feature& feature, bool& hasManual, bool& hasAuto, bool& hasOnePushAuto) const
		{
			::dc1394feature_modes_t modes;
			
			::dc1394error_t error = ::dc1394_feature_get_modes(this->camera, static_cast<::dc1394feature_t>(feature), &modes);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			hasManual = false;
			hasAuto = false;
			hasOnePushAuto = false;
			
			for (::std::size_t i = 0; i < modes.num; ++i)
			{
				if (::DC1394_FEATURE_MODE_MANUAL == modes.modes[i])
				{
					hasManual = true;
				}
				else if (::DC1394_FEATURE_MODE_AUTO == modes.modes[i])
				{
					hasAuto = true;
				}
				else if (::DC1394_FEATURE_MODE_ONE_PUSH_AUTO == modes.modes[i])
				{
					hasOnePushAuto = true;
				}
			}
		}
		
		unsigned int
		Dc1394Camera::getFeatureValue(const Feature& feature) const
		{
			unsigned int value;
			
			::dc1394error_t error = ::dc1394_feature_get_value(this->camera, static_cast<::dc1394feature_t>(feature), &value);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			return value;
		}
		
		float
		Dc1394Camera::getFeatureValueAbsolute(const Feature& feature) const
		{
			float value;
			
			::dc1394error_t error = ::dc1394_feature_get_absolute_value(this->camera, static_cast<::dc1394feature_t>(feature), &value);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			return value;
		}
		
		void
		Dc1394Camera::getFormat7(VideoMode& videoMode, ColorCoding& colorCoding, unsigned int& left, unsigned int& top, unsigned int& width, unsigned int& height) const
		{
			colorCoding = this->colorCoding;
			height = this->height;
			left = this->left;
			top = this->top;
			videoMode = this->videoMode;
			width = this->width;
		}
		
		void
		Dc1394Camera::getFormat7MaximumImageSize(const unsigned int& mode, unsigned int& width, unsigned& height) const
		{
			::dc1394error_t error = ::dc1394_format7_get_max_image_size(this->camera, static_cast<::dc1394video_mode_t>(mode), &width, &height);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
		}
		
		Dc1394Camera::Framerate
		Dc1394Camera::getFramerate() const
		{
			Framerate framerate;
			
			::dc1394error_t error = ::dc1394_video_get_framerate(this->camera, reinterpret_cast<::dc1394framerate_t*>(&framerate));
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			return framerate;
		}
		
		unsigned int
		Dc1394Camera::getNode() const
		{
			return this->node;
		}
		
		int
		Dc1394Camera::getNumCameras() const
		{
			return this->cameras;
		}
		
		Dc1394Camera::OperationMode
		Dc1394Camera::getOperationMode() const
		{
			OperationMode operationMode;
			
			::dc1394error_t error = ::dc1394_video_get_operation_mode(this->camera, reinterpret_cast<::dc1394operation_mode_t*>(&operationMode));
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			return operationMode;
		}
		
		unsigned int
		Dc1394Camera::getSize() const
		{
			unsigned int width;
			unsigned int height;
			
			::dc1394error_t error = ::dc1394_get_image_size_from_video_mode(this->camera, static_cast<::dc1394video_mode_t>(this->videoMode), &width, &height);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			return width * height * this->getBitsPerPixel() / 8;
		}
		
		Dc1394Camera::IsoSpeed
		Dc1394Camera::getSpeed() const
		{
			IsoSpeed speed;
			
			::dc1394error_t error = ::dc1394_video_get_iso_speed(this->camera, reinterpret_cast<::dc1394speed_t*>(&speed));
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			return speed;
		}
		
		::std::chrono::nanoseconds
		Dc1394Camera::getUpdateRate() const
		{
			double framerate;
			
			switch (this->framerate)
			{
			case Framerate::f1_875:
				framerate = 1.875;
				break;
			case Framerate::f3_75:
				framerate = 3.75;
				break;
			case Framerate::f7_5:
				framerate = 7.5;
				break;
			case Framerate::f15:
				framerate = 15.0;
				break;
			case Framerate::f30:
				framerate = 30.0;
				break;
			case Framerate::f60:
				framerate = 60.0;
				break;
			case Framerate::f120:
				framerate = 120.0;
				break;
			case Framerate::f240:
				framerate = 240.0;
				break;
			default:
				return ::std::chrono::nanoseconds::zero();
				break;
			}
			
			return ::std::chrono::duration_cast<::std::chrono::nanoseconds>(
				::std::chrono::duration<double>(1.0 / framerate * ::rl::math::constants::unit2nano)
			);
		}
		
		Dc1394Camera::VideoMode
		Dc1394Camera::getVideoMode() const
		{
			VideoMode videoMode;
			
			::dc1394error_t error = ::dc1394_video_get_mode(this->camera, reinterpret_cast<::dc1394video_mode_t*>(&videoMode));
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			return videoMode;
		}
		
		unsigned int
		Dc1394Camera::getWidth() const
		{
			unsigned int width;
			unsigned int height;
			
			::dc1394error_t error = ::dc1394_get_image_size_from_video_mode(this->camera, static_cast<::dc1394video_mode_t>(this->videoMode), &width, &height);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			return width;
		}
		
		void
		Dc1394Camera::grab(unsigned char* image)
		{
			::dc1394error_t error = ::dc1394_capture_dequeue(this->camera, ::DC1394_CAPTURE_POLICY_WAIT, &this->frame);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			::std::memcpy(image, this->frame->image, this->getSize());
			
			error = ::dc1394_capture_enqueue(this->camera, this->frame);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
		}
		
		bool
		Dc1394Camera::hasFeatureAbsoluteControl(const Feature& feature) const
		{
			::dc1394bool_t hasAbsoluteControl;
			
			::dc1394error_t error = ::dc1394_feature_has_absolute_control(this->camera, static_cast<::dc1394feature_t>(feature), &hasAbsoluteControl);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			return hasAbsoluteControl;
		}
		
		bool
		Dc1394Camera::isFeatureEnabled(const Feature& feature) const
		{
			::dc1394switch_t isFeatureOn;
			
			::dc1394error_t error = ::dc1394_feature_get_power(this->camera, static_cast<::dc1394feature_t>(feature), &isFeatureOn);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			return isFeatureOn;
		}
		
		bool
		Dc1394Camera::isFeaturePresent(const Feature& feature) const
		{
			::dc1394bool_t isFeaturePresent;
			
			::dc1394error_t error = ::dc1394_feature_is_present(this->camera, static_cast<::dc1394feature_t>(feature), &isFeaturePresent);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			return isFeaturePresent;
		}
		
		bool
		Dc1394Camera::isFeatureReadable(const Feature& feature) const
		{
			::dc1394bool_t canReadOut;
			
			::dc1394error_t error = ::dc1394_feature_is_readable(this->camera, static_cast<::dc1394feature_t>(feature), &canReadOut);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			return canReadOut;
		}
		
		bool
		Dc1394Camera::isFeatureSwitchable(const Feature& feature) const
		{
			::dc1394bool_t canTurnOnOff;
			
			::dc1394error_t error = ::dc1394_feature_is_switchable(this->camera, static_cast<::dc1394feature_t>(feature), &canTurnOnOff);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			return canTurnOnOff;
		}
		
		void
		Dc1394Camera::open()
		{
			::dc1394camera_list_t* list;
			
			::dc1394error_t error = ::dc1394_camera_enumerate(this->dc1394, &list);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			this->cameras = list->num;
			
			this->camera = ::dc1394_camera_new(this->dc1394, list->ids[this->node].guid);
			
			::dc1394_camera_free_list(list);
		}
		
		void
		Dc1394Camera::reset()
		{
			::dc1394error_t error = ::dc1394_camera_reset(this->camera);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
		}
		
		void
		Dc1394Camera::setFeatureAbsoluteControl(const Feature& feature, const bool& doOn)
		{
		}
		
		void
		Dc1394Camera::setFeatureEnabled(const Feature& feature, const bool& doOn)
		{
			::dc1394error_t error = ::dc1394_feature_set_power(this->camera, static_cast<::dc1394feature_t>(feature), static_cast<::dc1394switch_t>(doOn));
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
		}
		
		void
		Dc1394Camera::setFeatureMode(const Feature& feature, const FeatureMode& mode)
		{
			::dc1394error_t error = ::dc1394_feature_set_mode(this->camera, static_cast<::dc1394feature_t>(feature), static_cast<::dc1394feature_mode_t>(mode));
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
		}
		
		void
		Dc1394Camera::setFeatureValue(const Feature& feature, const unsigned int& value)
		{
			::dc1394error_t error = ::dc1394_feature_set_value(this->camera, static_cast<::dc1394feature_t>(feature), value);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
		}
		
		void
		Dc1394Camera::setFeatureValueAbsolute(const Feature& feature, const float& value)
		{
			::dc1394error_t error = ::dc1394_feature_set_absolute_value(this->camera, static_cast<::dc1394feature_t>(feature), value);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
		}
		
		void
		Dc1394Camera::setFormat7(const VideoMode& videoMode, const ColorCoding& colorCoding, const unsigned int& left, const unsigned int& top, const unsigned int& width, const unsigned int& height)
		{
			::dc1394error_t error = ::dc1394_format7_set_roi(
				this->camera,
				static_cast<::dc1394video_mode_t>(videoMode),
				static_cast<::dc1394color_coding_t>(colorCoding),
				DC1394_QUERY_FROM_CAMERA,
				left,
				top,
				width,
				height
			);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			this->colorCoding = colorCoding;
			this->height = height;
			this->left = left;
			this->top = top;
			this->videoMode = videoMode;
			this->width = width;
		}
		
		void
		Dc1394Camera::setFramerate(const Framerate& framerate)
		{
			::dc1394error_t error = ::dc1394_video_set_framerate(this->camera, static_cast<::dc1394framerate_t>(framerate));
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			this->framerate = framerate;
		}
		
		void
		Dc1394Camera::setNode(const unsigned int& node)
		{
			this->node = node;
		}
		
		void
		Dc1394Camera::setOperationMode(const OperationMode& mode)
		{
			::dc1394error_t error = ::dc1394_video_set_operation_mode(this->camera, static_cast<::dc1394operation_mode_t>(mode));
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
		}
		
		void
		Dc1394Camera::setSpeed(const IsoSpeed& speed)
		{
			::dc1394error_t error = ::dc1394_video_set_iso_speed(this->camera, static_cast<::dc1394speed_t>(speed));
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			this->speed = speed;
		}
		
		void
		Dc1394Camera::setVideoMode(const VideoMode& videoMode)
		{
			::dc1394error_t error = ::dc1394_video_set_mode(this->camera, static_cast<::dc1394video_mode_t>(videoMode));
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			this->videoMode = videoMode;
		}
		
		void
		Dc1394Camera::start()
		{
			::dc1394error_t error = ::dc1394_capture_setup(this->camera, this->buffer, 0);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			error = ::dc1394_camera_set_power(this->camera, ::DC1394_ON);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			error = ::dc1394_video_set_transmission(this->camera, ::DC1394_ON);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
		}
		
		void
		Dc1394Camera::step()
		{
		}
		
		void
		Dc1394Camera::stop()
		{
			::dc1394error_t error = ::dc1394_video_set_transmission(this->camera, ::DC1394_OFF);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			error = ::dc1394_camera_set_power(this->camera, ::DC1394_OFF);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			error = ::dc1394_capture_stop(this->camera);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
		}
		
		Dc1394Camera::Exception::Exception(const ::dc1394error_t& error) :
			DeviceException(""),
			error(error)
		{
		}
		
		Dc1394Camera::Exception::~Exception() throw()
		{
		}
		
		::dc1394error_t
		Dc1394Camera::Exception::getError() const
		{
			return this->error;
		}
		
		const char*
		Dc1394Camera::Exception::what() const throw()
		{
			switch (this->error)
			{
			case ::DC1394_FAILURE:
				return "Failure.";
				break;
			default:
				return ::dc1394_error_get_string(this->error);
				break;
			}
		}
	}
}
