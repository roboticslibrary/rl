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
#include <rl/math/Unit.h>

#include "Dc1394Camera.h"

namespace rl
{
	namespace hal
	{
		Dc1394Camera::Dc1394Camera(const ::std::string& filename, const unsigned int& node) :
			Camera(),
			CyclicDevice(::std::chrono::nanoseconds::zero()),
#if (LIBDC1394_VERSION_MAJOR > 10)
			buffer(8),
			camera(nullptr),
			cameras(0),
			colorCoding(COLOR_CODING_RAW8),
			dc1394(::dc1394_new()),
			filename(filename),
			frame(),
			framerate(static_cast<Framerate>(DC1394_FRAMERATE_MIN)),
			height(DC1394_USE_MAX_AVAIL),
			left(0),
			node(node),
			speed(ISO_SPEED_400),
			top(0),
			videoMode(VIDEO_MODE_640x480_RGB8),
			width(DC1394_USE_MAX_AVAIL)
#else
			buffer(8),
			camera(),
			cameras(0),
			channel(0),
			colorCoding(COLOR_CODING_RAW8),
			drop(1),
			filename(filename),
			framerate(FRAMERATE_MIN),
			handle(nullptr),
			height(USE_MAX_AVAIL),
			info(),
			left(0),
			node(node),
			nodes(nullptr),
			port(0),
			speed(::SPEED_400),
			top(0),
			videoMode(VIDEO_MODE_640x480_RGB8),
			width(USE_MAX_AVAIL)
#endif
		{
		}
		
		Dc1394Camera::~Dc1394Camera()
		{
#if (LIBDC1394_VERSION_MAJOR > 10)
			if (nullptr != this->dc1394)
			{
				::dc1394_free(this->dc1394);
			}
#else
			if (nullptr != this->nodes)
			{
				::dc1394_free_camera_nodes(this->nodes);
				this->nodes = nullptr;
			}
			
			if (nullptr != this->handle)
			{
				::dc1394_destroy_handle(this->handle);
				this->handle = nullptr;
			}
#endif
		}
		
		void
		Dc1394Camera::close()
		{
#if (LIBDC1394_VERSION_MAJOR > 10)
			if (nullptr != this->camera)
			{
				::dc1394_camera_free(this->camera);
			}
#else
			if (nullptr != this->nodes)
			{
				::dc1394_free_camera_nodes(this->nodes);
				this->nodes = nullptr;
			}
			
			if (nullptr != this->handle)
			{
				::dc1394_destroy_handle(this->handle);
				this->handle = nullptr;
			}
#endif
		}
		
		unsigned int
		Dc1394Camera::getBitsPerPixel() const
		{
			switch (this->videoMode)
			{
			case VIDEO_MODE_640x480_MONO8:
			case VIDEO_MODE_800x600_MONO8:
			case VIDEO_MODE_1024x768_MONO8:
			case VIDEO_MODE_1280x960_MONO8:
			case VIDEO_MODE_1600x1200_MONO8:
				return 8;
				break;
			case VIDEO_MODE_640x480_YUV411:
				return 12;
				break;
			case VIDEO_MODE_640x480_MONO16:
			case VIDEO_MODE_800x600_MONO16:
			case VIDEO_MODE_1024x768_MONO16:
			case VIDEO_MODE_1280x960_MONO16:
			case VIDEO_MODE_1600x1200_MONO16:
			case VIDEO_MODE_320x240_YUV422:
			case VIDEO_MODE_640x480_YUV422:
			case VIDEO_MODE_800x600_YUV422:
			case VIDEO_MODE_1024x768_YUV422:
			case VIDEO_MODE_1280x960_YUV422:
			case VIDEO_MODE_1600x1200_YUV422:
				return 16;
				break;
			case VIDEO_MODE_640x480_RGB8:
			case VIDEO_MODE_800x600_RGB8:
			case VIDEO_MODE_1024x768_RGB8:
			case VIDEO_MODE_1280x960_RGB8:
			case VIDEO_MODE_1600x1200_RGB8:
			case VIDEO_MODE_160x120_YUV444:
				return 24;
				break;
			case VIDEO_MODE_FORMAT7_0:
			case VIDEO_MODE_FORMAT7_1:
			case VIDEO_MODE_FORMAT7_2:
			case VIDEO_MODE_FORMAT7_3:
			case VIDEO_MODE_FORMAT7_4:
			case VIDEO_MODE_FORMAT7_5:
			case VIDEO_MODE_FORMAT7_6:
			case VIDEO_MODE_FORMAT7_7:
				switch (this->colorCoding)
				{
				case COLOR_CODING_MONO8:
				case COLOR_CODING_RAW8:
					return 8;
					break;
				case COLOR_CODING_YUV411:
					return 12;
					break;
				case COLOR_CODING_MONO16:
				case COLOR_CODING_MONO16S:
				case COLOR_CODING_RAW16:
				case COLOR_CODING_YUV422:
					return 16;
					break;
				case COLOR_CODING_RGB8:
				case COLOR_CODING_YUV444:
					return 24;
					break;
				case COLOR_CODING_RGB16:
				case COLOR_CODING_RGB16S:
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
			case VIDEO_MODE_640x480_MONO8:
			case VIDEO_MODE_800x600_MONO8:
			case VIDEO_MODE_1024x768_MONO8:
			case VIDEO_MODE_1280x960_MONO8:
			case VIDEO_MODE_1600x1200_MONO8:
			case VIDEO_MODE_640x480_RGB8:
			case VIDEO_MODE_800x600_RGB8:
			case VIDEO_MODE_1024x768_RGB8:
			case VIDEO_MODE_1280x960_RGB8:
			case VIDEO_MODE_1600x1200_RGB8:
			case VIDEO_MODE_640x480_YUV411:
			case VIDEO_MODE_320x240_YUV422:
			case VIDEO_MODE_640x480_YUV422:
			case VIDEO_MODE_800x600_YUV422:
			case VIDEO_MODE_1024x768_YUV422:
			case VIDEO_MODE_1280x960_YUV422:
			case VIDEO_MODE_1600x1200_YUV422:
			case VIDEO_MODE_160x120_YUV444:
				return 8;
				break;
			case VIDEO_MODE_640x480_MONO16:
			case VIDEO_MODE_800x600_MONO16:
			case VIDEO_MODE_1024x768_MONO16:
			case VIDEO_MODE_1280x960_MONO16:
			case VIDEO_MODE_1600x1200_MONO16:
				return 16;
				break;
			case VIDEO_MODE_FORMAT7_0:
			case VIDEO_MODE_FORMAT7_1:
			case VIDEO_MODE_FORMAT7_2:
			case VIDEO_MODE_FORMAT7_3:
			case VIDEO_MODE_FORMAT7_4:
			case VIDEO_MODE_FORMAT7_5:
			case VIDEO_MODE_FORMAT7_6:
			case VIDEO_MODE_FORMAT7_7:
				switch (this->colorCoding)
				{
				case COLOR_CODING_MONO8:
				case COLOR_CODING_RAW8:
				case COLOR_CODING_RGB8:
				case COLOR_CODING_YUV411:
				case COLOR_CODING_YUV422:
				case COLOR_CODING_YUV444:
					return 8;
					break;
				case COLOR_CODING_MONO16:
				case COLOR_CODING_MONO16S:
				case COLOR_CODING_RAW16:
				case COLOR_CODING_RGB16:
				case COLOR_CODING_RGB16S:
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
#if (LIBDC1394_VERSION_MAJOR > 10)
			unsigned int width;
			unsigned int height;
			
			::dc1394error_t error = ::dc1394_get_image_size_from_video_mode(this->camera, static_cast< ::dc1394video_mode_t>(this->videoMode), &width, &height); 
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			return height;
#else
			return this->camera.frame_height;
#endif
		}
		
		bool
		Dc1394Camera::getFeatureAbsoluteControl(const Feature& feature) const
		{
			::dc1394bool_t hasAbsoluteControl;
			
#if (LIBDC1394_VERSION_MAJOR > 10)
			::dc1394error_t error = ::dc1394_feature_has_absolute_control(this->camera, static_cast< ::dc1394feature_t>(feature), &hasAbsoluteControl);
#else
			int error = ::dc1394_has_absolute_control(this->handle, this->nodes[this->node], feature, &hasAbsoluteControl);
#endif
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			return hasAbsoluteControl;
		}
		
		void
		Dc1394Camera::getFeatureBoundaries(const Feature& feature, unsigned int& min, unsigned int& max) const
		{
#if (LIBDC1394_VERSION_MAJOR > 10)
			::dc1394error_t error = ::dc1394_feature_get_boundaries(this->camera, static_cast< ::dc1394feature_t>(feature), &min, &max); 
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
#else
			int error = ::dc1394_get_min_value(this->handle, this->nodes[this->node], feature, &min);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			error = ::dc1394_get_max_value(this->handle, this->nodes[this->node], feature, &max);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
#endif
		}
		
		void
		Dc1394Camera::getFeatureBoundariesAbsolute(const Feature& feature, float& min, float& max) const
		{
#if (LIBDC1394_VERSION_MAJOR > 10)
			::dc1394error_t error = ::dc1394_feature_get_absolute_boundaries(this->camera, static_cast< ::dc1394feature_t>(feature), &min, &max); 
#else
			int error = ::dc1394_query_absolute_feature_min_max(this->handle, this->nodes[this->node], feature, &min, &max);
#endif
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
		}
		
		Dc1394Camera::FeatureMode
		Dc1394Camera::getFeatureMode(const Feature& feature) const
		{
#if (LIBDC1394_VERSION_MAJOR > 10)
			::dc1394feature_mode_t mode;
			
			::dc1394error_t error = ::dc1394_feature_get_mode(this->camera, static_cast< ::dc1394feature_t>(feature), &mode); 
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			return static_cast<FeatureMode>(mode);
#else
			::dc1394bool_t isOnePushInOperation;
			
			int error = ::dc1394_is_one_push_in_operation(this->handle, this->nodes[this->node], feature, &isOnePushInOperation);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			if (isOnePushInOperation)
			{
				return FEATURE_MODE_ONE_PUSH_AUTO;
			}
			
			::dc1394bool_t isFeatureAuto;
			
			error = ::dc1394_is_feature_auto(this->handle, this->nodes[this->node], feature, &isFeatureAuto);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			if (isFeatureAuto)
			{
				return FEATURE_MODE_AUTO;
			}
			
			return FEATURE_MODE_MANUAL;
#endif
		}
		
		void
		Dc1394Camera::getFeatureModes(const Feature& feature, bool& hasManual, bool& hasAuto, bool& hasOnePushAuto) const
		{
#if (LIBDC1394_VERSION_MAJOR > 10)
			::dc1394feature_modes_t modes;
			
			::dc1394error_t error = ::dc1394_feature_get_modes(this->camera, static_cast< ::dc1394feature_t>(feature), &modes);
			
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
#else
			int error = ::dc1394_has_manual_mode(this->handle, this->nodes[this->node], feature, static_cast< ::dc1394bool_t*>(&hasManual));
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			error = ::dc1394_has_auto_mode(this->handle, this->nodes[this->node], feature, stataic_cast< ::dc1394bool_t*>(&hasAuto));
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			error = ::dc1394_has_one_push_auto(this->handle, this->nodes[this->node], feature, static_cast< ::dc1394bool_t*>(&hasOnePushAuto));
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
#endif
		}
		
		unsigned int
		Dc1394Camera::getFeatureValue(const Feature& feature) const
		{
			unsigned int value;
			
#if (LIBDC1394_VERSION_MAJOR > 10)
			::dc1394error_t error = ::dc1394_feature_get_value(this->camera, static_cast< ::dc1394feature_t>(feature), &value); 
#else
			int error = ::dc1394_get_feature_value(this->handle, this->nodes[this->node], feature, &value);
#endif
			
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
			
#if (LIBDC1394_VERSION_MAJOR > 10)
			::dc1394error_t error = ::dc1394_feature_get_absolute_value(this->camera, static_cast< ::dc1394feature_t>(feature), &value); 
#else
			int error = ::dc1394_query_absolute_feature_value(this->handle, this->nodes[this->node], feature, &value);
#endif
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			return value;
		}
		
		::std::string
		Dc1394Camera::getFilename() const
		{
			return this->filename;
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
#if (LIBDC1394_VERSION_MAJOR > 10)
			::dc1394error_t error = ::dc1394_format7_get_max_image_size(this->camera, static_cast< ::dc1394video_mode_t>(mode), &width, &height); 
#else
			int error = ::dc1394_query_format7_max_image_size(this->handle, this->nodes[this->node], mode, &width, &height);
#endif
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
		}
		
		Dc1394Camera::Framerate
		Dc1394Camera::getFramerate() const
		{
			Framerate framerate;
			
#if (LIBDC1394_VERSION_MAJOR > 10)
			::dc1394error_t error = ::dc1394_video_get_framerate(this->camera, reinterpret_cast< ::dc1394framerate_t*>(&framerate));
#else
			int error = ::dc1394_get_video_framerate(this->handle, this->nodes[this->node], reinterpret_cast<unsigned int*>(&framerate));
#endif
			
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
			
#if (LIBDC1394_VERSION_MAJOR > 10)
			::dc1394error_t error = ::dc1394_video_get_operation_mode(this->camera, reinterpret_cast< ::dc1394operation_mode_t*>(&operationMode)); 
#else
			int error = ::dc1394_get_operation_mode(this->handle, this->nodes[this->node], reinterpret_cast<unsigned int*>(&operationMode));
#endif
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			return operationMode;
		}
		
		unsigned int
		Dc1394Camera::getSize() const
		{
#if (LIBDC1394_VERSION_MAJOR > 10)
			unsigned int width;
			unsigned int height;
			
			::dc1394error_t error = ::dc1394_get_image_size_from_video_mode(this->camera, static_cast< ::dc1394video_mode_t>(this->videoMode), &width, &height); 
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			return width * height * this->getBitsPerPixel() / 8;
#else
			return this->getWidth() * this->getHeight() * this->getBitsPerPixel() / 8;
#endif
		}
		
		Dc1394Camera::IsoSpeed
		Dc1394Camera::getSpeed() const
		{
			IsoSpeed speed;
			
#if (LIBDC1394_VERSION_MAJOR > 10)
			::dc1394error_t error = ::dc1394_video_get_iso_speed(this->camera, reinterpret_cast< ::dc1394speed_t*>(&speed));
#else
			unsigned int channel;
			int error = ::dc1394_get_iso_channel_and_speed(this->handle, this->nodes[this->node], &channel, reinterpret_cast<unsigned int*>(&speed));
#endif
			
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
			case FRAMERATE_1_875:
				framerate = 1.875;
				break;
			case FRAMERATE_3_75:
				framerate = 3.75;
				break;
			case FRAMERATE_7_5:
				framerate = 7.5;
				break;
			case FRAMERATE_15:
				framerate = 15.0;
				break;
			case FRAMERATE_30:
				framerate = 30.0;
				break;
			case FRAMERATE_60:
				framerate = 60.0;
				break;
			case FRAMERATE_120:
				framerate = 120.0;
				break;
			case FRAMERATE_240:
				framerate = 240.0;
				break;
			default:
				return ::std::chrono::nanoseconds::zero();
				break;
			}
			
			return ::std::chrono::duration_cast< ::std::chrono::nanoseconds>(
				::std::chrono::duration<double>(1.0 / framerate * rl::math::UNIT2NANO)
			);
		}
		
		Dc1394Camera::VideoMode
		Dc1394Camera::getVideoMode() const
		{
			VideoMode videoMode;
			
#if (LIBDC1394_VERSION_MAJOR > 10)
			::dc1394error_t error = ::dc1394_video_get_mode(this->camera, reinterpret_cast< ::dc1394video_mode_t*>(&videoMode)); 
#else
			int error = ::dc1394_get_video_mode(this->handle, this->nodes[this->node], reinterpret_cast<unsigned int*>(&videoMode));
#endif
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			return videoMode;
		}
		
		unsigned int
		Dc1394Camera::getWidth() const
		{
#if (LIBDC1394_VERSION_MAJOR > 10)
			unsigned int width;
			unsigned int height;
			
			::dc1394error_t error = ::dc1394_get_image_size_from_video_mode(this->camera, static_cast< ::dc1394video_mode_t>(this->videoMode), &width, &height); 
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			return width;
#else
			return this->camera.frame_width;
#endif
		}
		
		void
		Dc1394Camera::grab(unsigned char* image)
		{
#if (LIBDC1394_VERSION_MAJOR > 10)
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
#else
			int error = ::dc1394_dma_multi_capture(&this->camera, 1);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			::std::memcpy(image, this->camera.capture_buffer, this->getSize());
			
			error = ::dc1394_dma_done_with_buffer(&this->camera);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
#endif
		}
		
		bool
		Dc1394Camera::hasFeatureAbsoluteControl(const Feature& feature) const
		{
			::dc1394bool_t hasAbsoluteControl;
			
#if (LIBDC1394_VERSION_MAJOR > 10)
			::dc1394error_t error = ::dc1394_feature_has_absolute_control(this->camera, static_cast< ::dc1394feature_t>(feature), &hasAbsoluteControl); 
#else
			int error = ::dc1394_has_absolute_control(this->handle, this->nodes[this->node], feature, &hasAbsoluteControl);
#endif
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			return hasAbsoluteControl;
		}
		
		bool
		Dc1394Camera::isFeatureEnabled(const Feature& feature) const
		{
#if (LIBDC1394_VERSION_MAJOR > 10)
			::dc1394switch_t isFeatureOn;
			
			::dc1394error_t error = ::dc1394_feature_get_power(this->camera, static_cast< ::dc1394feature_t>(feature), &isFeatureOn); 
#else
			::dc1394bool_t isFeatureOn;
			
			int error = ::dc1394_is_feature_on(this->handle, this->nodes[this->node], feature, &isFeatureOn);
#endif
			
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
			
#if (LIBDC1394_VERSION_MAJOR > 10)
			::dc1394error_t error = ::dc1394_feature_is_present(this->camera, static_cast< ::dc1394feature_t>(feature), &isFeaturePresent); 
#else
			int error = ::dc1394_is_feature_present(this->handle, this->nodes[this->node], feature, &isFeaturePresent);
#endif
			
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
			
#if (LIBDC1394_VERSION_MAJOR > 10)
			::dc1394error_t error = ::dc1394_feature_is_readable(this->camera, static_cast< ::dc1394feature_t>(feature), &canReadOut); 
#else
			int error = ::dc1394_can_read_out(this->handle, this->nodes[this->node], feature, &canReadOut);
#endif
			
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
			
#if (LIBDC1394_VERSION_MAJOR > 10)
			::dc1394error_t error = ::dc1394_feature_is_switchable(this->camera, static_cast< ::dc1394feature_t>(feature), &canTurnOnOff); 
#else
			int error = ::dc1394_can_turn_on_off(this->handle, this->nodes[this->node], feature, &canTurnOnOff);
#endif
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			return canTurnOnOff;
		}
		
		void
		Dc1394Camera::open()
		{
#if (LIBDC1394_VERSION_MAJOR > 10)
			::dc1394camera_list_t* list;
			
			::dc1394error_t error = ::dc1394_camera_enumerate(this->dc1394, &list);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			this->cameras = list->num;
			
			this->camera = ::dc1394_camera_new(this->dc1394, list->ids[this->node].guid);
			
			::dc1394_camera_free_list(list);
#else
			this->handle = ::dc1394_create_handle(0);
			
			if (nullptr == this->handle)
			{
				throw Exception("Handle creation failure.");
			}
			
			this->nodes = ::dc1394_get_camera_nodes(handle, &this->cameras, this->port);
			
			if (nullptr == this->nodes || 1 > this->cameras)
			{
				throw Exception("No cameras found.");
			}
			
			int error = ::dc1394_get_camera_info(this->handle, this->nodes[this->node], &this->info);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			error = ::dc1394_get_iso_channel_and_speed(this->handle, this->nodes[this->node], &this->channel, &this->speed); 
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			error = ::dc1394_get_video_framerate(this->handle, this->nodes[this->node], static_cast< ::dc1394framerate_t*>(&this->framerate));
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			error = ::dc1394_get_video_mode(this->handle, this->nodes[this->node], static_cast< ::dc1394video_mode_t*>(&this->videoMode));
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
#endif
		}
		
		void
		Dc1394Camera::reset()
		{
#if (LIBDC1394_VERSION_MAJOR > 10)
			::dc1394error_t error = ::dc1394_camera_reset(this->camera); 
#else
			int error = ::dc1394_init_camera(this->handle, this->nodes[this->node]);
#endif
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
		}
		
		void
		Dc1394Camera::setFeatureAbsoluteControl(const Feature& feature, const bool& doOn)
		{
#if (LIBDC1394_VERSION_MAJOR > 10)
#else
			int error = ::dc1394_absolute_setting_on_off(this->handle, this->nodes[this->node], feature, doOn);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
#endif
		}
		
		void
		Dc1394Camera::setFeatureEnabled(const Feature& feature, const bool& doOn)
		{
#if (LIBDC1394_VERSION_MAJOR > 10)
			::dc1394error_t error = ::dc1394_feature_set_power(this->camera, static_cast< ::dc1394feature_t>(feature), static_cast< ::dc1394switch_t>(doOn)); 
#else
			int error = ::dc1394_feature_on_off(this->handle, this->nodes[this->node], feature, doOn);
#endif
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
		}
		
		void
		Dc1394Camera::setFeatureMode(const Feature& feature, const FeatureMode& mode)
		{
#if (LIBDC1394_VERSION_MAJOR > 10)
			::dc1394error_t error = ::dc1394_feature_set_mode(this->camera, static_cast< ::dc1394feature_t>(feature), static_cast< ::dc1394feature_mode_t>(mode)); 
#else
			int error;
			
			switch (mode)
			{
			case FEATURE_MODE_MANUAL:
				error = ::dc1394_auto_on_off(this->handle, this->nodes[this->node], feature, false);
				break;
			case FEATURE_MODE_AUTO:
				error = ::dc1394_auto_on_off(this->handle, this->nodes[this->node], feature, true);
				break;
			case FEATURE_MODE_ONE_PUSH_AUTO:
				error = ::dc1394_start_one_push_operation(this->handle, this->nodes[this->node], feature);
				break;
			default:
				break;
			}
#endif
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
		}
		
		void
		Dc1394Camera::setFeatureValue(const Feature& feature, const unsigned int& value)
		{
#if (LIBDC1394_VERSION_MAJOR > 10)
			::dc1394error_t error = ::dc1394_feature_set_value(this->camera, static_cast< ::dc1394feature_t>(feature), value); 
#else
			int error = ::dc1394_set_feature_value(this->handle, this->nodes[this->node], feature, value);
#endif
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
		}
		
		void
		Dc1394Camera::setFeatureValueAbsolute(const Feature& feature, const float& value)
		{
#if (LIBDC1394_VERSION_MAJOR > 10)
			::dc1394error_t error = ::dc1394_feature_set_absolute_value(this->camera, static_cast< ::dc1394feature_t>(feature), value); 
#else
			int error = ::dc1394_set_absolute_feature_value(this->handle, this->nodes[this->node], feature, value);
#endif
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
		}
		
		void
		Dc1394Camera::setFilename(const ::std::string& filename)
		{
			this->filename = filename;
		}
		
		void
		Dc1394Camera::setFormat7(const VideoMode& videoMode, const ColorCoding& colorCoding, const unsigned int& left, const unsigned int& top, const unsigned int& width, const unsigned int& height)
		{
#if (LIBDC1394_VERSION_MAJOR > 10)
			::dc1394error_t error = ::dc1394_format7_set_roi(
				this->camera,
				static_cast< ::dc1394video_mode_t>(videoMode),
				static_cast< ::dc1394color_coding_t>(colorCoding),
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
#endif
			
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
#if (LIBDC1394_VERSION_MAJOR > 10)
			::dc1394error_t error = ::dc1394_video_set_framerate(this->camera, static_cast< ::dc1394framerate_t>(framerate));  
#else
			int error = ::dc1394_set_video_framerate(this->handle, this->nodes[node], framerate);
#endif
			
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
#if (LIBDC1394_VERSION_MAJOR > 10)
			::dc1394error_t error = ::dc1394_video_set_operation_mode(this->camera, static_cast< ::dc1394operation_mode_t>(mode)); 
#else
			int error = ::dc1394_set_operation_mode(this->handle, this->nodes[this->node], mode);
#endif
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
		}
		
		void
		Dc1394Camera::setSpeed(const IsoSpeed& speed)
		{
#if (LIBDC1394_VERSION_MAJOR > 10)
			::dc1394error_t error = ::dc1394_video_set_iso_speed(this->camera, static_cast< ::dc1394speed_t>(speed)); 
#else
			int error = ::dc1394_set_iso_channel_and_speed(this->handle, this->nodes[node], this->channel, speed);
#endif
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			this->speed = speed;
		}
		
		void
		Dc1394Camera::setVideoMode(const VideoMode& videoMode)
		{
#if (LIBDC1394_VERSION_MAJOR > 10)
			::dc1394error_t error = ::dc1394_video_set_mode(this->camera, static_cast< ::dc1394video_mode_t>(videoMode)); 
#else
			unsigned int format;
			
			if (static_cast<VideoMode>(MODE_FORMAT0_MIN) <= this->videoMode && static_cast<VideoMode>(MODE_FORMAT0_MAX) >= this->videoMode)
			{
				format = FORMAT_VGA_NONCOMPRESSED;
			}
			else if (static_cast<VideoMode>(MODE_FORMAT1_MIN) <= this->videoMode && static_cast<VideoMode>(MODE_FORMAT1_MAX) >= this->videoMode)
			{
				format = FORMAT_SVGA_NONCOMPRESSED_1;
			}
			else if (static_cast<VideoMode>(MODE_FORMAT2_MIN) <= this->videoMode && static_cast<VideoMode>(MODE_FORMAT2_MAX) >= this->videoMode)
			{
				format = FORMAT_SVGA_NONCOMPRESSED_2;
			}
			else if (static_cast<VideoMode>(MODE_FORMAT6_MIN) <= this->videoMode && static_cast<VideoMode>(MODE_FORMAT6_MAX) >= this->videoMode)
			{
				format = FORMAT_STILL_IMAGE;
			}
			
			int error = ::dc1394_set_video_format(this->handle, this->nodes[this->node], format);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			error = ::dc1394_set_video_mode(this->handle, this->nodes[this->node], videoMode);
#endif
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			this->videoMode = videoMode;
		}
		
		void
		Dc1394Camera::start()
		{
#if (LIBDC1394_VERSION_MAJOR > 10)
			::dc1394error_t error = ::DC1394_SUCCESS;// = ::dc1394_capture_set_device_filename(this->camera, this->filename.c_str()); 
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			error = ::dc1394_capture_setup(this->camera, this->buffer, 0);
			
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
#else
			int error;
			
			if (static_cast<VideoMode>(MODE_FORMAT7_MIN) <= this->videoMode && static_cast<VideoMode>(MODE_FORMAT7_MAX) >= this->videoMode)
			{
				error = ::dc1394_set_format7_color_coding_id(this->handle, this->nodes[this->node], this->videoMode, this->colorCoding);
				
				if (::DC1394_SUCCESS != error)
				{
					throw Exception(error);
				}
				
				error = ::dc1394_dma_setup_format7_capture(
					this->handle,
					this->nodes[this->node],
					this->channel,
					this->videoMode,
					this->speed,
					QUERY_FROM_CAMERA,
					this->left,
					this->top,
					this->width,
					this->height,
					this->buffer,
					this->drop,
					this->filename.c_str(),
					&this->camera
				);
			}
			else
			{
				unsigned int format;
				
				if (static_cast<VideoMode>(MODE_FORMAT0_MIN) <= this->videoMode && static_cast<VideoMode>(MODE_FORMAT0_MAX) >= this->videoMode)
				{
					format = FORMAT_VGA_NONCOMPRESSED;
				}
				else if (static_cast<VideoMode>(MODE_FORMAT1_MIN) <= this->videoMode && static_cast<VideoMode>(MODE_FORMAT1_MAX) >= this->videoMode)
				{
					format = FORMAT_SVGA_NONCOMPRESSED_1;
				}
				else if (static_cast<VideoMode>(MODE_FORMAT2_MIN) <= this->videoMode && static_cast<VideoMode>(MODE_FORMAT2_MAX) >= this->videoMode)
				{
					format = FORMAT_SVGA_NONCOMPRESSED_2;
				}
				else if (static_cast<VideoMode>(MODE_FORMAT6_MIN) <= this->videoMode && static_cast<VideoMode>(MODE_FORMAT6_MAX) >= this->videoMode)
				{
					format = FORMAT_STILL_IMAGE;
				}
				
				error = ::dc1394_dma_setup_capture(
					this->handle,
					this->nodes[this->node],
					this->channel,
					format,
					this->videoMode,
					this->speed,
					this->framerate,
					this->buffer,
					this->drop,
					this->filename.c_str(),
					&this->camera
				);
			}
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			error = ::dc1394_camera_on(this->handle, this->nodes[this->node]);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			error = ::dc1394_start_iso_transmission(this->handle, this->nodes[this->node]);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
#endif
		}
		
		void
		Dc1394Camera::step()
		{
		}
		
		void
		Dc1394Camera::stop()
		{
#if (LIBDC1394_VERSION_MAJOR > 10)
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
#else
			int error = ::dc1394_stop_iso_transmission(this->handle, this->nodes[this->node]);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			error = ::dc1394_camera_off(this->handle, this->nodes[this->node]);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			error = ::dc1394_dma_unlisten(this->handle, &this->camera);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
			
			error = ::dc1394_dma_release_camera(this->handle, &this->camera);
			
			if (::DC1394_SUCCESS != error)
			{
				throw Exception(error);
			}
#endif
		}
		
#if (LIBDC1394_VERSION_MAJOR > 10)
		Dc1394Camera::Exception::Exception(const ::dc1394error_t& error) :
#else
		Dc1394Camera::Exception::Exception(const int& error) :
#endif
			DeviceException(""),
			error(error)
		{
		}
		
		Dc1394Camera::Exception::~Exception() throw()
		{
		}
		
#if (LIBDC1394_VERSION_MAJOR > 10)
		::dc1394error_t
#else
		int
#endif
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
#if (LIBDC1394_VERSION_MAJOR > 10)
			default:
				return ::dc1394_error_get_string(this->error);
				break;
			} 
#else
			case ::DC1394_NO_FRAME:
				return "No frame.";
				break;
			case ::DC1394_NO_CAMERA:
				return "No camera.";
				break;
			default:
				return "Unknown error.";
				break;
			}
#endif
		}
	}
}
