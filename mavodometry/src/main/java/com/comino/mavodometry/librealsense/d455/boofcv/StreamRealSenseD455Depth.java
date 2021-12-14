/****************************************************************************
 *
 *   Copyright (c) 2021 Eike Mansfeld ecm@gmx.de. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


package com.comino.mavodometry.librealsense.d455.boofcv;

import java.util.ArrayList;
import java.util.List;

import com.comino.mavodometry.concurrency.OdometryPool;
import com.comino.mavodometry.librealsense.lib.Realsense2Library;
import com.comino.mavodometry.librealsense.lib.Realsense2Library.rs2_camera_info;
import com.comino.mavodometry.librealsense.lib.Realsense2Library.rs2_format;
import com.comino.mavodometry.librealsense.lib.Realsense2Library.rs2_intrinsics;
import com.comino.mavodometry.librealsense.lib.Realsense2Library.rs2_option;
import com.comino.mavodometry.librealsense.lib.Realsense2Library.rs2_rs400_visual_preset;
import com.comino.mavodometry.librealsense.lib.Realsense2Library.rs2_stream;
import com.comino.mavodometry.librealsense.lib.RealsenseDevice_lib;
import com.comino.mavodometry.librealsense.utils.LibRealSenseIntrinsics;
import com.comino.mavodometry.librealsense.utils.RealSenseInfo;
import com.sun.jna.Pointer;
import com.sun.jna.ptr.PointerByReference;

import boofcv.struct.calib.CameraPinholeBrown;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;

public class StreamRealSenseD455Depth extends RealsenseDevice_lib {
	
	private static final int FRAMERATE = 15;


	private static StreamRealSenseD455Depth instance;
	
	private final List<IDepthCallback> listeners;

	// image with depth information
	private final GrayU16 depth        = new GrayU16(1,1);
	private final Planar<GrayU8> rgb	 = new Planar<GrayU8>(GrayU8.class,1,1,3);

	private volatile Realsense2Library.rs2_device dev;
	private volatile Realsense2Library.rs2_pipeline pipeline;
	private volatile Realsense2Library.rs2_config config;

	private PointerByReference depth_sensor  = null;
	private PointerByReference rgb_sensor    = null;
	private PointerByReference motion_sensor = null;
	
	private  final byte[] input;

	private final RealSenseInfo          info;
	private LibRealSenseIntrinsics intrinsics;
	private float scale;

	private boolean is_running;

	
	public static StreamRealSenseD455Depth getInstance(RealSenseInfo info) {
		if(instance==null)
			instance = new StreamRealSenseD455Depth(info);
		return instance;
	}

	private StreamRealSenseD455Depth(RealSenseInfo info)
	{

		super();

		this.input = new byte[info.width * info.height * 3];

		this.listeners = new ArrayList<IDepthCallback>();
		this.info = info;

		dev = getDeviceByName("D455");

		// No depth sensor found => do not use this driver
		if(dev==null) {
			throw new IllegalArgumentException("No device found");
		}
		
		rs2.rs2_hardware_reset(dev, error);

		PointerByReference sensor_list = rs2.rs2_query_sensors(dev, error);	
		
		int sensor_count = rs2.rs2_get_sensors_count(sensor_list, error);
		System.out.println("D455 has "+sensor_count+" sensor(s):");
	
		
		// Stereo module
		depth_sensor = rs2.rs2_create_sensor(sensor_list, 0, error);
		System.out.println("  "+rs2.rs2_get_sensor_info(depth_sensor, 0, error).getString(0));
		checkError("Sensors",error);
		
		setOption(depth_sensor, rs2_option.RS2_OPTION_EMITTER_ENABLED,"RS2_OPTION_EMITTER_ENABLED",2 );
//		setOption(depth_sensor, rs2_option.RS2_OPTION_HOLES_FILL,"RS2_OPTION_HOLES_FILL",true );
		
		// RGB module
		rgb_sensor = rs2.rs2_create_sensor(sensor_list, 1, error);
		System.out.println("  "+rs2.rs2_get_sensor_info(rgb_sensor, 0, error).getString(0));
		checkError("Sensors",error);
			
		setOption(rgb_sensor, rs2_option.RS2_OPTION_ENABLE_AUTO_EXPOSURE,"RS2_OPTION_ENABLE_AUTO_EXPOSURE",true );
//		setOption(rgb_sensor, rs2_option.RS2_OPTION_EXPOSURE,"RS2_OPTION_EXPOSURE",2566*3 );
//		setOption(rgb_sensor, rs2_option.RS2_OPTION_GAIN,"RS2_OPTION_GAIN",16*4 );
		setOption(rgb_sensor, rs2_option.RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE,"RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE",true );
		
		// Motion module
		motion_sensor = rs2.rs2_create_sensor(sensor_list, 2, error);
		System.out.println("  "+rs2.rs2_get_sensor_info(motion_sensor, 0, error).getString(0));
		checkError("Sensors",error);
		
	
		scale = rs2.rs2_get_option(depth_sensor, rs2_option.RS2_OPTION_DEPTH_UNITS, error);

		depth.reshape(info.width,info.height);
		rgb.reshape(info.width,info.height);

		printDeviceInfo();


	}


	public StreamRealSenseD455Depth registerCallback(IDepthCallback listener) {
		listeners.add(listener);
		return this;
	}


	public void start() {

		if(dev == null)
			return;

		// /home/lquac/librealsense/build/examples/C/depth/rs-depth

		config = rs2.rs2_create_config(error);
		rs2.rs2_config_enable_device(config, rs2.rs2_get_device_info(dev, rs2_camera_info.RS2_CAMERA_INFO_SERIAL_NUMBER, error),error);
		pipeline = rs2.rs2_create_pipeline(ctx, error);

		// Configure streams
		rs2.rs2_config_enable_stream(config, rs2_stream.RS2_STREAM_COLOR, 0, info.width, info.height, rs2_format.RS2_FORMAT_RGB8, FRAMERATE, error);
		checkError("ColorStream",error);
		rs2.rs2_config_enable_stream(config, rs2_stream.RS2_STREAM_DEPTH, 0, info.width, info.height, rs2_format.RS2_FORMAT_Z16, FRAMERATE, error);
		checkError("DepthStream",error);

		//		PointerByReference profile_list = rs2.rs2_get_stream_profiles(sensor, error);
		//		PointerByReference profile = rs2.rs2_get_stream_profile(profile_list, 11, error);
		//		
		//		rs2_intrinsics rs_intrinsics = new rs2_intrinsics();
		//		rs2.rs2_get_video_stream_intrinsics(profile, rs_intrinsics, error);
		//		intrinsics = new LibRealSenseIntrinsics(rs_intrinsics);
		//		System.out.println(intrinsics);

		OdometryPool.submit(new CombineD455Thread());
		System.out.println("D455 depth estimation started");

	}

	public void stop() {

		if(dev == null)
			return;

		System.out.print("Try to stop D455...");
		is_running = false;
		try { Thread.sleep(200); } catch (InterruptedException e) { }
	}



	public float getScale() {
		return scale;
	}

	public boolean isInitialized() {
		return is_initialized;
	}


	private class CombineD455Thread extends Thread {

		private long   tms_rgb;
		private long   tms_depth;
		
		private Realsense2Library.rs2_frame frames;
		private Realsense2Library.rs2_frame  frame;
		
		rs2_intrinsics rs_intrinsics = new rs2_intrinsics();

		@Override
		public void run() {
			
			
			try { Thread.sleep(300); } catch (InterruptedException e) {  }

			rs2.rs2_pipeline_start_with_config(pipeline, config, error);

			try { Thread.sleep(200); } catch (InterruptedException e) {  }

			System.out.println("D455 pipeline started");
			is_running = true;	
			while( is_running ) {

				try {

					frames = rs2.rs2_pipeline_wait_for_frames(pipeline, 1000, error);

					frame = rs2.rs2_extract_frame(frames, 0, error);
					if(rs2.rs2_get_frame_data_size(frame, error) > 0) {
						tms_depth = (long)rs2.rs2_get_frame_timestamp(frame,error);
						bufferDepthToU16(rs2.rs2_get_frame_data(frame, error),depth);
					}

					if(intrinsics==null) {
						PointerByReference mode = rs2.rs2_get_frame_stream_profile(frame,error);
						rs2.rs2_get_video_stream_intrinsics(mode, rs_intrinsics, error);
						intrinsics = new LibRealSenseIntrinsics(rs_intrinsics);
						is_initialized = true;
					}

					rs2.rs2_release_frame(frame);
					
					frame = rs2.rs2_extract_frame(frames, 1, error);
					if(rs2.rs2_get_frame_data_size(frame, error) > 0) {
						tms_rgb = (long)rs2.rs2_get_frame_timestamp(frame,error);
						bufferRgbToMsU8(rs2.rs2_get_frame_data(frame, error),rgb);
					}

					rs2.rs2_release_frame(frame);


					if(listeners.size()>0 && is_initialized) {
						for(IDepthCallback listener : listeners)
							listener.process(rgb, depth, tms_rgb, tms_depth);
					}


					rs2.rs2_release_frame(frames);

				} catch(Exception e) {
					e.printStackTrace();
				}
			}
			rs2.rs2_pipeline_stop(pipeline, error);
			System.out.println("D455 stopped.");
		}
	}


//	public void bufferGrayToU8(Pointer input , GrayU8 output ) {
//		byte[] inp = input.getByteArray(0, output.width * output.height );
//		System.arraycopy(inp, 0, output.data, 0, output.width * output.height);
//	}


	public void bufferDepthToU16(Pointer input , GrayU16 output ) {
		//	output.data = input.getShortArray(0, 678400);
		input.read(0, output.data, 0, output.data.length);
	}

	public void bufferRgbToMsU8( Pointer inp , Planar<GrayU8> output ) {

		byte[] b0 = output.getBand(0).data;
		byte[] b1 = output.getBand(1).data;
		byte[] b2 = output.getBand(2).data;

		inp.read(0, input, 0, input.length);
		
		for(int  y = 0; y < output.height; y++ ) {
//		BoofConcurrency.loopFor(0, output.height, y -> {
			int indexIn  = y*output.stride * 3;
			int indexOut = output.startIndex + y*output.stride;
			for( int x = 0; x < output.width; x++ , indexOut++ ) {
				b0[indexOut] = input[indexIn++];
				b1[indexOut] = input[indexIn++];
				b2[indexOut] = input[indexIn++];

			}
	//			});
		}
	}



	public CameraPinholeBrown getIntrinsics() {
		return intrinsics;
	}

	public void printDeviceInfo() {	
		if(dev == null)
			return;
		try {

			System.out.println(rs2
					.rs2_get_device_info(dev, rs2_camera_info.RS2_CAMERA_INFO_NAME, error).getString(0));
			System.out.println(rs2
					.rs2_get_device_info(dev, rs2_camera_info.RS2_CAMERA_INFO_SERIAL_NUMBER, error)
					.getString(0));
			System.out.println(rs2
					.rs2_get_device_info(dev, rs2_camera_info.RS2_CAMERA_INFO_FIRMWARE_VERSION, error)
					.getString(0));
			System.out.println("API version "+Realsense2Library.RS2_API_VERSION_STR);
			System.out.println("Scale: "+scale+"m");
		} catch(Exception e ) { }

	}


}
