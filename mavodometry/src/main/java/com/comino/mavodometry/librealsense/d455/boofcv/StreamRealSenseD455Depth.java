/****************************************************************************
 *
 *   Copyright (c) 2017 Eike Mansfeld ecm@gmx.de. All rights reserved.
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
import com.comino.mavodometry.librealsense.lib.LibRealSense1Library;
import com.comino.mavodometry.librealsense.lib.Realsense2Library;
import com.comino.mavodometry.librealsense.lib.Realsense2Library.rs2_camera_info;
import com.comino.mavodometry.librealsense.lib.Realsense2Library.rs2_format;
import com.comino.mavodometry.librealsense.lib.Realsense2Library.rs2_intrinsics;
import com.comino.mavodometry.librealsense.lib.Realsense2Library.rs2_option;
import com.comino.mavodometry.librealsense.lib.Realsense2Library.rs2_stream;
import com.comino.mavodometry.librealsense.lib.Realsense2Library.rs2_stream_profile_list;
import com.comino.mavodometry.librealsense.lib.RealsenseDevice;
import com.comino.mavodometry.librealsense.utils.LibRealSenseIntrinsics;
import com.comino.mavodometry.librealsense.utils.RealSenseInfo;
import com.sun.jna.Pointer;
import com.sun.jna.ptr.PointerByReference;

import boofcv.concurrency.BoofConcurrency;
import boofcv.struct.calib.CameraPinholeBrown;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;

public class StreamRealSenseD455Depth extends RealsenseDevice {


	private List<IDepthCallback> listeners;

	// image with depth information
	private GrayU16 depth        = new GrayU16(1,1);
	private Planar<GrayU8> rgb	 = new Planar<GrayU8>(GrayU8.class,1,1,3);

	private volatile Realsense2Library.rs2_device dev;
	private volatile Realsense2Library.rs2_pipeline pipeline;
	private volatile Realsense2Library.rs2_config config;

	private PointerByReference sensor = null;
	private  byte[] input;

	private RealSenseInfo          info;
	private LibRealSenseIntrinsics intrinsics;
	private float scale;

	private boolean is_running;

	private long   tms;

	public StreamRealSenseD455Depth(int devno , RealSenseInfo info)
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


		// Settings some options
		PointerByReference sensor_list = rs2.rs2_query_sensors(dev, error);	
		sensor = rs2.rs2_create_sensor(sensor_list, 0, error);
		checkError("Sensors",error);

		rs2.rs2_set_option(sensor, Realsense2Library.rs2_option.RS2_OPTION_EMITTER_ALWAYS_ON, OPTION_ENABLE, error);
		rs2.rs2_set_option(sensor, Realsense2Library.rs2_option.RS2_OPTION_HISTOGRAM_EQUALIZATION_ENABLED, OPTION_DISABLE, error);

		scale = rs2.rs2_get_option(sensor, rs2_option.RS2_OPTION_DEPTH_UNITS, error);
		

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
		rs2.rs2_config_enable_stream(config, rs2_stream.RS2_STREAM_COLOR, 0, info.width, info.height, rs2_format.RS2_FORMAT_RGB8, 30, error);
		checkError("ColorStream",error);
		rs2.rs2_config_enable_stream(config, rs2_stream.RS2_STREAM_DEPTH, 0, info.width, info.height, rs2_format.RS2_FORMAT_Z16, 30, error);
		checkError("DepthStream",error);
		
//		PointerByReference profile_list = rs2.rs2_get_stream_profiles(sensor, error);
//		PointerByReference profile = rs2.rs2_get_stream_profile(profile_list, 11, error);
//		
//		rs2_intrinsics rs_intrinsics = new rs2_intrinsics();
//		rs2.rs2_get_video_stream_intrinsics(profile, rs_intrinsics, error);
//		intrinsics = new LibRealSenseIntrinsics(rs_intrinsics);
//		System.out.println(intrinsics);

		is_running = true;

		OdometryPool.submit(new CombineD455Thread());
		System.out.println("D455 depth estimation started");

	}

	public void stop() {

		if(dev == null)
			return;

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


		private Realsense2Library.rs2_frame  frame;


		@Override
		public void run() {
			rs2_intrinsics rs_intrinsics = new rs2_intrinsics();

			rs2.rs2_pipeline_start_with_config(pipeline, config, error);

			try { Thread.sleep(200); } catch (InterruptedException e) {  }
			
			System.out.println("D455 pipeline started");

			while( is_running ) {

				try {

					Realsense2Library.rs2_frame              frames = rs2.rs2_pipeline_wait_for_frames(pipeline, 2000, error);


					tms = (long)rs2.rs2_get_frame_timestamp(frames,error);


					frame = rs2.rs2_extract_frame(frames, 1, error);
					if(rs2.rs2_get_frame_data_size(frame, error) > 0) 
						bufferRgbToMsU8(rs2.rs2_get_frame_data(frame, error),rgb);

					rs2.rs2_release_frame(frame);

					frame = rs2.rs2_extract_frame(frames, 0, error);
					if(rs2.rs2_get_frame_data_size(frame, error) > 0) 
						bufferDepthToU16(rs2.rs2_get_frame_data(frame, error),depth);
					
					if(intrinsics==null) {
						PointerByReference mode = rs2.rs2_get_frame_stream_profile(frame,error);
						rs2.rs2_get_video_stream_intrinsics(mode, rs_intrinsics, error);
						intrinsics = new LibRealSenseIntrinsics(rs_intrinsics);
						is_initialized = true;
					}

					rs2.rs2_release_frame(frame);

					synchronized(this) {
						if(listeners.size()>0 && is_initialized) {
							for(IDepthCallback listener : listeners)
								listener.process(rgb, depth, tms, tms);
						}
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
	

	public void bufferGrayToU8(Pointer input , GrayU8 output ) {
		byte[] inp = input.getByteArray(0, output.width * output.height );
		System.arraycopy(inp, 0, output.data, 0, output.width * output.height);
	}


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
//		});
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
