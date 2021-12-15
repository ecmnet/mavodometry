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

import static org.bytedeco.librealsense2.global.realsense2.rs2_config_enable_device;
import static org.bytedeco.librealsense2.global.realsense2.rs2_config_enable_stream;
import static org.bytedeco.librealsense2.global.realsense2.rs2_create_config;
import static org.bytedeco.librealsense2.global.realsense2.rs2_create_pipeline;
import static org.bytedeco.librealsense2.global.realsense2.rs2_extract_frame;
import static org.bytedeco.librealsense2.global.realsense2.rs2_get_frame_data;
import static org.bytedeco.librealsense2.global.realsense2.rs2_get_frame_data_size;
import static org.bytedeco.librealsense2.global.realsense2.rs2_get_frame_stream_profile;
import static org.bytedeco.librealsense2.global.realsense2.rs2_get_frame_timestamp;
import static org.bytedeco.librealsense2.global.realsense2.rs2_get_video_stream_intrinsics;
import static org.bytedeco.librealsense2.global.realsense2.rs2_pipeline_start_with_config;
import static org.bytedeco.librealsense2.global.realsense2.rs2_pipeline_stop;
import static org.bytedeco.librealsense2.global.realsense2.rs2_pipeline_wait_for_frames;
import static org.bytedeco.librealsense2.global.realsense2.rs2_release_frame;

import java.util.ArrayList;
import java.util.List;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.javacpp.ShortPointer;
import org.bytedeco.librealsense2.rs2_config;
import org.bytedeco.librealsense2.rs2_device;
import org.bytedeco.librealsense2.rs2_frame;
import org.bytedeco.librealsense2.rs2_intrinsics;
import org.bytedeco.librealsense2.rs2_pipeline;
import org.bytedeco.librealsense2.rs2_sensor;
import org.bytedeco.librealsense2.rs2_sensor_list;
import org.bytedeco.librealsense2.rs2_stream_profile;
import org.bytedeco.librealsense2.global.realsense2;

import com.comino.mavodometry.concurrency.OdometryPool;
import com.comino.mavodometry.librealsense.javacpp.RealsenseDevice;
import com.comino.mavodometry.librealsense.utils.LibRealSenseIntrinsics;
import com.comino.mavodometry.librealsense.utils.RealSenseInfo;

import boofcv.struct.calib.CameraPinholeBrown;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;

public class StreamRealSenseD455Depth extends RealsenseDevice {
	
	private static final int FRAMERATE = 15;


	private static StreamRealSenseD455Depth instance;
	
	private final List<IDepthCallback> listeners;

	// image with depth information
	private final GrayU16 depth        = new GrayU16(1,1);
	private final Planar<GrayU8> rgb	 = new Planar<GrayU8>(GrayU8.class,1,1,3);

	private volatile rs2_device dev;
	private volatile rs2_pipeline pipeline;
	private volatile rs2_config config;

	private rs2_sensor         depth_sensor  = null;
	private rs2_sensor         rgb_sensor    = null;
	private rs2_sensor         motion_sensor = null;
	
	private  final byte[] input;

	private final RealSenseInfo          info;
	private LibRealSenseIntrinsics intrinsics;
	private float scale;

	private boolean is_running;

	
	public static StreamRealSenseD455Depth getInstance(RealSenseInfo info) throws Exception {
		if(instance==null)
			instance = new StreamRealSenseD455Depth(info);
		return instance;
	}

	private StreamRealSenseD455Depth(RealSenseInfo info) throws Exception
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
		
		hardwareReset(dev);

		rs2_sensor_list sensor_list = getSensorList(dev);	
		
		int sensor_count = getSensorCount(sensor_list);
		System.out.println("D455 has "+sensor_count+" sensor(s):");
	
		
		// Stereo module
		depth_sensor = createSensor(sensor_list, 0);
		System.out.println("  "+getSensorInfo(depth_sensor, 0));
		
		setSensorOption(depth_sensor, realsense2.RS2_OPTION_EMITTER_ENABLED,2 );
//		setSensorOption(depth_sensor, realsense2.RS2_OPTION_HOLES_FILL,true );
		
		// RGB module
		rgb_sensor = createSensor(sensor_list, 1);
		System.out.println("  "+getSensorInfo(rgb_sensor, 0));
			
		setSensorOption(rgb_sensor, realsense2.RS2_OPTION_ENABLE_AUTO_EXPOSURE,true );
//		setSensorOption(rgb_sensor, realsense2.RS2_OPTION_EXPOSURE,2566*3 );
//		setSensorOption(rgb_sensor, realsense2.RS2_OPTION_GAIN,16*4 );
		setSensorOption(rgb_sensor, realsense2.RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE,true );
		
		// Motion module
		motion_sensor = createSensor(sensor_list, 2);
		System.out.println("  "+getSensorInfo(motion_sensor, 0));
		
	
		scale = getSensorOption(depth_sensor, realsense2.RS2_OPTION_DEPTH_UNITS);

		depth.reshape(info.width,info.height);
		rgb.reshape(info.width,info.height);

		printDeviceInfo();


	}


	public StreamRealSenseD455Depth registerCallback(IDepthCallback listener) {
		listeners.add(listener);
		return this;
	}


	public void start() throws Exception {

		if(dev == null)
			return;

		// /home/lquac/librealsense/build/examples/C/depth/rs-depth

		config = rs2_create_config(error);
		checkError(error);
		
		rs2_config_enable_device(config, getDeviceInfo(dev, realsense2.RS2_CAMERA_INFO_SERIAL_NUMBER),error);
		checkError(error);
		
		
		pipeline = rs2_create_pipeline(ctx, error);
		checkError(error);
	

		// Configure streams
		
		rs2_config_enable_stream(config, realsense2.RS2_STREAM_COLOR, 0, info.width, info.height, realsense2.RS2_FORMAT_RGB8, FRAMERATE , error);
		checkError(error);
		
		rs2_config_enable_stream(config, realsense2.RS2_STREAM_DEPTH, 0, info.width, info.height, realsense2.RS2_FORMAT_Z16, FRAMERATE , error);
		checkError(error);
		

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
		
		private rs2_frame frames;
		private rs2_frame  frame;
		
		rs2_intrinsics rs_intrinsics = new rs2_intrinsics();

		@Override
		public void run() {
			
			
			try { Thread.sleep(300); } catch (InterruptedException e) {  }

			rs2_pipeline_start_with_config(pipeline, config, error);

			try { Thread.sleep(200); } catch (InterruptedException e) {  }

			System.out.println("D455 pipeline started");
			is_running = true;	
			while( is_running ) {

				try {

					frames = rs2_pipeline_wait_for_frames(pipeline, 1000, error);

					frame = rs2_extract_frame(frames, 0, error);
					if(rs2_get_frame_data_size(frame, error) > 0) {
						tms_depth = (long)rs2_get_frame_timestamp(frame,error);
						bufferDepthToU16(rs2_get_frame_data(frame, error),depth);
					}

					if(intrinsics==null) {
						rs2_stream_profile mode = rs2_get_frame_stream_profile(frame,error);
						rs2_get_video_stream_intrinsics(mode, rs_intrinsics, error);
						intrinsics = new LibRealSenseIntrinsics(rs_intrinsics);
						is_initialized = true;
					}

					rs2_release_frame(frame);
					
					frame = rs2_extract_frame(frames, 1, error);
					if(rs2_get_frame_data_size(frame, error) > 0) {
						tms_rgb = (long)rs2_get_frame_timestamp(frame,error);
						bufferRgbToMsU8(rs2_get_frame_data(frame, error),rgb);
					}

					rs2_release_frame(frame);


					if(listeners.size()>0 && is_initialized) {
						for(IDepthCallback listener : listeners)
							listener.process(rgb, depth, tms_rgb, tms_depth);
					}


					rs2_release_frame(frames);

				} catch(Exception e) {
					e.printStackTrace();
				}
			}
			rs2_pipeline_stop(pipeline, error);
			System.out.println("D455 stopped.");
		}
	}


//	public void bufferGrayToU8(Pointer input , GrayU8 output ) {
//		byte[] inp = input.getByteArray(0, output.width * output.height );
//		System.arraycopy(inp, 0, output.data, 0, output.width * output.height);
//	}


	public void bufferDepthToU16(Pointer input , GrayU16 output ) {
		ShortPointer input_short = new ShortPointer(input);
		input_short.get(output.data);
		input_short.close();
	}

	public void bufferRgbToMsU8( Pointer inp , Planar<GrayU8> output ) {

		BytePointer input = new BytePointer(inp);
		
		byte[] b0 = output.getBand(0).data;
		byte[] b1 = output.getBand(1).data;
		byte[] b2 = output.getBand(2).data;

		
		
		for(int  y = 0; y < output.height; y++ ) {
//		BoofConcurrency.loopFor(0, output.height, y -> {
			int indexIn  = y*output.stride * 3;
			int indexOut = output.startIndex + y*output.stride;
			for( int x = 0; x < output.width; x++ , indexOut++ ) {
				b0[indexOut] = input.get(indexIn++);
				b1[indexOut] = input.get(indexIn++);
				b2[indexOut] = input.get(indexIn++);

			}
	//			});
		}
		input.close();
	}



	public CameraPinholeBrown getIntrinsics() {
		return intrinsics;
	}

	public void printDeviceInfo() {	
		if(dev == null)
			return;
		try {

			System.out.println(getDeviceInfo(dev, realsense2.RS2_CAMERA_INFO_NAME));
			System.out.println(getDeviceInfo(dev, realsense2.RS2_CAMERA_INFO_SERIAL_NUMBER));
			System.out.println(getDeviceInfo(dev, realsense2.RS2_CAMERA_INFO_FIRMWARE_VERSION));
			System.out.println("API version "+realsense2.RS2_API_VERSION_STR);
			System.out.println("Scale: "+scale+"m");
		} catch(Exception e ) { }

	}


}
