package com.comino.mavodometry.librealsense.t265.boofcv;

/****************************************************************************
 *
 *   Copyright (c) 2020 Eike Mansfeld ecm@gmx.de. All rights reserved.
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

import com.comino.mavodometry.concurrency.OdometryPool;
import com.comino.mavodometry.librealsense.t265.wrapper.Realsense2Library;
import com.comino.mavodometry.librealsense.t265.wrapper.Realsense2Library.rs2_camera_info;
import com.sun.jna.Pointer;
import com.sun.jna.ptr.PointerByReference;
import com.sun.tools.javac.code.Attribute.Array;

import boofcv.struct.calib.CameraUniversalOmni;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.se.Se3_F64;

public class StreamRealSenseT265Pose {

	private static final int CYCLE_MS = 20;

	private static final int WIDTH  = 848;
	private static final int HEIGHT = 800;

	public static final  int POS_FOREWARD = 0;
	public static final  int POS_DOWNWARD = 1;

	private CameraUniversalOmni left_model;
	private CameraUniversalOmni right_model;

	private PointerByReference error = new PointerByReference();

	private Realsense2Library.rs2_device dev;

	private Realsense2Library.rs2_context ctx;
	private Realsense2Library.rs2_pipeline pipeline;
	private Realsense2Library.rs2_config config;
	private Realsense2Library.rs2_device_list device_list;

	private Realsense2Library.rs2_pose rawpose = new Realsense2Library.rs2_pose();

	private Realsense2Library.rs2_intrinsics intrinsics_left = new Realsense2Library.rs2_intrinsics();
	private PointerByReference mode_left = null;

	private Realsense2Library.rs2_intrinsics intrinsics_right = new Realsense2Library.rs2_intrinsics();
	private PointerByReference mode_right = null;

	private Realsense2Library.rs2_extrinsics extrinsics = new Realsense2Library.rs2_extrinsics();

	private IPoseCallback callback;

	private Planar<GrayU8> img  = new Planar<GrayU8>(GrayU8.class,WIDTH,HEIGHT,3);

	private long   tms0, tms;
	private int    fps;

	private boolean is_running;
	private boolean is_initialized;
	private boolean reset_request;

	private int x0,y0,x1,y1;
	private int mount;
	private int dev_count = 0;


	public StreamRealSenseT265Pose(int mount, IPoseCallback callback) {
		this(mount, WIDTH,HEIGHT,callback);
	}

	public StreamRealSenseT265Pose(int mount, int width, int height, IPoseCallback callback) {

		this.x0 = WIDTH/2 - width/2;
		this.y0 = HEIGHT/2 - height/2;
		this.x1 = x0 + width;
		this.y1 = y0 + height;
		this.mount = mount;

		this.callback = callback;

		ctx = Realsense2Library.INSTANCE.rs2_create_context(Realsense2Library.RS2_API_VERSION, error);
		if(!checkError(error)) {
			System.err.println("Check RS2_API_VERSION "+Realsense2Library.RS2_API_VERSION_STR);
			return;
		}

		device_list = Realsense2Library.INSTANCE.rs2_query_devices(ctx,error);
		checkError(error);
		dev_count = Realsense2Library.INSTANCE.rs2_get_device_count(device_list, error);
		if(dev_count < 1) {
			is_running = false;
			return;
		}

		dev = Realsense2Library.INSTANCE.rs2_create_device(device_list, 0, error);
		checkError(error);

		Realsense2Library.INSTANCE.rs2_hardware_reset(dev, error);
		try { Thread.sleep(200); } catch (InterruptedException e) {  }


		config = Realsense2Library.INSTANCE.rs2_create_config(error);

		//		Realsense2Library.INSTANCE.rs2_get_stream_profiles_count(dev, error);

		pipeline = Realsense2Library.INSTANCE.rs2_create_pipeline(ctx, error);
		checkError(error);

	}

	public void start() {
		if(dev_count < 1)
			return;
		System.out.println("T265 pose estimation started"+device_list);
		is_running = true;
		OdometryPool.submit(new CombineThread());
	}

	public void stop() {
		is_running = false;
		OdometryPool.close();
	}


	public Realsense2Library.rs2_pose getRawPose() {
		return rawpose;
	}

	public void reset() {
		reset_request = true;
	}

	public int getFrameRate() {
		return fps;
	}

	public int getMount() {
		return mount;
	}


	public CameraUniversalOmni getLeftModel() {
		return left_model;
	}

	public CameraUniversalOmni getRightModel() {
		return right_model;
	}

	public void printDeviceInfo() {

		System.out.println(Realsense2Library.INSTANCE
				.rs2_get_device_info(dev, rs2_camera_info.RS2_CAMERA_INFO_NAME, error).getString(0));
		System.out.println(Realsense2Library.INSTANCE
				.rs2_get_device_info(dev, rs2_camera_info.RS2_CAMERA_INFO_SERIAL_NUMBER, error)
				.getString(0));
		System.out.println(Realsense2Library.INSTANCE
				.rs2_get_device_info(dev, rs2_camera_info.RS2_CAMERA_INFO_FIRMWARE_VERSION, error)
				.getString(0));
		System.out.println("API version "+Realsense2Library.RS2_API_VERSION_STR);

	}

	public boolean isRunning() {
		return is_running;
	}

	private class CombineThread extends Thread {

		private Se3_F64    current_pose         = new Se3_F64();
		private Se3_F64    current_speed        = new Se3_F64();
		private Se3_F64    current_acceleration = new Se3_F64();

		private Realsense2Library.rs2_frame  frame;

		@Override
		public void run() {

			is_initialized = false;
			reset_request = false;

			// TODO: Pipelinestart failes with 0.2.0951 FW
			Realsense2Library.INSTANCE.rs2_pipeline_start_with_config(pipeline, config, error);
			try { Thread.sleep(200); } catch (InterruptedException e) {  }
			if(!checkError(error)) {
				is_running = false;
				return;
			}

			System.out.println("T265 pipeline started");


			long cycle_tms = System.currentTimeMillis(); long wait_ms;

			while(is_running) {

				wait_ms = CYCLE_MS - (System.currentTimeMillis() - cycle_tms );
				cycle_tms = System.currentTimeMillis();
				if(wait_ms > 0) {
					try { Thread.sleep(wait_ms); } catch (InterruptedException e) {  }
				}

				Realsense2Library.rs2_frame frames = Realsense2Library.INSTANCE.rs2_pipeline_wait_for_frames(pipeline, 500, error);
				tms = (long)Realsense2Library.INSTANCE.rs2_get_frame_timestamp(frames,error);

				frame = Realsense2Library.INSTANCE.rs2_extract_frame(frames, 0, error);
				if(Realsense2Library.INSTANCE.rs2_get_frame_data_size(frame, error) > 0) {
					bufferGrayToU8(Realsense2Library.INSTANCE.rs2_get_frame_data(frame, error),img);

					if(mode_left==null) {
						mode_left = Realsense2Library.INSTANCE.rs2_get_frame_stream_profile(frame,error);
						Realsense2Library.INSTANCE.rs2_get_video_stream_intrinsics(mode_left, intrinsics_left, error);
						left_model = createFisheyeModel(intrinsics_left);
					}

				}
				Realsense2Library.INSTANCE.rs2_release_frame(frame);

				frame = Realsense2Library.INSTANCE.rs2_extract_frame(frames, 1, error);
				if(Realsense2Library.INSTANCE.rs2_get_frame_data_size(frame, error) > 0) {
					if(mode_right==null) {
						mode_right = Realsense2Library.INSTANCE.rs2_get_frame_stream_profile(frame,error);
						Realsense2Library.INSTANCE.rs2_get_video_stream_intrinsics(mode_right, intrinsics_right, error);
						Realsense2Library.INSTANCE.rs2_get_extrinsics(mode_left, mode_right, extrinsics, error);

						right_model = createFisheyeModel(intrinsics_right);

						is_initialized = true;

					}
				}
				Realsense2Library.INSTANCE.rs2_release_frame(frame);

				frame = Realsense2Library.INSTANCE.rs2_extract_frame(frames, 4, error);
				if(Realsense2Library.INSTANCE.rs2_get_frame_data_size(frame, error) > 0) {
					Realsense2Library.INSTANCE.rs2_pose_frame_get_pose_data(frame, rawpose, error);
				}
				Realsense2Library.INSTANCE.rs2_release_frame(frame);


				Realsense2Library.INSTANCE.rs2_release_frame(frames);

				fps = (int)(1000.0f/(tms - tms0));
				tms0 = tms;

				switch(mount) {

				case POS_FOREWARD:

					current_pose.getTranslation().set( - rawpose.translation.z, rawpose.translation.x, - rawpose.translation.y);
					ConvertRotation3D_F64.quaternionToMatrix(
							rawpose.rotation.w,
						   -rawpose.rotation.z,
							rawpose.rotation.x,
						   -rawpose.rotation.y, current_pose.getRotation());

					current_speed.getTranslation().set(- rawpose.velocity.z, rawpose.velocity.x, - rawpose.velocity.y);
					current_speed.getRotation().set(current_pose.getRotation());

					current_acceleration.getTranslation().set(- rawpose.acceleration.z, rawpose.acceleration.x, - rawpose.acceleration.y);

					break;
				case POS_DOWNWARD:

					current_pose.getTranslation().set( -rawpose.translation.x, -rawpose.translation.z, - rawpose.translation.y);
					current_pose.R.zero();
					current_speed.getTranslation().set(- rawpose.velocity.x, -rawpose.velocity.z, - rawpose.velocity.y);
					current_speed.getRotation().set(current_pose.getRotation());

					current_acceleration.getTranslation().set(- rawpose.acceleration.x, -rawpose.acceleration.z, - rawpose.acceleration.y);

					break;
				}

				if(is_initialized)
					callback.handle(tms, rawpose, current_pose,current_speed, current_acceleration,
							img.subimage(x0, y0, x1, y1));


				if(reset_request) {
					try {
						synchronized(this) {
							reset_request = false;
							Realsense2Library.INSTANCE.rs2_pipeline_stop(pipeline, error);
							try { Thread.sleep(100); } catch (InterruptedException e) {  }
							Realsense2Library.INSTANCE.rs2_pipeline_start_with_config(pipeline, config, error);
							checkError(error);
						}
					} catch(Exception e) { e.printStackTrace(); }
				}

			}
			Realsense2Library.INSTANCE.rs2_pipeline_stop(pipeline, error);
		}
	}

	private void bufferGrayToU8(Pointer input , Planar<GrayU8> output ) {
		output.bands[0].data = input.getByteArray(0, 678400);
		output.bands[1].data = input.getByteArray(0, 678400);
		output.bands[2].data = input.getByteArray(0, 678400);
	}

	private boolean checkError(PointerByReference error) {
		if(error.getValue()!=null) {
			if(error.getValue().getString(1).length()>0) {
				System.err.println(error.getValue().getString(1));
				return false;
			}
			return true;
		}
		return true;
	}


	private CameraUniversalOmni createFisheyeModel(Realsense2Library.rs2_intrinsics in) {

		CameraUniversalOmni omni = new CameraUniversalOmni(5);

		omni.fx = in.fx;
		omni.fy = in.fy;
		omni.cx = in.ppx;
		omni.cy = in.ppy;
		omni.width  = in.width;
		omni.height = in.height;
		omni.t1 = 0;
		omni.t2 = 0;

		omni.mirrorOffset = 0.5;

		for(int i = 0; i< in.coeffs.length;i++)
			omni.radial[i] = in.coeffs[i];

		return omni;

	}

}
