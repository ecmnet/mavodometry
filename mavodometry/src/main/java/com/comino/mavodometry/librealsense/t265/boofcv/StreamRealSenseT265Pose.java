/****************************************************************************
 *
 *   Copyright (c) 2017,2021 Eike Mansfeld ecm@gmx.de. All rights reserved.
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

package com.comino.mavodometry.librealsense.t265.boofcv;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.Debug;

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
import com.comino.mavodometry.librealsense.lib.Realsense2Library;
import com.comino.mavodometry.librealsense.lib.Realsense2Library.rs2_camera_info;
import com.comino.mavodometry.librealsense.lib.Realsense2Library.rs2_option;
import com.comino.mavodometry.librealsense.lib.Realsense2Library.rs2_pipeline_profile;
import com.comino.mavodometry.librealsense.lib.RealsenseDevice;
import com.sun.jna.Pointer;
import com.sun.jna.ptr.PointerByReference;
import com.sun.jna.win32.StdCallLibrary.StdCallCallback;

import boofcv.struct.calib.CameraKannalaBrandt;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.se.Se3_F64;

public class StreamRealSenseT265Pose extends RealsenseDevice {

	private static StreamRealSenseT265Pose instance;

	private static final int WIDTH  = 848;
	private static final int HEIGHT = 800;

	public static final  int POS_FOREWARD     = 0;
	public static final  int POS_DOWNWARD     = 1; // UP
	public static final  int POS_DOWNWARD_180 = 2; // Jetson

	public static final  int CONFIDENCE_FAILED = 0;
	public static final  int CONFIDENCE_LOW    = 1;
	public static final  int CONFIDENCE_MEDIUM = 2;
	public static final  int CONFIDENCE_HIGH   = 3;

	public static final  int T265_EVENT_NOTIFICATION = 0;

	private CameraKannalaBrandt left_model;
	private CameraKannalaBrandt right_model;

	private volatile Realsense2Library.rs2_device dev;
	private volatile Realsense2Library.rs2_pipeline pipeline;
	private volatile Realsense2Library.rs2_config config;

	private PointerByReference sensor = null;

	private Realsense2Library.rs2_pose rawpose = new Realsense2Library.rs2_pose();
	private Realsense2Library.rs2_pose node = new Realsense2Library.rs2_pose();

	private Realsense2Library.rs2_intrinsics intrinsics_left = new Realsense2Library.rs2_intrinsics();
	private PointerByReference mode_left = null;

	private Realsense2Library.rs2_intrinsics intrinsics_right = new Realsense2Library.rs2_intrinsics();
	private PointerByReference mode_right = null;

	private Realsense2Library.rs2_extrinsics extrinsics = new Realsense2Library.rs2_extrinsics();

	private final List<IPoseCallback>         callbacks     = new ArrayList<IPoseCallback>();

	private final Planar<GrayU8> img     = new Planar<GrayU8>(GrayU8.class,WIDTH,HEIGHT,3);

	private long   tms0, tms, tms_offset;
	private int    fps;

	private boolean is_running;
	private boolean reset_request;

	private int x0,y0,x1,y1;
	private int mount;

	private Debug debug;


	private final DMatrixRMaj   rtY90  = CommonOps_DDRM.identity( 3 );
	private final DMatrixRMaj   rtY90P = CommonOps_DDRM.identity( 3 );
	private final DMatrixRMaj   tmp    = CommonOps_DDRM.identity( 3 );

	public static StreamRealSenseT265Pose getInstance(int mount, int width, int height, Debug debug) {
		if(instance==null)
			instance = new StreamRealSenseT265Pose(mount,width,height, debug);
		return instance;
	}


//	public class T265NotificationCallback implements Realsense2Library.rs2_notification_callback_ptr {
//
//		//	private PointerByReference notfication = new PointerByReference();
//
//		@Override
//		public void apply(Pointer rs2_notification, Pointer voidPtr1) {
//			if(is_initialized) {
//				for(INotificationCallback notification : notifications)
//					notification.notify(DataModel.getSynchronizedPX4Time_us(), T265_EVENT_NOTIFICATION);
//			}
//			//			notfication.setPointer(rs2_notification);
//			//			int category = rs2.rs2_get_notification_category(notfication, error);
//			//			System.out.println("Notification "+category);
//		}  
//	}

//	private  T265NotificationCallback cb = new T265NotificationCallback();

	private  StreamRealSenseT265Pose(int mount, int width, int height, Debug debug) {

		super();

		this.x0 = WIDTH/2 - width/2;
		this.y0 = HEIGHT/2 - height/2;
		this.x1 = x0 + width;
		this.y1 = y0 + height;
		this.mount = mount;
		this.debug = debug;

		ConvertRotation3D_F64.rotY(Math.PI/2,rtY90);
		ConvertRotation3D_F64.rotY(-Math.PI/2,rtY90P);

		dev = getDeviceByName("T265");

		// No depth sensor found => do not use this driver
		if(dev==null) {
			throw new IllegalArgumentException("No device found");
		}


		// Settings some options for the pose sensor

		PointerByReference sensor_list = rs2.rs2_query_sensors(dev, error);

		int sensor_count = rs2.rs2_get_sensors_count(sensor_list, error);
		System.out.println("T265 has "+sensor_count+" sensor(s)");

		sensor = rs2.rs2_create_sensor(sensor_list, 0, error);

		setOption(sensor,rs2_option.RS2_OPTION_ENABLE_POSE_JUMPING, "RS2_OPTION_ENABLE_POSE_JUMPING", false);
		setOption(sensor,rs2_option.RS2_OPTION_ENABLE_MAPPING, "RS2_OPTION_ENABLE_MAPPING", true);
		setOption(sensor,rs2_option.RS2_OPTION_ENABLE_MAP_PRESERVATION, "RS2_OPTION_ENABLE_MAP_PRESERVATION", true);
		setOption(sensor,rs2_option.RS2_OPTION_ENABLE_DYNAMIC_CALIBRATION, "RS2_OPTION_ENABLE_DYNAMIC_CALIBRATION", true);
		setOption(sensor,rs2_option.RS2_OPTION_ENABLE_RELOCALIZATION, "RS2_OPTION_ENABLE_RELOCALIZATION", true);

		setOption(sensor,rs2_option.RS2_OPTION_FRAMES_QUEUE_SIZE, "RS2_OPTION_FRAMES_QUEUE_SIZE", 3);

		// Callback setup: TODO: Not sure whether this works
//		System.out.println("   -> Notification callback registered");
//		rs2.rs2_set_notifications_callback(sensor,cb,null,error);

	}

	public StreamRealSenseT265Pose registerCallback(IPoseCallback callback) {
		this.callbacks.add(callback);
		return this;
	}

	public void start() {

		if(dev == null)
			return;

		if(rs2.rs2_get_device_info(dev, rs2_camera_info.RS2_CAMERA_INFO_FIRMWARE_VERSION, error)
				.getString(0).contains("951"))
		{
			System.out.println("T265 hardware reset performed..");
			rs2.rs2_hardware_reset(dev, error);
			try { Thread.sleep(200); } catch (InterruptedException e) {  }
		}

		config = rs2.rs2_create_config(error);
		rs2.rs2_config_enable_device(config, rs2.rs2_get_device_info(dev, rs2_camera_info.RS2_CAMERA_INFO_SERIAL_NUMBER, error),error);
		pipeline = rs2.rs2_create_pipeline(ctx, error);

		is_running = true;

		OdometryPool.submit(new CombineT265Thread());

		System.out.println("T265 pose estimation started");
	}

	public void stop() {
		if(dev == null)
			return;
		is_running = false;
		try { Thread.sleep(200); } catch (InterruptedException e) { }
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


	public CameraKannalaBrandt getLeftModel() {
		return left_model;
	}

	public CameraKannalaBrandt getRightModel() {
		return right_model;
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
		} catch(Exception e ) { }

	}

	public boolean isRunning() {
		return is_running;
	}

	private class CombineT265Thread extends Thread {

		private Se3_F64    current_pose         = new Se3_F64();
		private Se3_F64    current_speed        = new Se3_F64();
		private Se3_F64    current_acceleration = new Se3_F64();

		private Realsense2Library.rs2_frame  frame;

		@Override
		public void run() {

			is_initialized = false;
			reset_request = false;

			try { Thread.sleep(300); } catch (InterruptedException e) {  }

			//			rs2_pipeline_profile profile = 
			rs2.rs2_pipeline_start_with_config(pipeline, config, error);


			try { Thread.sleep(200); } catch (InterruptedException e) {  }


			System.out.println("T265 pipeline started");


			while(is_running) {

				try {


					Realsense2Library.rs2_frame	frames = rs2.rs2_pipeline_wait_for_frames(pipeline, 500, error);

					tms = (long)rs2.rs2_get_frame_timestamp(frames,error);

					if(tms_offset==0 && tms > 0)
						tms_offset = tms-System.currentTimeMillis();

					//					if(rs2.rs2_get_static_node(sensor, "node0", node.translation, node.rotation, error)!=0)
					//						System.out.println("Node0");

					frame = rs2.rs2_extract_frame(frames, 0, error);
					if(rs2.rs2_get_frame_data_size(frame, error) > 0) {

						if(mode_left==null) {
							mode_left = rs2.rs2_get_frame_stream_profile(frame,error);
							rs2.rs2_get_video_stream_intrinsics(mode_left, intrinsics_left, error);
							left_model = createFisheyeModel(intrinsics_left);
						} else {
							bufferGrayToU8(rs2.rs2_get_frame_data(frame, error),img);
						}

					}
					rs2.rs2_release_frame(frame);

					frame = rs2.rs2_extract_frame(frames, 1, error);
					if(rs2.rs2_get_frame_data_size(frame, error) > 0) {
						if(mode_right==null) {
							mode_right = rs2.rs2_get_frame_stream_profile(frame,error);
							rs2.rs2_get_video_stream_intrinsics(mode_right, intrinsics_right, error);
							rs2.rs2_get_extrinsics(mode_left, mode_right, extrinsics, error);

							right_model = createFisheyeModel(intrinsics_right);

							is_initialized = true;

						}
					}
					rs2.rs2_release_frame(frame);

					frame = rs2.rs2_extract_frame(frames, 4, error);
					if(rs2.rs2_get_frame_data_size(frame, error) > 0) {
						rs2.rs2_pose_frame_get_pose_data(frame, rawpose, error);
					}
					rs2.rs2_release_frame(frame);


					rs2.rs2_release_frame(frames);

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

						current_pose.getTranslation().set( -rawpose.translation.z, rawpose.translation.x, - rawpose.translation.y);

						ConvertRotation3D_F64.quaternionToMatrix(
								rawpose.rotation.w,
								-rawpose.rotation.z,
								rawpose.rotation.x,
								-rawpose.rotation.y, tmp);

						CommonOps_DDRM.mult(tmp, rtY90 , current_pose.getRotation());

						current_speed.getTranslation().set( -rawpose.velocity.z, rawpose.velocity.x, - rawpose.velocity.y);
						current_speed.getRotation().set(current_pose.getRotation());

						current_acceleration.getTranslation().set(- rawpose.acceleration.z, rawpose.acceleration.x, - rawpose.acceleration.y);

						break;


					case POS_DOWNWARD_180:

						current_pose.getTranslation().set( rawpose.translation.z, -rawpose.translation.x, - rawpose.translation.y);

						ConvertRotation3D_F64.quaternionToMatrix(
								rawpose.rotation.w,
								rawpose.rotation.z,
								-rawpose.rotation.x,
								-rawpose.rotation.y, tmp);

						CommonOps_DDRM.mult(tmp, rtY90P , current_pose.getRotation());

						current_speed.getTranslation().set( rawpose.velocity.z, -rawpose.velocity.x, - rawpose.velocity.y);
						current_speed.getRotation().set(current_pose.getRotation());

						current_acceleration.getTranslation().set(rawpose.acceleration.z, -rawpose.acceleration.x, - rawpose.acceleration.y);

						break;
					}


					if(reset_request) {
						try {
							synchronized(this) {
								reset_request = false;
								rs2.rs2_pipeline_stop(pipeline, error);
								try { Thread.sleep(200); } catch (InterruptedException e) {  }
								rs2.rs2_pipeline_start_with_config(pipeline, config, error);
								checkError("Restart pipeline",error);
								tms_offset=0;
							}
						} catch(Exception e) { e.printStackTrace(); }
						continue;
					}

					if(is_initialized) {
						for(IPoseCallback callback : callbacks)
							callback.handle(tms+tms_offset, rawpose, current_pose,current_speed, current_acceleration, img.subimage(x0, y0, x1, y1));
					}


				} catch(Exception e) {
					e.printStackTrace();
				}
			}
			rs2.rs2_pipeline_stop(pipeline, error);
			System.out.println("T265 stopped.");
		}
	}

	private synchronized void bufferGrayToU8(Pointer input , Planar<GrayU8> output ) {
		input.read(0, output.bands[0].data, 0, output.bands[0].data.length);
		output.bands[1].data = output.bands[0].data;
		output.bands[2].data = output.bands[0].data;
	}


	private CameraKannalaBrandt createFisheyeModel(Realsense2Library.rs2_intrinsics in) {

		CameraKannalaBrandt model = new CameraKannalaBrandt();

		model.fx = in.fx;
		model.fy = in.fy;

		model.width  = x1-x0;
		model.height = y1-y0;

		model.cx = model.width/2f;
		model.cy = model.height/2f;

		for(int i=0;i<5;i++) {
			model.coefSymm[i] = in.coeffs[i];
		}

		return model;

	}

}
