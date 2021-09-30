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

import java.time.Instant;
import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

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
import com.comino.mavodometry.librealsense.lib.Realsense2Library.rs2_extension;
import com.comino.mavodometry.librealsense.lib.Realsense2Library.rs2_format;
import com.comino.mavodometry.librealsense.lib.Realsense2Library.rs2_frame;
import com.comino.mavodometry.librealsense.lib.Realsense2Library.rs2_option;
import com.comino.mavodometry.librealsense.lib.Realsense2Library.rs2_sensor;
import com.comino.mavodometry.librealsense.lib.Realsense2Library.rs2_stream;
import com.comino.mavodometry.librealsense.lib.Realsense2Library.rs2_stream_profile;
import com.comino.mavodometry.librealsense.lib.RealsenseDevice;
import com.sun.jna.Pointer;
import com.sun.jna.ptr.PointerByReference;

import boofcv.struct.calib.CameraKannalaBrandt;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.se.Se3_F64;

public class StreamRealSenseT265Pose2_backup extends RealsenseDevice {

	private static StreamRealSenseT265Pose2_backup instance;

	private static final int WIDTH      = 848;
	private static final int HEIGHT     = 800;

	private static final int RATE_FPS   = 30;


	public static final  int POS_FOREWARD     			= 0;
	public static final  int POS_DOWNWARD    			= 1; // UP
	public static final  int POS_DOWNWARD_180 			= 2; // Jetson
	public static final  int POS_DOWNWARD_180_PREDICT 	= 3; // Jetson predict

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
	private Realsense2Library.rs2_pose prepose = new Realsense2Library.rs2_pose();


	private Realsense2Library.rs2_intrinsics intrinsics_left = new Realsense2Library.rs2_intrinsics();
	private PointerByReference mode_left = null;

	private Realsense2Library.rs2_intrinsics intrinsics_right = new Realsense2Library.rs2_intrinsics();
	private PointerByReference mode_right = null;

	private Realsense2Library.rs2_extrinsics extrinsics = new Realsense2Library.rs2_extrinsics();

	private final List<IPoseCallback>         callbacks     = new ArrayList<IPoseCallback>();

	private final Planar<GrayU8> img     = new Planar<GrayU8>(GrayU8.class,WIDTH,HEIGHT,3);

	private long   tms0, tms;
	private int    fps;

	private boolean is_running      = false;
	private boolean reset_request   = false;
	private boolean enable_video    = true;

	private int x0,y0,x1,y1;
	private int mount;

	private CombineT265Thread worker = null;


	private final DMatrixRMaj   rtY90  = CommonOps_DDRM.identity( 3 );
	private final DMatrixRMaj   rtY90P = CommonOps_DDRM.identity( 3 );
	private final DMatrixRMaj   tmp    = CommonOps_DDRM.identity( 3 );

	public static StreamRealSenseT265Pose2_backup getInstance(int mount, int width, int height) {
		if(instance==null)
			instance = new StreamRealSenseT265Pose2_backup(mount,width,height);
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



	//private final T265FrameCallback cb = new T265FrameCallback();

	private  StreamRealSenseT265Pose2_backup(int mount, int width, int height) {

		super();

		this.x0 = WIDTH/2 - width/2;
		this.y0 = HEIGHT/2 - height/2;
		this.x1 = x0 + width;
		this.y1 = y0 + height;
		this.mount = mount;

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
		setOption(sensor,rs2_option.RS2_OPTION_ENABLE_MAP_PRESERVATION, "RS2_OPTION_ENABLE_MAP_PRESERVATION", false);
		setOption(sensor,rs2_option.RS2_OPTION_ENABLE_DYNAMIC_CALIBRATION, "RS2_OPTION_ENABLE_DYNAMIC_CALIBRATION", true);
		setOption(sensor,rs2_option.RS2_OPTION_ENABLE_RELOCALIZATION, "RS2_OPTION_ENABLE_RELOCALIZATION", false);


		setOption(sensor,rs2_option.RS2_OPTION_FRAMES_QUEUE_SIZE, "RS2_OPTION_FRAMES_QUEUE_SIZE", 3);

		// Callback setup: TODO: Not sure whether this works
		//		System.out.println("   -> Notification callback registered");
		//		rs2.rs2_set_notifications_callback(sensor,cb,null,error);

	}

	public StreamRealSenseT265Pose2_backup registerCallback(IPoseCallback callback) {
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

		pipeline = rs2.rs2_create_pipeline(ctx, error);

		config = rs2.rs2_create_config(error);
		rs2.rs2_config_enable_device(config, rs2.rs2_get_device_info(dev, rs2_camera_info.RS2_CAMERA_INFO_SERIAL_NUMBER, error),error);


		// TODO: Selection of streams does not work
		//					rs2.rs2_config_enable_stream(config, rs2_stream.RS2_STREAM_POSE, -1, 0, 0, rs2_format.RS2_FORMAT_6DOF, 0 , error);
		//					rs2.rs2_config_enable_stream(config, rs2_stream.RS2_STREAM_FISHEYE, 0, WIDTH, HEIGHT, rs2_format.RS2_FORMAT_Y8, 0, error);
		//					rs2.rs2_config_enable_stream(config, rs2_stream.RS2_STREAM_FISHEYE, 1, WIDTH, HEIGHT, rs2_format.RS2_FORMAT_Y8, 0, error);
		if(enable_video) {

		}

		//	

		is_running = true;

		worker = new CombineT265Thread();
		OdometryPool.submit(worker);


		//	rs2.rs2_pipeline_start_with_config_and_callback(pipeline, config, cb, null, error);


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

	public void reset(boolean enable_video_stream) {
		reset_request = true;
		enable_video = enable_video_stream;
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

	//	public class T265FrameCallback implements Realsense2Library.rs2_frame_callback_ptr {
	//
	//		private Se3_F64    current_pose         = new Se3_F64();
	//		private Se3_F64    current_speed        = new Se3_F64();
	//		private Se3_F64    current_acceleration = new Se3_F64();
	//
	//		private int num_of_frames = 0;
	//
	//		@Override
	//		public void apply(Pointer rs2_frame, Pointer voidPtr1) {
	//			rs2_frame frames =  new rs2_frame(rs2_frame);
	//			num_of_frames = rs2.rs2_embedded_frames_count(frames, error);
	//
	//			// Images as frameset)
	//			if(num_of_frames > 0) {
	//
	//				rs2_frame frame = rs2.rs2_extract_frame(frames, 0, error);
	//				tms = (long)rs2.rs2_get_frame_timestamp(frame,error);
	//
	//				if(0 != rs2.rs2_is_frame_extendable_to(frame, rs2_extension.RS2_EXTENSION_VIDEO_FRAME, error) ) {
	//					if(mode_left==null) {
	//						mode_left = rs2.rs2_get_frame_stream_profile(frame,error);
	//						rs2.rs2_get_video_stream_intrinsics(mode_left, intrinsics_left, error);
	//						left_model = createFisheyeModel(intrinsics_left);
	//					} else {
	//						bufferGrayToU8(rs2.rs2_get_frame_data(frame, error),img);
	//					}
	//					worker.trigger();
	//				}
	//				rs2.rs2_release_frame(frame);
	//
	//			} else {
	//
	//				if(0 != rs2.rs2_is_frame_extendable_to(frames, rs2_extension.RS2_EXTENSION_POSE_FRAME, error)) {
	//					rs2.rs2_pose_frame_get_pose_data(frames, rawpose, error);
	//				}
	//			}
	//
	//
	//			rs2.rs2_release_frame(frames);
	//		}  
	//	}

	//	private class CombineT265Thread extends Thread {
	//
	//		private Se3_F64    current_pose         = new Se3_F64();
	//		private Se3_F64    current_speed        = new Se3_F64();
	//		private Se3_F64    current_acceleration = new Se3_F64();
	//
	//		private float      dt_s = 0;
	//
	//		private boolean    transfer             = false;
	//
	//
	//		@Override
	//		public void run() {
	//			while(is_running) {
	//				try {
	//
	//					synchronized(this) {
	//						while(!transfer)
	//							this.wait();
	//						System.out.println(tms+":"+rawpose.translation);
	//						for(IPoseCallback callback : callbacks)
	//							callback.handle(tms, rawpose, current_pose,current_speed, current_acceleration, img.subimage(x0, y0, x1, y1));
	//					transfer = false;
	//				}
	//
	//
	//			} catch (InterruptedException e) {
	//				e.printStackTrace();
	//			}
	//
	//		}
	//	}
	//
	//	public synchronized void trigger() {
	//		transfer = true;
	//		notify();	
	//	}
	//}


	private class CombineT265Thread extends Thread {

		private Se3_F64    current_pose         = new Se3_F64();
		private Se3_F64    current_speed        = new Se3_F64();
		private Se3_F64    current_acceleration = new Se3_F64();

		private float      dt_s = 0;


		@Override
		public void run() {

			is_initialized = false;
			reset_request = false;

			//	try { Thread.sleep(300); } catch (InterruptedException e) {  }

			//			rs2_pipeline_profile profile = 


			try { Thread.sleep(200); } catch (InterruptedException e) {  }

			int num_of_frames = 0;
			boolean left = false;

			rs2_frame frame;
			
			rs2.rs2_pipeline_start_with_config(pipeline, config, error);

			System.out.println("T265 pipeline started. Video is enabled : "+enable_video);


			while(is_running) {

				try {

					rs2_frame frames = rs2.rs2_pipeline_wait_for_frames(pipeline, 15000, error);

					num_of_frames = rs2.rs2_embedded_frames_count(frames, error);
					tms = (long)rs2.rs2_get_frame_timestamp(frames,error);
					if(tms!=tms0)
						fps = (int)(1000.0f/(tms - tms0));
					tms0 = tms;

					left = false; 

					for(int i = 0; i < num_of_frames;i++) {

						frame = rs2.rs2_extract_frame(frames, i, error);

						if(0 != rs2.rs2_is_frame_extendable_to(frame, rs2_extension.RS2_EXTENSION_POSE_FRAME, error)) {
							rs2.rs2_pose_frame_get_pose_data(frame, rawpose, error);
						//	System.out.println(rawpose.translation);
						}

						if(0 != rs2.rs2_is_frame_extendable_to(frame, rs2_extension.RS2_EXTENSION_VIDEO_FRAME, error)) {
							if(!left) {
								left = true;
								if(mode_left==null) {
									mode_left = rs2.rs2_get_frame_stream_profile(frame,error);
									rs2.rs2_get_video_stream_intrinsics(mode_left, intrinsics_left, error);
									left_model = createFisheyeModel(intrinsics_left);
								} else {
									bufferGrayToU8(rs2.rs2_get_frame_data(frame, error),img);
								} 
							} else {
								if(mode_right==null) {
									mode_right = rs2.rs2_get_frame_stream_profile(frame,error);
									rs2.rs2_get_video_stream_intrinsics(mode_right, intrinsics_right, error);
									rs2.rs2_get_extrinsics(mode_left, mode_right, extrinsics, error);
									right_model = createFisheyeModel(intrinsics_right);
									is_initialized = true;
								}
							}
						} 

						rs2.rs2_release_frame(frame);
					}

					rs2.rs2_release_frame(frames);


					switch(mount) {

					case POS_FOREWARD:

						current_pose.getTranslation().setTo( - rawpose.translation.z, rawpose.translation.x, - rawpose.translation.y);
						ConvertRotation3D_F64.quaternionToMatrix(
								rawpose.rotation.w,
								-rawpose.rotation.z,
								rawpose.rotation.x,
								-rawpose.rotation.y, current_pose.getRotation());

						current_speed.getTranslation().setTo(- rawpose.velocity.z, rawpose.velocity.x, - rawpose.velocity.y);
						current_speed.getRotation().setTo(current_pose.getRotation());

						current_acceleration.getTranslation().setTo(- rawpose.acceleration.z, rawpose.acceleration.x, - rawpose.acceleration.y);

						break;

					case POS_DOWNWARD:

						current_pose.getTranslation().setTo( -rawpose.translation.z, rawpose.translation.x, - rawpose.translation.y);

						ConvertRotation3D_F64.quaternionToMatrix(
								rawpose.rotation.w,
								-rawpose.rotation.z,
								rawpose.rotation.x,
								-rawpose.rotation.y, tmp);

						CommonOps_DDRM.mult(tmp, rtY90 , current_pose.getRotation());

						current_speed.getTranslation().setTo( -rawpose.velocity.z, rawpose.velocity.x, - rawpose.velocity.y);
						current_speed.getRotation().setTo(current_pose.getRotation());

						current_acceleration.getTranslation().setTo(- rawpose.acceleration.z, rawpose.acceleration.x, - rawpose.acceleration.y);

						break;


					case POS_DOWNWARD_180:


						current_pose.getTranslation().setTo( rawpose.translation.z, -rawpose.translation.x, - rawpose.translation.y);

						ConvertRotation3D_F64.quaternionToMatrix(
								rawpose.rotation.w,
								rawpose.rotation.z,
								-rawpose.rotation.x,
								-rawpose.rotation.y, tmp);

						CommonOps_DDRM.mult(tmp, rtY90P , current_pose.getRotation());

						current_speed.getTranslation().setTo( rawpose.velocity.z, -rawpose.velocity.x, - rawpose.velocity.y);
						current_speed.getRotation().setTo(current_pose.getRotation());

						current_acceleration.getTranslation().setTo(rawpose.acceleration.z, -rawpose.acceleration.x, - rawpose.acceleration.y);

						break;

					case POS_DOWNWARD_180_PREDICT:

						dt_s = (getUnixTime_ms() - tms) / 1000f;

						if(dt_s > 0) {

							prepose.velocity.x = (dt_s/2 * rawpose.acceleration.x + rawpose.velocity.x);
							prepose.velocity.y = (dt_s/2 * rawpose.acceleration.y + rawpose.velocity.y);
							prepose.velocity.z = (dt_s/2 * rawpose.acceleration.z + rawpose.velocity.z);

							prepose.translation.x = dt_s * prepose.velocity.x + rawpose.translation.x;
							prepose.translation.y = dt_s * prepose.velocity.y + rawpose.translation.y;
							prepose.translation.z = dt_s * prepose.velocity.z + rawpose.translation.z;


							// TODO: Predict rotation also
							//						rs2_vector W = {
							//								dt_s * (dt_s/2 * rawpose.angular_acceleration.x + rawpose.angular_velocity.x),
							//								dt_s * (dt_s/2 * rawpose.angular_acceleration.y + rawpose.angular_velocity.y),
							//								dt_s * (dt_s/2 * rawpose.angular_acceleration.z + rawpose.angular_velocity.z),
							//						};
							//						P.rotation = quaternion_multiply(quaternion_exp(W), pose.rotation);

						}

						current_pose.getTranslation().setTo( prepose.translation.z, -prepose.translation.x, - prepose.translation.y);

						ConvertRotation3D_F64.quaternionToMatrix(
								rawpose.rotation.w,
								rawpose.rotation.z,
								-rawpose.rotation.x,
								-rawpose.rotation.y, tmp);

						CommonOps_DDRM.mult(tmp, rtY90P , current_pose.getRotation());

						current_speed.getTranslation().setTo( prepose.velocity.z, -prepose.velocity.x, - prepose.velocity.y);
						current_speed.getRotation().setTo(current_pose.getRotation());

						current_acceleration.getTranslation().setTo(rawpose.acceleration.z, -rawpose.acceleration.x, - rawpose.acceleration.y);

						break;
					}


					if(reset_request) {
						try {
							synchronized(this) {
								reset_request = false; tms0 = 0;
								rs2.rs2_pipeline_stop(pipeline, error);
								try { Thread.sleep(200); } catch (InterruptedException e) {  }
								//								if(!enable_video) {
								//									rs2.rs2_config_disable_stream(config, 1, error);
								//									rs2.rs2_config_disable_stream(config, 2, error);
								//								}
								rs2.rs2_pipeline_start_with_config(pipeline, config, error);
								checkError("Restart pipeline",error);
							}
						} catch(Exception e) { e.printStackTrace(); }
						continue;
					}

					if(is_initialized && tms > 0) {
						for(IPoseCallback callback : callbacks)
							if(enable_video)
								callback.handle(tms, rawpose, current_pose,current_speed, current_acceleration, img.subimage(x0, y0, x1, y1));
							else
								callback.handle(tms, rawpose, current_pose,current_speed, current_acceleration, null);
					}

				} catch(Exception e) {
					e.printStackTrace();
				}
			}
			rs2.rs2_pipeline_stop(pipeline, error);
			rs2.rs2_delete_pipeline(pipeline);
			rs2.rs2_delete_config(config);
			System.out.println("T265 stopped.");
		}
	}

	private void bufferGrayToU8(Pointer input , Planar<GrayU8> output ) {
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

	public long getUnixTime_ms() {
		Instant ins = Instant.now();
		long now_ns = ins.getEpochSecond() * 1000000000L + ins.getNano();
		return now_ns/1000000L;
	}

}
