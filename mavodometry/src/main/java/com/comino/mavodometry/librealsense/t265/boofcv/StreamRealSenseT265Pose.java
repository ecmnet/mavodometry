package com.comino.mavodometry.librealsense.t265.boofcv;

import com.comino.mavodometry.concurrency.OdometryPool;
import com.comino.mavodometry.librealsense.t265.wrapper.Realsense2Library;
import com.comino.mavodometry.librealsense.t265.wrapper.Realsense2Library.rs2_camera_info;
import com.sun.jna.Pointer;
import com.sun.jna.ptr.PointerByReference;

import boofcv.struct.calib.CameraUniversalOmni;
import boofcv.struct.image.GrayU8;
import georegression.geometry.ConvertRotation3D_F32;
import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.GeoTuple3D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F32;
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

	private GrayU8 left  = new GrayU8(WIDTH,HEIGHT);
	private GrayU8 right = new GrayU8(WIDTH,HEIGHT);

	private long   tms0, tms;
	private int    fps;

	private boolean is_running;
	private boolean is_initialized;
	private boolean reset_request;

	private int x0,y0,x1,y1;
	private int mount;


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

		device_list = Realsense2Library.INSTANCE.rs2_query_devices(ctx,error);
		int dev_count = Realsense2Library.INSTANCE.rs2_get_device_count(device_list, error);
		if(dev_count < 1) {
			is_running = false;
			System.out.println("No device found for Pose estimation");
			return;
		}

		dev = Realsense2Library.INSTANCE.rs2_create_device(device_list, 0, error);

		//		Realsense2Library.INSTANCE.rs2_get_stream_profiles_count(dev, error);

		pipeline = Realsense2Library.INSTANCE.rs2_create_pipeline(ctx, error);
		config = Realsense2Library.INSTANCE.rs2_create_config(error);



		//	Realsense2Library.INSTANCE.rs2_get_video_stream_intrinsics(null, intrinsics, error);
	}

	public void start() {
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

	}

	public boolean isRunning() {
		return is_running;
	}

	private class CombineThread extends Thread {

		private Se3_F64    current_pose  = new Se3_F64();
		private Se3_F64    current_speed = new Se3_F64();

		private Realsense2Library.rs2_frame  frame;

		@Override
		public void run() {

			is_initialized = false;
			reset_request = false;
			Realsense2Library.INSTANCE.rs2_pipeline_start_with_config(pipeline, config, error);

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
					bufferGrayToU8(Realsense2Library.INSTANCE.rs2_get_frame_data(frame, error),left);

					if(mode_left==null) {
						mode_left = Realsense2Library.INSTANCE.rs2_get_frame_stream_profile(frame,error);
						Realsense2Library.INSTANCE.rs2_get_video_stream_intrinsics(mode_left, intrinsics_left, error);
						left_model = createFisheyeModel(intrinsics_left);
					}

				}
				Realsense2Library.INSTANCE.rs2_release_frame(frame);

				frame = Realsense2Library.INSTANCE.rs2_extract_frame(frames, 1, error);
				if(Realsense2Library.INSTANCE.rs2_get_frame_data_size(frame, error) > 0) {
					bufferGrayToU8(Realsense2Library.INSTANCE.rs2_get_frame_data(frame, error),right);
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

					break;
				case POS_DOWNWARD:
					// Note: Initialization of device fails: https://github.com/IntelRealSense/librealsense/issues/4080
					current_pose.getTranslation().set( rawpose.translation.x, -rawpose.translation.z, - rawpose.translation.y);

					break;
				}

				if(is_initialized)
					callback.handle(tms, rawpose, current_pose,current_speed,
							left.subimage(x0, y0, x1, y1),
							right.subimage(x0, y0, x1, y1));


				if(reset_request) {
					Realsense2Library.INSTANCE.rs2_pipeline_stop(pipeline, error);
					try { Thread.sleep(10); } catch (InterruptedException e) { }
					Realsense2Library.INSTANCE.rs2_pipeline_start_with_config(pipeline, config, error);
					reset_request = false;
				}

			}
			Realsense2Library.INSTANCE.rs2_pipeline_stop(pipeline, error);
		}
	}

	private void bufferGrayToU8(Pointer input , GrayU8 output ) {
		output.data = input.getByteArray(0, 678400);
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

		// TODO: Optimize mirrorOffset
		omni.mirrorOffset = 0.5;

		for(int i = 0; i< in.coeffs.length;i++)
			omni.radial[i] = in.coeffs[i];

		return omni;

	}

}
