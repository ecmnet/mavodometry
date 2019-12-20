package com.comino.mavodometry.librealsense.t265.boofcv;

import com.comino.mavodometry.concurrency.OdometryPool;
import com.comino.mavodometry.librealsense.t265.wrapper.Realsense2Library;
import com.sun.jna.ptr.PointerByReference;

import com.comino.mavodometry.librealsense.t265.wrapper.Realsense2Library.rs2_camera_info;

public class StreamRealSenseT265Pose {

	private Realsense2Library.rs2_context ctx;
	private Realsense2Library.rs2_pose rawpose = new Realsense2Library.rs2_pose();
	private IPoseCallback callback;

	private double tms0, tms;
	private int    fps;

	private boolean is_running;

	public StreamRealSenseT265Pose(IPoseCallback callback) {

		this.callback = callback;
		PointerByReference error = new PointerByReference();

		ctx = Realsense2Library.INSTANCE.rs2_create_context(Realsense2Library.RS2_API_VERSION, error);
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

	public int getFrameRate() {
		return fps;
	}

	public void printDeviceInfo() {

		PointerByReference error = new PointerByReference();

		Realsense2Library.rs2_device_list device_list = Realsense2Library.INSTANCE.rs2_query_devices(ctx,
				error);

		Realsense2Library.rs2_device dev = Realsense2Library.INSTANCE.rs2_create_device(device_list, 0, error);

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

		PointerByReference error = new PointerByReference();
		Realsense2Library.rs2_frame frame;

		@Override
		public void run() {

			Realsense2Library.rs2_device_list device_list = Realsense2Library.INSTANCE.rs2_query_devices(ctx,error);
			int dev_count = Realsense2Library.INSTANCE.rs2_get_device_count(device_list, error);
			if(dev_count < 1) {
				is_running = false;
				System.out.println("No device found for Pose estimation");
				return;
			}

			Realsense2Library.rs2_pipeline pipeline = Realsense2Library.INSTANCE.rs2_create_pipeline(ctx, error);
			Realsense2Library.rs2_config config = Realsense2Library.INSTANCE.rs2_create_config(error);
			Realsense2Library.INSTANCE.rs2_pipeline_start_with_config(pipeline, config, error);

			while(is_running) {

				Realsense2Library.rs2_frame frames = Realsense2Library.INSTANCE.rs2_pipeline_wait_for_frames(pipeline, 500, error);
				frame = Realsense2Library.INSTANCE.rs2_extract_frame(frames, 4, error);
				Realsense2Library.INSTANCE.rs2_pose_frame_get_pose_data(frame, rawpose, error);
				tms = Realsense2Library.INSTANCE.rs2_get_frame_timestamp(frame,error);
				fps = (int)(1000.0f/(tms - tms0));
				tms0 = tms;
				Realsense2Library.INSTANCE.rs2_release_frame(frame);
				Realsense2Library.INSTANCE.rs2_release_frame(frames);

				callback.handle(rawpose);

			}
			Realsense2Library.INSTANCE.rs2_pipeline_stop(pipeline, error);

		}


	}

}
