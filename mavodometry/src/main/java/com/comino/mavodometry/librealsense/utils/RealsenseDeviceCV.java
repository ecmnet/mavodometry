package com.comino.mavodometry.librealsense.utils;

import static org.bytedeco.librealsense2.global.realsense2.RS2_API_VERSION;
import static org.bytedeco.librealsense2.global.realsense2.rs2_create_context;
import static org.bytedeco.librealsense2.global.realsense2.rs2_create_device;
import static org.bytedeco.librealsense2.global.realsense2.rs2_create_sensor;
import static org.bytedeco.librealsense2.global.realsense2.rs2_get_device_count;
import static org.bytedeco.librealsense2.global.realsense2.rs2_get_device_info;
import static org.bytedeco.librealsense2.global.realsense2.rs2_get_error_message;
import static org.bytedeco.librealsense2.global.realsense2.rs2_get_failed_args;
import static org.bytedeco.librealsense2.global.realsense2.rs2_get_failed_function;
import static org.bytedeco.librealsense2.global.realsense2.rs2_get_option;
import static org.bytedeco.librealsense2.global.realsense2.rs2_get_sensor_info;
import static org.bytedeco.librealsense2.global.realsense2.rs2_get_sensors_count;
import static org.bytedeco.librealsense2.global.realsense2.rs2_get_frame_metadata;
import static org.bytedeco.librealsense2.global.realsense2.rs2_supports_frame_metadata;
import static org.bytedeco.librealsense2.global.realsense2.rs2_hardware_reset;
import static org.bytedeco.librealsense2.global.realsense2.rs2_query_devices;
import static org.bytedeco.librealsense2.global.realsense2.rs2_query_sensors;
import static org.bytedeco.librealsense2.global.realsense2.rs2_set_option;
import static org.bytedeco.librealsense2.global.realsense2.rs2_supports_device_info;
import static org.bytedeco.librealsense2.global.realsense2.rs2_supports_option;

import org.bytedeco.javacpp.Loader;
import org.bytedeco.javacv.FrameGrabber;
import org.bytedeco.javacv.FrameGrabber.Exception;
import org.bytedeco.javacv.RealSense2FrameGrabber;
import org.bytedeco.librealsense2.rs2_context;
import org.bytedeco.librealsense2.rs2_device;
import org.bytedeco.librealsense2.rs2_device_list;
import org.bytedeco.librealsense2.rs2_error;
import org.bytedeco.librealsense2.rs2_frame;
import org.bytedeco.librealsense2.rs2_options;
import org.bytedeco.librealsense2.rs2_sensor;
import org.bytedeco.librealsense2.rs2_sensor_list;
import org.bytedeco.librealsense2.global.realsense2;


public class RealsenseDeviceCV  {


	public static final float OPTION_ENABLE  = 1.0f;
	public static final float OPTION_DISABLE = 0.0f;

	private static FrameGrabber.Exception loadingException = null;

	public static void tryLoad() throws FrameGrabber.Exception {
		if (loadingException != null) {
			loadingException.printStackTrace();
			throw loadingException;
		} else {
			try {
				Loader.load(org.bytedeco.librealsense2.presets.realsense2.class);
			} catch (Throwable t) {
				throw loadingException = new FrameGrabber.Exception("Failed to load " + RealSense2FrameGrabber.class, t);
			}
		}
	}

	protected static rs2_error error = new rs2_error();
	protected static rs2_context ctx;
	protected static rs2_device_list device_list;
	protected static rs2_device[] devices;

	protected static int dev_count = 0;


	protected boolean is_initialized = false;
	

	public RealsenseDeviceCV() throws Exception {

		is_initialized = false;

		if(ctx == null) {

			System.out.println("Using JAVACV Realsense2 device driver version "+RS2_API_VERSION);

			ctx = rs2_create_context(RS2_API_VERSION, error);
			
			device_list = rs2_query_devices(ctx,error);
			dev_count = rs2_get_device_count(device_list, error);
			
			if(dev_count < 1) {
				throw new IllegalArgumentException("No realsense device found");
			}
			System.out.println(dev_count+" devices found");
			devices = new rs2_device[dev_count];
			for(int i=0;i<dev_count;i++) {
				devices[i] = createDevice(i);
				System.out.println("-> "+getDeviceInfo(devices[i], realsense2.RS2_CAMERA_INFO_NAME)+ 
						" (USB type "+ getDeviceInfo(devices[i], realsense2.RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR)+")");
			}
		}

	}

	public boolean isInitialized() {
		return is_initialized;
	}

	protected rs2_device getDeviceByName(String name) throws Exception {
		for(int i=0;i<dev_count;i++) {
			if(rs2_get_device_info(devices[i], realsense2.RS2_CAMERA_INFO_NAME, error).getString().contains(name)) {
				return devices[i];
			}
		}
		return null;
	}

	public void setSensorOption(rs2_sensor sensor, int optionIndex, boolean value) throws Exception {
		setSensorOption(sensor, optionIndex, value ? 1f : 0f);
	}

	public void setSensorOption(rs2_sensor sensor, int optionIndex, float value) throws Exception {
		rs2_options options = new rs2_options(sensor);
		boolean isSupported = toBoolean(rs2_supports_option(options, optionIndex, error));
		checkError(error);

		if (!isSupported) {
			throw new Exception("Option " + optionIndex + " is not supported!");
		}

		rs2_set_option(options, optionIndex, value, error);
		checkError(error);
		
		options.releaseReference();
	}

	public float getSensorOption(rs2_sensor sensor, int optionIndex)  throws Exception {
		rs2_options options = new rs2_options(sensor);
		boolean isSupported = toBoolean(rs2_supports_option(options, optionIndex, error));
		checkError(error);

		if (!isSupported) {
			throw new Exception("Option " + optionIndex + " is not supported!");
		}

		float val = rs2_get_option(options, optionIndex, error);
		checkError(error);
		
		options.releaseReference();

		return val;
	}

	protected static void checkError(rs2_error e) throws Exception {
		if (!e.isNull()) {
			System.err.println("LibRealsense2 error in: "+rs2_get_failed_function(e).getString()+": "+rs2_get_error_message(e).getString());
			throw new Exception(String.format("rs_error was raised when calling %s(%s):\n%s\n",
					rs2_get_failed_function(e).getString(),
					rs2_get_failed_args(e).getString(),
					rs2_get_error_message(e).getString()));
		}
	}

	protected rs2_context createContext() throws Exception {
		rs2_context context = rs2_create_context(RS2_API_VERSION, error);
		checkError(error);
		return context;
	}

	protected rs2_device_list createDeviceList() throws Exception {
		rs2_device_list deviceList = rs2_query_devices(ctx, error);
		checkError(error);
		return deviceList;
	}

	protected int getDeviceCount() throws Exception {
		int count = rs2_get_device_count(device_list, error);
		checkError(error);
		return count;
	}
	protected rs2_device createDevice(int index) throws Exception {
		rs2_device device = rs2_create_device(device_list, index, error);
		checkError(error);
		return device;
	}

	protected void hardwareReset(rs2_device device) throws Exception {
		rs2_hardware_reset(device, error);
		checkError(error);
	}

	protected rs2_sensor_list getSensorList(rs2_device device)  throws Exception {
		rs2_sensor_list sensors = rs2_query_sensors(device, error);
		checkError(error);
		return sensors;
	}

	protected int getSensorCount(rs2_sensor_list sensors)  throws Exception {
		int count = rs2_get_sensors_count(sensors, error);
		checkError(error);
		return count;
	}

	protected rs2_sensor createSensor(rs2_sensor_list sensors, int num)  throws Exception {
		rs2_sensor sensor = rs2_create_sensor(sensors, num, error);
		checkError(error);
		return sensor;
	}

	protected String getDeviceInfo(rs2_device device, int info) throws Exception {
		// check if info is supported
	//	rs2_error error = new rs2_error();
		boolean isSupported = toBoolean(rs2_supports_device_info(device, info, error));
		checkError(error);

		if (!isSupported)
			return null;

		// read device info
		String infoText = rs2_get_device_info(device, info, error).getString();
		checkError(error);

		return infoText;
	}

	protected String getSensorInfo(rs2_sensor sensor, int num) throws Exception {
		// check if info is supported
	//	rs2_error error = new rs2_error();

		// read device info
		String infoText = rs2_get_sensor_info(sensor, num, error).getString();
		checkError(error);

		return infoText;
	}
	
	protected long getFrameMetaData(rs2_frame frame, int type) throws Exception {
//		if(!toBoolean(rs2_supports_frame_metadata(frame,type,error)))
//			return -1;
		long result = rs2_get_frame_metadata(frame,type,error);
		checkError(error);
		return result;	
	}

	protected static boolean toBoolean(int value) {
		return value >= 1;
	}



}
