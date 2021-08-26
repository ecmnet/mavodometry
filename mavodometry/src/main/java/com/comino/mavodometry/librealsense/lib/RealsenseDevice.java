package com.comino.mavodometry.librealsense.lib;

import com.comino.mavodometry.librealsense.lib.Realsense2Library.rs2_camera_info;
import com.comino.mavodometry.librealsense.lib.Realsense2Library.rs2_option;
import com.sun.jna.ptr.PointerByReference;

public class RealsenseDevice  {

	public static Realsense2Library rs2 = Realsense2Library.INSTANCE;
	
	public static final float OPTION_ENABLE  = 1.0f;
	public static final float OPTION_DISABLE = 0.0f;

	public static Realsense2Library.rs2_context ctx = null;

	protected static PointerByReference error= new PointerByReference();
	protected static Realsense2Library.rs2_device_list device_list;
	protected static Realsense2Library.rs2_device[] devices;
	protected static int dev_count = 0;
	
	protected boolean is_initialized = false;

	public RealsenseDevice() {
		
		is_initialized = false;

		if(ctx == null) {

			System.out.println("Using realsense device driver");

			ctx = rs2.rs2_create_context(Realsense2Library.RS2_API_VERSION, error);
			
			device_list = rs2.rs2_query_devices(ctx,error);
			
			dev_count = rs2.rs2_get_device_count(device_list, error);
			if(dev_count < 1) {
				throw new IllegalArgumentException("No realsense device found");
			}
			System.out.println(dev_count+" devices found");
			devices = new Realsense2Library.rs2_device[dev_count];
			for(int i=0;i<dev_count;i++) {
				devices[i] = rs2.rs2_create_device(device_list, i, error);
				System.out.println("-> "+rs2.rs2_get_device_info(devices[i], rs2_camera_info.RS2_CAMERA_INFO_NAME, error).getString(0));
			}
		}
	}
	
	public boolean isInitialized() {
		return is_initialized;
	}

	protected Realsense2Library.rs2_device getDeviceByName(String name) {

		for(int i=0;i<dev_count;i++) {
			if(rs2.rs2_get_device_info(devices[i], rs2_camera_info.RS2_CAMERA_INFO_NAME, error).getString(0).contains(name)) {
				return devices[i];
			}
		}
		return null;
	}

	protected boolean checkError(String t, PointerByReference error) {
		//		Pointer s = Realsense2Library.INSTANCE.rs2_get_error_message(error);
		//		if(s!=null) {
		//			System.out.println("Error at "+t);
		//			return true;
		//		}
		return true;
	}
	
	protected boolean setOption(PointerByReference sensor, int option, String option_name, boolean enable) {
		if(enable)
		  rs2.rs2_set_option(sensor, option, OPTION_ENABLE, error);
		else
		  rs2.rs2_set_option(sensor, option, OPTION_DISABLE, error);
		float enabled = rs2.rs2_get_option(sensor, option, error);
		if(enabled!=0) {
			System.out.println("  -> Option "+option_name+" is enabled");
			return true;
		}
		System.out.println("  -> Option "+option_name+" is not enabled");
		return false;	
	}
	
	protected boolean setOption(PointerByReference sensor, int option, String option_name, int value) {
		  rs2.rs2_set_option(sensor, option, value, error);
		float value_r = rs2.rs2_get_option(sensor, option, error);
		if(value == value_r) {
			System.out.println("  -> Option "+option_name+" set to "+value);
			return true;
		}
		return false;
			
	}
	

}
