package com.comino.mavodometry.librealsense.t265.boofcv;

import com.comino.mavodometry.librealsense.t265.wrapper.Realsense2Library;

public interface IPoseCallback {

	public void handle(Realsense2Library.rs2_pose pose);

}
