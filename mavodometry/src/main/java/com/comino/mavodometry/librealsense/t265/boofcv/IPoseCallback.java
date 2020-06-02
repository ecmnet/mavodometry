package com.comino.mavodometry.librealsense.t265.boofcv;

import com.comino.mavodometry.librealsense.t265.wrapper.Realsense2Library;

import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import georegression.struct.se.Se3_F64;

public interface IPoseCallback {

	public void handle(long tms, Realsense2Library.rs2_pose rawpose , Se3_F64 pose, Se3_F64 speed, Se3_F64 acc, Planar<GrayU8> img);

}
