package com.comino.mavodometry.librealsense.t265.boofcv;

import org.bytedeco.librealsense2.rs2_pose;

import com.comino.mavodometry.librealsense.lib.Realsense2Library;

import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import georegression.struct.se.Se3_F64;

public interface IPoseCallback {

	public void handle(long tms, rs2_pose rawpose , Se3_F64 pose, Se3_F64 speed, Se3_F64 acc, Planar<GrayU8> img);

}
