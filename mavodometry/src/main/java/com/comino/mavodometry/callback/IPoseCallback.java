package com.comino.mavodometry.callback;


import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import georegression.struct.se.Se3_F64;

public interface IPoseCallback {

	public void handle(long tms, int tracker_confidence , Se3_F64 pose, Se3_F64 speed, Se3_F64 acc, Planar<GrayU8> img);

}
