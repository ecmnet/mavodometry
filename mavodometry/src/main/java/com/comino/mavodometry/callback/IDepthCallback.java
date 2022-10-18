package com.comino.mavodometry.callback;

import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;

public interface  IDepthCallback<T> {
	
	public void process(Planar<GrayU8> rgb, GrayU16 depth, T detections, long timeRgb, long timeDepth);

}
