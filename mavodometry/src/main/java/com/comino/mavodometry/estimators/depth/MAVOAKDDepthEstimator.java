package com.comino.mavodometry.estimators.depth;


import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.control.IMAVMSPController;
import com.comino.mavodometry.estimators.MAVAbstractEstimator;
import com.comino.mavodometry.video.IVisualStreamHandler;

import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;

public class MAVOAKDDepthEstimator extends MAVAbstractEstimator  {

	private boolean is_running = false;

	private IVisualStreamHandler<Planar<GrayU8>> stream;


	private boolean enableStream;

	private final int width;
	private final int height;


	public <T> MAVOAKDDepthEstimator(IMAVMSPController control,  MSPConfig config, int width, int height, int mode, IVisualStreamHandler<Planar<GrayU8>> stream) {
		super(control);
		this.stream = stream;
		this.width  = width;
		this.height = height;
	}

	// Main Image processing just returning quality
	private int processImage(GrayU8 gray, Planar<GrayU8> color, long tms) {
       return 100;
	}

	public void start() {

		
	}

	public void stop() {
		is_running = false;
	}

	public void enableStream(boolean enable) {
		this.enableStream = enable;
	}


	


}
