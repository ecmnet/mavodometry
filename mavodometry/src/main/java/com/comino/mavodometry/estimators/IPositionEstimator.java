package com.comino.mavodometry.estimators;

import com.comino.mavodometry.librealsense.r200.vio.odometry.MAVDepthVisualOdometry;
import com.comino.mavodometry.video.IVisualStreamHandler;

import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;

public interface IPositionEstimator {

	void registerDetector(IMapper detector);

	void registerStreams(IVisualStreamHandler stream);

	void start();

	void stop();

	boolean isRunning();

	void reset();

	void enableDetectors( boolean enable);

	MAVDepthVisualOdometry<GrayU8,GrayU16> getOdometry();

}