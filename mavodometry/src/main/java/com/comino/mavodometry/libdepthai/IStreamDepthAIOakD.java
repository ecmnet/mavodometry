package com.comino.mavodometry.libdepthai;

import com.comino.mavodometry.callback.IDepthCallback;

import boofcv.struct.calib.CameraPinholeBrown;

public interface IStreamDepthAIOakD {

	IStreamDepthAIOakD registerCallback(IDepthCallback listener);

	void start() throws Exception;

	void stop();

	void setRGBMode(boolean rgb);

	long getFrameCount();

	long getRGBTms();

	long getDepthTms();

	boolean isRunning();

	CameraPinholeBrown getIntrinsics();

}