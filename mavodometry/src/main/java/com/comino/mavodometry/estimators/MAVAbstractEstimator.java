package com.comino.mavodometry.estimators;

import com.comino.mavcom.control.IMAVMSPController;
import com.comino.mavcom.core.ControlModule;

public abstract class MAVAbstractEstimator extends ControlModule {
	
	public static final  int CONFIDENCE_FAILED = 0;
	public static final  int CONFIDENCE_LOW    = 1;
	public static final  int CONFIDENCE_MEDIUM = 2;
	public static final  int CONFIDENCE_HIGH   = 3;

	public MAVAbstractEstimator(IMAVMSPController control) {
		super(control);
	}
	
	public abstract void enableStream(boolean enable);

	public abstract void start() throws Exception;
	
	public abstract void stop();
	
}
