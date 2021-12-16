package com.comino.mavodometry.estimators;

import com.comino.mavcom.control.IMAVMSPController;
import com.comino.mavcom.core.ControlModule;

public abstract class MAVAbstractEstimator extends ControlModule {

	public MAVAbstractEstimator(IMAVMSPController control) {
		super(control);
	}
	
	public abstract void enableStream(boolean enable);

	public abstract void start() throws Exception;
	
	public abstract void stop();
}
