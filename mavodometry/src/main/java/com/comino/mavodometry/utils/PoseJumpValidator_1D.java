package com.comino.mavodometry.utils;

public class PoseJumpValidator_1D {

	private float a = 0;

	private float d1 = 0;

	private float threshold = 0;

	private boolean is_initialized = false;

	public PoseJumpValidator_1D(float threshold) {
		this.threshold = threshold;
	}
	
	public PoseJumpValidator_1D(double threshold) {
		this.threshold = (float)threshold;
	}
	
	public boolean isJump(double a0, float dt_sec) {
		return isJump((float) a0, dt_sec);
	}

	public boolean isJump(float a0, float dt_sec) {
		if(is_initialized && dt_sec < 1f && dt_sec > 0.010f) {
			d1 = Math.abs(a - a0) / dt_sec;
			a = a0; 
			return d1 > threshold;
		} else {
			a = a0;
			is_initialized = true;
			return false;
		}
	}
	
	
	public String toString() {
		return "Diff="+d1;
	}

	public void reset() {
		is_initialized = false;
	}


}
