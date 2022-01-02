package com.comino.mavodometry.estimators.drift;

import com.comino.mavcom.utils.MSP3DComplementaryFilter;

import georegression.struct.point.Vector3D_F64;

public class MAVDriftSpeedEstimator {
	
	private final MSP3DComplementaryFilter vel_pos_filtered;  // filtered velocities based on position
	private final MSP3DComplementaryFilter vel_vel_filtered;  // filtered velocities based on velocities
	
	private final Vector3D_F64 drift = new Vector3D_F64();

	private final double max_valid_speed_sq;
	private final int    min_count;
	
	public MAVDriftSpeedEstimator(double max_valid_speed, double factor, int min_valid_counts) {
		this.max_valid_speed_sq = max_valid_speed * max_valid_speed;
		this.min_count          = min_valid_counts;
		this.vel_pos_filtered   = new MSP3DComplementaryFilter(factor);
		this.vel_vel_filtered   = new MSP3DComplementaryFilter(factor);
	}
	
	public boolean estimate(Vector3D_F64 vel_vel, Vector3D_F64 vel_pos) {
		
		if(vel_vel.normSq() < max_valid_speed_sq && vel_pos.normSq() < max_valid_speed_sq ) {
			vel_vel_filtered.add(vel_vel);
			vel_pos_filtered.add(vel_pos);
		}
		
		if( vel_vel_filtered.getCount() > min_count ) {
			drift.x = vel_pos_filtered.getFiltered().x - vel_vel_filtered.getFiltered().x;
			drift.y = vel_pos_filtered.getFiltered().y - vel_vel_filtered.getFiltered().y;
			drift.z = vel_pos_filtered.getFiltered().z - vel_vel_filtered.getFiltered().z;
			return true;
		}		
		return false;	
	}
	
	public Vector3D_F64 get() {
		return drift;
	}
	
	public void reset() {
		vel_vel_filtered.clear();
		vel_pos_filtered.clear();
		drift.setTo(0,0,0);
	}

}
