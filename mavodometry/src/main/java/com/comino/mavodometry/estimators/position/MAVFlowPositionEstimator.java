package com.comino.mavodometry.estimators.position;

//refer to : https://github.com/PX4/PX4-Autopilot/blob/master/src/modules/local_position_estimator/sensors/flow.cpp


import com.comino.mavcom.control.IMAVMSPController;
import com.comino.mavcom.flow.MessageBus;
import com.comino.mavcom.flow.ModelSubscriber;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.Flow;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavodometry.estimators.MAVAbstractEstimator;

import georegression.geometry.GeometryMath_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;

public class MAVFlowPositionEstimator extends MAVAbstractEstimator  {
	
	private static final float FLOWSCALE = 1.0f;
	
	private Vector3D_F64     flow       = new Vector3D_F64();
	private Vector3D_F64     gyro       = new Vector3D_F64();
	private Vector3D_F64     vel_body   = new Vector3D_F64();
	private Vector3D_F64     vel_ned    = new Vector3D_F64();
	private Vector3D_F64     pos_ned    = new Vector3D_F64();
	
	private Se3_F64          to_ned     = new Se3_F64();
	
	private long tms = 0;
	
	private DataModel 	model;

	public MAVFlowPositionEstimator(IMAVMSPController control) {
		super(control);
		this.model = control.getCurrentModel();
	
		
		ModelSubscriber<Flow> flow_subscriber = new ModelSubscriber<Flow>(Flow.class,(n) -> {
			if(n.fq > 0)
			   estimate();
		});

		MessageBus.getInstance().subscribe(flow_subscriber);
		
	}
	
	public boolean estimate() {
		
		float dt_int = model.flow.ius * 1e-6f;
		float dt_sec = ( System.currentTimeMillis() - tms) / 1000f;
		tms = System.currentTimeMillis();
		
		if(dt_int > 0.5 || dt_int < 1e-6f || dt_sec > 0.5)
			return false;
		
		if (Math.abs(model.attitude.p) > 0.5f || Math.abs(model.attitude.r) > 0.5f)
			return false;
		
		if(!model.sys.isStatus(Status.MSP_ARMED)) {
			reset();
			return false;
		}
		
		MSP3DUtils.convertModelRotationToSe3_F64(model, to_ned);
		
		flow.setTo(model.flow.fX * FLOWSCALE, model.flow.fY * FLOWSCALE,0);
		gyro.setTo(model.flow.fgX,model.flow.fgY,0);
		
		float d = model.hud.ar * (float)Math.cos(model.attitude.p) * (float)Math.cos(model.attitude.r);
		
		vel_body.setTo( ( (flow.y - gyro.y) * d ) / dt_int,
				      (-(flow.x - gyro.x) * d ) / dt_int,
				      0);
		
		GeometryMath_F64.mult(to_ned.R, vel_body,vel_ned);
		
		vel_ned.scale(dt_sec);
		pos_ned.plusIP(vel_ned);
		
		model.vision.gx = (float)pos_ned.x;
		model.vision.gy = (float)pos_ned.y;
		model.vision.gz = 0f;
		
		return true;
		
	}
	
	public void reset() {
		pos_ned.setTo(model.state.l_x, model.state.l_y,model.state.l_z);
		vel_ned.setTo(model.state.l_vx, model.state.l_vy,model.state.l_vz);
		tms = 0;
	}
	
	public Vector3D_F64 getVeocity() {
		return vel_ned;
	}
	
	public Vector3D_F64 getPosition() {
		return pos_ned;
	}

	@Override
	public void enableStream(boolean enable) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void start() throws Exception {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void stop() {
		// TODO Auto-generated method stub
		
	}

}
