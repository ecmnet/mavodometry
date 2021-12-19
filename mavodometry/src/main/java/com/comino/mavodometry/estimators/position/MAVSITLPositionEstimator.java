package com.comino.mavodometry.estimators.position;

import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.MSP_CMD;
import org.mavlink.messages.MSP_COMPONENT_CTRL;
import org.mavlink.messages.lquac.msg_msp_command;
import org.mavlink.messages.lquac.msg_msp_vision;
import org.mavlink.messages.lquac.msg_odometry;
import org.mavlink.messages.lquac.msg_set_gps_global_origin;

import com.comino.mavcom.config.MSPParams;
import com.comino.mavcom.control.IMAVMSPController;
import com.comino.mavcom.log.MSPLogger;
import com.comino.mavcom.mavlink.IMAVLinkListener;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.model.segment.Vision;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavodometry.estimators.MAVAbstractEstimator;

import georegression.geometry.GeometryMath_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;

// Note: Requires odomtery enabled in SDF <send_odometry>1</send_odometry>

public class MAVSITLPositionEstimator extends MAVAbstractEstimator implements IMAVLinkListener  {


	private final DataModel 	     model;
	private final Vector3D_F64       vel_ned    = new Vector3D_F64();
	private final Vector3D_F64       vel_body   = new Vector3D_F64();
	private final Se3_F64            to_ned     = new Se3_F64();

	private final msg_msp_vision     msg        = new msg_msp_vision(2,1);
	private boolean                  is_running = false;

	private long                     tms = 0;

	public MAVSITLPositionEstimator(IMAVMSPController control) {
		super(control);
		this.model = control.getCurrentModel();
		control.addMAVLinkListener(this);
	}

	@Override
	public void received(Object o) {

		if( !(o instanceof msg_odometry))
			return;

		if(!is_running) {
			model.vision.setStatus(Vision.ENABLED, true);
			model.sys.setSensor(Status.MSP_OPCV_AVAILABILITY, true);
			model.vision.setStatus(Vision.POS_VALID, true);
			model.vision.setStatus(Vision.SPEED_VALID, true);
			model.vision.setStatus(Vision.PUBLISHED, true);

			System.out.println("SITL Vision PositionEstimator enabled");
			is_running = true;
		}

		// Just send msg_odometry as msp_vision message
		msg_odometry odometry = (msg_odometry)o;

		msg.x =  odometry.x;
		msg.y =  odometry.y;
		msg.z =  odometry.z;

		MSP3DUtils.convertModelToSe3_F64(model, to_ned);
		vel_body.setTo(odometry.vx,odometry.vy,odometry.vz);
		GeometryMath_F64.mult(to_ned.R, vel_body,vel_ned);

		msg.vx = (float)vel_ned.x;
		msg.vy = (float)vel_ned.y;
		msg.vz = (float)vel_ned.z;

		msg.p  = model.attitude.p;
		msg.r  = model.attitude.r;
		msg.h  = model.attitude.y;

		msg.quality = 100;
		msg.errors  = 0;
		msg.tms     = odometry.time_usec;
		msg.flags   = model.vision.flags;

		msg.fps     = 1000f / (System.currentTimeMillis() - tms);
		tms = System.currentTimeMillis();

		control.sendMAVLinkMessage(msg);

	}




	public void reset() {

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
