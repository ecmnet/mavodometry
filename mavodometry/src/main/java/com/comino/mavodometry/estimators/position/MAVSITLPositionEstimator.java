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
import com.comino.mavcom.utils.MSP3DComplementaryFilter;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavodometry.estimators.MAVAbstractEstimator;
import com.comino.mavodometry.estimators.drift.MAVDriftSpeedEstimator;
import com.comino.mavodometry.utils.TimeHysteris;
import com.comino.mavutils.workqueue.WorkQueue;

import georegression.geometry.GeometryMath_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;

// Note: Requires odomtery enabled in SDF <send_odometry>1</send_odometry>

public class MAVSITLPositionEstimator extends MAVAbstractEstimator implements IMAVLinkListener  {

	private final WorkQueue wq = WorkQueue.getInstance();

	private final DataModel 	     model;
	private final Vector3D_F64       pos_ned    = new Vector3D_F64();
	
	private final Vector3D_F64       vel_ned    = new Vector3D_F64();
	private final Vector3D_F64       vel_body   = new Vector3D_F64();
	
	private final Vector3D_F64       vpo_body   = new Vector3D_F64();
	private final Se3_F64            to_ned     = new Se3_F64();

	private final msg_msp_vision     msg        = new msg_msp_vision(2,1);
	private boolean                  is_running = false;

	private long                     tms = 0;

	

	public MAVSITLPositionEstimator(IMAVMSPController control) {
		super(control);
		this.model = control.getCurrentModel();
		control.addMAVLinkListener(this);

		control.sendMAVLinkMessage(msg);

		control.registerListener(msg_msp_command.class, new IMAVLinkListener() {
			@Override
			public void received(Object o) {
				msg_msp_command cmd = (msg_msp_command)o;
				switch(cmd.command) {
				case MSP_CMD.MSP_CMD_SET_HOMEPOS:
					setGlobalOrigin(cmd.param1 / 1e7f, cmd.param2 / 1e7f, cmd.param3 / 1e3f );
					break;
				}
			}
		});


		wq.addCyclicTask("LP", 30, () -> {

			if(is_running) {
				msg.fps     = 1000f / (System.currentTimeMillis() - tms);
				tms = System.currentTimeMillis();
			
				
				// Convert body velocities to NED
				MSP3DUtils.convertModelToSe3_F64(model, to_ned);
				GeometryMath_F64.mult(to_ned.R, vel_body,vel_ned);
				
				msg.x =  (float)pos_ned.x;
				msg.y =  (float)pos_ned.y;
				msg.z =  (float)pos_ned.z;

				msg.vx = (float)vel_ned.x;
				msg.vy = (float)vel_ned.y;
				msg.vz = (float)vel_ned.z;
				
				

				// Simulate drift estimation
				vpo_body.setTo(msg.vx+0.03+Math.random()/1000.0, msg.vy+Math.random()/1000.0,msg.vz);

				msg.p  = model.attitude.p;
				msg.r  = model.attitude.r;
				msg.h  = model.attitude.y;

				msg.quality = 100;
				msg.errors  = 0;
				msg.tms     = DataModel.getSynchronizedPX4Time_us();
				msg.flags   = model.vision.flags;
				
				control.sendMAVLinkMessage(msg);


			}
		});

	}

	@Override
	public void received(Object o) {

		if( !(o instanceof msg_odometry))
			return;

		if(!is_running) {
			model.vision.setStatus(Vision.ENABLED, true);
			model.vision.setStatus(Vision.AVAILABLE, true);
			model.sys.setSensor(Status.MSP_OPCV_AVAILABILITY, true);
			model.vision.setStatus(Vision.POS_VALID, true);
			model.vision.setStatus(Vision.SPEED_VALID, true);
			model.vision.setStatus(Vision.PUBLISHED, true);

			System.out.println("SITL Vision PositionEstimator enabled");
			is_running = true;
		}

		// Just send msg_odometry as msp_vision message
		msg_odometry odometry = (msg_odometry)o;

		pos_ned.setTo(odometry.x,odometry.y,odometry.z);
		vel_body.setTo(odometry.vx,odometry.vy,odometry.vz);
		
		


	}




	public void reset() {

	}


	@Override
	public void enableStream(boolean enable) {


	}

	@Override
	public void start() throws Exception {


	}

	@Override
	public void stop() {

	}

	// TODO: Move to commander
	private void setGlobalOrigin(double lat, double lon, double altitude) {

		if(model.sys.isSensorAvailable(Status.MSP_GPS_AVAILABILITY) || model.sys.isStatus(Status.MSP_GPOS_VALID))
			return;

		// Note: In SITL Set global origin causes BARO failure 
		// TODO: To be investigated in PX4

		//
		//			msg_set_gps_global_origin gor = new msg_set_gps_global_origin(1,1);
		//			gor.target_system = 1;
		//			gor.latitude = (long)(lat * 1e7);
		//			gor.longitude = (long)(lon * 1e7);
		//			gor.altitude = (int)(altitude * 1000);
		//			gor.time_usec = DataModel.getSynchronizedPX4Time_us();
		//
		//			control.sendMAVLinkMessage(gor);
		//
		//			MSPLogger.getInstance().writeLocalMsg("[msp] Setting reference position",
		//					MAV_SEVERITY.MAV_SEVERITY_INFO);

	}


}
