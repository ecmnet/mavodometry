package com.comino.mavodometry.estimators.position;

import org.ejml.dense.row.CommonOps_DDRM;
import org.mavlink.messages.MAV_ESTIMATOR_TYPE;
import org.mavlink.messages.MAV_FRAME;
import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.MSP_CMD;
import org.mavlink.messages.MSP_COMPONENT_CTRL;
import org.mavlink.messages.lquac.msg_msp_command;
import org.mavlink.messages.lquac.msg_msp_vision;
import org.mavlink.messages.lquac.msg_odometry;

import com.comino.gazebo.libvision.boofcv.StreamGazeboVision;
import com.comino.mavcom.config.MSPParams;
import com.comino.mavcom.control.IMAVMSPController;
import com.comino.mavcom.mavlink.IMAVLinkListener;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.EstStatus;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.model.segment.Vision;
import com.comino.mavcom.struct.Attitude3D_F64;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavodometry.estimators.MAVAbstractEstimator;
import com.comino.mavutils.workqueue.WorkQueue;

import georegression.geometry.GeometryMath_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;

// Note: Requires odomtery enabled in SDF <send_odometry>1</send_odometry>

public class MAVGazeboVisPositionEstimator extends MAVAbstractEstimator  {

	// MAVLink messages
	private final msg_msp_vision               msg = new msg_msp_vision(2,1);
	private final msg_odometry                 odo = new msg_odometry(1,1);


	private final WorkQueue wq = WorkQueue.getInstance();

	private final DataModel 	     model;
	private final Se3_F64            to_ned     = new Se3_F64();
	private final Se3_F64            to_body    = new Se3_F64();

	private final Se3_F64          ned      		= new Se3_F64();
	private final Se3_F64          ned_s    		= new Se3_F64();
	private final Se3_F64          body_s    		= new Se3_F64();
	private final Vector3D_F64     offset_pos_ned   = new Vector3D_F64();

	private boolean                  is_running = false;

	private long                     tms   = 0;
	private long                     tms_r = 0;
	private float 			    	 dt_sec  = 0;
	private float 			    	 dt_sec_1 = 0;

	private long                     tms_reset = 0;
	private long                     tms_start = 0;

	private int                      reset_count = 0;
	private int                      ekf_reset_counter = 0;

	private float velocity_error    = 0;

	private final StreamGazeboVision vis;
	private final Attitude3D_F64     att = new Attitude3D_F64();

	public MAVGazeboVisPositionEstimator(IMAVMSPController control) {
		super(control);
		this.model = control.getCurrentModel();
		this.vis   = StreamGazeboVision.getInstance(640,480);
		

		control.registerListener(msg_msp_command.class, new IMAVLinkListener() {
			@Override
			public void received(Object o) {
				msg_msp_command cmd = (msg_msp_command)o;
				switch(cmd.command) {
				case MSP_CMD.MSP_CMD_SET_HOMEPOS:
					setGlobalOrigin(cmd.param1 / 1e7f, cmd.param2 / 1e7f, cmd.param3 / 1e3f );
					break;
				case MSP_CMD.MSP_CMD_VISION:

					switch((int)cmd.param1) {
					case MSP_COMPONENT_CTRL.ENABLE:
						if(!model.vision.isStatus(Vision.ENABLED)) {
							init("enable");
							model.vision.setStatus(Vision.ENABLED, true);
						}
						break;
					case MSP_COMPONENT_CTRL.DISABLE:
						model.vision.setStatus(Vision.ENABLED, false);
						break;
					case MSP_COMPONENT_CTRL.RESET:
						init("Reset");
						break;
					}
				}
			}
		});

		control.getStatusManager().addListener(Status.MSP_ARMED, (n) -> {
			if(n.isStatus(Status.MSP_ARMED) && n.isStatus(Status.MSP_LANDED)) {
				init("armed");
			}


		});

		offset_pos_ned.setTo(0,0,0);

		vis.registerCallback((tms, confidence, p, s, a) ->  {
			

			if(tms_reset > 0 && ((System.currentTimeMillis() - tms_reset) < 2000)) {
				model.vision.setStatus(Vision.RESETTING, true);
				MSP3DUtils.convertCurrentPosition(model, offset_pos_ned);
				offset_pos_ned.scale(-1);
				offset_pos_ned.plusIP(p.T);
				offset_pos_ned.scale(-1);
				tms_start = System.currentTimeMillis();
				body_s.reset();
				publishPX4Odometry(ned.T,body_s.T,MAV_FRAME.MAV_FRAME_LOCAL_NED,true,confidence,tms);
				return;
			}

			model.vision.setStatus(Vision.RESETTING, false);
			model.vision.setStatus(Vision.ERROR, false);

			tms_reset = 0;

			if(!model.vision.isStatus(Vision.ENABLED)) {
				publishMSPFlags(DataModel.getSynchronizedPX4Time_us());
				return;
			}


			if(ekf_reset_counter != model.est.reset_counter) {
				ekf_reset_counter =  model.est.reset_counter;
				model.vision.setStatus(Vision.SPEED_VALID, false);
				model.vision.setStatus(Vision.POS_VALID, false);
				model.vision.setStatus(Vision.ERROR, true);
				init("EKF2 reset");
				return;
			}


			if(model.sys.isSensorAvailable(Status.MSP_GPS_AVAILABILITY) && 	model.est.isFlagSet(EstStatus.EKF_GPS_GLITCH)) {

				model.vision.setStatus(Vision.SPEED_VALID, false);
				model.vision.setStatus(Vision.POS_VALID, false);
				model.vision.setStatus(Vision.ERROR, true);		
				init("EKF2 Glitch");
				return;
			}

			MSP3DUtils.convertModelToSe3_F64(model, to_ned);
			CommonOps_DDRM.transpose(to_ned.R, to_body.R);

			ned.setTo(p);
			ned_s.setTo(s);

			// get euler angles
			att.setFromMatrix(ned.R);
			
			ned.T.plusIP(offset_pos_ned);

//			if((System.currentTimeMillis() -tms_start) > 30000 && tms_start > 0)
//				velocity_error += .005;
//
//			ned_s.T.x += velocity_error;
//			ned_s.T.y += velocity_error;


			GeometryMath_F64.mult( to_body.R, ned_s.T ,body_s.T );



			model.vision.setStatus(Vision.POS_VALID, true);
			model.vision.setStatus(Vision.SPEED_VALID, true);
			model.vision.setStatus(Vision.AVAILABLE, true);


			publishPX4Odometry(ned.T,body_s.T,MAV_FRAME.MAV_FRAME_LOCAL_NED,true,confidence,tms);
			// Publish to GCL
			publishMSPVision(ned,ned_s,tms);

			model.vision.setAttitude(att);
			model.vision.setPosition(ned.T);
			model.vision.setSpeed(ned_s.T);
			model.vision.tms = tms * 1000;

			model.vision.fps = vis.getFrameRate();

		});


	}


	public void reset() {

	}


	@Override
	public void enableStream(boolean enable) {


	}

	@Override
	public void start() throws Exception {
		System.out.println("Gazebo vision plugin started for SITL");
		vis.start();

	}

	@Override
	public void stop() {

	}

	public void init(String s) {
		tms_reset = System.currentTimeMillis();
		reset_count++; 
		if(s!=null)
			writeLogMessage(new LogMessage("[vio] Gazebo vision reset ["+s+"]", MAV_SEVERITY.MAV_SEVERITY_WARNING));
		model.vision.setStatus(Vision.RESETTING, true);
		model.vision.setStatus(Vision.POS_VALID, false);
		model.vision.setStatus(Vision.SPEED_VALID, false);
		publishMSPFlags(DataModel.getSynchronizedPX4Time_us());

		velocity_error = 0;
	}

	private void publishPX4Odometry(Vector3D_F64 pose, Vector3D_F64 speed, int frame, boolean pose_is_valid, int confidence, long tms) {

		odo.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION;
		odo.frame_id       = frame;
		odo.child_frame_id = MAV_FRAME.MAV_FRAME_BODY_FRD;

		odo.time_usec =  tms * 1000;

		if(pose_is_valid) {
			odo.x = (float) pose.x;
			odo.y = (float) pose.y;
			odo.z = (float) pose.z;
			//			build_covariance(odo.pose_covariance, confidence);
		} else {
			//			odo.x = Float.NaN;
			//			odo.y = Float.NaN;
			//	odo.z = Float.NaN;
			//			odo.x = (float)lpos.T.x;
			//			odo.y = (float)lpos.T.y;
			//			odo.z = (float) pose.z;
			//			odo.pose_covariance[0] = Float.NaN;
		}

		odo.vx = (float) speed.x;
		odo.vy = (float) speed.y;
		odo.vz = (float) speed.z;

		// Use EKF params
		odo.pose_covariance[0] = Float.NaN;
		odo.velocity_covariance[0] = Float.NaN;

		//		build_covariance(odo.velocity_covariance, confidence);

		//		ConvertRotation3D_F64.matrixToQuaternion(body.R, att_q);
		//		odo.q[0] = (float)att_q.w;
		//		odo.q[1] = (float)att_q.x;
		//		odo.q[2] = (float)att_q.y;
		//		odo.q[3] = (float)att_q.z;

		// do not use twist
		odo.q[0] = Float.NaN;


		odo.reset_counter = reset_count;

		control.sendMAVLinkMessage(odo);

		model.sys.setSensor(Status.MSP_OPCV_AVAILABILITY, true);
		model.vision.setStatus(Vision.PUBLISHED, true);

	}

	private void publishMSPVision(Se3_F64 pose, Se3_F64 speed, long tms) {


		msg.x =  (float) pose.T.x;
		msg.y =  (float) pose.T.y;
		msg.z =  (float) pose.T.z;

		msg.vx =  (float) speed.T.x;
		msg.vy =  (float) speed.T.y;
		msg.vz =  (float) speed.T.z;

		msg.h   = (float)att.getYaw();
		msg.r   = (float)att.getRoll();
		msg.p   = (float)att.getPitch();

		msg.quality = 100;
		msg.errors  = 0;
		msg.tms     = tms * 1000;
		msg.flags   = model.vision.flags;
		msg.fps     = model.vision.fps;

		control.sendMAVLinkMessage(msg);


	}

	private void publishMSPFlags(long tms) {

		msg.quality = (int)(100);
		msg.errors  = 0;
		msg.tms     = tms * 1000;
		msg.flags   = model.vision.flags;
		msg.fps     = model.vision.fps;

		control.sendMAVLinkMessage(msg);

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
