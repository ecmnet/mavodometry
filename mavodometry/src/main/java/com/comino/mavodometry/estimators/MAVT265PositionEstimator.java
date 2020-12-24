package com.comino.mavodometry.estimators;

/****************************************************************************
 *
 *   Copyright (c) 2020,21 Eike Mansfeld ecm@gmx.de. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

import java.awt.Color;
import java.awt.Graphics;
import java.util.concurrent.TimeUnit;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.mavlink.messages.ESTIMATOR_STATUS_FLAGS;
import org.mavlink.messages.MAV_ESTIMATOR_TYPE;
import org.mavlink.messages.MAV_FRAME;
import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.MSP_AUTOCONTROL_MODE;
import org.mavlink.messages.MSP_CMD;
import org.mavlink.messages.MSP_COMPONENT_CTRL;
import org.mavlink.messages.lquac.msg_debug_vect;
import org.mavlink.messages.lquac.msg_msp_command;
import org.mavlink.messages.lquac.msg_msp_vision;
import org.mavlink.messages.lquac.msg_odometry;
import org.mavlink.messages.lquac.msg_vision_position_estimate;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.control.IMAVMSPController;
import com.comino.mavcom.mavlink.IMAVLinkListener;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.model.segment.Vision;
import com.comino.mavcom.status.StatusManager;
import com.comino.mavcom.struct.Attitude3D_F64;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavcom.utils.SimpleLowPassFilter;
import com.comino.mavodometry.librealsense.t265.boofcv.StreamRealSenseT265Pose;
import com.comino.mavodometry.video.IVisualStreamHandler;
import com.comino.mavutils.legacy.ExecutorService;

import boofcv.abst.fiducial.FiducialDetector;
import boofcv.abst.fiducial.FiducialStability;
import boofcv.alg.distort.pinhole.LensDistortionPinhole;
import boofcv.factory.fiducial.ConfigFiducialBinary;
import boofcv.factory.fiducial.FactoryFiducial;
import boofcv.factory.filter.binary.ConfigThreshold;
import boofcv.factory.filter.binary.ThresholdType;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import georegression.geometry.GeometryMath_F64;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.point.Vector4D_F64;
import georegression.struct.se.Se3_F64;
import georegression.struct.so.Quaternion_F64;

public class MAVT265PositionEstimator {

	private static final String T265_PRECISION_LOCK      = "t265_precision_lock";
	private static final String T265_FIDUCIAL_SIZE       = "t265_fiducial_size";
	private static final String T265_OFFSET_X            = "t265_offset_x";
	private static final String T265_OFFSET_Y            = "t265_offset_y";
	private static final String T265_OFFSET_Z            = "t265_offset_z";
	private static final String T265_CHECK_SPEED_XY      = "t265_check_speed_xy";
	private static final String T265_CHECK_SPEED_Z       = "t265_check_speed_z";



	private static final int         FIDUCIAL            = 284;
	private static final float       FIDUCIAL_SIZE       = 0.168f;
	private static final int         FIDUCIAL_RATE       = 100;

	private static final int     	 MAX_ERRORS          = 15;

	private static final float       MAX_SPEED_DEVIATION = 0.3f;

	private static final long        LOCK_TIMEOUT        = 2000;

	// mounting offset in m
	private static final double   	   OFFSET_X =  0.00;
	private static final double        OFFSET_Y =  0.00;
	private static final double        OFFSET_Z =  0.00;

	// Modes
	public static final int  GROUNDTRUTH_MODE   = 1;
	public static final int  LPOS_VIS_MODE_NED  = 2;
	public static final int  LPOS_ODO_MODE_NED  = 3;
	public static final int  LPOS_ODO_MODE_BODY = 4;


	// MessageBus -> maybe used for failsafe actions
	// private static final MessageBus bus = MessageBus.getInstance();

	// MAVLink messages
	private final msg_msp_vision               msg = new msg_msp_vision(2,1);
	private final msg_vision_position_estimate sms = new msg_vision_position_estimate(1,2);
	private final msg_odometry                 odo = new msg_odometry(1,2);

	// Controls
	private StreamRealSenseT265Pose t265;
	private IMAVMSPController       control;
	private DataModel               model;

	// 3D transformation matrices
	private Se3_F64          to_ned              = new Se3_F64();
	private Se3_F64          to_fiducial_ned     = new Se3_F64();
	private Se3_F64          to_body             = new Se3_F64();

	private Se3_F64          ned      = new Se3_F64();
	private Se3_F64          ned_s    = new Se3_F64();
	private Se3_F64          body     = new Se3_F64();
	private Se3_F64          body_s   = new Se3_F64();
	private Se3_F64          lpos     = new Se3_F64();

	private DMatrixRMaj   tmp         = CommonOps_DDRM.identity( 3 );
	private DMatrixRMaj   initial_rot = CommonOps_DDRM.identity( 3 );

	// 3D helper structures
	private Vector3D_F64     offset     = new Vector3D_F64();
	private Vector3D_F64     offset_r   = new Vector3D_F64();
	private Vector3D_F64     lpos_s     = new Vector3D_F64();

	private Attitude3D_F64   att      = new Attitude3D_F64();

	private float             quality = 0;
	private long              tms_old = 0;
	private long            tms_reset = 0;
	private int        confidence_old = 0;

	private boolean       do_odometry = true;
	private boolean      enableStream = false;
	private boolean    is_initialized = false;


	// Validity checks
	private boolean    check_speed_xy = false;
	private boolean    check_speed_z  = false;

	private SimpleLowPassFilter   avg_z_speed_dev  = new SimpleLowPassFilter(0.75);
	private SimpleLowPassFilter  avg_xy_speed_dev  = new SimpleLowPassFilter(0.75);

	private boolean          is_fiducial        = false;
	private float            fiducial_size      = FIDUCIAL_SIZE;
	private int              fiducial_idx       = 0;
	private long             locking_tms        = 0;
	private long             fiducial_tms       = 0;
	private Se3_F64          targetToSensor     = new Se3_F64();
	private Se3_F64          precision_ned      = new Se3_F64();
	private Attitude3D_F64   fiducial_att       = new Attitude3D_F64();
	private Point2D_F64      fiducial_cen       = new Point2D_F64();
	private Vector4D_F64     precision_lock     = new Vector4D_F64();

	private  LensDistortionPinhole lensDistortion = null;

	private int           error_count = 0;
	private int           reset_count = 0;

	private FiducialDetector<GrayU8> detector = null;
	private FiducialStability        stability = new FiducialStability();

	// Stream data
	private int   width;
	private int   height;

	private final Color	bgColor_header    = new Color(128,128,128,130);
//	private final msg_debug_vect  debug   = new msg_debug_vect(1,2);


	public <T> MAVT265PositionEstimator(IMAVMSPController control,  MSPConfig config, int width, int height, int mode, IVisualStreamHandler<Planar<GrayU8>> stream) {

		this.control = control;
		this.width   = width;
		this.height  = height;
		this.model   = control.getCurrentModel();

		// read offsets from config
		offset.x = -config.getFloatProperty(T265_OFFSET_X, String.valueOf(OFFSET_X));
		offset.y = -config.getFloatProperty(T265_OFFSET_Y, String.valueOf(OFFSET_Y));
		offset.z = -config.getFloatProperty(T265_OFFSET_Z, String.valueOf(OFFSET_Z));

		check_speed_xy = config.getBoolProperty(T265_CHECK_SPEED_XY, "false");
		check_speed_z  = config.getBoolProperty(T265_CHECK_SPEED_Z, "false");

		fiducial_size   = config.getFloatProperty(T265_FIDUCIAL_SIZE,String.valueOf(FIDUCIAL_SIZE));
		System.out.println("Fiducial size: "+fiducial_size+"m");


		control.registerListener(msg_msp_command.class, new IMAVLinkListener() {
			@Override
			public void received(Object o) {
				msg_msp_command cmd = (msg_msp_command)o;
				switch(cmd.command) {
				case MSP_CMD.MSP_CMD_VISION:
					switch((int)cmd.param1) {
					case MSP_COMPONENT_CTRL.ENABLE:
						do_odometry = true; init("enable"); break;
					case MSP_COMPONENT_CTRL.DISABLE:
						do_odometry = false; break;
					case MSP_COMPONENT_CTRL.RESET:
						if(!t265.isRunning())
							start();
						else
							init("reset");
						break;
					}
					break;
				}
			}
		});


		// reset vision when armed
		control.getStatusManager().addListener( Status.MSP_ARMED, (n) -> {
			if(n.isStatus(Status.MSP_ARMED)) {
				init("armed");
			}
		});

		// reset vision when absolute position lost
		control.getStatusManager().addListener(StatusManager.TYPE_ESTIMATOR, ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS, StatusManager.EDGE_FALLING, (n) -> {
			init("Est.LPOS(abs)");
		});

		model.sys.setAutopilotMode(MSP_AUTOCONTROL_MODE.PRECISION_LOCK,config.getBoolProperty(T265_PRECISION_LOCK, "true"));

		detector = FactoryFiducial.squareBinary(new ConfigFiducialBinary(fiducial_size), ConfigThreshold.local(ThresholdType.LOCAL_MEAN, 25), GrayU8.class);

		try {
			t265 = new StreamRealSenseT265Pose(StreamRealSenseT265Pose.POS_DOWNWARD,width,height);
		} catch( IllegalArgumentException e) {
			System.out.println("No T265 device found");
			return;

		}


		t265.registerCallback((tms, raw, p, s, a, img) ->  {

			switch(raw.tracker_confidence) {
			case StreamRealSenseT265Pose.CONFIDENCE_FAILED:
				quality = 0.00f; error_count++; is_fiducial = false;
				return;
			case StreamRealSenseT265Pose.CONFIDENCE_LOW:
				quality = 0.33f; error_count++;
				return;
			case StreamRealSenseT265Pose.CONFIDENCE_MEDIUM:
				quality = 0.66f;
				break;
			case StreamRealSenseT265Pose.CONFIDENCE_HIGH:

				if(confidence_old != StreamRealSenseT265Pose.CONFIDENCE_HIGH) {
					quality = 1f; error_count = 0;
					//					printMatricesDebug("Before",p);
					//					tmpv.set(p.T); tmpv.scale(-1);
					//					MSP3DUtils.convertCurrentState(model, to_ned.T);
					//					to_ned.T.plusIP(tmpv);

					to_ned.T.z = - model.raw.di - offset_r.z;

					control.writeLogMessage(new LogMessage("[vio] T265 NED Translation init. (Z update)", MAV_SEVERITY.MAV_SEVERITY_DEBUG));
					//					printMatricesDebug("After",p);
				}
				break;
			default:
				System.out.println("TrackerConfidence is "+raw.tracker_confidence);
			}

			confidence_old = raw.tracker_confidence;


			// Reset procedure 

			// Note: This takes 1.5sec for T265
			if((System.currentTimeMillis() - tms_reset) < 2500) {
				model.vision.setStatus(Vision.RESETTING, true);
				quality = 0; error_count=0; tms_reset = 0; confidence_old = 0;

				// set initial T265 pose as origin
				to_body.setTranslation(- p.T.x , - p.T.y , - p.T.z );

				MSP3DUtils.convertModelToSe3_F64(model, to_ned);

				// set initial Z-position to LIDAR distance (configurable)
				to_ned.T.z = - model.raw.di;

				// Rotate offset to NED
				GeometryMath_F64.mult(to_ned.R, offset, offset_r );

				// Subtract offset from ned position
				offset_r.scale(-1);
				to_ned.T.plusIP(offset_r);

				CommonOps_DDRM.transpose(p.R, tmp);
				CommonOps_DDRM.mult( to_ned.R, tmp , initial_rot );

				if(lensDistortion == null) {
					lensDistortion = new LensDistortionPinhole(t265.getLeftModel());
					detector.setLensDistortion(lensDistortion,width, height);
				}
				precision_lock.set(Double.NaN,Double.NaN,Double.NaN, Double.NaN);
				model.vision.setStatus(Vision.FIDUCIAL_LOCKED, false);

				avg_z_speed_dev.clear();
				avg_xy_speed_dev.clear();

				is_fiducial = false;

				// IDEA: Publish Dummy Vision with LIDAR-Z to PX4

				return;
			}

			model.vision.setStatus(Vision.RESETTING, false);

			if(error_count > MAX_ERRORS) {
				init("quality");
				return;
			}

			if(!is_initialized)
				return;

			// Transformation to NED

			// Get current position and speeds
			MSP3DUtils.convertModelToSe3_F64(model, lpos);
			MSP3DUtils.convertCurrentSpeed(model, lpos_s);

			CommonOps_DDRM.transpose(p.R, to_body.R);
			p.concat(to_body, body);


			GeometryMath_F64.mult(to_body.R, s.T, body_s.T);

			// Get model attitude rotation
			MSP3DUtils.convertModelRotationToSe3_F64(model, to_ned);

			// rotate position and offset to ned
			body.concat(to_ned, ned);
			//	CommonOps_DDRM.mult( body.R, to_ned.R, ned_s.R );
			GeometryMath_F64.mult(to_ned.R, body_s.T,ned_s.T);


			GeometryMath_F64.mult(to_ned.R, offset, offset_r );

			// add rotated offset
			ned.T.plusIP(offset_r);

			// Set rotation to vision based rotation
			CommonOps_DDRM.mult( initial_rot, p.R , ned.R );
			ned_s.R.set(ned.R);

			// get euler angles
			att.setFromMatrix(ned.R);

			// Consistency checks

			avg_xy_speed_dev.add(MSP3DUtils.distance2D(ned_s.T, lpos_s));

			// Speed check: Is visual XY speed acceptable
			if(avg_xy_speed_dev.getMeanAbs() > MAX_SPEED_DEVIATION) {
				//	control.writeLogMessage(new LogMessage("[vio] T265 XY speed vs local", MAV_SEVERITY.MAV_SEVERITY_DEBUG));
				if(check_speed_xy) {
					error_count++;
					init("speedXY");
					return;
				}
			}

			// calculate average mean for Z-speed
			avg_z_speed_dev.add(Math.abs(ned_s.T.z - lpos_s.z));

			// Speed check: Is visual Z speed acceptable
			if(avg_z_speed_dev.getMeanAbs() > MAX_SPEED_DEVIATION) {
				//	control.writeLogMessage(new LogMessage("[vio] T265 Z speed vs local: "+f2.format(avg_z_speed_dev.getMean())+"m/s", MAV_SEVERITY.MAV_SEVERITY_DEBUG));
				if(check_speed_z) {
					error_count++;
					init("speedZ");
					return;
				}
			}


			// Precision lock procedure 
			if(model.sys.isAutopilotMode(MSP_AUTOCONTROL_MODE.PRECISION_LOCK) && 
					( (System.currentTimeMillis() - fiducial_tms) > FIDUCIAL_RATE)) {
				fiducial_tms = System.currentTimeMillis();
				model.vision.setStatus(Vision.FIDUCIAL_ACTIVE, true);

				if((System.currentTimeMillis() - locking_tms) > LOCK_TIMEOUT) {
					precision_lock.set(Double.NaN,Double.NaN,Double.NaN, Double.NaN);
					model.vision.setStatus(Vision.FIDUCIAL_LOCKED, false);
				}

				try {
					detector.detect(img.bands[0]);
					if(detector.totalFound()>0 && detector.is3D()) {
						for(int i = 0; i < detector.totalFound();i++ ) {
							is_fiducial = false;
							if(detector.getId(i)==FIDUCIAL) {
								fiducial_idx = i;
								is_fiducial = true;
								break;
							}
						} 
					} else 
						is_fiducial = false;

					if(is_fiducial) {

						if(detector.getFiducialToCamera(fiducial_idx, targetToSensor)) {

							detector.computeStability(fiducial_idx, 0.25f, stability);

							targetToSensor.T.z = - targetToSensor.T.z;
							detector.getCenter(fiducial_idx, fiducial_cen);

							MSP3DUtils.convertModelToSe3_F64(model, to_fiducial_ned);
							GeometryMath_F64.mult(to_fiducial_ned.R, targetToSensor.T,precision_ned.T);

							fiducial_att.setFromMatrix(targetToSensor.R);
							precision_lock.set(lpos.T.x-precision_ned.T.x,lpos.T.y-precision_ned.T.y,precision_ned.T.z,
									fiducial_att.getYaw()+model.attitude.y);

							model.vision.setStatus(Vision.FIDUCIAL_LOCKED, true);
							locking_tms = System.currentTimeMillis();

						}
					}
					
			} catch(Exception e ) {
				control.writeLogMessage(new LogMessage("[vio] Fiducial error: "+e.getMessage(), MAV_SEVERITY.MAV_SEVERITY_CRITICAL));
				precision_lock.set(Double.NaN,Double.NaN,Double.NaN, Double.NaN);
				model.vision.setStatus(Vision.FIDUCIAL_LOCKED, false);
				return;
			}
		} 

		// Publishing data

		switch(mode) {

		case GROUNDTRUTH_MODE:

			// just set ground truth in local model
			model.vision.setGroundTruth(ned.T);

			break;

		case LPOS_VIS_MODE_NED:


			// Publish position NED
			if(do_odometry)
				publishPX4VisionPos(ned,tms);
			publishMSPVision(precision_ned,ned,ned_s,precision_lock,tms);

			break;

		case LPOS_ODO_MODE_NED:


			// Publish position data NED frame, speed body frame; No Z update
			if(do_odometry)
				publishPX4Odometry(ned,body_s,MAV_FRAME.MAV_FRAME_LOCAL_FRD,raw.tracker_confidence > StreamRealSenseT265Pose.CONFIDENCE_LOW,true,tms);
			publishMSPVision(p,ned,ned_s,precision_lock,tms);

			break;

		case LPOS_ODO_MODE_BODY:


			// Publish position and speed data body frame
			if(do_odometry)
				publishPX4Odometry(body,body_s,MAV_FRAME.MAV_FRAME_LOCAL_FRD, raw.tracker_confidence > StreamRealSenseT265Pose.CONFIDENCE_LOW,true,tms);
			publishMSPVision(precision_ned,ned,ned_s,precision_lock,tms);

			break;

		}

		model.vision.setStatus(Vision.POS_VALID, true);

		// Transfer to local model
		model.vision.setAttitude(att);
		model.vision.setPrecisionOffset(precision_lock);
		model.vision.setPosition(ned.T);
		model.vision.setSpeed(ned_s.T);
		model.vision.tms = DataModel.getSynchronizedPX4Time_us();

		//			// Publish vision data to subscribers
		//			bus.publish(model.vision);

		// Add left camera to stream
		if(stream!=null && enableStream) {
			stream.addToStream(img, model, tms);
		}

	});

		if(stream != null && t265!=null){
			stream.registerOverlayListener((ctx,tms) -> {
				if(enableStream)
					overlayFeatures(ctx, tms);
			});
		}


		if(t265.getMount() == StreamRealSenseT265Pose.POS_DOWNWARD)
			System.out.println("T265 sensor initialized with mounting offset "+offset+" mounted downwards");
		else
			System.out.println("T265 sensor initialized with mounting offset "+offset+" mounted forewards");

}

public void enableStream(boolean enable) {
	this.enableStream = enable;
}

public void init(String s) {
	model.vision.setStatus(Vision.POS_VALID, false);
	reset_count++;
	is_initialized = false;
	t265.reset();
	control.writeLogMessage(new LogMessage("[vio] T265 reset ["+s+"]", MAV_SEVERITY.MAV_SEVERITY_WARNING));
	tms_reset = System.currentTimeMillis();
	ExecutorService.get().schedule(() -> {
		if(lensDistortion!=null)
			detector.setLensDistortion(lensDistortion, width, height);
		is_initialized = true;
	}, 2, TimeUnit.SECONDS);
}


public void start() {
	if(t265==null)
		return;
	t265.start();
	System.out.println("[vio] Starting T265....");
	t265.printDeviceInfo();
	ExecutorService.get().schedule(() -> {
		init("init");
		is_initialized = true;
	}, 10, TimeUnit.SECONDS);
}

public void stop() {
	if(t265==null)
		return;
	t265.stop();
}

private void overlayFeatures(Graphics ctx, long tms) {


	ctx.setColor(bgColor_header);
	ctx.fillRect(5, 5, width-10, 21);

	ctx.setXORMode(Color.white);  

	if(is_fiducial) {	
		ctx.drawLine((int)fiducial_cen.x-10, (int)fiducial_cen.y, (int)fiducial_cen.x+10, (int)fiducial_cen.y);
		ctx.drawLine((int)fiducial_cen.x, (int)fiducial_cen.y-10, (int)fiducial_cen.x, (int)fiducial_cen.y+10);
	} 
		
	ctx.drawLine(95, height/2, width/2-25, height/2); ctx.drawLine(width/2+25, height/2, width-95, height/2);
	ctx.drawLine(width/2, 60, width/2, height/2-20); ctx.drawLine(width/2, height/2+20, width/2, height-60);

	ctx.setPaintMode();
	ctx.setColor(Color.white);
	
	if(is_fiducial && Double.isFinite(precision_lock.z))
		ctx.drawString(String.format("%#.2fm",-precision_lock.z), width-40, 20);

	if(!Float.isNaN(model.sys.t_armed_ms) && model.sys.isStatus(Status.MSP_ARMED)) {
		ctx.drawString(String.format("%.1fs",model.sys.t_armed_ms/1000f), 20, 20);
	}

	// TODO: To be refactored
//	if(model.msg.text != null && (DataModel.getSynchronizedPX4Time_us()-model.msg.tms) < 1000000 
//			&& !model.msg.text.contains("[logger]")
//			&& model.msg.severity < MAV_SEVERITY.MAV_SEVERITY_DEBUG)
	
	if(model.msg.isNew(MAV_SEVERITY.MAV_SEVERITY_INFO,tms))
		ctx.drawString(model.msg.text, 10, height-5);

}


private void publishPX4Odometry(Se3_F64 pose, Se3_F64 speed, int frame, boolean pose_is_valid, boolean z_valid,long tms) {

	odo.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION;
	odo.frame_id       = frame;
	odo.child_frame_id = MAV_FRAME.MAV_FRAME_BODY_FRD;

	odo.time_usec = tms * 1000;


	if(pose_is_valid) {
		odo.x = (float) pose.T.x;
		odo.y = (float) pose.T.y;
		if(z_valid)
			odo.z = (float) pose.T.z;
		else
			odo.z = model.state.l_z;
	} else {
		odo.x = Float.NaN;
		odo.y = Float.NaN;
		odo.z = Float.NaN;
	}

	odo.vx = (float) speed.T.x;
	odo.vy = (float) speed.T.y;
	odo.vz = (float) speed.T.z;

	// Use EKF params
	odo.pose_covariance[0] = Float.NaN;
	odo.velocity_covariance[0] = Float.NaN;

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

}

private void publishPX4VisionPos(Se3_F64 pose, long tms) {


	sms.usec = tms * 1000;

	sms.x = (float) pose.T.x;
	sms.y = (float) pose.T.y;
	sms.z = (float) pose.T.z;

	sms.reset_counter = reset_count;

	sms.roll  = (float)att.getRoll();
	sms.pitch = (float)att.getPitch();
	sms.yaw   = (float)att.getYaw();
	//		sms.yaw   = Float.NaN;

	sms.covariance[0] = Float.NaN;

	control.sendMAVLinkMessage(sms);

	model.sys.setSensor(Status.MSP_OPCV_AVAILABILITY, true);

}


private void publishMSPVision(Se3_F64 orig,Se3_F64 pose, Se3_F64 speed,Vector4D_F64 offset,long tms) {

	msg.x =  (float) pose.T.x;
	msg.y =  (float) pose.T.y;
	msg.z =  (float) pose.T.z;

	msg.vx =  (float) speed.T.x;
	msg.vy =  (float) speed.T.y;
	msg.vz =  (float) speed.T.z;

	msg.gx =  (float) orig.T.x;
	msg.gy =  (float) orig.T.y;
	msg.gz =  (float) orig.T.z;

	msg.px =  (float)offset.x;
	msg.py =  (float)offset.y;
	msg.pz =  (float)offset.z;
	msg.pw =  (float)offset.w;

	msg.h   = (float)att.getYaw();
	msg.r   = (float)att.getRoll();
	msg.p   = (float)att.getPitch();

	msg.quality = (int)(quality * 100f);
	msg.errors  = error_count;
	msg.tms     = tms * 1000;
	msg.flags   = model.vision.flags;

	if(tms_old != tms) {
		msg.fps = 1000 / (tms - tms_old);
	}
	tms_old = tms;

	control.sendMAVLinkMessage(msg);


}

public void printMatricesDebug(String title,Se3_F64 pose) {

	System.out.println("--"+title+"---");
	System.out.println("P=         "+pose.T);
	System.out.println("ToBody=    "+to_body.T);
	System.out.println("Body=      "+body.T);
	System.out.println("ToNed=     "+to_ned.T);
	System.out.println("OffsetR=   "+offset_r);
	System.out.println("Ned=        "+ned.T);
	System.out.println("----------");

}

//	private void sendPoseDebug(GeoTuple3D_F64<?> pose) {
//
//		debug.x = (float)pose.getX();
//		debug.y = (float)pose.getY();
//		debug.z = (float)pose.getZ();
//
//		control.sendMAVLinkMessage(debug);
//
//	}



}
