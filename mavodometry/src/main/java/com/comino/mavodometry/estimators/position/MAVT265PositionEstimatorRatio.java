package com.comino.mavodometry.estimators.position;

/****************************************************************************
 *
 *   Copyright (c) 2020,2022 Eike Mansfeld ecm@gmx.de. All rights reserved.
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
import java.text.DecimalFormat;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.mavlink.messages.ESTIMATOR_STATUS_FLAGS;
import org.mavlink.messages.MAV_ESTIMATOR_TYPE;
import org.mavlink.messages.MAV_FRAME;
import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.MSP_AUTOCONTROL_MODE;
import org.mavlink.messages.MSP_CMD;
import org.mavlink.messages.MSP_COMPONENT_CTRL;
import org.mavlink.messages.lquac.msg_msp_command;
import org.mavlink.messages.lquac.msg_msp_vision;
import org.mavlink.messages.lquac.msg_odometry;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.config.MSPParams;
import com.comino.mavcom.control.IMAVMSPController;
import com.comino.mavcom.mavlink.IMAVLinkListener;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.model.segment.Vision;
import com.comino.mavcom.status.StatusManager;
import com.comino.mavcom.struct.Attitude3D_F64;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavcom.utils.MSPCovariance;
import com.comino.mavodometry.estimators.MAVAbstractEstimator;
import com.comino.mavodometry.librealsense.t265.boofcv.StreamRealSenseT265PoseCV;
import com.comino.mavodometry.utils.TimeHysteresis;
import com.comino.mavodometry.video.IVisualStreamHandler;
import com.comino.mavutils.MSPMathUtils;
import com.comino.mavutils.workqueue.WorkQueue;
import com.comino.mavutils.workqueue.WorkQueueException;

import boofcv.abst.fiducial.FiducialDetector;
import boofcv.abst.fiducial.FiducialStability;
import boofcv.alg.distort.pinhole.LensDistortionPinhole;
import boofcv.factory.fiducial.ConfigFiducialBinary;
import boofcv.factory.fiducial.FactoryFiducial;
import boofcv.factory.filter.binary.ConfigThreshold;
import boofcv.factory.filter.binary.ThresholdType;
import boofcv.struct.calib.CameraKannalaBrandt;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import georegression.geometry.ConvertRotation3D_F64;
import georegression.geometry.GeometryMath_F64;
import georegression.struct.EulerType;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.point.Vector4D_F64;
import georegression.struct.se.Se3_F64;

public class MAVT265PositionEstimatorRatio extends MAVAbstractEstimator {

	// Modes
	public static final int  GROUNDTRUTH_MODE       			= 1;
	public static final int  LPOS_ODO_MODE_POSITION 		    = 2;

	private static final int         FIDUCIAL_ID            	= 284;
	private static final float       FIDUCIAL_SIZE          	= 0.168f;
	private static final int         FIDUCIAL_RATE_SCAN     	= 500;
	private static final int         FIDUCIAL_RATE_ACTIVE   	= 200;

	private static final int         FIDUCIAL_HEIGHT     		= 360;
	private static final int         FIDUCIAL_WIDTH     		= 360;

	private static final float       DEFAULT_MAX_VEL_TESTRATIO  = 0.5f;
	private static final int     	 DEFAULT_MAX_ERRORS         = 20;

	private static final float       MAX_YAW_VARIANCE           = 1.0f;

	private static final long        LOCK_TIMEOUT_MS            = 1000;
	private static final long        VISION_SETTLE_MS           = 300;

	// mounting offset in m
	private static final double   	 OFFSET_X 					=   0.00;
	private static final double      OFFSET_Y 					=   0.00;
	private static final double      OFFSET_Z 					=   0.00;

	private static final double      FIDUCIAL_OFFSET_X 			=  -0.08;
	private static final double      FIDUCIAL_OFFSET_Y 			=   0.05;
	private static final double      FIDUCIAL_OFFSET_Z 			=   0.00;

	private final WorkQueue wq;

	private  boolean is_initialized      = false;

	// MAVLink messages
	private final msg_msp_vision               msg = new msg_msp_vision(2,1);
	private final msg_odometry                 odo = new msg_odometry(1,1);

	// Controls
	private StreamRealSenseT265PoseCV t265;

	// 3D transformation matrices
	private final Se3_F64          to_ned          	= new Se3_F64();
	private final Se3_F64          to_fiducial_ned 	= new Se3_F64();
	private final Se3_F64          to_body         	= new Se3_F64();
	private final Se3_F64          to_rotz90       	= new Se3_F64();
	private final Se3_F64          to_tmp          	= new Se3_F64();

	private final Se3_F64          ned      		= new Se3_F64();
	private final Se3_F64          gnd_ned  	 	= new Se3_F64();
	private final Se3_F64          ned_s    		= new Se3_F64();
	private final Se3_F64          body     		= new Se3_F64();
	private final Se3_F64          body_s   		= new Se3_F64();
	private final Se3_F64          body_a   		= new Se3_F64();
	private final Se3_F64          lpos     		= new Se3_F64();

	private final DMatrixRMaj   tmp         	= CommonOps_DDRM.identity( 3 );
	private final DMatrixRMaj   initial_rot 	= CommonOps_DDRM.identity( 3 );

	// 3D helper structures
	private final Vector3D_F64  offset     		= new Vector3D_F64();
	private final Vector3D_F64  offset_pos_ned  = new Vector3D_F64();
	private final Vector3D_F64  error_pos_ned   = new Vector3D_F64();
	private final Vector3D_F64  offset_vel_body = new Vector3D_F64();
	private final Vector3D_F64  angular_rates   = new Vector3D_F64();
	private final Vector3D_F64  lpos_current_s  = new Vector3D_F64();
	

	private final Attitude3D_F64   att      	= new Attitude3D_F64();
	
	private float             quality = 0;
	private long              tms_old = 0;
	private long            tms_reset = 0;
	private long       tms_last_reset = 0;
	private int        confidence_old = 0;
	private double             dt_sec = 0;
	private double           dt_sec_1 = 0;

	private boolean    enableStream   = false;

	private final Se3_F64          targetToSensor     = new Se3_F64();
	private final Se3_F64          precision_ned      = new Se3_F64();
	private final Attitude3D_F64   fiducial_att       = new Attitude3D_F64();
	private final Point2D_F64      fiducial_cen       = new Point2D_F64();
	private final Vector4D_F64     precision_lock     = new Vector4D_F64();
	private final Vector3D_F64     fiducial_offset    = new Vector3D_F64();

	private boolean          is_fiducial        = false;
	private float            fiducial_size      = FIDUCIAL_SIZE;
	private int              fiducial_idx       = 0;
	private long             locking_tms        = 0;
	private int              fiducial_x_offs    = 0;
	private int              fiducial_y_offs    = 0;

	private LensDistortionPinhole lensDistortion = null;

	private int           error_count = 0;
	private int           reset_count = 0;

	private FiducialDetector<GrayU8> detector = null;
	private FiducialStability        stability = new FiducialStability();

	private int   width4;

	private final DecimalFormat flocked2  = new DecimalFormat("LockAlt: #0.00m");
	private final DecimalFormat flocked1  = new DecimalFormat("LockAlt: #0.00m");
	private String stmp;

	private final GrayU8 fiducial         = new GrayU8(1,1);
	
	private int                            fiducial_worker;

	private final TimeHysteresis           att_hysteresis;

	private float                          check_veltestratio = DEFAULT_MAX_VEL_TESTRATIO;
	private int                            check_max_errors   = DEFAULT_MAX_ERRORS;                

	private MSPCovariance                  cov_s = new MSPCovariance(10);
	private float                          cov_s_f = 0;
	private float cov_velocity = 0.1f;

	@SuppressWarnings("unused")
	
	public <T> MAVT265PositionEstimatorRatio(IMAVMSPController control,  MSPConfig config, int width, int height, int mode, IVisualStreamHandler<Planar<GrayU8>> stream) 
			throws Exception {

		super(control);

		wq     = WorkQueue.getInstance();

		model.vision.clear();
		model.vision.setStatus(Vision.ENABLED, config.getBoolProperty(MSPParams.PUBLISH_ODOMETRY, "true"));
		model.sys.setAutopilotMode(MSP_AUTOCONTROL_MODE.PRECISION_LOCK,config.getBoolProperty(MSPParams.T265_PRECISION_LOCK, "true"));

		this.width4  = width / 4;

		// Subimage-Offsets for fiducial img
		this.fiducial_x_offs = (width - FIDUCIAL_WIDTH )   / 2;
		this.fiducial_y_offs = (height - FIDUCIAL_HEIGHT ) / 2;

		// read offsets from config
		offset.x = config.getFloatProperty(MSPParams.T265_OFFSET_X, String.valueOf(OFFSET_X));
		offset.y = config.getFloatProperty(MSPParams.T265_OFFSET_Y, String.valueOf(OFFSET_Y));
		offset.z = config.getFloatProperty(MSPParams.T265_OFFSET_Z, String.valueOf(OFFSET_Z));
		System.out.println("T265 Mounting offset: "+offset);



		check_veltestratio = config.getFloatProperty(MSPParams.T265_CHECK_VELTESTRATIO, Float.toString(DEFAULT_MAX_VEL_TESTRATIO));
		System.out.println("T265 check velocity test ratio: "+check_veltestratio);
		if(check_veltestratio <= 0)
			writeLogMessage(new LogMessage("[msp] T265 testratio check disabled",MAV_SEVERITY.MAV_SEVERITY_DEBUG));

		check_max_errors = config.getIntProperty(MSPParams.T265_CHECK_MAX_ERROR, Integer.toString(DEFAULT_MAX_ERRORS));
		System.out.println("T265 max errors: "+check_max_errors);
		if(check_max_errors <= 0)
			writeLogMessage(new LogMessage("[msp] T265 reset disabled",MAV_SEVERITY.MAV_SEVERITY_DEBUG));

		// Do not allow drift compensation with GPS
		// TOOD: Better check PX4 fusion parameter
		//		drift_compensation = config.getBoolProperty(MSPParams.T265_DRIFT_COMPENSATION, "false");
		//		if(model.sys.isSensorAvailable(Status.MSP_GPS_AVAILABILITY) && drift_compensation) {
		//			drift_compensation = false;
		//			writeLogMessage(new LogMessage("[vio] T265 drift compensation disabled as GPS is available", MAV_SEVERITY.MAV_SEVERITY_WARNING));
		//		} else
		//			System.out.println("T265 drift compensation enabled: "+drift_compensation);

		// fiducial settings
		fiducial_size   = config.getFloatProperty(MSPParams.T265_FIDUCIAL_SIZE,String.valueOf(FIDUCIAL_SIZE));
		System.out.println("T265 fiducial size: "+fiducial_size+"m");

		fiducial_offset.x = -config.getFloatProperty(MSPParams.T265_FIDUCIAL_OFFSET_X, String.valueOf(FIDUCIAL_OFFSET_X));
		fiducial_offset.y = -config.getFloatProperty(MSPParams.T265_FIDUCIAL_OFFSET_Y, String.valueOf(FIDUCIAL_OFFSET_Y));
		fiducial_offset.z = -config.getFloatProperty(MSPParams.T265_FIDUCIAL_OFFSET_Z, String.valueOf(FIDUCIAL_OFFSET_Z));
		System.out.println("T265 fiducial offset: "+fiducial_offset);

		ConvertRotation3D_F64.rotZ(Math.PI/2,to_rotz90.R);


		control.registerListener(msg_msp_command.class, new IMAVLinkListener() {
			@Override
			public void received(Object o) {
				msg_msp_command cmd = (msg_msp_command)o;
				switch(cmd.command) {
				case MSP_CMD.MSP_CMD_VISION:

					switch((int)cmd.param1) {
					case MSP_COMPONENT_CTRL.ENABLE:
						if(!model.vision.isStatus(Vision.ENABLED)) {
							init("enable");
							model.vision.setStatus(Vision.ENABLED, true);
						}
						config.updateProperty(MSPParams.PUBLISH_ODOMETRY, "true");
						break;
					case MSP_COMPONENT_CTRL.DISABLE:
						model.vision.setStatus(Vision.ENABLED, false);
						config.updateProperty(MSPParams.PUBLISH_ODOMETRY, "false");
						break;
					case MSP_COMPONENT_CTRL.RESET:
						if(t265!=null) {
							if(!t265.isRunning())
								start();
							else
								init("reset");
						}
						break;
					}
					break;
				}
			}
		});

		control.getStatusManager().addListener(Status.MSP_GPOS_VALID, (n) -> {
			if(model.gps.numsat > 8 && n.isStatus(Status.MSP_GPOS_VALID) && n.isStatus(Status.MSP_LANDED))
				init("gpos");

		});

		control.getStatusManager().addListener(Status.MSP_ARMED, (n) -> {
			if(n.isStatus(Status.MSP_ARMED) && n.isStatus(Status.MSP_LANDED))
				init("armed");

		});


		control.getStatusManager().addListener(StatusManager.TYPE_MSP_SERVICES,	Status.MSP_OPCV_AVAILABILITY, (n) -> {
			if (n.isSensorAvailable(Status.MSP_OPCV_AVAILABILITY))
				wq.addSingleTask("LP",1000,() -> init("init"));
		});

		control.getStatusManager().addListener(StatusManager.TYPE_MSP_AUTOPILOT,
				MSP_AUTOCONTROL_MODE.PRECISION_LOCK, StatusManager.EDGE_RISING, (n) -> {
					if(model.sys.isAutopilotMode(MSP_AUTOCONTROL_MODE.PRECISION_LOCK))
						writeLogMessage(new LogMessage("[vio] PrecisionLock enabled", MAV_SEVERITY.MAV_SEVERITY_NOTICE));
				});

		detector = FactoryFiducial.squareBinary(new ConfigFiducialBinary(fiducial_size), ConfigThreshold.local(ThresholdType.LOCAL_MEAN, 25), GrayU8.class);

		
		att_hysteresis = new TimeHysteresis(0.5f,TimeHysteresis.EDGE_RISING);

		t265 = StreamRealSenseT265PoseCV.getInstance(StreamRealSenseT265PoseCV.POS_DOWNWARD_180_PREDICT,width,height);
		t265.registerCallback((tms, confidence, p, s, a, img) ->  {

			// Bug in CB; sometimes called twice
			if((tms-tms_old) < 3)
				return;

			switch(confidence) {
			case CONFIDENCE_FAILED:
				quality = 0.00f; is_fiducial = false; 
				is_initialized = false;
				break;
			case CONFIDENCE_LOW:
				quality = 0.33f; 
				break;
			case CONFIDENCE_MEDIUM:
				quality = 0.66f; 
				break;
			case CONFIDENCE_HIGH:
				if(confidence_old != CONFIDENCE_HIGH) {
					quality = 1f; 
				}
				break;
			default:
				System.out.println("TrackerConfidence is "+confidence);
			}

			model.vision.setStatus(Vision.ERROR, false);


			// Reset procedure ------------------------------------------------------------------------------------------------
			// Note: This takes 2sec for T265;

			if((System.currentTimeMillis() - tms_reset) < 1800 || !is_initialized) {
				tms_reset = 0; confidence_old = 0; is_initialized = true; error_count = 0;

				// set initial T265 pose as origin
				to_body.setTranslation(- p.T.x , - p.T.y , - p.T.z );

				MSP3DUtils.convertModelToSe3_F64(model, to_ned);

				// Rotate offset to NED
				GeometryMath_F64.mult(to_ned.R, offset, offset_pos_ned );

				// Subtract offset from ned position
				offset_pos_ned.scale(-1);
				to_ned.T.plusIP(offset_pos_ned);

				CommonOps_DDRM.transpose(p.R, tmp);
				CommonOps_DDRM.mult( to_ned.R, tmp , initial_rot );

				if(lensDistortion == null) {
					CameraKannalaBrandt model = t265.getLeftModel();
					// Adjust intrinsics as fiducial size is changed
					if(model!=null) {
						model.set(FIDUCIAL_WIDTH,FIDUCIAL_HEIGHT);
						lensDistortion = new LensDistortionPinhole(model);
						detector.setLensDistortion(lensDistortion,FIDUCIAL_WIDTH, FIDUCIAL_HEIGHT);
					}
				}

				if(t265.isVideoEnabled() && !wq.isInQueue("LP",fiducial_worker)) {
					System.out.println("T265 starting fiducial worker...");
					fiducial_worker = wq.addCyclicTask("LP", FIDUCIAL_RATE_SCAN, new FiducialHandler());
				} 

				precision_lock.setTo(Double.NaN,Double.NaN,Double.NaN, Double.NaN);
				model.vision.setStatus(Vision.FIDUCIAL_LOCKED, false);

				MSP3DUtils.convertCurrentSpeed(model, ned_s.T);

				is_fiducial = false;

				if(t265.isVideoEnabled() && stream!=null && enableStream && img != null) {
					stream.addToStream("DOWN",img, model, tms);
				}

				tms_old  = tms; 

				error_pos_ned.setTo(0,0,0);
				att_hysteresis.reset();

				tms_last_reset = System.currentTimeMillis();

				return;
			}

			model.vision.setStatus(Vision.AVAILABLE, true);
			if(!t265.isVideoEnabled()) {
				model.sys.setAutopilotMode(MSP_AUTOCONTROL_MODE.PRECISION_LOCK, false);
			}

			if(confidence <= CONFIDENCE_LOW && confidence_old != confidence) {

				if(++error_count > check_max_errors && check_max_errors > 0)
					init("Low confidence");

				model.vision.setStatus(Vision.SPEED_VALID, false);
				model.vision.setStatus(Vision.POS_VALID, false);
				model.vision.setStatus(Vision.ERROR, true);

				publishMSPFlags(tms);
				// Add left camera to stream
				if(stream!=null && enableStream) {
					stream.addToStream("DOWN",img, model, tms);
				}

				return;
			}

			confidence_old = confidence;



			//No flight controller connected => publish raw pose and speed for debugging purpose
			if(!model.sys.isStatus(Status.MSP_CONNECTED)) {
				if(stream!=null && enableStream) {
					stream.addToStream(getClass().getName(),img, model, tms);
				}
				model.vision.setStatus(Vision.PUBLISHED, false);
				publishMSPVision(p,s,a,precision_lock,tms);
				return;
			}

			// Transformation to Body frame and then to NED

			// Get current position and speeds
			MSP3DUtils.convertModelToSe3_F64(model, lpos);
			MSP3DUtils.convertCurrentSpeed(model, lpos_current_s);

			CommonOps_DDRM.transpose(p.R, to_body.R);

			p.concat(to_body, body);

			// timea and fps
			dt_sec   = (tms - tms_old) / 1000f;
			dt_sec_1 = 1.0 / dt_sec;
			model.vision.fps = (float)dt_sec_1;
			tms_old = tms;

			// rotate sensor velocities and acceleration to body frame
			GeometryMath_F64.mult(to_body.R, s.T, body_s.T);
			GeometryMath_F64.mult(to_body.R, a.T, body_a.T);

			// eventually calculate rr,pr,yr and do not use model rates
			// should compesate rotations in speed

			angular_rates.setTo(model.attitude.rr, model.attitude.pr, model.attitude.yr);
			offset_vel_body.crossSetTo(angular_rates, offset);
			offset_vel_body.scale(-1);
			body_s.T.plusIP(offset_vel_body);

			// Get model attitude rotation
			MSP3DUtils.convertModelRotationToSe3_F64(model, to_ned);

			// rotate position, speed and offset to ned
			body.concat(to_ned, ned);
			//	CommonOps_DDRM.mult( body.R, to_ned.R, ned_s.R );
			GeometryMath_F64.mult(to_ned.R, body_s.T,ned_s.T);
			GeometryMath_F64.mult(to_ned.R, offset, offset_pos_ned );

			// add rotated offset
			offset_pos_ned.scale(-1);
			ned.T.plusIP(offset_pos_ned);

			// Set rotation to vision based rotation
			CommonOps_DDRM.mult( initial_rot, p.R , ned.R );
			ned_s.R.setTo(ned.R);

			// get euler angles
			att.setFromMatrix(ned.R);

			if(att_hysteresis.check(Math.abs(att.getYaw() - model.attitude.y) > MAX_YAW_VARIANCE)) {
				model.vision.setStatus(Vision.ATT_VALID, false);
			} else {
				model.vision.setStatus(Vision.ATT_VALID, true);
			}

			// Assume vision position and velocity valid
			model.vision.setStatus(Vision.POS_VALID, true);
			model.vision.setStatus(Vision.SPEED_VALID, true);

			
            // 200ms settle time after reset
			if((System.currentTimeMillis() - tms_last_reset) < 200)
				return;

			model.vision.setStatus(Vision.RESETTING, false);

			// Test vision vs. lpos velocity cov
			cov_s_f = (float)Math.abs(cov_s.determine(ned_s.T.norm(), lpos_current_s.norm(), false)*10f);
			model.debug.x = cov_s_f;
			

			// Velocity testRatio check if enabled, GPS is available but not when GPS glitches reported
			if(check_veltestratio > 0 
					&&  model.est.velRatio > check_veltestratio 
					&&	model.sys.isSensorAvailable(Status.MSP_GPS_AVAILABILITY)
					&&	(System.currentTimeMillis() - tms_last_reset) > VISION_SETTLE_MS
					&&  (model.est.flags & ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH) != ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH
					)  {

				model.vision.setStatus(Vision.SPEED_VALID, false);
				model.vision.setStatus(Vision.POS_VALID, false);
				model.vision.setStatus(Vision.ERROR, true);	

				error_count++;

				publishMSPFlags(tms);

				// Add left camera to stream
				if(stream!=null && enableStream) {
					stream.addToStream("DOWN",img, model, tms);
				}

				if(error_count > check_max_errors) {
					init("Testratio");	
				}

				return;
			}

			error_count = 0;
			cov_velocity = Float.NaN;


			// Fiducial detection
			model.vision.setStatus(Vision.FIDUCIAL_ENABLED, model.sys.isAutopilotMode(MSP_AUTOCONTROL_MODE.PRECISION_LOCK));
			if(img!=null) 
				img.bands[0].subimage(fiducial_x_offs, fiducial_y_offs, width-fiducial_x_offs, height-fiducial_y_offs, fiducial);

			// Publishing data

			if(!model.vision.isStatus(Vision.ENABLED)) {

				// Add left camera to stream
				if(stream!=null && enableStream) {
					stream.addToStream("DOWN",img, model, tms);
				}

				model.vision.setStatus(Vision.PUBLISHED, false);
				model.vision.setGroundTruth(ned.T);
				publishMSPFlags(tms);
				return;
			}

			// Publishing data
			switch(mode) {

			case GROUNDTRUTH_MODE:

				// just set ground truth in local model and send to GCL
				model.vision.setGroundTruth(ned.T);
				publishMSPFlags(tms);

				break;


			case LPOS_ODO_MODE_POSITION:

				// Publish position data NED frame, speed body frame;  with ground truth
				//		publishPX4Odometry(ned.T,body_s.T,MAV_FRAME.MAV_FRAME_LOCAL_NED,model.vision.isStatus(Vision.POS_VALID),confidence,tms);

				// Do not publish data if error detected
				if(!model.vision.isStatus(Vision.ERROR))
					publishPX4Odometry(ned.T,body_s.T,MAV_FRAME.MAV_FRAME_LOCAL_NED,cov_velocity,confidence,tms);

				// Publish to GCL
				publishMSPVision(ned,ned_s,body_a,precision_lock,tms);

				model.vision.setGroundTruth(gnd_ned.T);

				break;

			}

			// Transfer to local model
			model.vision.setAttitude(att);
			model.vision.setPosition(ned.T);
			model.vision.setSpeed(ned_s.T);
			model.vision.tms = tms * 1000;

			// Add left camera to stream
			if(stream!=null && enableStream) {
				stream.addToStream("DOWN",img, model, tms);
			}
		});


		if(stream != null && t265!=null){
			stream.registerOverlayListener((ctx,n,tms) -> {
				if(enableStream)
					overlayFeatures(ctx, tms);
			});
		}


		if(t265.getMount() == StreamRealSenseT265PoseCV.POS_FOREWARD)
			System.out.println("T265 sensor initialized with mounting offset "+offset+" mounted forewards");
		else
			System.out.println("T265 sensor initialized with mounting offset "+offset+" mounted downwards");

	}

	public void enableStream(boolean enable) {
		this.enableStream = enable;
	}

	public void init(String s) {
		if(t265!=null) {
			tms_reset = System.currentTimeMillis();
			reset_count++; 
			quality = 0; 
			ned_s.reset(); body_s.reset(); body_a.reset();
			t265.reset();
			if(s!=null)
				writeLogMessage(new LogMessage("[vio] T265 reset ["+s+"]", MAV_SEVERITY.MAV_SEVERITY_WARNING));
			model.vision.setStatus(Vision.RESETTING, true);
			model.vision.setStatus(Vision.POS_VALID, false);
			model.vision.setStatus(Vision.SPEED_VALID, false);
			publishMSPFlags(DataModel.getSynchronizedPX4Time_us());
		}
	}

	public void start() {
		if(t265==null)
			return;

		is_initialized = false;
		t265.start();

		System.out.println("[vio] Starting T265....");
		t265.printDeviceInfo();	

	}

	public void stop() {
		wq.removeTask("LP", fiducial_worker);
		if(t265==null)
			return;
		t265.stop();
		is_initialized = false;
	}


	private void overlayFeatures(Graphics ctx, long tms) {

		ctx.setColor(Color.white);

		if(model.sys.isAutopilotMode(MSP_AUTOCONTROL_MODE.PRECISION_LOCK)) {

			drawFiducialArea(ctx,fiducial_x_offs,fiducial_y_offs,fiducial_x_offs+FIDUCIAL_WIDTH,fiducial_y_offs+FIDUCIAL_HEIGHT);

			if(model.vision.isStatus(Vision.FIDUCIAL_LOCKED)) {	
				int fx = (int)fiducial_cen.x + fiducial_x_offs;
				int fy = (int)fiducial_cen.y + fiducial_y_offs;
				ctx.drawLine(fx-10,fy,fx+10,fy);
				ctx.drawLine(fx,fy-10,fx,fy+10);
			} 
		}


		if(model.vision.isStatus(Vision.FIDUCIAL_LOCKED) && Double.isFinite(precision_lock.z)) {
			if(precision_lock.z > 1)
				stmp = flocked1.format(-precision_lock.z);
			else
				stmp = flocked2.format(-precision_lock.z);	
			ctx.drawString(stmp, width4*3 - ctx.getFontMetrics().stringWidth(stmp)/2, 20);
		}
	}

	private void drawFiducialArea(Graphics ctx, int x0, int y0, int x1, int y1) {

		final int ln = 20;

		ctx.drawLine(x0,y0,x0+ln,y0);
		ctx.drawLine(x0,y0,x0,y0+ln);

		ctx.drawLine(x0,y1,x0,y1-ln);
		ctx.drawLine(x0,y1,x0+ln,y1);

		ctx.drawLine(x1,y0,x1-ln,y0);
		ctx.drawLine(x1,y0,x1,y0+ln);

		ctx.drawLine(x1,y1,x1-ln,y1);
		ctx.drawLine(x1,y1,x1,y1-ln);

	}


	private void publishPX4Odometry(Vector3D_F64 pose, Vector3D_F64 speed, int frame, float cov_vel, int confidence, long tms) {

		odo.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION;
		odo.frame_id       = frame;
		odo.child_frame_id = MAV_FRAME.MAV_FRAME_BODY_FRD;

		odo.time_usec =  tms * 1000;

		odo.x = (float) pose.x;
		odo.y = (float) pose.y;
		odo.z = (float) pose.z;

		odo.vx = (float) speed.x;
		odo.vy = (float) speed.y;
		odo.vz = (float) speed.z;

		// Use EKF params
		odo.pose_covariance[0] = Float.NaN;

		if(Float.isFinite(cov_vel))
			build_covariance(cov_vel, odo.velocity_covariance);
		else
			odo.velocity_covariance[0] = Float.NaN;

		// do not use twist
		odo.q[0] = Float.NaN;


		odo.reset_counter = reset_count;

		control.sendMAVLinkMessage(odo);

		model.sys.setSensor(Status.MSP_OPCV_AVAILABILITY, true);
		model.vision.setStatus(Vision.PUBLISHED, true);

	}


	private void publishMSPVision(Se3_F64 pose, Se3_F64 speed, Se3_F64 acc_body, Vector4D_F64 offset,long tms) {

		msg.x =  (float) pose.T.x;
		msg.y =  (float) pose.T.y;
		msg.z =  (float) pose.T.z;

		msg.vx =  (float) speed.T.x;
		msg.vy =  (float) speed.T.y;
		msg.vz =  (float) speed.T.z;

		msg.ax =  (float) acc_body.T.x;
		msg.ay =  (float) acc_body.T.y;
		msg.az =  (float) acc_body.T.z;

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
		msg.fps     = model.vision.fps;

		control.sendMAVLinkMessage(msg);


	}

	private void publishMSPFlags(long tms) {

		msg.quality = (int)(quality * 100f);
		msg.errors  = error_count;
		msg.tms     = tms * 1000;
		msg.flags   = model.vision.flags;
		msg.fps     = model.vision.fps;

		control.sendMAVLinkMessage(msg);

	}

	public void printMatricesDebug(String title,Se3_F64 pose) {

		System.out.println("--"+title+"---");
		System.out.println("P=         "+pose.T);
		System.out.println("ToBody=    "+to_body.T);
		System.out.println("Body=      "+body.T);
		System.out.println("ToNed=     "+to_ned.T);
		System.out.println("OffsetR=   "+offset_pos_ned);
		System.out.println("Ned=        "+ned.T);
		System.out.println("----------");

	}


	private void build_covariance(float cov_in, float[] cov) {

		cov[0]  = cov_in;   cov[1]  = 0; cov[2]  = 0; cov[3]  = 0; cov[4]  = 0; cov[5]  = 0;
		cov[6]  = cov_in;   cov[7]  = 0; cov[8]  = 0; cov[9]  = 0; cov[10] = 0;
		cov[11] = cov_in;   cov[12] = 0; cov[13] = 0; cov[14] = 0;
		cov[15] = 0; cov[16] = 0; cov[17] = 0;
		cov[18] = 0; cov[19] = 0; 
		cov[20] = 0;

	}

	private class FiducialHandler implements Runnable {

		@Override
		public void run() {
			// Precision lock procedure 
			if(!model.sys.isAutopilotMode(MSP_AUTOCONTROL_MODE.PRECISION_LOCK)) {
				model.vision.setStatus(Vision.FIDUCIAL_LOCKED, false);
				return;
			}

			if((System.currentTimeMillis() - locking_tms) > LOCK_TIMEOUT_MS) {
				precision_lock.setTo(Double.NaN,Double.NaN,Double.NaN, Double.NaN);
				model.vision.setPrecisionOffset(precision_lock);
				model.vision.setStatus(Vision.FIDUCIAL_LOCKED, false);
			}

			try {

				detector.detect(fiducial);

				if(detector.totalFound()>0 && detector.is3D()) {
					for(int i = 0; i < detector.totalFound();i++ ) {
						is_fiducial = false;
						if(detector.getId(i)==FIDUCIAL_ID) {
							fiducial_idx = i;
							is_fiducial = true;
							break;
						}
					} 
				} else 
					is_fiducial = false;

				if(is_fiducial) {


					if(detector.getFiducialToCamera(fiducial_idx, to_tmp)) {

						detector.computeStability(fiducial_idx, 0.25f, stability);

						// rotate into YX axis (Sensor orientation)
						to_tmp.concat(to_rotz90, targetToSensor);

						targetToSensor.T.z = - targetToSensor.T.z;
						detector.getCenter(fiducial_idx, fiducial_cen);

						targetToSensor.T.plusIP(fiducial_offset);

						// check attitude because of rotated camera
						MSP3DUtils.convertModelToSe3_F64(model, to_fiducial_ned);
						GeometryMath_F64.mult(to_fiducial_ned.R, targetToSensor.T,precision_ned.T);

						// Add fiducial offset (left eye camera in body frame)
						fiducial_att.setFromMatrix(targetToSensor.R, EulerType.XYZ);

						precision_lock.setTo(lpos.T.x-precision_ned.T.x,lpos.T.y-precision_ned.T.y,precision_ned.T.z,
								MSPMathUtils.normAngle((float)fiducial_att.getYaw()-(float)Math.PI+model.attitude.y));

						model.vision.setPrecisionOffset(precision_lock);


						model.vision.setStatus(Vision.FIDUCIAL_LOCKED, true);
						locking_tms = System.currentTimeMillis();
					}
				} 

			} catch(Exception e ) {
				System.out.println(e.getMessage());
				precision_lock.setTo(Double.NaN,Double.NaN,Double.NaN, Double.NaN);
				model.vision.setStatus(Vision.FIDUCIAL_LOCKED, false);
			}

			// Adjust fiducial scan rate
			try {
				if(model.vision.isStatus(Vision.FIDUCIAL_LOCKED))
					wq.changeCycle("LP", fiducial_worker, FIDUCIAL_RATE_ACTIVE);
				else
					wq.changeCycle("LP", fiducial_worker, FIDUCIAL_RATE_SCAN);
			} catch(WorkQueueException e) {
				writeLogMessage(new LogMessage("[vio] "+e.getMessage(), MAV_SEVERITY.MAV_SEVERITY_DEBUG));
			}

		} 

	}

}
