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
import java.awt.image.BufferedImage;
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
import org.mavlink.messages.lquac.msg_vision_position_estimate;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.config.MSPParams;
import com.comino.mavcom.control.IMAVMSPController;
import com.comino.mavcom.core.ControlModule;
import com.comino.mavcom.mavlink.IMAVLinkListener;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.model.segment.Vision;
import com.comino.mavcom.status.StatusManager;
import com.comino.mavcom.struct.Attitude3D_F64;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavcom.utils.SimpleLowPassFilter;
import com.comino.mavcom.utils.SimplePoseJumpDetector;
import com.comino.mavodometry.librealsense.t265.boofcv.StreamRealSenseT265Pose;
import com.comino.mavodometry.video.IVisualStreamHandler;
import com.comino.mavutils.MSPMathUtils;
import com.comino.mavutils.workqueue.WorkQueue;
import com.comino.mavutils.workqueue.WorkQueueException;

import boofcv.abst.fiducial.FiducialDetector;
import boofcv.abst.fiducial.FiducialStability;
import boofcv.alg.distort.pinhole.LensDistortionPinhole;
import boofcv.alg.filter.misc.AverageDownSampleOps;
import boofcv.factory.fiducial.ConfigFiducialBinary;
import boofcv.factory.fiducial.FactoryFiducial;
import boofcv.factory.filter.binary.ConfigThreshold;
import boofcv.factory.filter.binary.ThresholdType;
import boofcv.factory.transform.pyramid.FactoryPyramid;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.struct.calib.CameraKannalaBrandt;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.ImageType;
import boofcv.struct.image.Planar;
import boofcv.struct.pyramid.PyramidDiscrete;
import georegression.geometry.ConvertRotation3D_F64;
import georegression.geometry.GeometryMath_F64;
import georegression.struct.EulerType;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.point.Vector4D_F64;
import georegression.struct.se.Se3_F64;

public class MAVT265PositionEstimator extends ControlModule {

	private static final int         FIDUCIAL_ID            = 284;
	private static final float       FIDUCIAL_SIZE          = 0.168f;
	private static final int         FIDUCIAL_RATE_SCAN     = 500;
	private static final int         FIDUCIAL_RATE_ACTIVE   = 100;

	private static final int         FIDUCIAL_HEIGHT     	= 360;
	private static final int         FIDUCIAL_WIDTH     	= 360;

	private static final int     	 MAX_ERRORS          	= 30;

	private static final float       MAX_SPEED_DEVIATION   	= 0.3f;    
	private static final float       MAX_SPEED_Z_DEVIATION 	= 0.15f;  
	private static final float       MAX_ATT_DEVIATION     	= 0.20f;
	private static final float       MAX_ALT_DEVIATION     	= 0.30f;

	private static final long        LOCK_TIMEOUT        	= 1000;

	// mounting offset in m
	private static final double   	   OFFSET_X 			=  0.00;
	private static final double        OFFSET_Y 			=  0.00;
	private static final double        OFFSET_Z 			=  0.00;

	private static final double   	   FIDUCIAL_OFFSET_X 	=  -0.08;
	private static final double        FIDUCIAL_OFFSET_Y 	=   0.05;
	private static final double        FIDUCIAL_OFFSET_Z 	=   0.00;

	// Modes
	public static final int  GROUNDTRUTH_MODE       		= 1;
	public static final int  LPOS_ODO_MODE_NED_GND  		= 2;
	public static final int  LPOS_ODO_MODE_NED_PREC 		= 3;


	// Derived constants
	private static final float MAX_ATT_DEVIATION_SQ   		= MAX_ATT_DEVIATION * MAX_ATT_DEVIATION;

	// MessageBus -> maybe used for failsafe actions
	// private static final MessageBus bus = MessageBus.getInstance();

	private final WorkQueue wq = WorkQueue.getInstance();

	// MAVLink messages
	private final msg_msp_vision               msg = new msg_msp_vision(2,1);
	private final msg_vision_position_estimate sms = new msg_vision_position_estimate(1,1);
	private final msg_odometry                 odo = new msg_odometry(1,1);

	// Controls
	private StreamRealSenseT265Pose t265;

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
	private final Se3_F64          lpos     		= new Se3_F64();

	private final DMatrixRMaj   tmp         	= CommonOps_DDRM.identity( 3 );
	private final DMatrixRMaj   initial_rot 	= CommonOps_DDRM.identity( 3 );

	// 3D helper structures
	private final Vector3D_F64  offset     		= new Vector3D_F64();
	private final Vector3D_F64  offset_r   		= new Vector3D_F64();
	private final Vector3D_F64  lpos_s     		= new Vector3D_F64();

	private final Attitude3D_F64   att      	= new Attitude3D_F64();

	private float             quality = 0;
	private long              tms_old = 0;
	private long            tms_reset = 0;
	private int        confidence_old = 0;
	private float              dt_sec = 0;

	private boolean      enableStream = false;
	private boolean    is_originset   = false;

	// Validity checks
	private boolean    check_speed_xy = false;
	private boolean    check_speed_z  = false;

	private final SimpleLowPassFilter        avg_z_speed_dev  = new SimpleLowPassFilter(0.25);
	private final SimpleLowPassFilter        avg_xy_speed_dev = new SimpleLowPassFilter(0.75);
	private final SimpleLowPassFilter        avg_att_dev      = new SimpleLowPassFilter(0.05);
	private final SimplePoseJumpDetector     xy_pos_jump      = new SimplePoseJumpDetector(5.0f);


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
	private Vector3D_F64     fiducial_offset    = new Vector3D_F64();
	private int              fiducial_x_offs    = 0;
	private int              fiducial_y_offs    = 0;

	private LensDistortionPinhole lensDistortion = null;

	private int           error_count = 0;
	private int           reset_count = 0;

	private FiducialDetector<GrayU8> detector = null;
	private FiducialStability        stability = new FiducialStability();

	// Stream data
	private int   width;
	private int   width4;

	private final DecimalFormat flocked  = new DecimalFormat("LockAlt: #0.0m");
	private String stmp;

	private GrayU8 fiducial = new GrayU8(1,1);
	private int fiducial_worker;



	public <T> MAVT265PositionEstimator(IMAVMSPController control,  MSPConfig config, int width, int height, int mode, IVisualStreamHandler<Planar<GrayU8>> stream) {

		super(control);

		model.vision.clear();
		model.vision.setStatus(Vision.ENABLED, config.getBoolProperty(MSPParams.PUBLISH_ODOMETRY, "true"));
		model.sys.setAutopilotMode(MSP_AUTOCONTROL_MODE.PRECISION_LOCK,config.getBoolProperty(MSPParams.T265_PRECISION_LOCK, "true"));

		this.width   = width;
		this.width4  = width / 4;

		// Subimage-Offsets for fiducial img
		this.fiducial_x_offs = (width - FIDUCIAL_WIDTH )   / 2;
		this.fiducial_y_offs = (height - FIDUCIAL_HEIGHT ) / 2;

		// read offsets from config
		offset.x = -config.getFloatProperty(MSPParams.T265_OFFSET_X, String.valueOf(OFFSET_X));
		offset.y = -config.getFloatProperty(MSPParams.T265_OFFSET_Y, String.valueOf(OFFSET_Y));
		offset.z = -config.getFloatProperty(MSPParams.T265_OFFSET_Z, String.valueOf(OFFSET_Z));

		System.out.println("Mounting offset: "+offset);

		check_speed_xy = config.getBoolProperty(MSPParams.T265_CHECK_SPEED_XY, "false");
		check_speed_z  = config.getBoolProperty(MSPParams.T265_CHECK_SPEED_Z, "false");

		fiducial_size   = config.getFloatProperty(MSPParams.T265_FIDUCIAL_SIZE,String.valueOf(FIDUCIAL_SIZE));
		System.out.println("Fiducial size: "+fiducial_size+"m");

		fiducial_offset.x = -config.getFloatProperty(MSPParams.T265_FIDUCIAL_OFFSET_X, String.valueOf(FIDUCIAL_OFFSET_X));
		fiducial_offset.y = -config.getFloatProperty(MSPParams.T265_FIDUCIAL_OFFSET_Y, String.valueOf(FIDUCIAL_OFFSET_Y));
		fiducial_offset.z = -config.getFloatProperty(MSPParams.T265_FIDUCIAL_OFFSET_Z, String.valueOf(FIDUCIAL_OFFSET_Z));

		System.out.println("Fiducial offset: "+fiducial_offset);

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

		control.getStatusManager().addListener(StatusManager.TYPE_MSP_AUTOPILOT,
				MSP_AUTOCONTROL_MODE.PRECISION_LOCK, StatusManager.EDGE_RISING, (n) -> {
					if(model.sys.isAutopilotMode(MSP_AUTOCONTROL_MODE.PRECISION_LOCK))
						writeLogMessage(new LogMessage("[vio] PrecisionLock enabled", MAV_SEVERITY.MAV_SEVERITY_NOTICE));
				});


		// reset vision when absolute position lost and odometry if published
		control.getStatusManager().addListener(StatusManager.TYPE_ESTIMATOR, ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS, StatusManager.EDGE_FALLING, (n) -> {
			if(model.vision.isStatus(Vision.ENABLED))
				init("EKF2");
		});

		detector = FactoryFiducial.squareBinary(new ConfigFiducialBinary(fiducial_size), ConfigThreshold.local(ThresholdType.LOCAL_MEAN, 25), GrayU8.class);

		try {
			t265 = StreamRealSenseT265Pose.getInstance(StreamRealSenseT265Pose.POS_DOWNWARD_180,width,height);
		} catch( IllegalArgumentException e) {
			System.out.println("No T265 device found");
			return;

		}


		t265.registerCallback((tms, raw, p, s, a, img) ->  {

			// Bug in CB; sometimes called twice
			if((tms-tms_old) < 3)
				return;

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
				}
				break;
			default:
				System.out.println("TrackerConfidence is "+raw.tracker_confidence);
			}

			model.vision.setStatus(Vision.AVAILABLE, true);
			
			if(raw.tracker_confidence <= StreamRealSenseT265Pose.CONFIDENCE_LOW && confidence_old != raw.tracker_confidence) {
				control.writeLogMessage(new LogMessage("[vio] T265 Tracker confidence low", MAV_SEVERITY.MAV_SEVERITY_WARNING));
				// TODO: Action here
			}

			confidence_old = raw.tracker_confidence;

			// Reset procedure 

			// Note: This takes 1.5sec for T265;

			if((System.currentTimeMillis() - tms_reset) < 2000) {
				quality = 0; error_count=0; tms_reset = 0; confidence_old = 0;

				model.vision.setStatus(Vision.POS_VALID, false);

				// set initial T265 pose as origin
				to_body.setTranslation(- p.T.x , - p.T.y , - p.T.z );

				MSP3DUtils.convertModelToSe3_F64(model, to_ned);

				if(!is_originset) {
					to_ned.T.set(0,0,0);
					publishPX4OdometryZero(MAV_FRAME.MAV_FRAME_LOCAL_NED,tms);
					is_originset = true;
				}

				// Rotate offset to NED
				GeometryMath_F64.mult(to_ned.R, offset, offset_r );

				// Subtract offset from ned position
				offset_r.scale(-1);
				to_ned.T.plusIP(offset_r);

				CommonOps_DDRM.transpose(p.R, tmp);
				CommonOps_DDRM.mult( to_ned.R, tmp , initial_rot );

				if(lensDistortion == null) {
					CameraKannalaBrandt model = t265.getLeftModel();
					// Adjust intrinsics as fiducial size is changed
					model.set(FIDUCIAL_WIDTH,FIDUCIAL_HEIGHT);
					lensDistortion = new LensDistortionPinhole(model);
				}
				detector.setLensDistortion(lensDistortion,FIDUCIAL_WIDTH, FIDUCIAL_HEIGHT);

				precision_lock.set(Double.NaN,Double.NaN,Double.NaN, Double.NaN);
				model.vision.setStatus(Vision.FIDUCIAL_LOCKED, false);

				ned_s.T.set(Double.NaN,Double.NaN, Double.NaN);

				// Clear validators
				avg_z_speed_dev.clear();
				avg_xy_speed_dev.clear();
				avg_att_dev.clear();

				xy_pos_jump.reset();

				is_fiducial = false;
				error_count = 0;

				if(stream!=null && enableStream) {
					stream.addToStream(img, model, tms);
				}
				model.vision.setStatus(Vision.RESETTING, true);
				tms_old = tms;
				return;
			}

			model.vision.setStatus(Vision.RESETTING, false);

			if(error_count > MAX_ERRORS) {
				init("quality");
				return;
			}

			is_originset = true;
			dt_sec = (tms - tms_old) / 1000f;
			model.vision.fps = 1f / dt_sec;
			tms_old = tms;

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
				if(check_speed_xy) {
					writeLogMessage(new LogMessage("[vio] T265 XY speed drift vs local.", MAV_SEVERITY.MAV_SEVERITY_INFO));
					error_count++;
					//			init("speedXY");
					avg_xy_speed_dev.clear();
					return;
				}
			}
			
			// Put EV acceleration into debug
			model.debug.x = (float)a.T.x;
			model.debug.y = (float)a.T.y;
			model.debug.z = (float)a.T.z;

			// XYPos jump detection
			if(xy_pos_jump.isJump(ned.T.x, ned.T.y, dt_sec)) {
				writeLogMessage(new LogMessage("[vio] T265 XY position jump detected.", MAV_SEVERITY.MAV_SEVERITY_ERROR));
				error_count++;
				//init("posJump");
				return;
			}

			if(Math.abs(ned.T.z - lpos.T.z) > MAX_ALT_DEVIATION) {
				writeLogMessage(new LogMessage("[vio] EKF2 altitude not aligned to EV.", MAV_SEVERITY.MAV_SEVERITY_ERROR));
				error_count++;
				init("altitude");
				return;
			}

			// PoseCeck: Difference between LPOS and Vision (Avoid EKF2 reset of horizontal position)
			if(  ( (ned.T.x - lpos.T.x) * (lpos.T.x - lpos.T.x) + (ned.T.y - lpos.T.y) * (lpos.T.y - lpos.T.y) ) > 0.5f) {
				writeLogMessage(new LogMessage("[vio] T265 XY position inconsistency.", MAV_SEVERITY.MAV_SEVERITY_ERROR));
				error_count++;
				init("pose");
			}

			// calculate average mean for Z-speed on ground
			if(!model.sys.isStatus(Status.MSP_ARMED)) {
				avg_z_speed_dev.add(Math.abs(ned_s.T.z));

				// Speed check: Is visual Z speed acceptable
				if(avg_z_speed_dev.getMeanAbs() > MAX_SPEED_Z_DEVIATION) {
					if(check_speed_z) {
						writeLogMessage(new LogMessage("[vio] T265 Z speed drift on ground.", MAV_SEVERITY.MAV_SEVERITY_CRITICAL));
						error_count++;
						init("speedZ");
						avg_z_speed_dev.clear();
						return;
					}
				}
			} else
				avg_z_speed_dev.clear();

			// check attitude drift
			avg_att_dev.add((model.attitude.p - att.getPitch()) * (model.attitude.p - att.getPitch())
					+ (model.attitude.r - att.getRoll())  * (model.attitude.r - att.getRoll()) 
					);

			if(avg_att_dev.getMeanAbs() > MAX_ATT_DEVIATION_SQ ) {
				writeLogMessage(new LogMessage("[vio] T265 attitude drift detected.", MAV_SEVERITY.MAV_SEVERITY_INFO));
				avg_att_dev.clear();
				init("attitude");
				return;
			}


			model.vision.setStatus(Vision.FIDUCIAL_ENABLED, model.sys.isAutopilotMode(MSP_AUTOCONTROL_MODE.PRECISION_LOCK));
			img.bands[0].subimage(fiducial_x_offs, fiducial_y_offs, width-fiducial_x_offs, height-fiducial_y_offs, fiducial);

			// Publishing data

			if(!model.vision.isStatus(Vision.ENABLED)) {

				// Add left camera to stream
				if(stream!=null && enableStream) {
					stream.addToStream(img, model, tms);
				}

				model.vision.setStatus(Vision.PUBLISHED, false);
				publishMSPGroundTruth(ned);
				return;
			}


			model.vision.setStatus(Vision.POS_VALID, true);

			switch(mode) {

			case GROUNDTRUTH_MODE:

				// just set ground truth in local model and send to GCL
				publishMSPGroundTruth(ned);
				break;


			case LPOS_ODO_MODE_NED_GND:

				// Publish position data NED frame, speed body frame;  with ground truth
				publishPX4Odometry(ned,body_s,MAV_FRAME.MAV_FRAME_LOCAL_NED,true,tms);
				publishMSPVision(gnd_ned,ned,ned_s,precision_lock,tms);

				break;

			case LPOS_ODO_MODE_NED_PREC:


				// Publish position and speed data body frame; with precision lock
				publishPX4Odometry(body,body_s,MAV_FRAME.MAV_FRAME_LOCAL_NED, true,tms);
				publishMSPVision(precision_ned,ned,ned_s,precision_lock,tms);

				break;

			}

			// Transfer to local model
			model.vision.setAttitude(att);
			model.vision.setPrecisionOffset(precision_lock);
			model.vision.setPosition(ned.T);
			model.vision.setSpeed(ned_s.T);
			model.vision.tms = tms * 1000;

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
		if(t265!=null) {
			tms_reset = System.currentTimeMillis();
			reset_count++;
			t265.reset();
			control.writeLogMessage(new LogMessage("[vio] T265 reset ["+s+"]", MAV_SEVERITY.MAV_SEVERITY_WARNING));
		}
	}


	public void start() {
		if(t265==null)
			return;
		t265.start();
		System.out.println("[vio] Starting T265....");
		t265.printDeviceInfo();
		wq.addSingleTask("LP", 4000, () -> { 
			if(!model.sys.isStatus(Status.MSP_ARMED)) { 
				is_originset = false; 
				init("init"); 
			} 
		});

		fiducial_worker = wq.addCyclicTask("LP", FIDUCIAL_RATE_SCAN, new FiducialHandler());
	}

	public void stop() {
		wq.removeTask("LP", fiducial_worker);
		if(t265==null)
			return;
		t265.stop();
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
			stmp = flocked.format(-precision_lock.z);
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


	private void publishPX4Odometry(Se3_F64 pose, Se3_F64 speed, int frame, boolean pose_is_valid, long tms) {

		odo.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION;
		odo.frame_id       = frame;
		odo.child_frame_id = MAV_FRAME.MAV_FRAME_BODY_FRD;

		//		System.out.println(DataModel.getSynchronizedPX4Time_us()+"->"+(tms*1000));

		odo.time_usec =  tms * 1000;


		if(pose_is_valid) {
			odo.x = (float) pose.T.x;
			odo.y = (float) pose.T.y;
			odo.z = (float) pose.T.z;
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
		model.vision.setStatus(Vision.PUBLISHED, true);

	}

	private void publishPX4OdometryZero(int frame, long tms) {

		odo.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION;
		odo.frame_id       = frame;
		odo.child_frame_id = MAV_FRAME.MAV_FRAME_BODY_FRD;

		odo.time_usec = tms * 1000;

		odo.x = 0f;
		odo.y = 0f;
		odo.z = 0f;


		odo.vx = 0f;
		odo.vy = 0f;
		odo.vz = 0f;

		// Use EKF params
		odo.pose_covariance[0] = Float.NaN;
		odo.velocity_covariance[0] = Float.NaN;

		// do not use twist
		odo.q[0] = Float.NaN;

		odo.reset_counter = reset_count;

		control.sendMAVLinkMessage(odo);

	}



	private void publishMSPGroundTruth(Se3_F64 pose) {

		model.vision.setGroundTruth(pose.T);
		msg.gx    =  model.vision.gx;
		msg.gy    =  model.vision.gy;
		msg.gz    =  model.vision.gz;
		msg.flags = model.vision.flags;
		control.sendMAVLinkMessage(msg);

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
		msg.fps     = model.vision.fps;

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

	private class FiducialHandler implements Runnable {

		@Override
		public void run() {
			// Precision lock procedure 
			if(!model.sys.isAutopilotMode(MSP_AUTOCONTROL_MODE.PRECISION_LOCK)) {
				model.vision.setStatus(Vision.FIDUCIAL_LOCKED, false);
				return;
			}

			//	&& ( (System.currentTimeMillis() - fiducial_tms) > FIDUCIAL_RATE)) {
			fiducial_tms = System.currentTimeMillis();

			if((System.currentTimeMillis() - locking_tms) > LOCK_TIMEOUT) {
				precision_lock.set(Double.NaN,Double.NaN,Double.NaN, Double.NaN);
				model.vision.setStatus(Vision.FIDUCIAL_LOCKED, false);
			}

			try {

				//TODO: Enhance Image
				//				ImageStatistics.histogram(gray,0, histogram);
				//				EnhanceImageOps.equalize(histogram, transform);
				//				EnhanceImageOps.applyTransform(gray, transform, adjusted);

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

						precision_lock.set(lpos.T.x-precision_ned.T.x,lpos.T.y-precision_ned.T.y,precision_ned.T.z,
								MSPMathUtils.normAngle((float)fiducial_att.getYaw()-(float)Math.PI+model.attitude.y));

						// TODO: Check consistency of lock with LIDAR data or altitude above ground

						model.vision.setStatus(Vision.FIDUCIAL_LOCKED, true);
						locking_tms = System.currentTimeMillis();
					}
				}

			} catch(Exception e ) {
				System.out.println(e.getMessage());
				precision_lock.set(Double.NaN,Double.NaN,Double.NaN, Double.NaN);
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
