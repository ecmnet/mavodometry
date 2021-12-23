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
import org.mavlink.messages.MAV_ESTIMATOR_TYPE;
import org.mavlink.messages.MAV_FRAME;
import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.MSP_AUTOCONTROL_MODE;
import org.mavlink.messages.MSP_CMD;
import org.mavlink.messages.MSP_COMPONENT_CTRL;
import org.mavlink.messages.lquac.msg_msp_command;
import org.mavlink.messages.lquac.msg_msp_vision;
import org.mavlink.messages.lquac.msg_odometry;
import org.mavlink.messages.lquac.msg_set_gps_global_origin;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.config.MSPParams;
import com.comino.mavcom.control.IMAVMSPController;
import com.comino.mavcom.log.MSPLogger;
import com.comino.mavcom.mavlink.IMAVLinkListener;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.model.segment.Vision;
import com.comino.mavcom.status.StatusManager;
import com.comino.mavcom.struct.Attitude3D_F64;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavcom.utils.SimpleLowPassFilter;
import com.comino.mavodometry.estimators.MAVAbstractEstimator;
import com.comino.mavodometry.librealsense.t265.boofcv.StreamRealSenseT265PoseCV;
import com.comino.mavodometry.librealsense.t265.boofcv.StreamRealSenseT265PoseLegacy;
import com.comino.mavodometry.librealsense.utils.RealSenseInfo;
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
import georegression.struct.so.Quaternion_F64;

public class MAVT265PositionEstimator extends MAVAbstractEstimator {

	// Modes
	public static final int  GROUNDTRUTH_MODE       			= 1;
	public static final int  LPOS_ODO_MODE_POSITION 		    = 2;

	private static final int         FIDUCIAL_ID            	= 284;
	private static final float       FIDUCIAL_SIZE          	= 0.168f;
	private static final int         FIDUCIAL_RATE_SCAN     	= 500;
	private static final int         FIDUCIAL_RATE_ACTIVE   	= 200;

	private static final int         FIDUCIAL_HEIGHT     		= 360;
	private static final int         FIDUCIAL_WIDTH     		= 360;

	private static final int     	 MAX_ERRORS          		= 200;
	private static final boolean     DO_EV_REPOSITION       	= false;

	private static final float       MAX_ATT_DEVIATION_SQ       = 0.5f * 0.5f;
	private static final float       MAX_XY_POS_DEVIATION_SQ    = 0.3f * 0.3f;

	private static final long        LOCK_TIMEOUT_MS            = 1000;

	// mounting offset in m
	private static final double   	 OFFSET_X 					=   0.00;
	private static final double      OFFSET_Y 					=   0.00;
	private static final double      OFFSET_Z 					=   0.00;

	private static final double      FIDUCIAL_OFFSET_X 			=  -0.08;
	private static final double      FIDUCIAL_OFFSET_Y 			=   0.05;
	private static final double      FIDUCIAL_OFFSET_Z 			=   0.00;


	private final WorkQueue wq = WorkQueue.getInstance();
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
	private final Se3_F64          body_old         = new Se3_F64();
	private final Se3_F64          body_s   		= new Se3_F64();
	private final Se3_F64          lpos     		= new Se3_F64();

	private final DMatrixRMaj   tmp         	= CommonOps_DDRM.identity( 3 );
	private final DMatrixRMaj   initial_rot 	= CommonOps_DDRM.identity( 3 );

	// 3D helper structures
	private final Vector3D_F64  offset     		= new Vector3D_F64();
	private final Vector3D_F64  offset_pos_ned  = new Vector3D_F64();
	private final Vector3D_F64  offset_vel_body = new Vector3D_F64();
	private final Vector3D_F64  angular_rates   = new Vector3D_F64();
	private final Vector3D_F64  lpos_current_s  = new Vector3D_F64();
	private final Vector3D_F64  vpos_current_s  = new Vector3D_F64();
	private final Vector3D_F64  vpos_delta_s    = new Vector3D_F64();
	private final Vector3D_F64  reposition_ned  = new Vector3D_F64();

	private final Attitude3D_F64   att      	= new Attitude3D_F64();
	private final Quaternion_F64   att_q        = new Quaternion_F64();

	private float             quality = 0;
	private long              tms_old = 0;
	private long            tms_reset = 0;
	private int        confidence_old = 0;
	private double             dt_sec = 0;
	private double           dt_sec_1 = 0;

	private boolean    enableStream   = false;

	private final SimpleLowPassFilter        avg_att_dev      = new SimpleLowPassFilter(0.05);

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

	// Stream data
	private int   width4;

	private final DecimalFormat flocked2  = new DecimalFormat("LockAlt: #0.00m");
	private final DecimalFormat flocked1 = new DecimalFormat("LockAlt: #0.00m");
	private String stmp;

	private GrayU8 fiducial = new GrayU8(1,1);
	private int fiducial_worker;



	@SuppressWarnings("unused")
	public <T> MAVT265PositionEstimator(IMAVMSPController control,  MSPConfig config, int width, int height, int mode, IVisualStreamHandler<Planar<GrayU8>> stream) 
			throws Exception {

		super(control);

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

		fiducial_size   = config.getFloatProperty(MSPParams.T265_FIDUCIAL_SIZE,String.valueOf(FIDUCIAL_SIZE));
		System.out.println("Fiducial size: "+fiducial_size+"m");

		fiducial_offset.x = -config.getFloatProperty(MSPParams.T265_FIDUCIAL_OFFSET_X, String.valueOf(FIDUCIAL_OFFSET_X));
		fiducial_offset.y = -config.getFloatProperty(MSPParams.T265_FIDUCIAL_OFFSET_Y, String.valueOf(FIDUCIAL_OFFSET_Y));
		fiducial_offset.z = -config.getFloatProperty(MSPParams.T265_FIDUCIAL_OFFSET_Z, String.valueOf(FIDUCIAL_OFFSET_Z));

		System.out.println("T265 Fiducial offset: "+fiducial_offset);

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
				case MSP_CMD.MSP_CMD_SET_HOMEPOS:
					setGlobalOrigin(cmd.param1 / 1e7f, cmd.param2 / 1e7f, cmd.param3 / 1e3f );
					break;
				}
			}
		});

		control.getStatusManager().addListener(StatusManager.TYPE_MSP_AUTOPILOT,
				MSP_AUTOCONTROL_MODE.PRECISION_LOCK, StatusManager.EDGE_RISING, (n) -> {
					if(model.sys.isAutopilotMode(MSP_AUTOCONTROL_MODE.PRECISION_LOCK))
						writeLogMessage(new LogMessage("[vio] PrecisionLock enabled", MAV_SEVERITY.MAV_SEVERITY_NOTICE));
				});

		detector = FactoryFiducial.squareBinary(new ConfigFiducialBinary(fiducial_size), ConfigThreshold.local(ThresholdType.LOCAL_MEAN, 25), GrayU8.class);

		t265 = StreamRealSenseT265PoseCV.getInstance(StreamRealSenseT265PoseCV.POS_DOWNWARD_180,width,height);
		t265.registerCallback((tms, confidence, p, s, a, img) ->  {

			// Bug in CB; sometimes called twice
			if((tms-tms_old) < 3)
				return;

			switch(confidence) {
			case CONFIDENCE_FAILED:
				quality = 0.00f; error_count++; is_fiducial = false; 
				is_initialized = false;
				break;
			case CONFIDENCE_LOW:
				quality = 0.33f; error_count++;
				break;
			case CONFIDENCE_MEDIUM:
				quality = 0.66f; error_count = 0;
				break;
			case CONFIDENCE_HIGH:
				if(confidence_old != CONFIDENCE_HIGH) {
					quality = 1f; error_count = 0;
				}
				break;
			default:
				System.out.println("TrackerConfidence is "+confidence);
			}


			// Reset procedure ------------------------------------------------------------------------------------------------
			// Note: This takes 1.5sec for T265;

			if((System.currentTimeMillis() - tms_reset) < 2500 || !is_initialized) {
				tms_reset = 0; confidence_old = 0; is_initialized = true; error_count = 0;

				// set initial T265 pose as origin
				to_body.setTranslation(- p.T.x , - p.T.y , - p.T.z );

				MSP3DUtils.convertModelToSe3_F64(model, to_ned);
				reposition_ned.setTo(0,0,0);

				// Rotate offset to NED
				GeometryMath_F64.mult(to_ned.R, offset, offset_pos_ned );

				// Subtract offset from ned position
				offset_pos_ned.scale(-1);
				to_ned.T.plusIP(offset_pos_ned);

				CommonOps_DDRM.transpose(p.R, tmp);
				CommonOps_DDRM.mult( to_ned.R, tmp , initial_rot );

				p.concat(to_body, body_old);

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

				// Clear validators
				avg_att_dev.clear();

				is_fiducial = false;

				if(t265.isVideoEnabled() && stream!=null && enableStream && img != null) {
					stream.addToStream(img, model, tms);
				}

				body_old.setTo(body);
				tms_old  = tms; 
				return;
			}

			model.vision.setStatus(Vision.AVAILABLE, true);
			if(!t265.isVideoEnabled()) {
				model.sys.setAutopilotMode(MSP_AUTOCONTROL_MODE.PRECISION_LOCK, false);
			}

			if(confidence <= CONFIDENCE_LOW && confidence_old != confidence) {
				control.writeLogMessage(new LogMessage("[vio] T265 Tracker confidence low", MAV_SEVERITY.MAV_SEVERITY_WARNING));
				// TODO: Action here
			}

			confidence_old = confidence;

			// timea and fps
			dt_sec   = (tms - tms_old) / 1000f;
			dt_sec_1 = 1.0 / dt_sec;
			model.vision.fps = (float)dt_sec_1;
			tms_old = tms;

			model.vision.setStatus(Vision.RESETTING, false);

			if(error_count > MAX_ERRORS) {
				init("maxErrors");
				return;
			}


			//No flight controller connected => publish raw pose and speed for debugging purpose
			if(!model.sys.isStatus(Status.MSP_CONNECTED)) {
				if(stream!=null && enableStream) {
					stream.addToStream(img, model, tms);
				}
				model.vision.setStatus(Vision.PUBLISHED, true);
				publishMSPVision(gnd_ned,p,s,precision_lock,tms);
				return;
			}

			// Transformation to NED

			// Get current position and speeds
			MSP3DUtils.convertModelToSe3_F64(model, lpos);
			MSP3DUtils.convertCurrentSpeed(model, lpos_current_s);

			CommonOps_DDRM.transpose(p.R, to_body.R);

			body_old.setTo(body);
			p.concat(to_body, body);

			// calculate body speed based on position
			vpos_current_s.x = ( body.getX() - body_old.getX() ) * dt_sec_1;
			vpos_current_s.y = ( body.getY() - body_old.getY() ) * dt_sec_1;
			vpos_current_s.z = ( body.getZ() - body_old.getZ() ) * dt_sec_1;

			// Debug body speed based on position
			// model.debug.set(vpos_current_s);

			// rotate sensor velocities to body frame and correct by offset
			GeometryMath_F64.mult(to_body.R, s.T, body_s.T);

			// eventually calculate rr,pr,yr and do not use model rates
			angular_rates.setTo(model.attitude.rr, model.attitude.pr, model.attitude.yr);
			offset_vel_body.crossSetTo(angular_rates, offset);
			offset_vel_body.scale(-1);
			body_s.T.plusIP(offset_vel_body);
			
			// Debug speed offsets 
			model.debug.set(offset_vel_body);

			// Get model attitude rotation
			MSP3DUtils.convertModelRotationToSe3_F64(model, to_ned);

			// rotate position and offset to ned
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

			// Consistency checks
			model.vision.setStatus(Vision.POS_VALID, true);
			model.vision.setStatus(Vision.SPEED_VALID, true);

			ned.T.plusIP(reposition_ned);

			// speed variance body speed between pos based and vision
			vpos_delta_s.setTo(body_s.T);
			vpos_delta_s.scale(-1);
			vpos_delta_s.plusIP(vpos_current_s);


			// Speed variance too high => assuming position is not correct, e.g. PoseJump occurred
			if(vpos_delta_s.norm() > 0.1) {
				model.vision.setStatus(Vision.POS_VALID, false);
				if(DO_EV_REPOSITION) {
					do_repositioning("XYSpeedDev");
				}
				//System.out.println("VisionSpeed vs. VisionPosSpeed: "+vpos_delta_s);
			}


			// Validate XY position
			if(((ned.T.x - lpos.T.x) * (ned.T.x - lpos.T.x) + (ned.T.y - lpos.T.y) * (ned.T.y - lpos.T.y)) >  MAX_XY_POS_DEVIATION_SQ) {
				// TODO: Trigger repositioning event
				model.vision.setStatus(Vision.POS_VALID, false);
				if(DO_EV_REPOSITION) {
					do_repositioning("XYDev");
					return;
				}
			}


			// Validate attitude drift
			avg_att_dev.add((model.attitude.p - att.getPitch()) * (model.attitude.p - att.getPitch())
					+ (model.attitude.r - att.getRoll())  * (model.attitude.r - att.getRoll()) 
					);
			if(avg_att_dev.getMeanAbs() > MAX_ATT_DEVIATION_SQ ) {
				writeLogMessage(new LogMessage("[vio] T265 attitude drift.", MAV_SEVERITY.MAV_SEVERITY_INFO));
				avg_att_dev.clear();
				model.vision.setStatus(Vision.POS_VALID, false);
				model.vision.setStatus(Vision.SPEED_VALID, false);
				init("attitude");
				return;
			}

			// TODO: Z validation

			// Fiducial detection
			model.vision.setStatus(Vision.FIDUCIAL_ENABLED, model.sys.isAutopilotMode(MSP_AUTOCONTROL_MODE.PRECISION_LOCK));
			if(img!=null) 
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

			// Publishing data
			switch(mode) {

			case GROUNDTRUTH_MODE:

				// just set ground truth in local model and send to GCL
				publishMSPGroundTruth(ned);
				break;


			case LPOS_ODO_MODE_POSITION:

				// Publish position data NED frame, speed body frame;  with ground truth
				publishPX4Odometry(ned,body_s,MAV_FRAME.MAV_FRAME_LOCAL_NED,true,tms);
				publishMSPVision(gnd_ned,ned,ned_s,precision_lock,tms);

				break;

			}

			// Transfer to local model
			model.vision.setAttitude(att);
			model.vision.setPosition(ned.T);
			model.vision.setSpeed(ned_s.T);
			model.vision.tms = tms * 1000;

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


		if(t265.getMount() == StreamRealSenseT265PoseCV.POS_DOWNWARD)
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
			quality = 0; error_count=0; 
			t265.reset();
			if(s!=null)
				writeLogMessage(new LogMessage("[vio] T265 reset ["+s+"]", MAV_SEVERITY.MAV_SEVERITY_WARNING));
			model.vision.setStatus(Vision.RESETTING, true);
			model.vision.setStatus(Vision.POS_VALID, false);
			model.vision.setStatus(Vision.SPEED_VALID, false);
			publishMSPFlags(DataModel.getSynchronizedPX4Time_us());
		}
	}

	public void do_repositioning(String s) {
		reposition_ned.x += (lpos.T.x - ned.T.x);
		reposition_ned.y += (lpos.T.y - ned.T.y);
		reposition_ned.z  = 0;
		if(s!=null && model.sys.isStatus(Status.MSP_ARMED))
			writeLogMessage(new LogMessage("[vio] T265 Pos. adjustment ["+s+"]", MAV_SEVERITY.MAV_SEVERITY_WARNING));
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


	// TODO: Mpve to commander
	private void setGlobalOrigin(double lat, double lon, double altitude) {

		if(model.sys.isSensorAvailable(Status.MSP_GPS_AVAILABILITY) || model.sys.isStatus(Status.MSP_GPOS_VALID))
			return;

		// Note: In SITL Set global origin causes BARO failure 
		// TODO: To be investigated in PX4
		if(control.isSimulation())
			return;

		msg_set_gps_global_origin gor = new msg_set_gps_global_origin(1,1);
		gor.target_system = 1;
		gor.latitude = (long)(lat * 1e7);
		gor.longitude = (long)(lon * 1e7);
		gor.altitude = (int)(altitude * 1000);
		gor.time_usec = DataModel.getSynchronizedPX4Time_us();

		control.sendMAVLinkMessage(gor);

		MSPLogger.getInstance().writeLocalMsg("[msp] Setting reference position",
				MAV_SEVERITY.MAV_SEVERITY_INFO);
		init("init");

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


	private void publishPX4Odometry(Se3_F64 pose, Se3_F64 speed, int frame, boolean pose_is_valid, long tms) {

		odo.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION;
		odo.frame_id       = frame;
		odo.child_frame_id = MAV_FRAME.MAV_FRAME_BODY_FRD;

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

	private void publishMSPGroundTruth(Se3_F64 pose) {

		model.vision.setGroundTruth(pose.T);
		msg.gx    =  model.vision.gx;
		msg.gy    =  model.vision.gy;
		msg.gz    =  model.vision.gz;
		msg.flags = model.vision.flags;
		control.sendMAVLinkMessage(msg);

		msg.quality = (int)(quality * 100f);
		msg.errors  = error_count;
		msg.tms     = DataModel.getSynchronizedPX4Time_us();
		msg.flags   = model.vision.flags;
		msg.fps     = model.vision.fps;

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

	private class FiducialHandler implements Runnable {

		//		private final DriftEstimator driftEstimator = new DriftEstimator(5.0);
		//		private final Vector4D_F64   drift           = new Vector4D_F64();

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

					// TODO: Drift speed estimation !!

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

						//						if(!driftEstimator.add(precision_lock)) {
						//							if(driftEstimator.estimate(drift))
						//								System.out.println(drift);
						//							driftEstimator.clear();
						//						}

						// TODO: Check consistency of lock with LIDAR data or altitude above ground

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

	//	private class DriftEstimator {
	//
	//		private final List<Vector4D_F64> lock_positions  = new ArrayList<Vector4D_F64>();
	//		private final Vector4D_F64       lock_start      = new Vector4D_F64();
	//
	//		private final Vector4D_F64       tmp             = new Vector4D_F64();
	//
	//		private double                   max_time        = 5.0;
	//		private boolean                  estimated       = false;
	//
	//
	//		public DriftEstimator(double max_time) {
	//			this.max_time = max_time;
	//		}
	//
	//		public boolean add(double t_sec, Vector4D_F64 pos) {
	//			if(lock_positions.size() > 0 && (t_sec - lock_positions.get(0).w) >= max_time)
	//				return false;
	//			if(lock_positions.isEmpty()) {
	//				lock_start.setTo(pos); lock_start.scale(-1);
	//			}
	//			lock_positions.add(new Vector4D_F64(pos.x,pos.y,pos.z, t_sec));
	//			return true;
	//		}
	//
	//		public boolean add(Vector4D_F64 pos) {
	//			return add(System.currentTimeMillis()/1000.0,pos);
	//		}
	//
	//		public void clear() {
	//			lock_positions.clear();
	//			estimated = false;
	//		}
	//
	//		public int size() {
	//			return lock_positions.size();
	//		}
	//		
	//		public boolean isEstimated() {
	//			return estimated;
	//		}
	//
	//		public boolean estimate(Vector4D_F64 drift_estimate) {
	//			tmp.setTo(0,0,0,0); estimated = false;
	//
	//			if(lock_positions.size() < 2)
	//				return estimated;
	//
	//			final double dt_1 = 1 / (lock_positions.get(lock_positions.size()-1).w - lock_positions.get(0).w);
	//
	//			for(int i=0;i<lock_positions.size();i++) {
	//				// TODO: Variance
	//				tmp.plusIP(lock_positions.get(i));
	//			}
	//			tmp.scale(dt_1);
	//
	//			// TODO: Check validity
	//			drift_estimate.setTo(tmp);
	//			estimated = true;
	//			
	//			return estimated;
	//		}
	//	}
}
