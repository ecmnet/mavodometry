package com.comino.mavodometry.estimators;

/****************************************************************************
 *
 *   Copyright (c) 2020 Eike Mansfeld ecm@gmx.de. All rights reserved.
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

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.mavlink.messages.MAV_FRAME;
import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.MSP_CMD;
import org.mavlink.messages.MSP_COMPONENT_CTRL;
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
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavodometry.librealsense.t265.boofcv.StreamRealSenseT265Pose;
import com.comino.mavodometry.struct.Attitude3D_F64;
import com.comino.mavodometry.video.IVisualStreamHandler;

import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import georegression.geometry.GeometryMath_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;

public class MAVT265PositionEstimator {

	private static final int     	 MAX_ERRORS = 10;
	private static final float  	  MAX_SPEED = 1.0f;

	// mounting offset in m
	private static final double   	   OFFSET_X =  0.00;
	private static final double        OFFSET_Y =  0.00;
	private static final double        OFFSET_Z =  0.00;

	// Modes
	public static final int  GROUNDTRUTH_MODE   = 1;
	public static final int  LPOS_MODE_NED      = 2;
	public static final int  LPOS_MODE_BODY     = 3;

	// MAVLink messages
	private final msg_vision_position_estimate sms = new msg_vision_position_estimate(1,2);
	private final msg_msp_vision               msg = new msg_msp_vision(2,1);
	private final msg_odometry                 odo = new msg_odometry(2,1);

	// Controls
	private StreamRealSenseT265Pose t265;
	private IMAVMSPController       control;
	private DataModel               model;

	// 3D transformation matrices
	private Se3_F64          to_ned   = new Se3_F64();
	private Se3_F64          to_body  = new Se3_F64();

	private Se3_F64          ned      = new Se3_F64();
	private Se3_F64          body     = new Se3_F64();

	private DMatrixRMaj   tmp         = CommonOps_DDRM.identity( 3 );
	private DMatrixRMaj   initial_rot = CommonOps_DDRM.identity( 3 );

	//private Quaternion_F64   qbody    = new Quaternion_F64();

	// 3D helper structures
	private Vector3D_F64     offset     = new Vector3D_F64();
	private Vector3D_F64     offset_r   = new Vector3D_F64();

	private Attitude3D_F64   att      = new Attitude3D_F64();

	private float             quality = 0;
	private long              tms_old = 0;
	private long            tms_reset = 0;

	private boolean         do_odometry = true;

	private int           error_count = 0;
	private int           reset_count = 0;

	private Planar<GrayU8>  img = null;

	// Stream data
	private int   width;
	private int   height;

	private final Color	bgColor = new Color(128,128,128,130);

	public <T> MAVT265PositionEstimator(IMAVMSPController control,  MSPConfig config, int width, int height,int mode) {
		this(control,config,width,height,mode,null);
	}

	public <T> MAVT265PositionEstimator(IMAVMSPController control,  MSPConfig config, int width, int height,int mode, IVisualStreamHandler<Planar<GrayU8>> stream) {


		this.control = control;
		this.width   = width;
		this.height  = height;
		this.model   = control.getCurrentModel();

		this.img = new Planar<GrayU8>(GrayU8.class,width,height,1);

		// read offsets from config
		offset.x = -config.getFloatProperty("t265_offset_x", String.valueOf(OFFSET_X));
		offset.y = -config.getFloatProperty("t265_offset_y", String.valueOf(OFFSET_Y));
		offset.z = -config.getFloatProperty("t265_offset_z", String.valueOf(OFFSET_Z));


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
						init("reset"); break;
					}
					break;
				}
			}
		});


		// reset vision when armed
//		control.getStatusManager().addListener( Status.MSP_ARMED, (n) -> {
//			if(n.isStatus(Status.MSP_ARMED)) {
//				init("armed");
//			}
//		});


		//reset ned transformation when GPOS gets valid
		control.getStatusManager().addListener(Status.MSP_GPOS_VALID, (n) -> {
			//if((n.isStatus(Status.MSP_GPOS_VALID)))
			init("gpos");
		});

		if(stream!=null) {
			stream.registerOverlayListener(ctx -> {
				overlayFeatures(ctx);
			});
		}


		t265 = new StreamRealSenseT265Pose(StreamRealSenseT265Pose.POS_FOREWARD,width,height,(tms, raw, p, s, a, left, right) ->  {

			if(raw.tracker_confidence == 0) {
				quality = 0f; error_count++;
			}
			else if(raw.tracker_confidence == 1)
				quality = 0.33f;
			else if(raw.tracker_confidence == 2)
				quality = 0.66f;
			else if(raw.tracker_confidence == 3)
				quality = 1f;

			// Initializing odometry
			// Note: This takes 1.5sec for T265
			if((System.currentTimeMillis() - tms_reset) < 1500) {
				quality = 0; error_count=0;

				// set initial T265 pose as origin
				to_body.setTranslation(- p.T.x , - p.T.y , - p.T.z );

				MSP3DUtils.convertModelToSe3_F64(model, to_ned);

				// Rotate offset to NED
				GeometryMath_F64.mult(to_ned.R, offset, offset_r );

				// Subtract offset from ned position
				offset_r.scale(-1);
				to_ned.T.plusIP(offset_r);

				CommonOps_DDRM.transpose(p.R, tmp);
				CommonOps_DDRM.mult( to_ned.R, tmp , initial_rot );

				return;
			}


			tms_reset = 0;

			// Valdidations

			if(error_count > MAX_ERRORS) {
				init("quality");
				return;
			}

			if(s.T.norm()>MAX_SPEED) {
				init("speed");
				return;
			}

			// rotate to body => visual attitude = 0
			CommonOps_DDRM.transpose(p.R, to_body.R);
			p.concat(to_body, body);


			// Get model attitude rotation
			MSP3DUtils.convertModelRotationToSe3_F64(model, to_ned);

			// rotate position and offset to ned
			body.concat(to_ned, ned);
			GeometryMath_F64.mult(to_ned.R, offset, offset_r );

			// add rotated offset
			ned.T.plusIP(offset_r);

			//			System.out.println("P=         "+p.T);
			//			System.out.println("ToBody=    "+to_body.T);
			//			System.out.println("Body=      "+body.T);
			//			System.out.println("ToNed=     "+to_ned.T);
			//			System.out.println("OffsetR=   "+offset_r);
			//			System.out.println("Ned=        "+ned.T);
			//			System.out.println("----------");

			// Set rotation to vision based rotation
			CommonOps_DDRM.mult( initial_rot, p.R , ned.R );

			// get euler angles
			att.setFromMatrix(ned.R);

			switch(mode) {

			case GROUNDTRUTH_MODE:

				// just set ground truth in local model
				model.vision.gx =  (float)ned.getX();
				model.vision.gy =  (float)ned.getY();
				model.vision.gz =  (float)ned.getZ();

				break;

				// use msg_vision
			case LPOS_MODE_NED:

				// Publish position data NED
				if(do_odometry)
					publishPX4Vision(ned,tms);
				publishMSPVision(body,ned,tms);

				break;

				// use msg_odometry
			case LPOS_MODE_BODY:

				// Publish position data body frame
				if(do_odometry)
					publishPX4Odometry(body,tms);
				publishMSPVision(p,ned,tms);

				break;
			}

			// Add left camera to stream
			if(stream!=null) {
				img.bands[0] = left;
				stream.addToStream(img, model, tms);
			}
		});

	System.out.println("T265 controller initialized with mounting offset "+offset);

	}

	public void init(String s) {
		reset_count++;
		t265.reset();
		this.control.writeLogMessage(new LogMessage("[vio] Estimation init ["+s+"]", MAV_SEVERITY.MAV_SEVERITY_NOTICE));
		tms_reset = System.currentTimeMillis();
	}


	public void start() {
		t265.start();
		System.out.println("[vio] Starting T265....");
		t265.printDeviceInfo();
		tms_reset = System.currentTimeMillis()+1000;
	}

	public void stop() {
		t265.stop();
	}

	private void overlayFeatures(Graphics ctx) {

		ctx.setColor(bgColor);
		ctx.fillRect(5, 5, width-10, 21);
		ctx.setColor(Color.white);

		ctx.drawLine(width/2-10, height/2, width/2+10, height/2);
		ctx.drawLine(width/2, height/2-10, width/2, height/2+10);

		if(!Float.isNaN(model.sys.t_armed_ms) && model.sys.isStatus(Status.MSP_ARMED)) {
			ctx.drawString(String.format("%.1f sec",model.sys.t_armed_ms/1000), 20, 20);
		}

		if(model.msg.text != null && (model.sys.getSynchronizedPX4Time_us()-model.msg.tms) < 1000000)
			ctx.drawString(model.msg.text, 10, height-5);

	}


	private void publishPX4Vision(Se3_F64 pose, long tms) {


		sms.usec = tms * 1000;

		sms.x = (float) pose.T.x;
		sms.y = (float) pose.T.y;
		sms.z = (float) pose.T.z;

		sms.reset_counter = reset_count;

		sms.roll  = (float)att.getRoll();
		sms.pitch = (float)att.getPitch();
		sms.yaw   = (float)att.getYaw();

		sms.covariance[0] = Float.NaN;

		control.sendMAVLinkMessage(sms);
		//
		//		smv.usec = tms;
		//
		//		smv.x = (float) speed.x;
		//		smv.y = (float) speed.y;
		//		smv.z = (float) speed.z;
		//
		//		control.sendMAVLinkMessage(smv);

		model.sys.setSensor(Status.MSP_OPCV_AVAILABILITY, true);

	}

	private void publishMSPVision(Se3_F64 orig,Se3_F64 pose, long tms) {

		msg.x =  (float) pose.T.x;
		msg.y =  (float) pose.T.y;
		msg.z =  (float) pose.T.z;

		msg.gx =  (float) orig.T.x;
		msg.gy =  (float) orig.T.y;
		msg.gz =  (float) orig.T.z;

		msg.h   = (float)att.getYaw();
		msg.r   = (float)att.getRoll();
		msg.p   = (float)att.getPitch();

		msg.quality = (int)(quality * 100f);
		msg.errors  = error_count;
		msg.tms = tms * 1000;

		if(tms_old > 0) {
			msg.fps = 1000 / (tms - tms_old);
		}
		tms_old = tms;

		control.sendMAVLinkMessage(msg);


	}

	private void publishPX4Odometry(Se3_F64 pose, long tms) {

		odo.time_usec = tms * 1000;

		odo.x = (float) pose.T.x;
		odo.y = (float) pose.T.y;
		odo.z = (float) pose.T.z;

		odo.frame_id = MAV_FRAME.MAV_FRAME_LOCAL_FRD;

		//		ConvertRotation3D_F64.matrixToQuaternion(pose.R, qbody);
		//		odo.q[0] = (float)qbody.w;
		//		odo.q[1] = (float)qbody.x;
		//		odo.q[2] = (float)qbody.y;
		//		odo.q[3] = (float)qbody.z;

		control.sendMAVLinkMessage(odo);

		model.sys.setSensor(Status.MSP_OPCV_AVAILABILITY, true);

	}



}
