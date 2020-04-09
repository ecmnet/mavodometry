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

import static boofcv.factory.distort.LensDistortionFactory.narrow;

import java.awt.Color;
import java.awt.Graphics;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.control.IMAVMSPController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavodometry.libnano.detetction.NanoObjectDetection;
import com.comino.mavodometry.libnano.segmentation.NanoSegmentation;
import com.comino.mavodometry.libnano.trail.NanoTrailDetection;
import com.comino.mavodometry.libnano.utils.ImageConversionUtil;
import com.comino.mavodometry.libnano.wrapper.JetsonNanoLibrary;
import com.comino.mavodometry.librealsense.r200.RealSenseInfo;
import com.comino.mavodometry.librealsense.r200.boofcv.StreamRealSenseVisDepth;
import com.comino.mavodometry.librealsense.r200.boofcv.StreamRealSenseVisDepth.Listener;
import com.comino.mavodometry.utils.DepthUtils;
import com.comino.mavodometry.video.IVisualStreamHandler;
import com.comino.mavutils.hw.HardwareAbstraction;
import com.sun.jna.ptr.PointerByReference;

import boofcv.alg.distort.PointToPixelTransform_F32;
import boofcv.alg.sfm.DepthSparse3D;
import boofcv.struct.distort.DoNothing2Transform2_F32;
import boofcv.struct.distort.Point2Transform2_F64;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import georegression.geometry.GeometryMath_F64;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;

public class MAVR200DepthEstimator {

	private static final boolean DO_AI = false;


	private static final float MAX_DISTANCE                = 7f;


	// mounting offset in m
	private static final double   	   OFFSET_X =  0.00;
	private static final double        OFFSET_Y =  0.00;
	private static final double        OFFSET_Z =  0.00;

	private StreamRealSenseVisDepth 	realsense	= null;
	private RealSenseInfo               info        = null;
	private DataModel                   model       = null;

	private boolean                     isRunning   = false;

	// Stream data
	private int   width;
	private int   height;

	// Window
	private int   base = 0;
	private int   top  = 40;

	private final Color	bgColor    = new Color(128,128,128,130);
	private final Color	depthColor = new Color(33, 100, 122,80);

	private Se3_F64                to_ned     = new Se3_F64();

	private Vector3D_F64     offset           = new Vector3D_F64();


	private NanoObjectDetection detect    = null;
	private NanoTrailDetection  trail     = null;
	private NanoSegmentation    segment   = null;

	private final Point2D_F64    norm    = new Point2D_F64();
	private Point2Transform2_F64 p2n;


	public <T> MAVR200DepthEstimator(IMAVMSPController control,ITargetListener targetListener, MSPConfig config, int width, int height,  IMAVMapper mapper) {
		this(control,targetListener, config,width,height, mapper,null);
	}

	public <T> MAVR200DepthEstimator(IMAVMSPController control, ITargetListener targetListener, MSPConfig config, int width, int height,
			IMAVMapper mapper, IVisualStreamHandler<Planar<GrayU8>> stream) {

		this.width   = width;
		this.height  = height;
		this.model   = control.getCurrentModel();

		// Vertical window for obstacle detection
		this.base    = height * 2 / 3 - 70;
		this.top     = height * 2 / 3 + 10;

		this.info = new RealSenseInfo(width,height, RealSenseInfo.MODE_RGB);

		this.realsense = new StreamRealSenseVisDepth(0,info);
		this.p2n = (narrow(realsense.getIntrinsics())).undistort_F64(true,false);

		this.realsense.setAutoExposureArea(0, height-base, width, top - base);

		if(HardwareAbstraction.instance().getArchId()==HardwareAbstraction.JETSON && DO_AI) {
			this.detect = new NanoObjectDetection(width,height,stream);
			this.detect.configure(narrow(realsense.getIntrinsics()), NanoObjectDetection.CLASS_PERSON);
			ImageConversionUtil.getInstance(width, height);
		}

		//		this.trail = new NanoTrailDetection(width,height,stream);

		//		this.segment = new NanoSegmentation(width,height,stream);

		// read offsets from config
		offset.x = -config.getFloatProperty("r200_offset_x", String.valueOf(OFFSET_X));
		offset.y = -config.getFloatProperty("r200_offset_y", String.valueOf(OFFSET_Y));
		offset.z = -config.getFloatProperty("r200_offset_z", String.valueOf(OFFSET_Z));



		if(stream!=null) {
			stream.registerOverlayListener(ctx -> {
				overlayFeatures(ctx);
			});
		}



		realsense.registerListener(new Listener() {

			int y0=0; int x; int y; int depth_z; int raw_z;

			Point3D_F64 raw_pt  =  new Point3D_F64();
			Point3D_F64 body_pt =  new Point3D_F64();
			Point3D_F64 ned_pt  =  new Point3D_F64();

			long tms = 0; int quality = 0;

			@Override
			public void process(Planar<GrayU8> rgb, GrayU16 depth, long timeRgb, long timeDepth) {

				quality = 0;
				model.slam.fps = (float)Math.round(10000.0f / (System.currentTimeMillis() - tms))/10.0f;
				tms = System.currentTimeMillis();

				MSP3DUtils.convertModelToSe3_F64(model, to_ned);

				if(detect!=null) {
					ImageConversionUtil.getInstance().convertToByteBuffer(rgb);
					detect.process(ImageConversionUtil.getInstance().getImage(), depth, to_ned);
					if(detect.hasObjectsDetected()) {
						targetListener.update(detect.getFirstObject().getPosNED(), detect.getFirstObject().getPosBODY());
					}

					//				trail.process(ImageConversionUtil.getInstance().getImage(), depth, to_ned);
					//				segment.process(ImageConversionUtil.getInstance().getImage(), depth, to_ned);
				}

				// Add rgb image to stream
				if(stream!=null) {
					//					ImageConversionUtil.getInstance().convertToPlanar(seg);
					stream.addToStream(rgb, model, timeDepth);
				}


				//				System.out.println();

				// Read minimum depth in a band around BASE
				for( x = 0; x < width; x++ ) {
					depth_z = Integer.MAX_VALUE;
					for( y = base; y < top; y++ ) {
						raw_z = depth.unsafe_get(x, y);
						if(raw_z > 20 && raw_z < depth_z && raw_z < 15000) {
							depth_z =  raw_z; y0 = y;
						}
					}

					if(depth_z == Integer.MAX_VALUE) {
						//						System.out.println(x+":");
						continue;
					}


					p2n.compute(x,y0,norm);
					raw_pt.z = depth_z*1e-3f;
					raw_pt.x = raw_pt.z*norm.x;
					raw_pt.y = raw_pt.z*norm.y;

					//					System.out.println(x+":"+raw_pt.x+" -> "+depth_z/1000f);

					quality++;


					if(raw_pt.z > MAX_DISTANCE)
						raw_pt.z = 20;

					body_pt.set(raw_pt.z, raw_pt.x, raw_pt.y);
					body_pt.plusIP(offset);

					// rotate in NED frame if NED available
					if(!to_ned.T.isNaN()) {
						GeometryMath_F64.mult(to_ned.R, body_pt, ned_pt );
						ned_pt.plusIP(to_ned.T);
					} else {
						ned_pt.set(body_pt);
					}

					// put into map if map available
					if(mapper!=null) {
						mapper.update(model.state.l_x, model.state.l_y,ned_pt);
					}
				}
				model.slam.quality = quality * 100 / width;
				model.slam.tms = model.sys.getSynchronizedPX4Time_us();

			}


		});

		System.out.println("R200 depth estimator initialized with offset:"+offset);
	}

	public void start() {
		isRunning = true;
		if(realsense!=null)
			realsense.start();
	}

	public void stop() {
		if(isRunning) {
			realsense.stop();
		}
		isRunning=false;
	}
	private void overlayFeatures(Graphics ctx) {

		ctx.setColor(bgColor);
		ctx.fillRect(5, 5, width-10, 21);

		ctx.setColor(depthColor);
		ctx.fillRect(0, base, width, top-base);

		ctx.setColor(Color.white);

		ctx.drawString(String.format("%2.1f fps (obs.)",model.slam.fps), width-95, 20);

		if(!Float.isNaN(model.sys.t_armed_ms) && model.sys.isStatus(Status.MSP_ARMED)) {
			ctx.drawString(String.format("%.1f sec",model.sys.t_armed_ms/1000), 20, 20);
		}

		if(model.msg.text != null && (model.sys.getSynchronizedPX4Time_us()-model.msg.tms) < 1000000)
			ctx.drawString(model.msg.text, 10, height-5);

	}

}
