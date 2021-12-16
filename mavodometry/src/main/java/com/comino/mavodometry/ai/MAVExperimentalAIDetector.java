package com.comino.mavodometry.ai;

/****************************************************************************
 *
 *   Copyright (c) 2021 Eike Mansfeld ecm@gmx.de. All rights reserved.
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
import java.awt.image.BufferedImage;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.control.IMAVMSPController;
import com.comino.mavcom.core.ControlModule;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavmap.map.map3D.impl.octree.LocalMap3D;
import com.comino.mavodometry.callback.IDepthCallback;
import com.comino.mavodometry.estimators.ITargetListener;
import com.comino.mavodometry.librealsense.d455.boofcv.StreamRealSenseD455Depth;
import com.comino.mavodometry.librealsense.utils.RealSenseInfo;
import com.comino.mavodometry.video.IVisualStreamHandler;

import boofcv.struct.distort.Point2Transform2_F64;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import edu.mines.jtk.awt.ColorMap;
import georegression.geometry.GeometryMath_F64;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;

public class MAVExperimentalAIDetector extends ControlModule  {

//	private static final boolean DO_DETECT          = false;
//	private static final boolean DO_TRAIL           = false;
//	private static final boolean DO_SEGMENT         = false;



	private static final float MIN_ALTITUDE         = 0.3f;

	private static final int         DEPTH_HEIGHT   = 60;
	private static final int         DEPTH_WIDTH    = 580;


	// mounting offset in m
	private static final double   	   OFFSET_X     =  0.00;
	private static final double        OFFSET_Y     =  0.00;
	private static final double        OFFSET_Z     =  0.00;

	private StreamRealSenseD455Depth 	realsense	= null;
	private RealSenseInfo               info        = null;

	// Window
	private int              depth_x_offs    = 0;
	private int              depth_y_offs    = 0;

//	private final Color	bgColor           = new Color(128,128,128,140);
//	private final Color	depthColor        = new Color(33, 100, 122,70);

	private Se3_F64       to_ned          = new Se3_F64();
	private Vector3D_F64  offset          = new Vector3D_F64();

//	private NanoObjectDetection detect    = null;
//	private NanoTrailDetection  trail     = null;
//	private NanoSegmentation    segment   = null;

	private Point2Transform2_F64 p2n      = null;

	private BufferedImage             img = null;
	private boolean          enableStream = false;



	@SuppressWarnings("unused")
	public <T> MAVExperimentalAIDetector(IMAVMSPController control, ITargetListener targetListener, LocalMap3D map, MSPConfig config, int width, int height,
			IVisualStreamHandler<Planar<GrayU8>> stream) {

		super(control);


		this.depth_x_offs = (width - DEPTH_WIDTH ) / 2;
		this.depth_y_offs = (height - DEPTH_HEIGHT ) / 2;

		this.img = new BufferedImage(width, height, BufferedImage.TYPE_BYTE_INDEXED, ColorMap.setAlpha(ColorMap.JET,0.4));


		this.info = new RealSenseInfo(width,height, RealSenseInfo.MODE_RGB);

		try {
			
			/// TODO: should not create a new instance
			
			this.realsense = StreamRealSenseD455Depth.getInstance(info);
		} catch( Exception e) {
			System.out.println("No D455 device found");
			return;

		}
		//		this.p2n = (narrow(realsense.getIntrinsics())).undistort_F64(true,false);
		//
		//		this.realsense.setAutoExposureArea(0, height-base, width, top - base);
		//
		//		if(HardwareAbstraction.instance().getArchId()==HardwareAbstraction.JETSON && DO_DETECT) {
		//			this.detect = new NanoObjectDetection(width,height,stream);
		//			this.detect.configure(narrow(realsense.getIntrinsics()), NanoObjectDetection.CLASS_PERSON);
		//			ImageConversionUtil.getInstance(width, height);
		//		}
		//
		//		if(HardwareAbstraction.instance().getArchId()==HardwareAbstraction.JETSON && DO_TRAIL) {
		//			this.trail = new NanoTrailDetection(width,height,stream);
		//			this.trail.configure(narrow(realsense.getIntrinsics()));
		//			ImageConversionUtil.getInstance(width, height);
		//		}
		//
		//		if(HardwareAbstraction.instance().getArchId()==HardwareAbstraction.JETSON && DO_SEGMENT) {
		//			this.segment = new NanoSegmentation(width,height,stream);
		//			ImageConversionUtil.getInstance(width, height);
		//		}

		
		realsense.registerCallback(new IDepthCallback() {

			int y0=0; int x; int y; int depth_z; int raw_z;

			Point3D_F64 raw_pt  =  new Point3D_F64();
			Point3D_F64 body_pt =  new Point3D_F64();
			Point3D_F64 ned_pt  =  new Point3D_F64();

			long tms = 0; int quality = 0;

			@Override
			public void process(Planar<GrayU8> rgb, GrayU16 depth, long timeRgb, long timeDepth) {


				quality = 0;

				// Bug in CB; sometimes called twice
				if((timeRgb-tms) < 10)
					return;

				// Initializing with Intrinsics
				if(p2n==null) {	
					p2n = (narrow(realsense.getIntrinsics())).undistort_F64(true,false);

				}

				MSP3DUtils.convertModelToSe3_F64(model, to_ned);


				if(!model.sys.isStatus(Status.MSP_LPOS_VALID)) {
					return;
				}

				//				// AI networks to be processed
				//				if(detect!=null || trail!=null || segment!=null) {
				//					ImageConversionUtil.getInstance().convertToByteBuffer(rgb);
				//
				//					if(detect!=null) {
				//						detect.process(ImageConversionUtil.getInstance().getImage(), depth, to_ned);
				//						if(detect.hasObjectsDetected()) {
				//							targetListener.update(detect.getFirstObject().getPosNED(), detect.getFirstObject().getPosBODY());
				//						}
				//					}
				//
				//					if(trail!=null) {
				//						trail.process(ImageConversionUtil.getInstance().getImage(), depth, to_ned);
				//					}
				//
				//					if(segment!=null) {
				//						segment.process(ImageConversionUtil.getInstance().getImage(), depth, to_ned);
				//					}
				//				}

				
			}


		});

		System.out.println("AI Detector initialized");
	}

	public void start() throws Exception {
		if(realsense!=null)
			realsense.start();
	}

	public void stop() {
		if(realsense!=null) {
			realsense.stop();
		}
	}

	public void enableStream(boolean enable) {
		if(realsense!=null)
			this.enableStream = enable;
		else
			this.enableStream = false;
	}


	private void overlayFeatures(Graphics ctx, long tms) {

		if(!enableStream)
			return;

		drawDepthArea(ctx,depth_x_offs,depth_y_offs,depth_x_offs+DEPTH_WIDTH,depth_y_offs+DEPTH_HEIGHT);
		//
		//		if(!DO_DEPTH_OVERLAY) {
		//			ctx.setColor(depthColor);
		//			ctx.fillRect(0, base, width, height);
		//		}

	}

	private void drawDepthArea(Graphics ctx, int x0, int y0, int x1, int y1) {

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


//	private BufferedImage overlayDepth(GrayU16 depth_area, BufferedImage image) {
//		WritableRaster raster = image.getRaster(); int[] pixel = new int[1];
//		for(int x=0;x<depth_area.width;x++) {
//			for(int y=0;y<depth_area.height;y++) {
//				pixel[0] = depth_area.get(x, y) / 50;
//				raster.setPixel(x,y,pixel);
//			}
//		}
//		return image;
//	}

}
