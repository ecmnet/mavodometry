package com.comino.mavodometry.estimators;

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
import java.awt.image.WritableRaster;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.control.IMAVMSPController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavmap.map.map3D.impl.octree.LocalMap3D;
import com.comino.mavodometry.libnano.detetction.NanoObjectDetection;
import com.comino.mavodometry.libnano.segmentation.NanoSegmentation;
import com.comino.mavodometry.libnano.trail.NanoTrailDetection;
import com.comino.mavodometry.libnano.utils.ImageConversionUtil;
import com.comino.mavodometry.librealsense.d455.boofcv.IDepthCallback;
import com.comino.mavodometry.librealsense.d455.boofcv.StreamRealSenseD455Depth;
import com.comino.mavodometry.librealsense.r200.boofcv.StreamRealSenseR200Depth;
import com.comino.mavodometry.librealsense.r200.boofcv.StreamRealSenseR200Depth.Listener;
import com.comino.mavodometry.librealsense.utils.RealSenseInfo;
import com.comino.mavodometry.video.IVisualStreamHandler;
import com.comino.mavutils.hw.HardwareAbstraction;

import boofcv.core.image.ConvertImage;
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

public class MAVD455DepthEstimator  {

	private static final boolean DO_DETECT          = false;
	private static final boolean DO_TRAIL           = false;
	private static final boolean DO_SEGMENT         = false;

	private static final boolean DO_DEPTH_OVERLAY   = false;

	private static final float MAX_DISTANCE         = 12.0f;


	// mounting offset in m
	private static final double   	   OFFSET_X     =  0.00;
	private static final double        OFFSET_Y     =  0.00;
	private static final double        OFFSET_Z     =  0.00;

	private StreamRealSenseD455Depth 	realsense	= null;
	private RealSenseInfo               info        = null;
	private DataModel                   model       = null;

	private boolean                     isRunning   = false;

	// Stream data
	private int   width;
	private int   height;

	// Window
	private int   base = 0;
	private int   top  = 40;

	private final Color	bgColor           = new Color(128,128,128,140);
	private final Color	depthColor        = new Color(33, 100, 122,70);

	private Se3_F64       to_ned          = new Se3_F64();
	private Vector3D_F64  offset          = new Vector3D_F64();

	private NanoObjectDetection detect    = null;
	private NanoTrailDetection  trail     = null;
	private NanoSegmentation    segment   = null;

	private final Point2D_F64    norm     = new Point2D_F64();
	private Point2Transform2_F64 p2n      = null;

	private BufferedImage             img = null;
	private boolean          enableStream = false;



	@SuppressWarnings("unused")
	public <T> MAVD455DepthEstimator(IMAVMSPController control, ITargetListener targetListener, LocalMap3D map, MSPConfig config, int width, int height,
			IVisualStreamHandler<Planar<GrayU8>> stream) {

		this.width   = width;
		this.height  = height;
		this.model   = control.getCurrentModel();

		this.img = new BufferedImage(width, height, BufferedImage.TYPE_BYTE_INDEXED, ColorMap.setAlpha(ColorMap.JET,0.4));


		this.info = new RealSenseInfo(width,height, RealSenseInfo.MODE_RGB);

		try {
			this.realsense = new StreamRealSenseD455Depth(0,info);
		} catch( IllegalArgumentException e) {
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

		// read offsets from config
		offset.x = -config.getFloatProperty("r200_offset_x", String.valueOf(OFFSET_X));
		offset.y = -config.getFloatProperty("r200_offset_y", String.valueOf(OFFSET_Y));
		offset.z = -config.getFloatProperty("r200_offset_z", String.valueOf(OFFSET_Z));




		if(stream!=null) {
			stream.registerOverlayListener((ctx,tms) -> {
				if(enableStream) {
					overlayFeatures(ctx,tms);
					if(DO_DEPTH_OVERLAY)
						ctx.drawImage(img, 0, 0, null);
				}
			});
		}

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

				model.slam.fps = model.slam.fps * 0.7f +(float)Math.round(10000.0f / (timeRgb - tms))/10.0f *0.3f;
				tms = timeRgb;

				MSP3DUtils.convertModelToSe3_F64(model, to_ned);


				//
				if(DO_DEPTH_OVERLAY && enableStream)
					overlayDepth(depth, img);


				if(!model.sys.isStatus(Status.MSP_LPOS_VALID)) {
					if(stream!=null && enableStream) {
						stream.addToStream(rgb, model, timeDepth);
					}
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

				// Add rgb image to stream
				if(stream!=null && enableStream) {
					stream.addToStream(rgb, model, timeDepth);
				}

//				for( x = 0; x < width; x = x + 2 ) {
//					for( y = 0; y < height; y = y + 2 ) {
//						raw_z = depth.unsafe_get(x, y);
//
//						if(raw_z < 20 || raw_z >= 20000)
//							continue;
//
//						quality++;
//
//						// transform to 3D body frame
//						p2n.compute(x,y,norm);
//				        raw_pt.y = -raw_pt.z*norm.y;
//						raw_pt.z =  raw_z*1e-3;
//						raw_pt.x =  raw_pt.z*norm.x;
//						
//
//						if(raw_pt.z > MAX_DISTANCE)
//							continue;
//
//						body_pt.set(raw_pt.z, raw_pt.x, raw_pt.y);
//						body_pt.plusIP(offset);
//
//						// rotate in NED frame if NED available
//						if(!to_ned.T.isNaN() &&  !control.isSimulation()) {
//							GeometryMath_F64.mult(to_ned.R, body_pt, ned_pt );
//							ned_pt.plusIP(to_ned.T);
//						} else {
//							ned_pt.set(body_pt);
//						}
//
////						// put into map if map available
////						if(map!=null) {
////							map.update(to_ned.T, ned_pt);
////						}
//					}
//				}
				model.slam.quality = quality * 400 / ( width * height );
				model.slam.tms = DataModel.getSynchronizedPX4Time_us();

			}


		});

		System.out.println("D455 depth estimator initialized with offset:"+offset);
	}

	public void start() {
		isRunning = true;
		if(realsense!=null)
			realsense.start();
	}

	public void stop() {
		if(realsense!=null) {
			realsense.stop();
		}
		isRunning=false;
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
		//
		//		if(!DO_DEPTH_OVERLAY) {
		//			ctx.setColor(depthColor);
		//			ctx.fillRect(0, base, width, height);
		//		}

	}


	private BufferedImage overlayDepth(GrayU16 depth_area, BufferedImage image) {
		WritableRaster raster = image.getRaster(); int[] pixel = new int[1];
		for(int x=0;x<depth_area.width;x++) {
			for(int y=0;y<depth_area.height;y++) {
				pixel[0] = depth_area.get(x, y) / 50;
				raster.setPixel(x,y,pixel);
			}
		}
		return image;
	}

}
