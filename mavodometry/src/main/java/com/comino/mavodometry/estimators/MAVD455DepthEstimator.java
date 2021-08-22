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

import java.awt.Graphics;
import java.awt.image.BufferedImage;
import java.awt.image.WritableRaster;
import java.text.DecimalFormat;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.config.MSPParams;
import com.comino.mavcom.control.IMAVMSPController;
import com.comino.mavcom.core.ControlModule;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavmap.map.map3D.impl.octree.LocalMap3D;
import com.comino.mavodometry.librealsense.d455.boofcv.IDepthCallback;
import com.comino.mavodometry.librealsense.d455.boofcv.StreamRealSenseD455Depth;
import com.comino.mavodometry.librealsense.utils.RealSenseInfo;
import com.comino.mavodometry.video.IVisualStreamHandler;
import com.comino.mavutils.workqueue.WorkQueue;

import boofcv.concurrency.BoofConcurrency;
import boofcv.struct.distort.Point2Transform2_F64;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import edu.mines.jtk.awt.ColorMap;
import georegression.geometry.GeometryMath_F64;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point2D_I32;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;

public class MAVD455DepthEstimator extends ControlModule  {

	private static final boolean DO_DEPTH_OVERLAY   = true; 
	private static final float  WARN_OBS_DISTANCE   = 1.5f;

	private static final int         DEPTH_HEIGHT   = 90;
	private static final int         DEPTH_WIDTH    = 540;

	private static final float       quality_factor = 100f / ( DEPTH_WIDTH * DEPTH_HEIGHT) ;

	// mounting offset in m
	private static final double   	   OFFSET_X     =  0.00;
	private static final double        OFFSET_Y     =  0.00;
	private static final double        OFFSET_Z     =  0.00;
	
	private static final int             DEPTH_RATE = 135;

	private StreamRealSenseD455Depth 	realsense	= null;
	private RealSenseInfo               info        = null;


	// Window
	private int              depth_x_offs    = 0;
	private int              depth_y_offs    = 0;

	//	private final Color	bgColor           = new Color(128,128,128,140);
	//	private final Color	depthColor        = new Color(33, 100, 122,70);

	private long          frame_count     = 0;

	private Se3_F64       to_ned          = new Se3_F64();
	private Vector3D_F64  offset          = new Vector3D_F64();

	private final Point2D_F64    norm     = new Point2D_F64();
	private Point2Transform2_F64 p2n      = null;

	private Point2D_I32 mindist_pt        =  new Point2D_I32();

	private BufferedImage             img = null;
	private boolean          enableStream = false;
	private boolean         depth_overlay = false;
	private float warn_obs_distance       = WARN_OBS_DISTANCE;

	private String tmp;
	private final DecimalFormat fdistance  = new DecimalFormat("Obst: #0.0m");
	private final int width4;

	private GrayU16 sub  = new GrayU16(1,1);
	private GrayU16 proc = new GrayU16(1,1);
	
	private final WorkQueue wq = WorkQueue.getInstance();
	private int depth_worker;


	@SuppressWarnings("unused")
	public <T> MAVD455DepthEstimator(IMAVMSPController control, ITargetListener targetListener, LocalMap3D map, MSPConfig config, int width, int height,
			IVisualStreamHandler<Planar<GrayU8>> stream) {

		super(control);

		this.sub.reshape(DEPTH_WIDTH, DEPTH_HEIGHT);
		this.proc.reshape(DEPTH_WIDTH, DEPTH_HEIGHT);

		this.width4 = width/4;

		this.depth_x_offs = (width - DEPTH_WIDTH ) / 2;
		this.depth_y_offs = (height - DEPTH_HEIGHT ) / 2;

		this.img = new BufferedImage(width, height, BufferedImage.TYPE_BYTE_INDEXED, ColorMap.setAlpha(ColorMap.JET,0.4));


		this.info = new RealSenseInfo(width,height, RealSenseInfo.MODE_RGB);

		try {
			this.realsense = StreamRealSenseD455Depth.getInstance(info);
		} catch( IllegalArgumentException e) {
			System.out.println("No D455 device found");
			return;

		}

		// configs
		offset.x = -config.getFloatProperty(MSPParams.D455_OFFSET_X, String.valueOf(OFFSET_X));
		offset.y = -config.getFloatProperty(MSPParams.D455_OFFSET_Y, String.valueOf(OFFSET_Y));
		offset.z = -config.getFloatProperty(MSPParams.D455_OFFSET_Z, String.valueOf(OFFSET_Z));
		warn_obs_distance = config.getFloatProperty(MSPParams.D455_WARN_OBS, String.valueOf(WARN_OBS_DISTANCE));
		System.out.println("D455 warning distance obstacles: "+warn_obs_distance+"m");
		depth_overlay     = config.getBoolProperty(MSPParams.D455_DEPTH_OVERLAY, String.valueOf(DO_DEPTH_OVERLAY));
		System.out.println("D455 depth overlay: "+depth_overlay);

		if(stream!=null) {
			stream.registerOverlayListener((ctx,tms) -> {
				if(enableStream) {
					overlayFeatures(ctx,tms);
					if(depth_overlay)
						ctx.drawImage(img, 0, 0, null);
				}
			});
		}

		realsense.registerCallback(new IDepthCallback() {

//			int y0=0; int x; int y; int depth_z; int raw_z;
//
//			Point3D_F64 raw_pt      =  new Point3D_F64();
//			Point3D_F64 body_pt     =  new Point3D_F64();
//			Point3D_F64 ned_pt      =  new Point3D_F64();
//			Point3D_F64 body_pt_n   =  new Point3D_F64();
//			Point3D_F64 ned_pt_n    =  new Point3D_F64();
//
//			double min_distance;
//			double distance;
//
			long tms = 0; 
//			int quality = 0;

			@Override
			public void process(Planar<GrayU8> rgb, GrayU16 depth, long timeRgb, long timeDepth) {

				frame_count++;	
				//quality = 0;

				// Bug in CB; sometimes called twice
				if((timeRgb-tms) < 3)
					return;
				
				model.slam.tms = DataModel.getSynchronizedPX4Time_us();
//				model.slam.fps = model.slam.fps * 0.7f +(float)Math.round(10000.0f / (timeRgb - tms))/10.0f *0.3f;
				model.slam.fps = 1000f / (timeRgb - tms);
				tms = timeRgb;

				// Initializing with Intrinsics
				if(p2n==null) {	
					p2n = (narrow(realsense.getIntrinsics())).undistort_F64(true,false);
					return;

				}
				
				// Add rgb image to stream
				if(stream!=null && enableStream) {
					stream.addToStream(rgb, model, timeDepth);
				}
				if(depth_overlay && enableStream)
					overlayDepth(sub, img);
				
				// make a copy of the depth area for asybchronous processing
				depth.subimage(depth_x_offs, depth_y_offs, depth_x_offs+DEPTH_WIDTH, depth_y_offs+DEPTH_HEIGHT, sub);	

			}
		});

		System.out.println("D455 depth estimator initialized with offset:"+offset);
	}

	public void start() {
		if(realsense!=null)
			realsense.start();
		depth_worker = wq.addCyclicTask("LP",DEPTH_RATE, new DepthHandler());
	}

	public void stop() {
		if(realsense!=null) {
			wq.removeTask("LP", depth_worker);
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

		if(model.sys.isStatus(Status.MSP_ARMED) && Float.isFinite(model.slam.dm) && model.slam.dm < warn_obs_distance) {
			tmp = fdistance.format(model.slam.dm);
			ctx.drawString(tmp, width4*3 - ctx.getFontMetrics().stringWidth(tmp)/2, 20);
			drawMinDist(ctx,depth_x_offs+mindist_pt.x,depth_y_offs+mindist_pt.y);
		}

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

	private void drawMinDist(Graphics ctx, int x0, int y0) {

		final int ln = 10;

		ctx.drawLine(x0-ln,y0-ln,x0+ln,y0-ln);
		ctx.drawLine(x0-ln,y0-ln,x0,y0+ln);
		ctx.drawLine(x0,y0+ln,x0+ln,y0-ln);

	}


	private BufferedImage overlayDepth(GrayU16 depth_area, BufferedImage image) {
		WritableRaster raster = image.getRaster(); int[] pixel = new int[1];
		for(int x=0;x<DEPTH_WIDTH;x++) {
			for(int y=0;y<DEPTH_HEIGHT;y++) {
				pixel[0] = depth_area.get(x, y) / 50;
				raster.setPixel(x+depth_x_offs,y+depth_y_offs,pixel);
			}
		}
		return image;
	}
	
	private class DepthHandler implements Runnable {
		
		Point3D_F64 raw_pt      =  new Point3D_F64();
		Point3D_F64 body_pt     =  new Point3D_F64();
		Point3D_F64 ned_pt      =  new Point3D_F64();
		Point3D_F64 body_pt_n   =  new Point3D_F64();
		Point3D_F64 ned_pt_n    =  new Point3D_F64();

		double min_distance;
		double distance;
		
		int depth_z; int raw_z;
		long tms = 0; int quality = 0;
		
		int y0=0; int x; int y;

		@Override
		public void run() {
			
			quality = 0;
			
			MSP3DUtils.convertModelToSe3_F64(model, to_ned);

			proc.setTo(sub);


			min_distance = Double.MAX_VALUE;
			// TODO: Eventually BOOF Concurrency here
		//				BoofConcurrency.loopFor(0, DEPTH_WIDTH, x -> {
			for(x = 0; x < DEPTH_WIDTH;x++) {
				for(y = 0; y < DEPTH_HEIGHT;y++) {
					raw_z = proc.unsafe_get(x, y);

					if(raw_z < 20 || raw_z >= 15000)
						continue;

					quality++;

					p2n.compute(x,y,norm);
					raw_pt.z =  raw_z*1e-3;
					raw_pt.y = -raw_pt.z*norm.y;
					raw_pt.x =  raw_pt.z*norm.x;
					

					body_pt.set(raw_pt.z, raw_pt.x, raw_pt.y);
//					body_pt.plusIP(offset);
//					GeometryMath_F64.mult(to_ned.R, body_pt, ned_pt );
//					ned_pt.plusIP(to_ned.T);

//					System.out.println(norm.x+"/"+norm.y);

					distance = body_pt.norm();
					if(distance < min_distance) {
						mindist_pt.set(x,y);
						min_distance = distance;
						body_pt_n.set(body_pt);
					}

					//						if(ned_pt.z > MIN_ALTITUDE)
					//							continue;
					//
					//						if(map!=null) {
					//							map.update(to_ned.T, ned_pt);
					//						}

				}
			}
//					});

			model.slam.quality = model.slam.quality * 0.7f + (quality * quality_factor ) * 0.3f;

			if(model.slam.quality > 30) {

				GeometryMath_F64.mult(to_ned.R, body_pt_n, ned_pt_n );
				ned_pt_n.plusIP(to_ned.T);

				model.slam.ox = (float)ned_pt_n.x;
				model.slam.oy = (float)ned_pt_n.y;
				model.slam.oz = (float)ned_pt_n.z;
				
//				map.update(to_ned.T, ned_pt_n, 1);

				model.slam.dm = (float)min_distance; 
				
			} else
				model.slam.dm = Float.NaN; 

		}
		
	}

}
