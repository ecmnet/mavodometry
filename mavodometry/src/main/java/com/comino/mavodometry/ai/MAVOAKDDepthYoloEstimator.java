package com.comino.mavodometry.ai;

/****************************************************************************
*
*   Copyright (c) 2022 Eike Mansfeld ecm@gmx.de. All rights reserved.
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
import java.text.DecimalFormat;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;


import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.control.IMAVMSPController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavodometry.callback.IDepthCallback;
import com.comino.mavodometry.estimators.MAVAbstractEstimator;
import com.comino.mavodometry.libdepthai.StreamDepthAIOakD;
import com.comino.mavodometry.video.IVisualStreamHandler;
import com.comino.mavutils.workqueue.WorkQueue;

import boofcv.struct.distort.Point2Transform2_F64;
import boofcv.struct.geo.Point2D3D;
import boofcv.struct.image.GrayF32;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import georegression.geometry.GeometryMath_F64;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;

public class MAVOAKDDepthYoloEstimator extends MAVAbstractEstimator  {

	private static final int             DEPTH_RATE     = 100;

	private final static int            DEPTH_SEG_W 	= 32;
	private final static int            DEPTH_SEG_H 	= 32;

	private final static int            MIN_DEPTH_MM 	= 350;
	private final static int            MAX_DEPTH_MM 	= 8000;

	private final GrayF32        		seg_distance;


	private StreamDepthAIOakD			oakd 			= null;

	private boolean 					enableStream  	= false;
	private boolean 					depth_overlay 	= false;
	private Point2Transform2_F64 		p2n      		= null;

	private final Se3_F64       		to_ned          = new Se3_F64();
	private final Point2D3D             nearest         = new Point2D3D();

	private long   						tms 			= 0;
	private int                         width34         = 0;
	
	private final DecimalFormat fdistance  = new DecimalFormat("Obst: #0.0m");

	private final WorkQueue wq = WorkQueue.getInstance();
	private final BlockingQueue<GrayU16> transfer_depth = new ArrayBlockingQueue<GrayU16>(1);
	private int depth_worker;

	public <T> MAVOAKDDepthYoloEstimator(IMAVMSPController control,  MSPConfig config, int width, int height, IVisualStreamHandler<Planar<GrayU8>> stream) {
		super(control);

		try {
			this.oakd   = StreamDepthAIOakD.getInstance(width, height);
			this.oakd.setRGBMode(!depth_overlay);
		} catch (Exception e) {
			e.printStackTrace();
		}
		
		this.width34 = width * 3 / 4;

		this.seg_distance 	= new GrayF32(width/DEPTH_SEG_W,height/DEPTH_SEG_H);

		if(stream!=null) {
			stream.registerOverlayListener((ctx,tms) -> {
				if(enableStream) {
					overlayFeatures(ctx,tms);
					if(depth_overlay)
						overlayDepthSegments(ctx, seg_distance);

				}
			});
		}

		oakd.registerCallback(new IDepthCallback() {

			@Override
			public void process(final Planar<GrayU8> rgb, final GrayU16 depth, long timeRgb, long timeDepth) {

				model.slam.tms = DataModel.getSynchronizedPX4Time_us();

				// transfer depth data to depth handler 
				if(transfer_depth.isEmpty()) {
					try {
						MSP3DUtils.convertModelToSe3_F64(model, to_ned);
						transfer_depth.put(depth);
					} catch (InterruptedException e) {	}
				}
				// Add image to stream
				if(stream!=null && enableStream) {
					stream.addToStream(rgb, model, timeRgb);
				}
			}
		});

	}

	public void start() throws Exception {
		if(oakd!=null) {
			oakd.start();
			p2n = (narrow(oakd.getIntrinsics())).undistort_F64(true,false);
			depth_worker = wq.addCyclicTask("LP",DEPTH_RATE, new DepthHandler());
		}
	}

	public void stop() {
		if(oakd!=null) {
			oakd.stop();
			wq.removeTask("LP", depth_worker);
		}
	}

	public void enableStream(boolean enable) {
		this.enableStream = enable;
	}

	private void overlayFeatures(Graphics ctx, long tms) {

		if(!enableStream)
			return;

		drawMinDist(ctx,(int)nearest.observation.x, (int)nearest.observation.y);

	}

	private void drawMinDist(Graphics ctx, int x0, int y0) {

		final int ln = 10;

		ctx.drawLine(x0-ln,y0-ln,x0+ln,y0-ln);
		ctx.drawLine(x0-ln,y0-ln,x0,y0+ln);
		ctx.drawLine(x0,y0+ln,x0+ln,y0-ln);
		
		String tmp = fdistance.format(model.slam.dm);
		ctx.drawString(fdistance.format(model.slam.dm), width34 - ctx.getFontMetrics().stringWidth(tmp)/2, 20);

	}

	final Point2D3D p = new Point2D3D();
	private void overlayDepthSegments(Graphics ctx, GrayF32 seg) {
		for(int x = 0; x < seg.width;x++) {
			// Skip first line
			for(int y = 1; y < seg.height;y++) {
				ctx.drawRect(x*DEPTH_SEG_W, y*DEPTH_SEG_H, DEPTH_SEG_W, DEPTH_SEG_H);
				if(seg_distance.get(x, y)> 0) {
					ctx.drawString(String.format("%+#.1f",seg.get(x, y)),x*DEPTH_SEG_W+5, y*DEPTH_SEG_H+10);
				}
			}
		}
	}


	/*
	 * Processing depth data at 10 Hz.
	 */
	private class DepthHandler implements Runnable {
		
		private final static int DEPTH_SEG_W_2 = DEPTH_SEG_W/2;
		private final static int DEPTH_SEG_H_2 = DEPTH_SEG_H/2;

		private final Point3D_F64 ned_pt_n = new Point3D_F64();
		private final Point2D_F64 norm     = new Point2D_F64();
		
		@Override
		public void run() {

			if(transfer_depth.isEmpty())
				return;

			try {

				GrayU16 depth = transfer_depth.take();

				model.slam.quality = buildMeanDepthSegments(depth,seg_distance);

				model.slam.fps = 1000f / (System.currentTimeMillis() - tms) + 0.5f;
				tms = System.currentTimeMillis();

				getNearestSegment(seg_distance, nearest);
				model.slam.dm = (float)nearest.location.x; 

				GeometryMath_F64.mult(to_ned.R, nearest.location, ned_pt_n );
				ned_pt_n.plusIP(to_ned.T);

				model.slam.ox = (float)ned_pt_n.x;
				model.slam.oy = (float)ned_pt_n.y;
				model.slam.oz = (float)ned_pt_n.z;


			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}	

		// Build depth segments by calculating the min distance
		private int buildMeanDepthSegments(GrayU16 in, GrayF32 out) {

			int dist_mm = 0; int valid_count=0; float quality = 0; int d = 0;

			final int xr = in.width  / out.width; 
			final int yr = in.height / out.height;

			for(int x = 0; x < out.width;x++) {
				for(int y = 0; y < out.height;y++) {

					dist_mm = 99999; valid_count = 0;

					for(int ix=0;ix<xr;ix++) {
						for(int iy=0;iy<yr;iy++) {
							d = in.get(x*xr+ix, y*yr+iy);
							if(d < MIN_DEPTH_MM || d > MAX_DEPTH_MM)
								continue;
							valid_count++;
							if(d < dist_mm)
								dist_mm = d;
						}	
					}

					if(valid_count > 0) {
						quality += 1;
						out.set(x, y, (float)dist_mm/1000);
					}
					else
						out.set(x, y, -1);
				}
			}
			return (int)(quality * 100f / out.data.length);
		}

		private Point2D3D getNearestSegment(GrayF32 seg, Point2D3D nearest ) {
			float distance = Float.MAX_VALUE; p.setTo(0,0,0,0,0);
			for(int x = 0; x < seg.width;x++) {
				for(int y = 0; y < seg.height;y++) {
					if(seg_distance.get(x, y)> 0 && seg_distance.get(x, y) < distance) {
						getSegmentPositionBody(x, y,seg, p);
						distance = seg_distance.get(x, y);
					}
				}
			}	
			nearest.setTo(p);
			return p;
		}

		// Return 3D point of depth segment in body frame
		private void getSegmentPositionBody(int x, int y, GrayF32 seg, Point2D3D p) {

			p.observation.x = x * DEPTH_SEG_W + DEPTH_SEG_W_2;
			p.observation.y = y * DEPTH_SEG_H + DEPTH_SEG_H_2;

			p2n.compute(p.observation.x,p.observation.y,norm);

			p.location.x = seg.get(x, y);
			p.location.y =   p.location.x * norm.x;
			p.location.z = - p.location.x * norm.y;

		}

	}

}
