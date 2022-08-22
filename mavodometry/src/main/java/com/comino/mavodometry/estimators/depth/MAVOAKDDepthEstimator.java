package com.comino.mavodometry.estimators.depth;

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
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;

import org.apache.commons.math3.util.Precision;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.config.MSPParams;
import com.comino.mavcom.control.IMAVMSPController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavmap.map.map3D.impl.octree.LocalMap3D;
import com.comino.mavodometry.callback.IDepthCallback;
import com.comino.mavodometry.estimators.MAVAbstractEstimator;
import com.comino.mavodometry.libdepthai.StreamDepthAIOakD;
import com.comino.mavodometry.video.IVisualStreamHandler;
import com.comino.mavutils.workqueue.WorkQueue;

import boofcv.struct.distort.Point2Transform2_F64;
import boofcv.struct.geo.Point2D3D;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import georegression.geometry.GeometryMath_F64;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;


public class MAVOAKDDepthEstimator extends MAVAbstractEstimator  {

	private static final int              DEPTH_RATE    = 100;

	// mounting offset in m
	private static final double   	      OFFSET_X 		=  -0.06;
	private static final double      	  OFFSET_Y 		=   0.00;
	private static final double      	  OFFSET_Z 		=   0.00;

	private final static float            MIN_DEPTH_M  	= 0.3f;
	private final static float            MAX_DEPTH_M 	= 5.0f;

	private final static int              DEPTH_SCALE   = 2; 
	private final static int 			  DEPTH_OFFSET  = 20;

	private StreamDepthAIOakD			oakd 			= null;

	private boolean 					enableStream  	= false;
	private Point2Transform2_F64 		p2n      		= null;

	private final Se3_F64       		to_ned          = new Se3_F64();
	private final Point2D3D             nearest_body    = new Point2D3D();
	private final Vector3D_F64          offset_body		= new Vector3D_F64();

	private long   						tms 			= 0;
	private int                         width34         = 0;

	private final DecimalFormat fdistance  = new DecimalFormat("Obst: #0.0m");
	private final DecimalFormat faltitude  = new DecimalFormat("at #0.0m;at -#0.0m");

	private final WorkQueue wq = WorkQueue.getInstance();
	private final BlockingQueue<GrayU16> transfer_depth = new ArrayBlockingQueue<GrayU16>(10);
	private int depth_worker;

	private final LocalMap3D map;
	private final IVisualStreamHandler<Planar<GrayU8>> stream;

	private final Planar<GrayU8> depth_colored;


	public <T> MAVOAKDDepthEstimator(IMAVMSPController control,  MSPConfig config, LocalMap3D map, int width, int height, IVisualStreamHandler<Planar<GrayU8>> stream) {
		super(control);

		this.stream = stream;


		// read offset settings
		offset_body.x = config.getFloatProperty(MSPParams.OAKD_OFFSET_X, String.valueOf(OFFSET_X));
		offset_body.y = config.getFloatProperty(MSPParams.OAKD_OFFSET_Y, String.valueOf(OFFSET_Y));
		offset_body.z = config.getFloatProperty(MSPParams.OAKD_OFFSET_Z, String.valueOf(OFFSET_Z));
		System.out.println("OAK-D Mounting offset: "+offset_body);

		try {
			this.oakd   = StreamDepthAIOakD.getInstance(width, height);
			//		this.oakd   = StreamNNDepthAIOakD.getInstance(width, height,"yolo-v3-tiny-tf_openvino_2021.4_6shave.blob", 416,416);
			this.oakd.setRGBMode(true);
			//	this.oakd.setRGBMode(false);
		} catch (Exception e) {
			e.printStackTrace();
		}

		this.width34 = width * 3 / 4;
		this.map = map;

		this.depth_colored = new Planar<GrayU8>(GrayU8.class,width,height,3);


		if(stream!=null) {
			stream.registerOverlayListener((ctx,n,tms) -> {
				if(enableStream) {
					overlayFeatures(ctx,n,tms);

				}
			});
		}

		oakd.registerCallback(new IDepthCallback() {

			@Override
			public void process(final Planar<GrayU8> rgb, final GrayU16 depth, long timeRgb, long timeDepth) {

				model.slam.tms = DataModel.getSynchronizedPX4Time_us();
				model.sys.setSensor(Status.MSP_SLAM_AVAILABILITY, true);

				// transfer depth data to depth handler 

				if(transfer_depth.offer(depth))
					MSP3DUtils.convertModelToSe3_F64(model, to_ned);

				// Add image to stream
				if(stream!=null && enableStream) {
					stream.addToStream("RGB",rgb, model, timeRgb);
					stream.addToStream("DEPTH",depth_colored, model, System.currentTimeMillis());	
				}

				model.slam.fps = model.slam.fps * 0.75f + ((float)(1000f / (System.currentTimeMillis()-tms))) * 0.25f;
				tms = System.currentTimeMillis();			
			}
		});

	}

	public void start() throws Exception {
		if(oakd!=null) {

			oakd.start();
			p2n = (narrow(oakd.getIntrinsics())).undistort_F64(true,false);

			Thread.sleep(200);
			depth_worker = wq.addCyclicTask("LP",DEPTH_RATE, new DepthHandler());
			System.out.println("Depth worker started");

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

	private void overlayFeatures(Graphics ctx, String stream, long tms) {

		if(!enableStream)
			return;

		drawMinDist(ctx, stream, (int)nearest_body.observation.x, (int)nearest_body.observation.y);

	}

	private void drawMinDist(Graphics ctx, String stream,int x0, int y0) {

		if((x0==0 && y0 == 0))
			return;

		if(stream.contains("DEPTH") && Float.isFinite(model.slam.dm)) {

			final int ln = 5;
			
			ctx.drawLine(x0-ln,y0-ln,x0+ln,y0-ln);
			ctx.drawLine(x0-ln,y0-ln,x0,y0+ln);
			ctx.drawLine(x0,y0+ln,x0+ln,y0-ln);

		}

		String tmp = fdistance.format(model.slam.dm)+" "+faltitude.format(model.slam.oz);
		ctx.drawString(tmp, width34 - ctx.getFontMetrics().stringWidth(tmp)/2, 20);

	}


	/*
	 * Processing depth data at 10 Hz.
	 */
	private class DepthHandler implements Runnable {

		private final Point2D_F64 norm     = new Point2D_F64();
		private final Point3D_F64 ned_pt_n = new Point3D_F64();

		private final Point2D3D   tmp_p = new Point2D3D();
		private final Point2D3D   ned_p = new Point2D3D();

		public DepthHandler() {

		}

		@Override
		public void run() {

			if(transfer_depth.isEmpty())
				return;

			try {

				GrayU16 depth = transfer_depth.take();


				model.slam.quality = depthMapping(depth);

				GeometryMath_F64.mult(to_ned.R, nearest_body.location, ned_pt_n );
				ned_pt_n.plusIP(to_ned.T);

				model.slam.dm = (float)nearest_body.location.x; 
				model.slam.ox = (float)ned_pt_n.x;
				model.slam.oy = (float)ned_pt_n.y;
				model.slam.oz = (float)ned_pt_n.z;		

				transfer_depth.clear();


			} catch (InterruptedException e) {
				e.printStackTrace();
			}

		}	

		// map depth on a 640/DEPTH_SCALE x 480/DEPTH_SCALE basis
		private int depthMapping(GrayU16 in) {

			int quality = 0; nearest_body.location.x = Double.MAX_VALUE;

			for(int x = DEPTH_OFFSET; x < in.width-DEPTH_OFFSET;x = x + DEPTH_SCALE) {
				for(int y = DEPTH_OFFSET; y < in.height-DEPTH_OFFSET;y = y + DEPTH_SCALE) {

					colorize(x,y,in,depth_colored, 9000);

					if(getSegmentPositionBody(x,y,in,tmp_p)) {
						GeometryMath_F64.mult(to_ned.R, tmp_p.location, ned_p.location );
						ned_p.location.plusIP(to_ned.T);
						if( Float.isFinite(model.hud.at) &&
								tmp_p.location.x< nearest_body.location.x && 
								ned_p.location.z < (-(model.hud.at+0.1f)) ) // consider terrain as ground
							nearest_body.setTo(tmp_p);

						if(control.isSimulation() || !model.sys.isStatus(Status.MSP_LANDED))
							map.update(to_ned.T,ned_p.location);   // Incremental probability
						//	  map.update(to_ned.T,ned_p.location,1); // Absolute probability

						quality++;
					}
				}
			}

			if(nearest_body.location.x > MAX_DEPTH_M)
				nearest_body.location.setTo(Double.NaN,Double.NaN,Double.NaN);

			return (int)(quality * 1600f / in.data.length);
		}

		private void colorize(int x, int y, GrayU16 in, Planar<GrayU8> out, int max) {

			int r, g, b; 			
			int v = in.get(x, y);

			if (v == 0 || v > max) {
				r = b = g = 60;
			} else {
				b = 255*v/max;
				r = 255*(max - v - 1)/max;
				g = r / 8;
			}
			for(int xs = 0; xs < DEPTH_SCALE;xs++)
				for(int ys = 0; ys < DEPTH_SCALE;ys++)
					out.set24u8(x+xs, y+ys, r << 16 | g << 8 | b );
		}


		// Return 3D point of depth segment in body frame
		private boolean getSegmentPositionBody(int x, int y, GrayU16 in, Point2D3D p) {

			p.observation.x = x;
			p.observation.y = y;

			p.location.x = in.get(x, y) * 1e-3;

			if(p.location.x < MIN_DEPTH_M || p.location.x > MAX_DEPTH_M) {
				p.location.x = Double.MAX_VALUE;
				return false;
			}

			p2n.compute(p.observation.x,p.observation.y,norm);

			p.location.y =   p.location.x * norm.x;
			p.location.z =   p.location.x * norm.y;


			p.location.plusIP(offset_body);

			return true;

		}
	}

}
