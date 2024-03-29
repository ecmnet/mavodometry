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

import java.awt.Color;
import java.awt.Graphics2D;
import java.util.List;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.config.MSPParams;
import com.comino.mavcom.control.IMAVMSPController;
import com.comino.mavcom.messaging.MessageBus;
import com.comino.mavcom.messaging.msgs.msp_msg_nn_object;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.model.segment.Vision;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavmap.map.map3D.impl.octomap.MAVOctoMap3D;
import com.comino.mavmap.map.map3D.impl.octomap.tools.MAVOctoMapTools;
import com.comino.mavodometry.callback.IDepthCallback;
import com.comino.mavodometry.estimators.MAVAbstractEstimator;
import com.comino.mavodometry.estimators.inference.YoloDetection;
import com.comino.mavodometry.libdepthai.IStreamDepthAIOakD;
import com.comino.mavodometry.libdepthai.StreamYoloDepthAIOakD;
import com.comino.mavodometry.video.IVisualStreamHandler;
import com.comino.mavodometry.video.impl.AbstractOverlayListener;
import com.comino.mavutils.file.MSPFileUtils;
import com.comino.mavutils.workqueue.WorkQueue;

import boofcv.abst.distort.FDistort;
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
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.jOctoMap.pointCloud.PointCloud;


public class MAVOAKDDepthEstimator extends MAVAbstractEstimator  {

	private static final int              DEPTH_RATE    = 200;

	// mounting offset in m
	private static final double   	      OFFSET_X 		=  -0.06;
	private static final double      	  OFFSET_Y 		=   0.00;
	private static final double      	  OFFSET_Z 		=   0.00;

	private final static float            MIN_DEPTH_M  	= 0.3f;
	private final static float            MAX_DEPTH_M 	= 5.0f;

	private final static int              DEPTH_SCALE   = 2; 
	private final static int 			  DEPTH_OFFSET  = 10;

	private IStreamDepthAIOakD			oakd 			= null;

	private boolean 					enableStream  	= false;
	private Point2Transform2_F64 		p2n      		= null;

	private final Se3_F64       		to_ned          = new Se3_F64();
	private final Point2D3D             nearest_body    = new Point2D3D();
	private final Vector3D_F64          offset_body		= new Vector3D_F64();

	private long   						tms 			= 0;

	private final WorkQueue  wq  = WorkQueue.getInstance();
	private final MessageBus bus = MessageBus.getInstance();

	private final BlockingQueue<GrayU16> transfer_depth = new ArrayBlockingQueue<GrayU16>(10);
	private int   depth_worker;

	private final MAVOctoMap3D map;
	

	private final Planar<GrayU8>       depth_colored;
	private       List<YoloDetection>  detection;
	private final Point2D3D            per_p = new Point2D3D();
	private final DepthHandler         depth_handler;

	public <T> MAVOAKDDepthEstimator(IMAVMSPController control,  MSPConfig config, MAVOctoMap3D map, int width, int height, IVisualStreamHandler<Planar<GrayU8>> stream) {
		super(control);

		this.per_p.location.setTo(Double.NaN,Double.NaN,Double.NaN);
		
		depth_handler = new DepthHandler(width,height);

		// read offset settings
		offset_body.x = config.getFloatProperty(MSPParams.OAKD_OFFSET_X, String.valueOf(OFFSET_X));
		offset_body.y = config.getFloatProperty(MSPParams.OAKD_OFFSET_Y, String.valueOf(OFFSET_Y));
		offset_body.z = config.getFloatProperty(MSPParams.OAKD_OFFSET_Z, String.valueOf(OFFSET_Z));
		System.out.println("OAK-D Mounting offset: "+offset_body);

		boolean yolo_enabled = config.getBoolProperty(MSPParams.OAKD_YOLO_ENABLED, String.valueOf(false));

		try {
			if(yolo_enabled) {
				model.vision.setStatus(Vision.NN_ENABLED,true);
				String nn_path = MSPFileUtils.getJarContainingFolder(getClass())+"/../networks/yolov5n_coco_416x416.blob";
				if(MSPFileUtils.exists(nn_path)) {
					System.out.println("NN Network found :"+nn_path);
					this.oakd   = StreamYoloDepthAIOakD.getInstance(width, height,nn_path, 416,416);
				}
				else {
					nn_path = MSPFileUtils.getJarContainingFolder(getClass())+"/networks/yolov5n_coco_416x416.blob";
					if(MSPFileUtils.exists(nn_path)) {
						System.out.println("NN Network found :"+nn_path);
						this.oakd   = StreamYoloDepthAIOakD.getInstance(width, height,nn_path, 416,416);
					}
				}

				if(oakd==null)
					this.oakd   = StreamYoloDepthAIOakD.getInstance(width, height,null, 416,416);
			}
			else {
				model.vision.setStatus(Vision.NN_ENABLED,false);
				this.oakd   = StreamYoloDepthAIOakD.getInstance(width, height,null, 416,416);
			}

			model.vision.setStatus(Vision.NN_ENABLED, yolo_enabled);
			model.sys.setSensor(Status.MSP_AI_AVAILABILITY, model.vision.isStatus(Vision.NN_ENABLED));

			this.oakd.setRGBMode(true);
		} catch (Exception e) {
			e.printStackTrace();
		}

		this.map = map;

		this.depth_colored = new Planar<GrayU8>(GrayU8.class,width,height,3);

		if(stream!=null) {
			stream.registerOverlayListener(new DepthOverlayListener(model));
			if(oakd.isInference())
				stream.registerOverlayListener(new YoloOverlayListener(model));
		}

		oakd.registerCallback(new IDepthCallback<List<YoloDetection>>() {

			@Override
			public void process(final Planar<GrayU8> rgb, final GrayU16 depth, List<YoloDetection> d, long timeRgb, long timeDepth) {
				//
				if((System.currentTimeMillis() - tms) < 10)
					return;
				

				model.slam.fps = (model.slam.fps * 0.95f + ((float)(1000f / (System.currentTimeMillis()-tms)) -0.5f) * 0.05f);
				tms = System.currentTimeMillis();	

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

				detection = d;

			}
		});

	}

	public void start() throws Exception {
		if(oakd!=null) {

			oakd.start();
			p2n = (narrow(oakd.getIntrinsics())).undistort_F64(true,false);

			Thread.sleep(100);
			depth_worker = wq.addCyclicTask("LP",DEPTH_RATE, depth_handler);
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


	private class YoloOverlayListener extends AbstractOverlayListener {

		public YoloOverlayListener(DataModel model) {
			super(model);
		}

		@Override
		public void processOverlay(Graphics2D ctx, String stream_name, long tms_usec) {

			if(!enableStream)
				return;

			if(stream_name.contains("RGB")) {

				if(detection == null  || detection.size() == 0)
					return;

				ctx.drawLine(10,8,10,29);
				ctx.setFont(small);

				ctx.drawString("yolo",15,29);

				if(Double.isFinite(per_p.location.x)) {
					ctx.drawLine(70,8,70,29);
					ctx.setFont(big);
					ctx.drawString(onedecimal.format(per_p.location.x),75,18);
					ctx.drawString("Person",15,18);
					ctx.setFont(small);
					ctx.drawString("distance",75,29);

				} else {
					ctx.setFont(big);
					ctx.drawString("Active",15,18);
				}


				if(detection == null  || detection.size() == 0)
					return;

				ctx.setFont(small);

				for(YoloDetection n : detection) {
					ctx.drawRect(n.xmin, n.ymin, n.xmax - n.xmin, n.ymax - n.ymin);
					ctx.drawString(n.getLabel(),n.xmin, n.ymin-2);
				}


			}

			if(stream_name.contains("DEPTH")) {

				if(detection == null  || detection.size() == 0)
					return;

				for(YoloDetection n : detection) {
					ctx.drawRect(n.xmin, n.ymin, n.xmax - n.xmin, n.ymax - n.ymin);
					ctx.drawString(n.getLabel(),n.xmin, n.ymin-2);
				}
			}
		}
	}


	/*
	 * Overlay for nearest obstacle
	 * TODO: Overlay for person detected
	 */
	private class DepthOverlayListener extends AbstractOverlayListener {

		public DepthOverlayListener(DataModel model) {
			super(model);
		}

		@Override
		public void processOverlay(Graphics2D ctx, String stream_name, long tms_usec) {

			if(!enableStream)
				return;

			drawMinDist(ctx, stream_name, (int)nearest_body.observation.x, (int)nearest_body.observation.y);

		}

		private void drawMinDist(Graphics2D ctx, String stream,int x0, int y0) {

			if(stream.contains("DEPTH")) {

				if((x0==0 && y0 == 0))
					return;

				if(Float.isFinite(model.slam.dm)) {
					drawTriangle(ctx,x0,y0, Color.WHITE);
				}

				if(Double.isFinite(per_p.location.x)) {
					drawTriangle(ctx,(int)per_p.observation.x, (int)per_p.observation.y,Color.CYAN);
				}

				ctx.drawLine(10,8,10,29);
				ctx.setFont(big);
				if(Float.isFinite(model.slam.dm))
					ctx.drawString(onedecimal.format(model.slam.dm), 15, 18);
				else
					ctx.drawString("-", 15, 18);
				ctx.setFont(small);
				ctx.drawString("distance",15,29);

			}

		}
	}

	private void drawTriangle(Graphics2D ctx, int x0, int y0, Color color) {
		final int ln = 5;
		ctx.setColor(color);
		ctx.drawLine(x0-ln,y0-ln,x0+ln,y0-ln);
		ctx.drawLine(x0-ln,y0-ln,x0,y0+ln);
		ctx.drawLine(x0,y0+ln,x0+ln,y0-ln);
		ctx.setColor(Color.WHITE);
	}


	/*
	 * Processing depth data at 10 Hz.
	 */
	private class DepthHandler implements Runnable {

		private final Point2D_F64 norm     = new Point2D_F64();
		private final Point3D_F64 ned_pt_n = new Point3D_F64();

		private final Point2D3D   tmp_p = new Point2D3D();
		private final Point2D3D   ned_p = new Point2D3D();

		private final PointCloud scan = new PointCloud();

		private msp_msg_nn_object person = new msp_msg_nn_object();
		
		private int quality;
		
		private GrayU16 depth;

		public DepthHandler(int width, int height) {
         
		}

		@Override
		public void run() {

			if(transfer_depth.isEmpty())
				return;

			try {

				depth = transfer_depth.take();
				
				model.slam.quality = depthMapping(depth);

				GeometryMath_F64.mult(to_ned.R, nearest_body.location, ned_pt_n );
				ned_pt_n.plusIP(to_ned.T);

				if(detection != null) {
					synchronized(this) {
						// TODO: Put detections in Queue
						for(YoloDetection n : detection) {
							// check for persion and estimate the position
							if(n.id == 0 && determineObjectPosition(n, depth, per_p)) {
								person.tms = System.currentTimeMillis();
								break;
							}	
						}
					}
				}


				if(person.tms > 0 && (System.currentTimeMillis() - person.tms) > 500) {
					person.tms = 0;
					bus.publish(person);
					per_p.location.setTo(Double.NaN,Double.NaN,Double.NaN);
				}

				transfer_depth.clear();

			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}	

		private boolean determineObjectPosition(YoloDetection n, GrayU16 in, Point2D3D p) {

			// TODO: This is dangerous as another object detected of not detected could be in front
			//       of the person => wrong depth estimation

			int xc = (n.xmax-n.xmin) / 2 + n.xmin ; int yc = (n.ymax-n.ymin) / 2 + n.ymin;

			// if person covers too much of the picture => no valid estimation
			if((n.xmax-n.xmin) > 320)
				return false;

			// use multiple measurement points around the center for depth estimation
			int count = 0; int min_d = 0; int tmp_d = 0;
			tmp_d = in.get(xc  , yc   ); if(tmp_d < 6000 && tmp_d > 200) { min_d += tmp_d; count++; }
			tmp_d = in.get(xc+10,yc   ); if(tmp_d < 6000 && tmp_d > 200) { min_d += tmp_d; count++; }
			tmp_d = in.get(xc-10,yc   ); if(tmp_d < 6000 && tmp_d > 200) { min_d += tmp_d; count++; }
			tmp_d = in.get(xc  , yc+10); if(tmp_d < 6000 && tmp_d > 200) { min_d += tmp_d; count++; }
			tmp_d = in.get(xc  , yc-10); if(tmp_d < 6000 && tmp_d > 200) { min_d += tmp_d; count++; }
			tmp_d = in.get(xc  , yc+30); if(tmp_d < 6000 && tmp_d > 200) { min_d += tmp_d; count++; }
			tmp_d = in.get(xc  , yc-30); if(tmp_d < 6000 && tmp_d > 200) { min_d += tmp_d; count++; }

			if(count == 0)
				return false;

			min_d = min_d / count;

			p.observation.x = xc;
			p.observation.y = yc;

			if(min_d > 6000 || min_d < 200 ) {
				return false;
			}

			p2n.compute(p.observation.x,p.observation.y,norm);

			p.location.x =  min_d * 1e-3;		
			p.location.y =  p.location.x * norm.x;
			p.location.z =  p.location.x * norm.y;

			GeometryMath_F64.mult(to_ned.R, p.location, person.position );
			person.position.plusIP(to_ned.T);
			person.object_id = 0;

			if(control.isSimulation())
				return true;

			bus.publish(person);

			return true;

		}

		// map depth on a 640/DEPTH_SCALE x 480/DEPTH_SCALE basis
		//
		// TODO: Do depth estimation at original resolution  
		//
		private int depthMapping(GrayU16 in) {

		    quality = 0; nearest_body.location.x = Double.MAX_VALUE;

			scan.clear();
			

			for(int x = DEPTH_OFFSET; x < in.width-DEPTH_OFFSET;x = x + DEPTH_SCALE) {
				for(int y = 5; y < in.height-5;y = y + DEPTH_SCALE) {

					colorize(x,y,in,depth_colored, 9000);

					if(getSegmentPositionBody(x,y,in,tmp_p)) {

						GeometryMath_F64.mult(to_ned.R, tmp_p.location, ned_p.location );
						ned_p.location.plusIP(to_ned.T);
						if( Float.isFinite(model.hud.at) &&
								tmp_p.location.x< nearest_body.location.x && 
								// consider terrain as ground
								( ned_p.location.z < (-(model.hud.at+0.2f)) || Float.isNaN(model.hud.at)) ) 
							nearest_body.setTo(tmp_p);

						if(!model.sys.isStatus(Status.MSP_LANDED) && 
								
								// consider terrain as ground
								( ned_p.location.z < (-(model.hud.at+0.2f)) || Float.isNaN(model.hud.at))) {			
							MAVOctoMapTools.addToPointCloud(scan, ned_p.location);
						}
						quality++;
					}
				}
			}

			map.getTree().insertPointCloud(scan, new Point3D(to_ned.T.x, to_ned.T.y, -to_ned.T.z));

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

