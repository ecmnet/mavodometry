package com.comino.mavodometry.estimators.depth;

import java.awt.Graphics;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.control.IMAVMSPController;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavmap.map.map3D.impl.octree.LocalMap3D;
import com.comino.mavmap.struct.Point2D3DW;
import com.comino.mavodometry.estimators.MAVAbstractEstimator;
import com.comino.mavodometry.video.IVisualStreamHandler;
import com.comino.mavutils.workqueue.WorkQueue;

import boofcv.alg.sfm.robust.GenerateSe2_PlanePtPixel;
import boofcv.struct.geo.Point2D3D;
import boofcv.struct.image.GrayF32;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import georegression.geometry.GeometryMath_F64;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;

public class MAVSimDepthSegmentEstimator extends MAVAbstractEstimator  {

	private static final int             DEPTH_RATE      = 100;

	private final static int            DEPTH_SEG_W 	 = 16;
	private final static int            DEPTH_SEG_H 	 = 16;

	private final static int            MIN_DEPTH_MM 	 = 350;
	private final static int            MAX_DEPTH_MM 	 = 8000;

	private static final float          MAP_MAX_DISTANCE = 8.0f;
	private static final float          MAP_DELTADOWN    = 0.4f;

	private final GrayF32        		seg_distance;

	private boolean 					enableStream  	= false;
	private boolean 					depth_overlay 	= false;

	private final Se3_F64       		to_ned          = new Se3_F64();
	private final Point2D3DW            nearest_body    = new Point2D3DW();
	private final List<Point2D3DW>      segments_ned    = new ArrayList<Point2D3DW>();

	private long   						tms 			= 0;
	private int                         width34         = 0;

	private int 						depth_worker;

	private final DecimalFormat fdistance  = new DecimalFormat("Obst: #0.0m");

	private final WorkQueue wq = WorkQueue.getInstance();


	private final LocalMap3D map;

	/* Idea: 
	 * 1. Generate a sequence of body frame depth-segment-frames either by program of from file
	 * 2. Rotate it to NED 
	 * 3. Put it into the map
	 * Purpose: Test how depth segment frames can be used to build the map and to replay recorded data
	 * Notes:
	 * 
	 */

	public <T> MAVSimDepthSegmentEstimator(IMAVMSPController control,  MSPConfig config, LocalMap3D map, int width, int height, IVisualStreamHandler<Planar<GrayU8>> stream) {
		super(control);


		this.width34 = width * 3 / 4;
		this.map = map;

		this.seg_distance 	= new GrayF32(width/DEPTH_SEG_W,height/DEPTH_SEG_H);

		for(int i=0; i < seg_distance.data.length;i++)
			this.segments_ned.add(new Point2D3DW());

		if(stream!=null) {
			stream.registerOverlayListener((ctx,n,tms) -> {
				if(enableStream) {
					overlayFeatures(ctx,tms);
					if(depth_overlay)
						overlayDepthSegments(ctx, seg_distance);

				}
			});
		}

	}

	public void start() throws Exception {
		System.out.println("Depth simulation started");
		depth_worker = wq.addCyclicTask("LP",DEPTH_RATE, new DepthHandler());
	}

	public void stop() {
		wq.removeTask("LP", depth_worker);
	}

	public void enableStream(boolean enable) {
		this.enableStream = enable;
	}

	private void overlayFeatures(Graphics ctx, long tms) {

		if(!enableStream)
			return;

		drawMinDist(ctx,(int)nearest_body.observation.x, (int)nearest_body.observation.y);

	}

	private void drawMinDist(Graphics ctx, int x0, int y0) {

		if(x0==0 && y0 == 0)
			return;

		final int ln = 10;

		ctx.drawLine(x0-ln,y0-ln,x0+ln,y0-ln);
		ctx.drawLine(x0-ln,y0-ln,x0,y0+ln);
		ctx.drawLine(x0,y0+ln,x0+ln,y0-ln);

		String tmp = fdistance.format(model.slam.dm);
		ctx.drawString(fdistance.format(model.slam.dm), width34 - ctx.getFontMetrics().stringWidth(tmp)/2, 20);

	}

	final Point2D3DW p = new Point2D3DW();
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

		private Point3D_F64 tmp_p;
		private Point3D_F64 tmp_r           = new Point3D_F64();
		private double   distance;

		private long start_tms  = System.currentTimeMillis();
//		private long start_tms2 = System.currentTimeMillis();
		
		private int state = 0;
		
		private int cycle = 0;

		@Override
		public void run() {

			long since_tms = System.currentTimeMillis() - start_tms;
			
			MSP3DUtils.convertModelToSe3_F64(model, to_ned);
			
			if(map.size() == 0) {
				state = 0; cycle = 0;
			}
			
			for(int i=0; i < seg_distance.data.length;i++) 
				segments_ned.get(i).setTo(0, 0, Double.NaN, Double.NaN, Double.NaN);

			// TODO: Build segments from file or sim

			if(since_tms > 2000 && state == 0) {
				System.out.println("State:"+state);
				segments_ned.get(10).setTo(1, 1, 0, 1, 0,0.1);
				//			    	segments_ned.get(11).setTo(1, 1, 1, 0, -1.1,0.1);
				//			    	segments_ned.get(12).setTo(1, 1, 1, 0, -1.2,0.1);
				
				for(int k=0; k <850;k++) {
					segments_ned.get(k+20).setTo(1, 1, Math.random()*2-1+to_ned.T.x, 
							1+Math.random()/5-0.1 +to_ned.T.y,
							-Math.random()*2+to_ned.T.z
							, 0.1);
				}

				state = 1;
				}


			if(since_tms > 6000 && state == 1) {
				System.out.println("State:"+state);
				segments_ned.get(13).setTo(1, 1, 0, 2, 0,  0.1);
				//			    	segments_ned.get(14).setTo(1, 1, 2.7, 0, -1.1,0.2);
				//			    	segments_ned.get(15).setTo(1, 1, 2.7, 0, -1.2,0.2);
				
				for(int k=0; k <250;k++) {
					segments_ned.get(k+20).setTo(1, 1, Math.random()*3-1.5, 2+Math.random()/5-0.1, -Math.random()*3, 0.1);
				}

				state = 2;

			}
			
			if(since_tms > 9000 && state == 2) {
				System.out.println("State:"+state);
				segments_ned.get(14).setTo(1, 1, 0, 3 ,0,  0.1);
				//			    	segments_ned.get(14).setTo(1, 1, 2.7, 0, -1.1,0.2);
				//			    	segments_ned.get(15).setTo(1, 1, 2.7, 0, -1.2,0.2);
				
				for(int k=0; k <250;k++) {
					segments_ned.get(k+20).setTo(1, 1, Math.random()*4-2, 3+Math.random()/5-0.1, -Math.random()*4, 0.1);
				}

				start_tms = System.currentTimeMillis();
				if(++cycle > 20)
					state = -1;
				else
				state = 0;

			}
			
			transferToMap();


			model.slam.quality = 100;

			model.slam.fps = 1000f / (System.currentTimeMillis() - tms) + 0.5f;
			tms = System.currentTimeMillis();

			// Build segmentlist

			// TODO: Get and publish nearest segment

			model.slam.dm = (float)nearest_body.location.x; 

			GeometryMath_F64.mult(to_ned.R, nearest_body.location, ned_pt_n );
			ned_pt_n.plusIP(to_ned.T);

			model.slam.ox = (float)ned_pt_n.x;
			model.slam.oy = (float)ned_pt_n.y;
			model.slam.oz = (float)ned_pt_n.z;
		}	

		private void transferToMap() {
			int ext_ratio = 1; double offset=0; double v=0;
			for(int i=0; i < seg_distance.data.length;i++) {
				tmp_p = segments_ned.get(i).location;
				if(Double.isFinite(tmp_p.x) && tmp_p.z < ( to_ned.T.z + MAP_DELTADOWN)) {
					distance = MSP3DUtils.distance3D(tmp_p, to_ned.T);
					if( distance< MAP_MAX_DISTANCE && distance > 0)
						// TODO: Scale segment size according to distance;
						// Question: Really needed as free space grows with occupied along the distance
						// For now => do nothing here
						// - Build up a list for projected segment size
						// - if projected segment size > map block size, ensure that all map blocks are set in the perpendicular plane
						// - maybe a simplified approach: consider 3, 5 or 7 distances
					//	ext_ratio  = (int)(segments_ned.get(i).extension / map.getMapInfo().getCellSize()+0.5);

				//	if(ext_ratio <= 1)
						map.update(to_ned.T,tmp_p);
//					else {
//						offset = ext_ratio * map.getMapInfo().getCellSize() / 2d;
//						// Works basically - TODO: Cover roundings (include edges) and rotate  tmp_r ned to NED draw into plane
//						tmp_r.x = tmp_p.x;
//						for(int y = 0; y < ext_ratio; y++) {
//							v = y  * map.getMapInfo().getCellSize();
//							if(v < offset)
//								tmp_r.y = tmp_p.y + v - offset - 0.01;
//							else
//								tmp_r.y = tmp_p.y + v - offset + 0.01;
//							for(int z = 0; z < ext_ratio; z++) {
//								v = z * map.getMapInfo().getCellSize();
//								if(v < offset)
//									tmp_r.z = tmp_p.z + z * map.getMapInfo().getCellSize() - offset + 0.01; 
//								else
//									tmp_r.z = tmp_p.z + z * map.getMapInfo().getCellSize() - offset -0.01; 
//								map.update(to_ned.T,tmp_r);
//							}
//						}
//
//					}
				}
			}
			
			 
		}

		// Build depth segments by calculating the min distance
		private int buildMinDepthSegments(GrayU16 in, GrayF32 out) {

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


		public String toString() {
			StringBuilder s = new StringBuilder();
			for(int i=0;i<segments_ned.size();i++) {
				s.append(segments_ned.get(i).location); s.append('\n');
			}
			return s.toString();
		}

	}

}
