package com.comino.mavodometry.estimators.depth;


import static boofcv.factory.distort.LensDistortionFactory.narrow;

import java.awt.Graphics;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.control.IMAVMSPController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavodometry.callback.IDepthCallback;
import com.comino.mavodometry.estimators.MAVAbstractEstimator;
import com.comino.mavodometry.libdepthai.StreamDepthAIOakD;
import com.comino.mavodometry.video.IVisualStreamHandler;

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

public class MAVOAKDDepthSegmentEstimator extends MAVAbstractEstimator  {

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
	private final Point3D_F64 			ned_pt_n    	= new Point3D_F64();
	
	private final Point2D3D             nearest         = new Point2D3D();

	private long   						tms 			= 0;


	public <T> MAVOAKDDepthSegmentEstimator(IMAVMSPController control,  MSPConfig config, int width, int height, IVisualStreamHandler<Planar<GrayU8>> stream) {
		super(control);

		try {
			this.oakd   = StreamDepthAIOakD.getInstance(width, height);
			this.oakd.setRGBMode(!depth_overlay);
		} catch (Exception e) {
			e.printStackTrace();
		}

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

				model.slam.fps = 1000f / (timeRgb - tms);
				tms = timeRgb;
				model.slam.tms = DataModel.getSynchronizedPX4Time_us();

				MSP3DUtils.convertModelToSe3_F64(model, to_ned);

				model.slam.quality = buildMeanDepthSegments(depth,seg_distance);

			    getNearestSegment(seg_distance, nearest);
				model.slam.dm = (float)nearest.location.x; 

				GeometryMath_F64.mult(to_ned.R, nearest.location, ned_pt_n );
				ned_pt_n.plusIP(to_ned.T);

				model.slam.ox = (float)ned_pt_n.x;
				model.slam.oy = (float)ned_pt_n.y;
				model.slam.oz = (float)ned_pt_n.z;


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
		}
	}

	public void stop() {
		if(oakd!=null) {
			oakd.stop();
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

	final Point2D3D p = new Point2D3D();
	private void overlayDepthSegments(Graphics ctx, GrayF32 seg) {
		for(int x = 0; x < seg.width;x++) {
			// Skip first line
			for(int y = 1; y < seg.height;y++) {
				ctx.drawRect(x*DEPTH_SEG_W, y*DEPTH_SEG_H, DEPTH_SEG_W, DEPTH_SEG_H);
				if(seg_distance.get(x, y)> 0) {
					getSegmentPositionBody(x, y,seg, p);
					ctx.drawString(String.format("%+#.1f",p.location.x),x*DEPTH_SEG_W+5, y*DEPTH_SEG_H+10);
					//				  ctx.drawString(String.format("%+#.1f",p.y),x*DEPTH_SEG_W+5, y*DEPTH_SEG_H+20);
					//				  ctx.drawString(String.format("%+#.1f",p.z),x*DEPTH_SEG_W+5, y*DEPTH_SEG_H+30);
				}

			}
		}
	}

	// Return 3D point of depth segment in body frame
	final Point2D_F64 norm = new Point2D_F64();
	private void getSegmentPositionBody(int x, int y, GrayF32 seg, Point2D3D p) {
		
		p.observation.x = x*DEPTH_SEG_W+DEPTH_SEG_W/2;
		p.observation.y = y*DEPTH_SEG_H+DEPTH_SEG_H/2;
		
		p2n.compute(p.observation.x,p.observation.y,norm);
		
		p.location.x = seg.get(x, y);
		p.location.y =   p.location.x * norm.x;
		p.location.z = - p.location.x * norm.y;
		
	}

	// Build depth segments by calculating the mean distance
	private int buildMeanDepthSegments(GrayU16 in, GrayF32 out) {

		int mean_dist_mm = 0; int valid_count=0; float quality = 0; int d = 0;

		final int xr = in.width  / out.width; 
		final int yr = in.height / out.height;

		for(int x = 0; x < out.width;x++) {
			for(int y = 0; y < out.height;y++) {

				mean_dist_mm = 0; valid_count = 0;

				for(int ix=0;ix<xr;ix++) {
					for(int iy=0;iy<yr;iy++) {
						d = in.get(x*xr+ix, y*yr+iy);
						if(d < MIN_DEPTH_MM || d > MAX_DEPTH_MM)
							continue;
						mean_dist_mm = mean_dist_mm + d; valid_count += 1000;
					}	
				}

				if(valid_count > 0) {
					quality += 1;//(float)valid_count / tc;
					out.set(x, y, (float)mean_dist_mm / valid_count);
				}
				else
					out.set(x, y, -1);

			}
		}
		return (int)(quality * 100f / out.data.length);
	}

}
