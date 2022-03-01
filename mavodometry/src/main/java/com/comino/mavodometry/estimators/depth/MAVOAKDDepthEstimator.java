package com.comino.mavodometry.estimators.depth;


import static boofcv.factory.distort.LensDistortionFactory.narrow;

import java.awt.Graphics;
import java.awt.image.BufferedImage;
import java.awt.image.WritableRaster;
import java.text.DecimalFormat;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.control.IMAVMSPController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavodometry.callback.IDepthCallback;
import com.comino.mavodometry.estimators.MAVAbstractEstimator;
import com.comino.mavodometry.libdepthai.StreamDepthAIOakD;
import com.comino.mavodometry.librealsense.d455.boofcv.StreamRealSenseD4xxDepthCV;
import com.comino.mavodometry.video.IVisualStreamHandler;
import com.comino.mavutils.workqueue.WorkQueue;

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

public class MAVOAKDDepthEstimator extends MAVAbstractEstimator  {

	private static final int         DEPTH_HEIGHT   = 90;
	private static final int         DEPTH_WIDTH    = 540;

	private static final float       quality_factor = 300f / ( DEPTH_WIDTH * DEPTH_HEIGHT) ;

	private static final float  WARN_OBS_DISTANCE   = 1.5f;
	private static final int             DEPTH_RATE = 135;

	private IVisualStreamHandler<Planar<GrayU8>> stream;
	private StreamDepthAIOakD oakd = null;

	private final BufferedImage    img;

	private boolean enableStream  = false;
	private boolean depth_overlay = false;

	private final GrayU16 sub;
	private GrayU16       proc;

	private Se3_F64       to_ned          = new Se3_F64();
	//	private Vector3D_F64  offset          = new Vector3D_F64();
	private float warn_obs_distance       = WARN_OBS_DISTANCE;

	private final Point2D_F64    norm     = new Point2D_F64();
	private Point2Transform2_F64 p2n      = null;

	private Point2D_I32 mindist_pt        =  new Point2D_I32();
	private String tmp;
	private final DecimalFormat fdistance  = new DecimalFormat("Obst: #0.0m");
	private final int width4;

	private long   tms = 0;

	// Window
	private int              depth_x_offs    = 0;
	private int              depth_y_offs    = 0;

	private final WorkQueue wq = WorkQueue.getInstance();
	private int depth_worker;


	public <T> MAVOAKDDepthEstimator(IMAVMSPController control,  MSPConfig config, int width, int height, IVisualStreamHandler<Planar<GrayU8>> stream) {
		super(control);

		this.sub = new GrayU16(DEPTH_WIDTH, DEPTH_HEIGHT);
		this.proc = new GrayU16(DEPTH_WIDTH, DEPTH_HEIGHT);

		this.width4 = width/4;

		this.depth_x_offs = (width - DEPTH_WIDTH ) / 2;
		this.depth_y_offs = (height - DEPTH_HEIGHT ) / 2;

		try {
			this.oakd   = StreamDepthAIOakD.getInstance(width, height);
		} catch (Exception e) {
			e.printStackTrace();
		}

		this.img = new BufferedImage(width, height, BufferedImage.TYPE_BYTE_INDEXED, ColorMap.setAlpha(ColorMap.JET,0.4));

		if(stream!=null) {
			stream.registerOverlayListener((ctx,tms) -> {
				if(enableStream) {
					overlayFeatures(ctx,tms);
					if(depth_overlay)
						ctx.drawImage(img, 0, 0, null);
				}
			});
		}
		
		oakd.registerCallback(new IDepthCallback() {

			@Override
			public void process(final Planar<GrayU8> rgb, final GrayU16 depth, long timeRgb, long timeDepth) {
				//				frame++;
				//				System.out.println(frame+": "+timeRgb);

				model.slam.fps = 1000f / (timeRgb - tms);
				tms = timeRgb;
				model.slam.tms = DataModel.getSynchronizedPX4Time_us();


				// make a copy of the depth area for asybchronous processing
				depth.subimage(depth_x_offs, depth_y_offs, depth_x_offs+DEPTH_WIDTH, depth_y_offs+DEPTH_HEIGHT, sub);	

				// Add rgb image to stream
				if(stream!=null && enableStream) {
					stream.addToStream(rgb, model, timeDepth);
				}
				if(depth_overlay && enableStream)
					overlayDepth(sub, img);

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
			wq.removeTask("LP", depth_worker);
			oakd.stop();
		}
	}

	public void enableStream(boolean enable) {
		this.enableStream = enable;
	}

	private void overlayFeatures(Graphics ctx, long tms) {

		if(!enableStream)
			return;

		drawDepthArea(ctx,depth_x_offs,depth_y_offs,depth_x_offs+DEPTH_WIDTH,depth_y_offs+DEPTH_HEIGHT);

		if((model.sys.isStatus(Status.MSP_ARMED) || control.isSimulation()) && Float.isFinite(model.slam.dm) && model.slam.dm < warn_obs_distance) {
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
				pixel[0] = depth_area.get(x, y) / 10;
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

		int raw_z;
		int quality = 0;

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

					if(raw_z < 20 || raw_z >= 8000)
						continue;

					quality++;

					p2n.compute(x,y,norm);
					raw_pt.z =  raw_z*1e-3;
					raw_pt.y = -raw_pt.z*norm.y;
					raw_pt.x =  raw_pt.z*norm.x;
					


					body_pt.setTo(raw_pt.z, raw_pt.x, raw_pt.y);
					
					
					//					body_pt.plusIP(offset);
					//					GeometryMath_F64.mult(to_ned.R, body_pt, ned_pt );
					//					ned_pt.plusIP(to_ned.T);

					//					System.out.println(norm.x+"/"+norm.y);

					distance = body_pt.norm();
					if(distance < min_distance) {
						mindist_pt.setTo(x,y);
						min_distance = distance;
						body_pt_n.setTo(body_pt);
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
