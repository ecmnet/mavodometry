package com.comino.mavodometry.estimators;

import static boofcv.factory.distort.LensDistortionFactory.narrow;

import java.awt.Color;
import java.awt.Graphics;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.control.IMAVMSPController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavcom.utils.MSP3DUtils;
import com.comino.mavodometry.librealsense.r200.RealSenseInfo;
import com.comino.mavodometry.librealsense.r200.boofcv.StreamRealSenseVisDepth;
import com.comino.mavodometry.librealsense.r200.boofcv.StreamRealSenseVisDepth.Listener;
import com.comino.mavodometry.video.IVisualStreamHandler;

import boofcv.alg.distort.PointToPixelTransform_F32;
import boofcv.alg.sfm.DepthSparse3D;
import boofcv.struct.distort.DoNothing2Transform2_F32;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import georegression.geometry.GeometryMath_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;

public class MAVR200DepthEstimator {

	private static final float COLLISION_WARNING_DISTANCE  = 1f;
	private static final float MAX_DISTANCE                = 10f;
	private static final int   CYCLE                       = 100;

	private StreamRealSenseVisDepth 	realsense	= null;
	private RealSenseInfo               info        = null;
	private DataModel                   model       = null;

	private boolean                     isRunning   = false;

	// Stream data
	private int   width;
	private int   height;

	private final Color	bgColor = new Color(128,128,128,130);

	private DepthSparse3D<GrayU16> pixel2Body = null;
	private Se3_F64                to_ned     = new Se3_F64();

	private double     	current_min_distance  = 0.0f;

	public <T> MAVR200DepthEstimator(IMAVMSPController control, MSPConfig config, int width, int height,  IMAVMapper mapper) {
		this(control,config,width,height, mapper,null);
	}

	public <T> MAVR200DepthEstimator(IMAVMSPController control, MSPConfig config, int width, int height,
			 IMAVMapper mapper, IVisualStreamHandler<Planar<GrayU8>> stream) {


		this.width   = width;
		this.height  = height;
		this.model   = control.getCurrentModel();

		this.pixel2Body = new DepthSparse3D.I<GrayU16>(1e-3);

		this.info = new RealSenseInfo(width,height, RealSenseInfo.MODE_RGB);

		this.realsense = new StreamRealSenseVisDepth(0,info);

		PointToPixelTransform_F32 visToDepth_pixel = new PointToPixelTransform_F32(new DoNothing2Transform2_F32());
		this.pixel2Body.configure(narrow(realsense.getIntrinsics()),visToDepth_pixel);



		if(stream!=null) {
			stream.registerOverlayListener(ctx -> {
				overlayFeatures(ctx);
			});
		}


		realsense.registerListener(new Listener() {

			final int BASE = height / 2; final int VIEW = 20;

			int x; int y; int depth_z; int raw_z;

			Point3D_F64 raw_pt  =  null;
			Point3D_F64 body_pt =  new Point3D_F64();
			Point3D_F64 ned_pt  =  new Point3D_F64();

			long tms = 0; int quality = 0;

			@Override
			public void process(Planar<GrayU8> rgb, GrayU16 depth, long timeRgb, long timeDepth) {

				// Add rgb image to stream
				if(stream!=null) {
					stream.addToStream(rgb, model, timeDepth);
				}

				// limit processing to 10Hz
				if((System.currentTimeMillis() - tms) < CYCLE)
					return;

				model.slam.fps = (float)Math.round(10000.0f / (System.currentTimeMillis() - tms))/10.0f;
				tms = System.currentTimeMillis();

				pixel2Body.setDepthImage(depth);
				MSP3DUtils.convertModelToSe3_F64(model, to_ned);

				current_min_distance = Double.MAX_VALUE;
				quality = 0;

				// Read minimum depth in a band around BASE
				for( x = 0; x < depth.width; x++ ) {

					depth_z = Integer.MAX_VALUE;

					for( y  = BASE - VIEW; y < BASE + VIEW; y++ ) {
						raw_z = depth.get(x, y);
						if(raw_z > 1 && raw_z < depth_z) {
							depth_z = raw_z;
						}
					}

					// valid depth found, transform and put into map
					if(depth_z < Integer.MAX_VALUE) {

						// transform to world coordinates in body frame
						pixel2Body.process(x, y);
						pixel2Body.getWorldPt();
						raw_pt = pixel2Body.getWorldPt();

					//	System.out.println(x+": "+raw_pt);

						quality++;

						// Maximum distance handled
						if(raw_pt.z > MAX_DISTANCE)
							continue;

						body_pt.set(raw_pt.z, raw_pt.x, raw_pt.y);


						// get min obstacle distance
						if(body_pt.x < current_min_distance)
							current_min_distance = body_pt.x;

						// put into map if map available
						if(mapper!=null) {

							// rotate in NED frame
							GeometryMath_F64.mult(to_ned.R, body_pt, ned_pt );
							ned_pt.plusIP(to_ned.T);

							mapper.update(model.state.l_x, model.state.l_y,ned_pt);

						}
					}

				}

				model.slam.quality = quality * 100 / width;
				model.slam.tms = model.sys.getSynchronizedPX4Time_us();

				if(current_min_distance<COLLISION_WARNING_DISTANCE)
					model.slam.dm = (float)current_min_distance;
				else
					model.slam.dm = Float.NaN;

			}

		});

		System.out.println("R200 depth estimator initialized");
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
		ctx.setColor(Color.white);

		ctx.drawLine(width/2-10, height/2, width/2+10, height/2);
		ctx.drawLine(width/2, height/2-10, width/2, height/2+10);

		if(!Float.isNaN(model.sys.t_armed_ms) && model.sys.isStatus(Status.MSP_ARMED)) {
			ctx.drawString(String.format("%.1f sec",model.sys.t_armed_ms/1000), 20, 20);
		}


		if(model.msg.text != null && (model.sys.getSynchronizedPX4Time_us()-model.msg.tms) < 1000000)
			ctx.drawString(model.msg.text, 10, height-5);

	}

	private void convert(GrayU16 in, Planar<GrayU8> output) {
		int value = 0;
		for( int y = 0; y < in.height; y++ ) {
			for( int x = 0; x < in.width; x++ ) {
					value = in.get(x, y);
				output.bands[0].set(x, y, (byte)(value >> 0 & 0x00FF)  );
				output.bands[1].set(x, y, (byte)(value >> 1 & 0x00FF)  );
				output.bands[2].set(x, y, (byte)(value >> 2 & 0x00FF) );
			}
		}
	}

}
