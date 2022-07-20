package com.comino.mavodometry.estimators.simple;


import java.awt.image.BufferedImage;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.control.IMAVMSPController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavodometry.callback.IDepthCallback;
import com.comino.mavodometry.estimators.MAVAbstractEstimator;
import com.comino.mavodometry.libdepthai.StreamRGBAIOakD;
import com.comino.mavodometry.video.IVisualStreamHandler;

import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import edu.mines.jtk.awt.ColorMap;

public class MAVOAKDCamEstimator extends MAVAbstractEstimator  {


	private IVisualStreamHandler<Planar<GrayU8>> stream;
	private StreamRGBAIOakD oakd = null;

	private boolean enableStream  = false;
	
	private long   tms = 0;


	public <T> MAVOAKDCamEstimator(IMAVMSPController control,  MSPConfig config, int width, int height, IVisualStreamHandler<Planar<GrayU8>> stream) {
		super(control);

		try {
			this.oakd   = StreamRGBAIOakD.getInstance(width, height);
		} catch (Exception e) {
			e.printStackTrace();
		}

		oakd.registerCallback(new IDepthCallback() {

			@Override
			public void process(final Planar<GrayU8> rgb, final GrayU16 depth, long timeRgb, long timeDepth) {
				//				frame++;
				//				System.out.println(frame+": "+timeRgb);

				model.slam.fps = 1000f / (timeRgb - tms);
				tms = timeRgb;
				model.slam.quality = 100;
				model.slam.tms = DataModel.getSynchronizedPX4Time_us();
	

				// Add rgb image to stream
				if(stream!=null && enableStream) {
					stream.addToStream(getClass().getName(),rgb, model, timeDepth);
				}

			}
		});

	}

	public void start() throws Exception {
		if(oakd!=null) {
			oakd.start();
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
}
