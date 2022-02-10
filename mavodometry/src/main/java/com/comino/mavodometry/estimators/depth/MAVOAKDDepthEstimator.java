package com.comino.mavodometry.estimators.depth;


import com.comino.mavcom.config.MSPConfig;
import com.comino.mavcom.control.IMAVMSPController;
import com.comino.mavodometry.callback.IDepthCallback;
import com.comino.mavodometry.estimators.MAVAbstractEstimator;
import com.comino.mavodometry.libdepthai.StreamDepthAIOakD;
import com.comino.mavodometry.librealsense.d455.boofcv.StreamRealSenseD4xxDepthCV;
import com.comino.mavodometry.video.IVisualStreamHandler;

import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;

public class MAVOAKDDepthEstimator extends MAVAbstractEstimator  {

	private boolean is_running = false;

	private IVisualStreamHandler<Planar<GrayU8>> stream;
	private StreamDepthAIOakD oakd = null;


	private boolean enableStream;

	private long  tms = 0;


	public <T> MAVOAKDDepthEstimator(IMAVMSPController control,  MSPConfig config, int width, int height, IVisualStreamHandler<Planar<GrayU8>> stream) {
		super(control);
		
		try {
			this.oakd   = StreamDepthAIOakD.getInstance(width, height);
		} catch (Exception e) {
			e.printStackTrace();
		}
		
		oakd.registerCallback(new IDepthCallback() {

			@Override
			public void process(Planar<GrayU8> rgb, GrayU16 depth, long timeRgb, long timeDepth) {
				
				
				model.slam.fps = 1000f / (timeRgb - tms);
				model.slam.quality = 100;
				tms = timeRgb;
			
				if(stream!=null && enableStream) {
					stream.addToStream(rgb, model, timeRgb);
				}
				
			}
			
		});
		
	}

	public void start() {
	  if(oakd!=null)
         oakd.start();
		
	}

	public void stop() {
		 if(oakd!=null)
			 oakd.stop();
		is_running = false;
	}

	public void enableStream(boolean enable) {
		this.enableStream = enable;
	}


	


}
