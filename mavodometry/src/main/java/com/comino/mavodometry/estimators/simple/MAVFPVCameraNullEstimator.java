//package com.comino.mavodometry.estimators.simple;
//
//
//import com.comino.mavcom.config.MSPConfig;
//import com.comino.mavcom.control.IMAVMSPController;
//import com.comino.mavcom.model.DataModel;
//import com.comino.mavodometry.concurrency.OdometryPool;
//import com.comino.mavodometry.estimators.MAVAbstractEstimator;
//import com.comino.mavodometry.libwebcam.StreamWebcam;
//import com.comino.mavodometry.libwebcam.StreamWebcam.SimpleSequence;
//import com.comino.mavodometry.video.IVisualStreamHandler;
//
//import boofcv.struct.image.GrayU8;
//import boofcv.struct.image.ImageType;
//import boofcv.struct.image.Planar;
//
//public class MAVFPVCameraNullEstimator extends MAVAbstractEstimator  {
//
//	private final static float FRAME_RATE = 15.0f;
//
//	private boolean is_running = false;
//
//	private IVisualStreamHandler<Planar<GrayU8>> stream;
//
//
//	private boolean enableStream;
//
//	private final int width;
//	private final int height;
//
//
//	public <T> MAVFPVCameraNullEstimator(IMAVMSPController control,  MSPConfig config, int width, int height, int mode, IVisualStreamHandler<Planar<GrayU8>> stream) {
//		super(control);
//		this.stream = stream;
//		this.width  = width;
//		this.height = height;
//	}
//
//	// Main Image processing just returning quality
//	private int processImage(GrayU8 gray, Planar<GrayU8> color, long tms) {
//       return 100;
//	}
//
//	public void start() {
//
//		System.out.println("[vio] Starting camera....");
//
//		is_running = true;
//		OdometryPool.submit(new WebCamThread(model));
//
//
//	}
//
//	public void stop() {
//		is_running = false;
//	}
//
//	public void enableStream(boolean enable) {
//		this.enableStream = enable;
//	}
//
//
//	private class WebCamThread extends Thread {
//
//		private Planar<GrayU8> img;
//		private StreamWebcam webcam;
//		private SimpleSequence<Planar<GrayU8>> sequence;
//
//		private int count=0;
//		private DataModel model;
//
//		public WebCamThread(DataModel model) {
//
//			this.model = model;
//			this.webcam = new StreamWebcam();
//			this.sequence = (SimpleSequence<Planar<GrayU8>>) webcam.open("0",width,height,FRAME_RATE, ImageType.pl(3,GrayU8.class) );
//
//		}
//
//		@Override
//		public void run() {
//
//			long tms =System.currentTimeMillis();
//			long dt  = 0;
//
//			while(is_running) {
//
//				if(sequence.hasNext()) {
//
//					img = sequence.next(); count++;
//
//					model.slam.tms = webcam.getGrabber().getTimestamp();
//					model.slam.quality =processImage(img.bands[0], img, model.slam.tms);
//
//					// Add rgb image to stream
//					if(stream!=null && enableStream && count > 0  && img != null) 
//						stream.addToStream(img, model, System.currentTimeMillis());
//
//					
//					dt = (System.currentTimeMillis() - tms);
//					model.slam.fps = model.slam.fps * (0.75f) + 1000.0f * 0.25f / dt;
//					tms = System.currentTimeMillis();
//
//				}
//
//			}	
//			model.slam.quality =0; model.slam.fps=0;
//			webcam.stop();
//
//		}
//	}
//
//
//
//}
