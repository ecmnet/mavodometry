//package com.comino.mavodometry.estimators.simple;
//
//
//import org.gazebosim.transport.Node;
//import org.gazebosim.transport.Subscriber;
//
//import com.comino.mavcom.config.MSPConfig;
//import com.comino.mavcom.control.IMAVMSPController;
//import com.comino.mavodometry.estimators.MAVAbstractEstimator;
//import com.comino.mavodometry.video.IVisualStreamHandler;
//
//import boofcv.abst.distort.FDistort;
//import boofcv.alg.interpolate.InterpolationType;
//import boofcv.struct.border.BorderType;
//import boofcv.struct.image.GrayU8;
//import boofcv.struct.image.Planar;
//import gazebo.msgs.ImageOuterClass.Image;
//import gazebo.msgs.ImageStampedOuterClass.ImageStamped;
//
//public class MAVGazeboFpvNullEstimator extends MAVAbstractEstimator  {
//
//	private static Subscriber<ImageStamped> sub;
//
//	private static Node node;
//	private boolean is_running = false;
//	private Image image;
//
//	private Planar<GrayU8> img;
//	private GrayU8 imo;
//	private GrayU8 ims;
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
//	public <T> MAVGazeboFpvNullEstimator(IMAVMSPController control,  MSPConfig config, int width, int height, int mode, IVisualStreamHandler<Planar<GrayU8>> stream) {
//		super(control);
//		this.stream = stream;
//		this.width  = width;
//		this.height = height;
//
//		this.img = new Planar<GrayU8>(GrayU8.class, width,height,3);
//		this.ims = new GrayU8(width,height);
//		this.imo = new GrayU8(160,120);
//
//
//		Node node = new Node("default");
//     	FDistort scaler = new FDistort(imo, ims).interp(InterpolationType.NEAREST_NEIGHBOR).border(BorderType.ZERO);
//		try {
//			node.waitForConnection();
//			System.out.println("Gazebo camera connected...");
//
//			sub = node.subscribe("iris/fpv_cam/link/camera/image", ImageStamped.getDefaultInstance(), 
//					(msg) -> {
//						if(is_running) {
//							image = msg.getImage();
//
//							byte[] buf = image.toByteArray();
//							int i=0;
//							for(int y=0;y<image.getHeight();y++) {
//								for(int x=0;x<image.getWidth();x++) {
//									imo.data[x+imo.stride*y] = buf[i++];
//									i++;i++;
//								}
//							}
//							
//							scaler.scale().apply();
//							img.getBands()[0].data = ims.data;
//							img.getBands()[1].data = ims.data;
//							img.getBands()[2].data = ims.data;
//
//							if(stream!=null && enableStream && img != null) 
//								stream.addToStream(img, model, System.currentTimeMillis());
//
//							synchronized(sub) {
//								sub.notifyAll();
//							}
//						}
//					});
//
//
//
//		} catch (Exception e) {
//
//			e.printStackTrace();
//		}
//
//	}
//
//	// Main Image processing just returning quality
//	private int processImage(GrayU8 gray, Planar<GrayU8> color, long tms) {
//		return 100;
//	}
//
//	public void start() {
//
//		System.out.println("[vio] Starting GazeboFpV....");
//
//		is_running = true;
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
//
//}
