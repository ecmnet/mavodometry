package com.comino.mavodometry.libdepthai;

import java.awt.image.BufferedImage;
import java.nio.ByteBuffer;
import java.nio.ShortBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;

import org.bytedeco.depthai.ColorCamera;
import org.bytedeco.depthai.ColorCameraProperties;
import org.bytedeco.depthai.ColorCameraProperties.ColorOrder;
import org.bytedeco.depthai.DataOutputQueue;
import org.bytedeco.depthai.Device;
import org.bytedeco.depthai.ImgFrame;
import org.bytedeco.depthai.MonoCamera;
import org.bytedeco.depthai.MonoCameraProperties;
import org.bytedeco.depthai.Pipeline;
import org.bytedeco.depthai.StereoDepth;
import org.bytedeco.depthai.StereoDepth.PresetMode;
import org.bytedeco.depthai.XLinkOut;
import org.bytedeco.depthai.global.depthai.CameraBoardSocket;
import org.bytedeco.depthai.global.depthai.MedianFilter;
import org.bytedeco.depthai.presets.depthai.Callback;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.PointerScope;

import com.comino.mavodometry.callback.IDepthCallback;
import com.comino.mavodometry.concurrency.OdometryPool;

import boofcv.struct.calib.CameraPinholeBrown;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;


public class StreamDepthAIOakD {

	private final static boolean  USE_USB2 = false;

	private static final int RGB_FRAME   = 0;
	private static final int DEPTH_FRAME = 1;

	private static StreamDepthAIOakD instance;

	private final    List<IDepthCallback> listeners;
	private final    Planar<GrayU8>       rgb;
	private final    GrayU16              depth;
	
	private final CameraPinholeBrown  intrinsics;


	private long             rgb_tms  = 0;
	private long           depth_tms  = 0;
	private int            frameCount = 0;

	private boolean is_running = false;

	private Device device;

	private CombineOAKDCallback callback;
	private final BlockingQueue<ImgFrame> transfer_rgb   = new ArrayBlockingQueue<ImgFrame>(4);
	private final BlockingQueue<ImgFrame> transfer_depth = new ArrayBlockingQueue<ImgFrame>(4);

	private int width;
	private int height;

	public static StreamDepthAIOakD getInstance(int width, int height) throws Exception {

		if(instance==null) {
			instance = new StreamDepthAIOakD(width,height);
		}
		return instance;
	}

	public StreamDepthAIOakD(int width, int height) {

		this.listeners  = new ArrayList<IDepthCallback>();
		this.rgb        = new Planar<GrayU8>(GrayU8.class,width,height,3);
		this.depth      = new GrayU16(width,height);
		this.intrinsics = new CameraPinholeBrown();
		this.width      = width;
		this.height     = height;

	}

	public StreamDepthAIOakD registerCallback(IDepthCallback listener) {
		listeners.add(listener);
		return this;
	}


	public void start() throws Exception {

		is_running = true;

		OdometryPool.submit(
		new Thread(()-> {
			while(is_running) {

				try {
					try (PointerScope scope = new PointerScope()) {	
						ImgFrame frame = transfer_rgb.take();
						if(frame != null && !frame.isNull()) {
							rgb_tms = System.currentTimeMillis();
							bufferRgbToMsU8(frame.getData().asByteBuffer(),rgb);
							frameCount = frame.getSequenceNum();
						}
					}
					
					try (PointerScope scope = new PointerScope()) {	
						ImgFrame frame = transfer_depth.take();
						if(frame != null && !frame.isNull()) {
							bufferDepthToU16(frame.getData().asByteBuffer().asShortBuffer(),depth);
						}
					}

					if(listeners.size()>0 ) {
						for(IDepthCallback listener : listeners)
							listener.process(rgb, depth, rgb_tms, depth_tms);
					}

				} catch (InterruptedException e) { }

			}

		}));
		

		callback = new CombineOAKDCallback();
		if(!is_running)
			throw new Exception("No OAKD camera found");
		callback.deallocate(false);

		System.out.println("OAK-D Lite depth pipeline started.");
	}

	public void stop() {
		is_running = false;
		if(device != null)
			device.close();
	}

	public int getFrameCount() {
		return frameCount;
	}

	public long getRGBTms() {
		return rgb_tms;
	}
	
	public long getDepthTms() {
		return depth_tms;
	}

	public boolean isRunning() {
		return is_running;
	}
	
	public CameraPinholeBrown getIntrinsics() {
		return intrinsics;
	}

	private class CombineOAKDCallback extends Callback {

		DataOutputQueue queue;

		public CombineOAKDCallback() {

			final Pipeline p = new Pipeline();
			p.deallocate(false);

			XLinkOut xlinkOut = p.createXLinkOut();
			xlinkOut.deallocate(false);
			xlinkOut.setStreamName("preview");

			StereoDepth depth = p.createStereoDepth();
			depth.setDefaultProfilePreset(PresetMode.HIGH_DENSITY);
			depth.initialConfig().setMedianFilter(MedianFilter.KERNEL_7x7);
			depth.setRectification(true);
			depth.setLeftRightCheck(false);
			depth.setExtendedDisparity(false);
			depth.setSubpixel(false);
			depth.initialConfig().setConfidenceThreshold(150);
			
			MonoCamera monoLeft = p.createMonoCamera();
			monoLeft.setResolution(MonoCameraProperties.SensorResolution.THE_480_P);
			monoLeft.setBoardSocket(CameraBoardSocket.LEFT);

			MonoCamera monoRight = p.createMonoCamera();
			monoRight.setResolution(MonoCameraProperties.SensorResolution.THE_480_P);
			monoRight.setBoardSocket(CameraBoardSocket.RIGHT);

			monoLeft.out().link(depth.left());
			monoRight.out().link(depth.right());

			ColorCamera colorCam = p.createColorCamera();
			colorCam.deallocate(false);
			colorCam.setPreviewSize(rgb.width, rgb.height);
			colorCam.setResolution(ColorCameraProperties.SensorResolution.THE_1080_P);
			colorCam.setColorOrder(ColorOrder.RGB);
			colorCam.setInterleaved(false);
			colorCam.preview().link(xlinkOut.input());
			
			depth.depth().link(xlinkOut.input());


			try {
				
				device = new Device(p,USE_USB2);
				device.deallocate(false);
				is_running = device.isPipelineRunning();
			} catch(RuntimeException e) {
				is_running = false;
				return;
			}

			try (@SuppressWarnings("unchecked")
			PointerScope scope = new PointerScope()) {	
				
				float[][] in = device.readCalibration().getCameraIntrinsics(CameraBoardSocket.LEFT).get();
				intrinsics.fx = in[0][0];
				intrinsics.fy = in[1][1];
				intrinsics.cx = in[0][2];
				intrinsics.cy = in[1][2];
				intrinsics.width  = width;
				intrinsics.height = height;
				
				IntPointer cameras = device.getConnectedCameras();
				for (int i = 0; i < cameras.limit(); i++) {
					System.out.print(cameras.get(i) + " ");
				}
				System.out.println("- "+device.getUsbSpeed());
				System.out.println(intrinsics);
			}

			queue = device.getOutputQueue("preview", 4, true);
			queue.deallocate(false);
			queue.addCallback(this);


		}

		public void call() {

			ImgFrame imgFrame = queue.getImgFrame();
			if(imgFrame!=null && !imgFrame.isNull() ) {
				try {
					switch(imgFrame.getInstanceNum()) {
					case RGB_FRAME:
						rgb_tms = System.currentTimeMillis();
						transfer_rgb.put(imgFrame);
						break;
					case DEPTH_FRAME:
						depth_tms = System.currentTimeMillis();
						transfer_depth.put(imgFrame);
						break;
					}
				} catch (InterruptedException e) {	}
			} 
		}	
	}
	
	private void bufferDepthToU16(ShortBuffer input , GrayU16 output ) {
		input.get(output.data);
	}


	private void bufferRgbToMsU8(ByteBuffer input,Planar<GrayU8> output) {	
		input.get(output.getBand(0).data);
		input.get(output.getBand(1).data);
		input.get(output.getBand(2).data);	
	}
	
	
	public static void main(String[] args) throws InterruptedException  {

		System.out.println("OAKD-Test");	

		BufferedImage im = new BufferedImage(640, 480, BufferedImage.TYPE_3BYTE_BGR);

		StreamDepthAIOakD oakd;
		try {
			oakd = StreamDepthAIOakD.getInstance(im.getWidth(), im.getHeight());
			oakd.registerCallback((image,np,t1,t2) -> {
				System.out.println(oakd.getFrameCount()+" "+oakd.getRGBTms()+" "+oakd.getDepthTms());
				//	ConvertBufferedImage.convertTo_U8(((Planar<GrayU8>)image), im, true);
			});
			oakd.start();
		} catch (Exception e) {
			e.printStackTrace();
		}

		while(true) {
			Thread.sleep(1000);
		}
	}


}


