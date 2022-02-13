package com.comino.mavodometry.libdepthai;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;

import org.bytedeco.depthai.ADatatype;
import org.bytedeco.depthai.ColorCamera;
import org.bytedeco.depthai.ColorCameraProperties;
import org.bytedeco.depthai.ColorCameraProperties.ColorOrder;
import org.bytedeco.depthai.DataOutputQueue;
import org.bytedeco.depthai.Device;
import org.bytedeco.depthai.ImgFrame;
import org.bytedeco.depthai.Pipeline;
import org.bytedeco.depthai.RawBuffer;
import org.bytedeco.depthai.RawImgFrame;
import org.bytedeco.depthai.XLinkOut;
import org.bytedeco.depthai.presets.depthai.Callback;
import org.bytedeco.depthai.presets.depthai.MessageCallback;
import org.bytedeco.depthai.presets.depthai.NameMessageCallback;
import org.bytedeco.depthai.presets.depthai.RawBufferCallback;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.javacpp.PointerScope;

import com.comino.mavodometry.callback.IDepthCallback;

import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;

public class StreamDepthAIOakD {

	private final static boolean             USE_USB2 = false;


	private static StreamDepthAIOakD instance;

	private final List<IDepthCallback> listeners;
	private final Planar<GrayU8>       rgb;

	private boolean is_running = false;

	private static Device device;
	private static MessageCallback callback;

	public static StreamDepthAIOakD getInstance(int width, int height) throws Exception {

		if(instance==null) {

			Pipeline p = new Pipeline();
			p.deallocate(false);


			//	try (PointerScope scope = new PointerScope()) {	

			XLinkOut xlinkOut = p.createXLinkOut();
			xlinkOut.deallocate(false);
			xlinkOut.setStreamName("preview");

			ColorCamera colorCam = p.createColorCamera();
			colorCam.deallocate(false);


			colorCam.setPreviewSize(width, height);
			colorCam.setResolution(ColorCameraProperties.SensorResolution.THE_1080_P);
			colorCam.setColorOrder(ColorOrder.RGB);
			colorCam.setInterleaved(true);
			colorCam.preview().link(xlinkOut.input());
			//	}

			instance = new StreamDepthAIOakD(p,width,height);
		}
		return instance;
	}

	public static StreamDepthAIOakD getInstance(Pipeline p, int width, int height) throws Exception {
		if(instance==null)
			instance = new StreamDepthAIOakD(p,width,height);
		return instance;
	}

	public StreamDepthAIOakD(Pipeline p, int width, int height) {

		this.listeners = new ArrayList<IDepthCallback>();
		this.rgb       = new Planar<GrayU8>(GrayU8.class,width,height,3);
		
		callback = new ColorCameraOakDCallback(rgb);
		callback.deallocate(false);

		try {
			device = new Device(p,USE_USB2);
			device.deallocate(false);
			is_running = device.isPipelineRunning();
		} catch(RuntimeException e) {
			is_running = false;
			return;
		}

		DataOutputQueue queue = device.getOutputQueue("preview", 4, false);
		queue.deallocate(false);
		queue.addCallback(callback);  


		System.out.print("OAK-D cameras: ");

		try (PointerScope scope = new PointerScope()) {	
			IntPointer cameras = device.getConnectedCameras();
			for (int i = 0; i < cameras.limit(); i++) {
				System.out.print(cameras.get(i) + " ");
			}
		}


		System.out.println();

	}

	public StreamDepthAIOakD registerCallback(IDepthCallback listener) {
		listeners.add(listener);
		return this;
	}


	public void start() {
		if(is_running)
			System.out.println("OAK-D Lite pipeline started");
	}

	public void stop() {
		if(device == null)
			return;
		device.close();
		is_running = false;
	}

	public boolean isRunning() {
		return is_running;
	}

	private class ColorCameraOakDCallback extends MessageCallback {

		int          frameno=0;
		Planar<GrayU8> output;
		ADatatype data;
		//		DataOutputQueue queue;

		public ColorCameraOakDCallback(final Planar<GrayU8> output) {
			super();
			this.output = output;
			//			this.queue  = queue;
		}

		@Override 
		public void call(Pointer p) {
			p.deallocate(false);
			data = p.getPointer(ADatatype.class);
			
	    	try (PointerScope scope = new PointerScope()) {	
			   ImgFrame imgFrame = data.getRaw().getPointer(ImgFrame.class);
				
				frameno++;
				System.out.println(imgFrame.getSequenceNum());
				//				if(data!=null && !data.isNull()) {
				//					ImgFrame imgFrame = data.getPointer(ImgFrame.class);
				//					if(imgFrame!=null && !imgFrame.isNull()) {
				//						System.out.println(imgFrame.getSequenceNum());
				//					}
				//				}
			}

			//			if(listeners.size()>0 ) {
			//				for(IDepthCallback listener : listeners)
			//					listener.process(rgb, null, System.currentTimeMillis(), System.currentTimeMillis());
			//			}

		}

		public void bufferRgbToMsU8(ByteBuffer input ) {

			byte[] b0 = output.getBand(0).data;
			byte[] b1 = output.getBand(1).data;
			byte[] b2 = output.getBand(2).data;

			for(int  y = 0; y < output.height; y++ ) {
				int indexOut = output.startIndex + y*output.stride;
				for( int x = 0; x < output.width; x++ , indexOut++ ) {
					b0[indexOut] = input.get();
					b1[indexOut] = input.get();
					b2[indexOut] = input.get();
				}
			}

		}
	}

}
