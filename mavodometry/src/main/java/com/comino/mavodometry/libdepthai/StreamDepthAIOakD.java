package com.comino.mavodometry.libdepthai;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;

import org.bytedeco.depthai.ColorCamera;
import org.bytedeco.depthai.ColorCameraProperties;
import org.bytedeco.depthai.ColorCameraProperties.ColorOrder;
import org.bytedeco.depthai.DataOutputQueue;
import org.bytedeco.depthai.Device;
import org.bytedeco.depthai.ImgFrame;
import org.bytedeco.depthai.Pipeline;
import org.bytedeco.depthai.XLinkOut;
import org.bytedeco.depthai.presets.depthai.Callback;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.PointerScope;

import com.comino.mavodometry.callback.IDepthCallback;

import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;

public class StreamDepthAIOakD {
	
	private final static boolean             USE_USB2 = true;
	

	private static StreamDepthAIOakD instance;

	private final List<IDepthCallback> listeners;
	private final Pipeline             pipe;
	private final Planar<GrayU8>       rgb;

	private boolean is_running = false;

	private Device device;

	public static StreamDepthAIOakD getInstance(int width, int height) throws Exception {
		if(instance==null) {
			
			Pipeline p = new Pipeline();

			ColorCamera colorCam = p.createColorCamera();

			XLinkOut xlinkOut = p.createXLinkOut();
			xlinkOut.setStreamName("preview");

			colorCam.setPreviewSize(width, height);
			colorCam.setResolution(ColorCameraProperties.SensorResolution.THE_1080_P);
			colorCam.setColorOrder(ColorOrder.RGB);
			colorCam.setInterleaved(true);

			colorCam.preview().link(xlinkOut.input());

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
		this.pipe      = p;
		this.listeners = new ArrayList<IDepthCallback>();
		this.rgb       = new Planar<GrayU8>(GrayU8.class,width,height,3);
	}

	public StreamDepthAIOakD registerCallback(IDepthCallback listener) {
		listeners.add(listener);
		return this;
	}


	public void start() {

		if(pipe == null)
			return;

		is_running = true;
		
		try {
			device = new Device(pipe,USE_USB2);
		} catch(RuntimeException e) {
			e.printStackTrace();
			is_running = false;
			return;
		}
		
		System.out.print("Connected cameras: ");
		IntPointer cameras = device.getConnectedCameras();
		for (int i = 0; i < cameras.limit(); i++) {
			System.out.print(cameras.get(i) + " ");
		}
		System.out.println();

		DataOutputQueue colorQueue = device.getOutputQueue("preview", 4, true);
		colorQueue.addCallback(new ColorCameraOakDCallback(colorQueue, rgb));

		System.out.println("OAK-D Lite pipeline started");

	}

	public void stop() {
		pipe.close();
		device.close();
		is_running = false;

	}

	public boolean isRunning() {
		return is_running;
	}

	private class ColorCameraOakDCallback extends Callback {

		Planar<GrayU8> output;
		DataOutputQueue queue;

		public ColorCameraOakDCallback(DataOutputQueue queue, Planar<GrayU8> output) {
			super();
			this.output = output;
			this.queue  = queue;
		}

		@Override
		public void call() {
			try (PointerScope scope = new PointerScope()) {	
				ImgFrame imgFrame = queue.getImgFrame();
				bufferRgbToMsU8(imgFrame.getData().asBuffer());
				imgFrame.close();
			}
			
			if(listeners.size()>0 ) {
				for(IDepthCallback listener : listeners)
					listener.process(rgb, null, System.currentTimeMillis(), System.currentTimeMillis());
			}
		}
		
		public void bufferRgbToMsU8(ByteBuffer input ) {

			byte[] b0 = output.getBand(0).data;
			byte[] b1 = output.getBand(1).data;
			byte[] b2 = output.getBand(2).data;

			for(int  y = 0; y < output.height; y++ ) {
				int indexOut = output.startIndex + y*output.stride;
				for( int x = 0; x < output.width; x++ , indexOut++ ) {
					if(input.remaining()>3) {
						b0[indexOut] = input.get();
						b1[indexOut] = input.get();
						b2[indexOut] = input.get();
					}
				}
			}

		}
	}

}
