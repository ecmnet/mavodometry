package com.comino.mavodometry.libdepthai;

import java.awt.image.BufferedImage;

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
import org.bytedeco.depthai.Pipeline;
import org.bytedeco.depthai.XLinkOut;
import org.bytedeco.depthai.presets.depthai.Callback;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.PointerScope;
import org.bytedeco.opencv.opencv_core.Mat;

import com.comino.mavodometry.callback.IDepthCallback;
import com.comino.mavodometry.concurrency.OdometryPool;

import boofcv.io.image.ConvertBufferedImage;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;


public class StreamDepthAIOakD {

	private final static boolean  USE_USB2 = false;


	private static StreamDepthAIOakD instance;

	private final    List<IDepthCallback> listeners;
	private final    Planar<GrayU8>       rgb;


	private long                 tms  = 0;

	private boolean is_running = false;

	private Device device;

	private CombineOAKDCallback callback;
	private final BlockingQueue<ImgFrame> transfer = new ArrayBlockingQueue<ImgFrame>(30);

	public static StreamDepthAIOakD getInstance(int width, int height) throws Exception {

		if(instance==null) {
			instance = new StreamDepthAIOakD(width,height);
		}
		return instance;
	}

	public StreamDepthAIOakD(int width, int height) {

		this.listeners = new ArrayList<IDepthCallback>();
		this.rgb       = new Planar<GrayU8>(GrayU8.class,width,height,3);

	}

	public StreamDepthAIOakD registerCallback(IDepthCallback listener) {
		listeners.add(listener);
		return this;
	}


	public void start() {

		is_running = true;

		final byte[] image_buffer = new byte[rgb.width*rgb.height*rgb.getNumBands()];

		Thread s = new Thread(()-> {
			while(is_running) {

				try {
					try (PointerScope scope = new PointerScope()) {	
						ImgFrame frame = transfer.take();
						if(frame != null && !frame.isNull()) {
							frame.getData().get(image_buffer);
							tms = System.currentTimeMillis();
						}
					}
					bufferRgbToMsU8(image_buffer,rgb);

					if(listeners.size()>0 ) {
						for(IDepthCallback listener : listeners)
							listener.process(rgb, null, tms, tms);
					}

				} catch (InterruptedException e) { }

			}

		});
		s.start();

		callback = new CombineOAKDCallback();

		System.out.println("OAK-D Lite pipeline started.");
	}

	public void stop() {
		is_running = false;
		if(device != null)
			device.close();

	}

	public boolean isRunning() {
		return is_running;
	}

	private class CombineOAKDCallback extends Callback {

		DataOutputQueue queue;

		public CombineOAKDCallback() {

			final Pipeline p = new Pipeline();
			p.deallocate(false);

			XLinkOut xlinkOut = p.createXLinkOut();
			xlinkOut.deallocate(false);
			xlinkOut.setStreamName("preview");

			ColorCamera colorCam = p.createColorCamera();
			colorCam.deallocate(false);
			colorCam.setPreviewSize(rgb.width, rgb.height);
			colorCam.setResolution(ColorCameraProperties.SensorResolution.THE_1080_P);
			colorCam.setColorOrder(ColorOrder.RGB);
			colorCam.setInterleaved(true);
			colorCam.preview().link(xlinkOut.input());

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
				IntPointer cameras = device.getConnectedCameras();
				for (int i = 0; i < cameras.limit(); i++) {
					System.out.print(cameras.get(i) + " ");
				}
				System.out.println();
			}

			queue = device.getOutputQueue("preview", 4, true);
			queue.deallocate(false);
			queue.addCallback(this);
			is_running = true;
		}

		public void call() {

			ImgFrame imgFrame = queue.getImgFrame();
			if(imgFrame!=null && !imgFrame.isNull() ) {
				try {
					transfer.put(imgFrame);
				} catch (InterruptedException e) {	}
			} 
		}	
	}


	private void bufferRgbToMsU8(byte[] input,Planar<GrayU8> output) {

		byte[] b0 = output.getBand(0).data;
		byte[] b1 = output.getBand(1).data;
		byte[] b2 = output.getBand(2).data;
		int inIndex = 0;
		for(int  y = 0; y < output.height; y++ ) {
			int indexOut = output.startIndex + y*output.stride;
			for( int x = 0; x < output.width; x++ , indexOut++ ) {
				b0[indexOut] = input[inIndex++];
				b1[indexOut] = input[inIndex++];
				b2[indexOut] = input[inIndex++];
			}
		}
	}

	static int i=0;
	public static void main(String[] args) throws InterruptedException  {

		System.out.println("OAKD-Test");	

		BufferedImage im = new BufferedImage(640, 480, BufferedImage.TYPE_3BYTE_BGR);

		StreamDepthAIOakD oakd;
		try {
			oakd = StreamDepthAIOakD.getInstance(im.getWidth(), im.getHeight());
			oakd.registerCallback((image,np,t1,t2) -> {
				System.out.println((i++)+" "+System.currentTimeMillis()+" ");
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


