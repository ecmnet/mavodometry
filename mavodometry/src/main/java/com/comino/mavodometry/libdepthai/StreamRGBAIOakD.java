package com.comino.mavodometry.libdepthai;

import java.awt.image.BufferedImage;
import java.nio.ByteBuffer;
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
import org.bytedeco.depthai.presets.*;
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

import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;


public class StreamRGBAIOakD {

	private final static boolean  USE_USB2 = true;


	private static StreamRGBAIOakD instance;

	private final    List<IDepthCallback> listeners;
	private final    Planar<GrayU8>       rgb;

	private long             rgb_tms  = 0;
	private int            frameCount = 0;

	private boolean is_running = false;

	private Device device;

	private CombineOAKDCallback callback;
	private final BlockingQueue<ImgFrame> transfer = new ArrayBlockingQueue<ImgFrame>(30);

	public static StreamRGBAIOakD getInstance(int width, int height) throws Exception {

		if(instance==null) {
			instance = new StreamRGBAIOakD(width,height);
		}
		return instance;
	}

	public StreamRGBAIOakD(int width, int height) {

		this.listeners = new ArrayList<IDepthCallback>();
		this.rgb       = new Planar<GrayU8>(GrayU8.class,width,height,3);

	}

	public StreamRGBAIOakD registerCallback(IDepthCallback listener) {
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
						ImgFrame frame = transfer.take();
						if(frame != null && !frame.isNull()) {
							bufferRgbToMsU8(frame.getData().asByteBuffer(),rgb);
							frameCount = frame.getSequenceNum();
						}
					}

					if(listeners.size()>0 ) {
						for(IDepthCallback listener : listeners)
							listener.process(rgb, null, rgb_tms, rgb_tms);
					}

				} catch (InterruptedException e) { }

			}

		}));

		callback = new CombineOAKDCallback();
		if(!is_running)
			throw new Exception("No OAKD camera found");
		callback.deallocate(false);

		System.out.println("OAK-D Lite RGB pipeline started.");
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
					System.out.println("- "+device.getUsbSpeed());
				}

				queue = device.getOutputQueue("preview", 4, true);
				queue.deallocate(false);
				queue.addCallback(this);


		}

		public void call() {

			ImgFrame imgFrame = queue.getImgFrame();
			if(imgFrame!=null && !imgFrame.isNull() ) {
				try {
					rgb_tms = System.currentTimeMillis();
					transfer.put(imgFrame);
				} catch (InterruptedException e) {	}
			} 
		}	
	}
	
	
	private void bufferRgbToMsU8(ByteBuffer input,Planar<GrayU8> output) {

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


	public static void main(String[] args) throws InterruptedException  {

		System.out.println("OAKD-Test");	

		BufferedImage im = new BufferedImage(640, 480, BufferedImage.TYPE_3BYTE_BGR);

		StreamRGBAIOakD oakd;
		try {
			oakd = StreamRGBAIOakD.getInstance(im.getWidth(), im.getHeight());
			oakd.registerCallback((image,np,t1,t2) -> {
				System.out.println(oakd.getFrameCount()+" "+oakd.getRGBTms()+" ");
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


