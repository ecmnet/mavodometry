package com.comino.mavodometry.libdepthai;

/****************************************************************************
*
*   Copyright (c) 2022 Eike Mansfeld ecm@gmx.de. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/

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
	private long           frameCount = 0;

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
	
	public long getFrameCount() {
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
				colorCam.setInterleaved(false);
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
		input.get(output.getBand(0).data);
		input.get(output.getBand(1).data);
		input.get(output.getBand(2).data);	
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


