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
import org.bytedeco.depthai.RawStereoDepthConfig;
import org.bytedeco.depthai.RawStereoDepthConfig.AlgorithmControl.DepthUnit;
import org.bytedeco.depthai.RawStereoDepthConfig.PostProcessing;
import org.bytedeco.depthai.RawStereoDepthConfig.PostProcessing.DecimationFilter;
import org.bytedeco.depthai.RawStereoDepthConfig.PostProcessing.SpatialFilter;
import org.bytedeco.depthai.RawStereoDepthConfig.PostProcessing.SpeckleFilter;
import org.bytedeco.depthai.RawStereoDepthConfig.PostProcessing.TemporalFilter;
import org.bytedeco.depthai.StereoDepth;
import org.bytedeco.depthai.StereoDepth.PresetMode;
import org.bytedeco.depthai.StereoDepthConfig;
import org.bytedeco.depthai.XLinkOut;
import org.bytedeco.depthai.global.depthai.CameraBoardSocket;
import org.bytedeco.depthai.global.depthai.MedianFilter;
import org.bytedeco.depthai.global.depthai.UsbSpeed;
import org.bytedeco.depthai.presets.depthai.Callback;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.PointerScope;

import com.comino.mavodometry.callback.IDepthCallback;
import com.comino.mavodometry.concurrency.OdometryPool;

import boofcv.struct.calib.CameraPinholeBrown;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;


public class StreamDepthAIOakD implements IStreamDepthAIOakD {

	private final static UsbSpeed  USE_USB2         = UsbSpeed.HIGH;
	//	private final static UsbSpeed  USE_USB2         = UsbSpeed.SUPER_PLUS;

	// TODO: Only USB2 possible due to interference with GPS. RGB stream rate therefore very low
	// Try to encode RGB on device and decode it on HOST, do overlay eventually on GC side

	private final static int      DEPTH_CONFIDENCE = 120;
	//	private final static int      DEPTH_SIGMA      = 150;

	private final static float    FPS_COLOR        = 15;
	private final static float    FPS_MONO         = 15;

	private static final int      RGB_FRAME   	   = 0;
	private static final int      MONO_FRAME       = 1;
	private static final int      DEPTH_FRAME      = 2;

	private static IStreamDepthAIOakD instance;

	private final    List<IDepthCallback> listeners;
	private final    Planar<GrayU8>       rgb;
	private final    GrayU16              depth;

	private final CameraPinholeBrown  intrinsics;


	private long             rgb_tms  = 0;
	private long           depth_tms  = 0;
	private long           frameCount = 0;

	private boolean is_running = false;

	private Device device;

	private CombineOAKDCallback callback;
	private final BlockingQueue<ImgFrame> transfer_rgb   = new ArrayBlockingQueue<ImgFrame>(5);
	private final BlockingQueue<ImgFrame> transfer_mono  = new ArrayBlockingQueue<ImgFrame>(5);
	private final BlockingQueue<ImgFrame> transfer_depth = new ArrayBlockingQueue<ImgFrame>(5);

	private int width;
	private int height;

	private boolean rgb_mode = false;

	public static IStreamDepthAIOakD getInstance(int width, int height) throws Exception {

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

	@Override
	public IStreamDepthAIOakD registerCallback(IDepthCallback listener) {
		listeners.add(listener);
		return this;
	}


	@Override
	public void start() throws Exception {

		is_running = true;

		OdometryPool.submit(
				new Thread(()-> {
					while(is_running) {

						try {
							try (PointerScope scope = new PointerScope()) {	
								if(!transfer_rgb.isEmpty())	{
									ImgFrame frame = transfer_rgb.take();
									if(frame != null && !frame.isNull()) {
										rgb_tms = System.currentTimeMillis();
										bufferRgbToMsU8(frame.getData().asByteBuffer(),rgb);
										frameCount = frame.getSequenceNum();

									}
								} 
							}

							try (PointerScope scope = new PointerScope()) {	
								if(!transfer_mono.isEmpty())	{
									ImgFrame frame = transfer_mono.take();
									if(frame != null && !frame.isNull()) {
										rgb_tms = System.currentTimeMillis();
										bufferMonoToMsU8(frame.getData().asByteBuffer(),rgb);
										frameCount = frame.getSequenceNum();
									}
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
									listener.process(rgb, depth, null, rgb_tms, depth_tms);
							}

							Thread.sleep(3);


						} catch (InterruptedException e) { }

					}

				}));

		Thread.sleep(50);

		callback = new CombineOAKDCallback();
		if(!is_running)
			throw new Exception("No OAKD camera found");

	}
	
	@Override
	public boolean isInference() {
		return false;
	}

	@Override
	public void stop() {
		is_running = false;
		if(device != null)
			device.close();
	}

	@Override
	public void setRGBMode(boolean rgb) {
		this.rgb_mode  = rgb;
	}

	@Override
	public long getFrameCount() {
		return frameCount;
	}

	@Override
	public long getRGBTms() {
		return rgb_tms;
	}

	@Override
	public long getDepthTms() {
		return depth_tms;
	}

	@Override
	public boolean isRunning() {
		return is_running;
	}

	@Override
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

			ColorCamera colorCam = p.createColorCamera();
			colorCam.deallocate(false);
			colorCam.setPreviewSize(width, height);
			colorCam.setFps(FPS_COLOR);
			//			colorCam.setBoardSocket(CameraBoardSocket.RGB);
			colorCam.setResolution(ColorCameraProperties.SensorResolution.THE_1080_P);
			colorCam.setColorOrder(ColorOrder.RGB);
			colorCam.setInterleaved(false);

			StereoDepth depth = p.createStereoDepth();

			depth.setDefaultProfilePreset(PresetMode.HIGH_ACCURACY);
			StereoDepthConfig config = depth.initialConfig();
			
			config.setMedianFilter(MedianFilter.KERNEL_7x7);
			config.setConfidenceThreshold(DEPTH_CONFIDENCE);
			config.setDepthUnit(DepthUnit.MILLIMETER);
			
			// Postprocessing lags a bit 
			
//			RawStereoDepthConfig sconfig = config.get();		
//			PostProcessing    pp = sconfig.postProcessing();
//			SpeckleFilter    spf = new SpeckleFilter(); spf.enable(true); spf.speckleRange(80); pp.speckleFilter(spf);
//			TemporalFilter   tmf = new TemporalFilter(); tmf.enable(true);  pp.temporalFilter(tmf); 
//			SpatialFilter    saf = new SpatialFilter(); saf.enable(true); saf.holeFillingRadius((byte)3); saf.numIterations(1); pp.spatialFilter(saf);
//			DecimationFilter def = new DecimationFilter(); def.decimationFactor(1); pp.decimationFilter(def);
//			config.set(sconfig);

			depth.setLeftRightCheck(true);
			depth.setExtendedDisparity(false);
			depth.setSubpixel(true);
			depth.setRectification(true);
			depth.useHomographyRectification(false);
			depth.setNumFramesPool(1);
			depth.setDepthAlign(colorCam.getBoardSocket());


			MonoCamera monoLeft = p.createMonoCamera();
			monoLeft.setResolution(MonoCameraProperties.SensorResolution.THE_480_P);
			monoLeft.setBoardSocket(CameraBoardSocket.LEFT);
			monoLeft.setFps(FPS_MONO);

			MonoCamera monoRight = p.createMonoCamera();
			monoRight.setResolution(MonoCameraProperties.SensorResolution.THE_480_P);
			monoRight.setBoardSocket(CameraBoardSocket.RIGHT);
			monoRight.setFps(FPS_MONO);

			monoLeft.out().link(depth.left());
			monoRight.out().link(depth.right());

			if(rgb_mode)
				colorCam.preview().link(xlinkOut.input());
			else
				monoLeft.out().link(xlinkOut.input());

			depth.depth().link(xlinkOut.input());

			try {

				device = new Device(p,USE_USB2);
				device.deallocate(false);

				is_running = device.isPipelineRunning();
			} catch(RuntimeException e) {
				is_running = false;
				return;
			}

			queue = device.getOutputQueue("preview", 10, true);
			queue.deallocate(false);
			int id = queue.addCallback(this);

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

			System.out.println("OAK-D Lite depth pipeline "+id+" started: "+device.isPipelineRunning());

		}

		public void call() {

			try {

				ImgFrame imgFrame = queue.tryGetImgFrame();
				if(imgFrame!=null && !imgFrame.isNull() ) {
					switch(imgFrame.getInstanceNum()) {
					case RGB_FRAME:
						rgb_tms = System.currentTimeMillis();
						if(rgb_mode)
							transfer_rgb.offer(imgFrame);
						else
							imgFrame.close();
						break;
					case MONO_FRAME:
						rgb_tms = System.currentTimeMillis();
						if(!rgb_mode)
							transfer_mono.offer(imgFrame);
						else
							imgFrame.close();  // Shouldn't this be called always??
						break;
					case DEPTH_FRAME:
						depth_tms = System.currentTimeMillis();
						transfer_depth.offer(imgFrame);
						break;
					}

				} 
			} catch (Exception e) {	}
		}
	}

	private void bufferDepthToU16(ShortBuffer input , GrayU16 output ) {
		input.get(output.data);
	}

	private void bufferMonoToMsU8(ByteBuffer input,Planar<GrayU8> output) {	
		input.get(output.getBand(0).data);
		output.getBand(1).data = output.getBand(0).data;
		output.getBand(2).data = output.getBand(0).data;
	}


	private void bufferRgbToMsU8(ByteBuffer input,Planar<GrayU8> output) {	
		input.get(output.getBand(0).data);
		input.get(output.getBand(1).data);
		input.get(output.getBand(2).data);	
	}


	public static void main(String[] args) throws InterruptedException  {

		System.out.println("OAKD-Test");	

		BufferedImage im = new BufferedImage(640, 480, BufferedImage.TYPE_3BYTE_BGR);

		IStreamDepthAIOakD oakd;
		try {
			oakd = StreamDepthAIOakD.getInstance(im.getWidth(), im.getHeight());
			oakd.registerCallback((image,np,d,t1,t2) -> {
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


