package com.comino.mavodometry.libdepthai;

/****************************************************************************
 *
 *   Copyright (c) 2023 Eike Mansfeld ecm@gmx.de. All rights reserved.
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
import org.bytedeco.depthai.ImageManip;
import org.bytedeco.depthai.ImgDetections;
import org.bytedeco.depthai.ImgFrame;
import org.bytedeco.depthai.IntVector;
import org.bytedeco.depthai.MonoCamera;
import org.bytedeco.depthai.MonoCameraProperties;
import org.bytedeco.depthai.Path;
import org.bytedeco.depthai.Pipeline;
import org.bytedeco.depthai.RawStereoDepthConfig;
import org.bytedeco.depthai.RawStereoDepthConfig.AlgorithmControl.DepthAlign;
import org.bytedeco.depthai.RawStereoDepthConfig.AlgorithmControl.DepthUnit;
import org.bytedeco.depthai.StereoDepth;
import org.bytedeco.depthai.StereoDepth.PresetMode;
import org.bytedeco.depthai.StereoDepthConfig;
import org.bytedeco.depthai.StringIntVectorMap;
import org.bytedeco.depthai.XLinkOut;
import org.bytedeco.depthai.YoloDetectionNetwork;
import org.bytedeco.depthai.global.depthai.CameraBoardSocket;
import org.bytedeco.depthai.global.depthai.MedianFilter;
import org.bytedeco.depthai.global.depthai.UsbSpeed;
import org.bytedeco.depthai.presets.depthai.Callback;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.PointerScope;

import com.comino.mavodometry.callback.IDepthCallback;
import com.comino.mavodometry.estimators.inference.YoloDetection;

import boofcv.struct.calib.CameraPinholeBrown;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;


public class StreamYoloDepthAIOakD implements IStreamDepthAIOakD {

	private final static UsbSpeed  USE_USB2         = UsbSpeed.HIGH;
	private final static int       FPS              = 20;
	private final static int       FPS_DEPTH        = 5;
	private final static int       DEPTH_CONFIDENCE = 100;

	private static final int      RGB_FRAME   = 0;
	private static final int      MONO_FRAME  = 1;
	private static final int      DEPTH_FRAME = 2;
	
	private static final int      DEPTH_WIDTH   = 640;
	private static final int      DEPTH_HEIGHT  = 480;


	private static StreamYoloDepthAIOakD instance;

	private final List<IDepthCallback<List<YoloDetection>>>    listeners;
	private final Planar<GrayU8>             rgb;
	private final GrayU16                    depth;
	private final CameraPinholeBrown         intrinsics;


	private long             rgb_tms  = 0;
	private long           depth_tms  = 0;
	private long           frameCount = 0;

	private boolean is_running = false;

	private Device device;

	private final BlockingQueue<ImgDetections> transfer_det = new ArrayBlockingQueue<ImgDetections>(5);


	private int width;
	private int height;

	private int width_nn;
	private int height_nn;

	private boolean use_nn = false;
	private String nn_path;

	private DataOutputQueue rgb_queue; 
	private DataOutputQueue depth_queue; 
	private DataOutputQueue nn_queue; 

	private boolean rgb_mode = true;

	private OAKDImageCallback imageCallback;
	private OAKDDepthCallback depthCallback;
	private OAKDNNCallback    nnCallback;

	public static IStreamDepthAIOakD getInstance(int width, int height) throws Exception {

		if(instance==null) {
			instance = new StreamYoloDepthAIOakD(width,height, null, 0, 0);
		}
		return instance;
	}

	public static StreamYoloDepthAIOakD getInstance(int width, int height, String nn_path, int width_nn, int height_nn) throws Exception {

		if(instance==null) {
			instance = new StreamYoloDepthAIOakD(width,height, nn_path, width_nn, height_nn);
		}
		return instance;
	}

	public StreamYoloDepthAIOakD(int width, int height, String nn_path, int width_nn, int height_nn) {

		this.listeners  = new ArrayList<IDepthCallback<List<YoloDetection>>>();
		this.rgb        = new Planar<GrayU8>(GrayU8.class,width,height,3);
		this.depth      = new GrayU16(DEPTH_WIDTH,DEPTH_HEIGHT);
		this.intrinsics = new CameraPinholeBrown();
		this.width      = width;
		this.height     = height;

		if(nn_path!=null) {
			this.use_nn = true;
			this.nn_path    = nn_path;
			this.width_nn   = width_nn;
			this.height_nn  = height_nn;
		}

	}

	@SuppressWarnings("unchecked")
	public StreamYoloDepthAIOakD registerCallback(IDepthCallback<?> listener) {
		listeners.add((IDepthCallback<List<YoloDetection>>) listener);
		return this;
	}


	public void start() throws Exception {

		ArrayList<YoloDetection> detections = new ArrayList<YoloDetection>();

		is_running = true;
		
	
		 buildPipeline();
		 
		 if(!is_running) 
			throw new Exception("No OAKD camera found");
		
		rgb_queue = device.getOutputQueue("preview", 1, false);
		rgb_queue.deallocate(false);
		imageCallback = new OAKDImageCallback();
		rgb_queue.addCallback(imageCallback);
		
		depth_queue = device.getOutputQueue("depth", 1, false);
		depth_queue.deallocate(false);
		depthCallback = new OAKDDepthCallback();
		depth_queue.addCallback(depthCallback);

		if(use_nn) {
			nn_queue = device.getOutputQueue("nn", 5, false);
			nn_queue.deallocate(false);
			nnCallback = new OAKDNNCallback();
			nn_queue.addCallback(nnCallback);
		}

	}

	public void stop() {
		is_running = false;
		if(device != null)
			device.close();
	}

	public void setRGBMode(boolean rgb) {
		this.rgb_mode  = rgb;
	}

	public long getFrameCount() {
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

	@Override
	public boolean isInference() {
		return true;
	}

	public CameraPinholeBrown getIntrinsics() {
		return intrinsics;
	}

	private void buildPipeline() {
		
		final Pipeline p = new Pipeline();
		p.deallocate(false);


		XLinkOut xlinkOut = p.createXLinkOut();
		xlinkOut.deallocate(false);
		xlinkOut.setStreamName("preview");
		
		XLinkOut xlinkDepth = p.createXLinkOut();
		xlinkDepth.deallocate(false);
		xlinkDepth.setStreamName("depth");
		
		System.out.println("...building RGB pipeline at "+FPS+"Hz");

		ColorCamera colorCam = p.createColorCamera();
		colorCam.deallocate(false);
		colorCam.setPreviewSize(width, height);
		//			colorCam.setBoardSocket(CameraBoardSocket.RGB);
		colorCam.setResolution(ColorCameraProperties.SensorResolution.THE_1080_P);

		colorCam.setColorOrder(ColorOrder.BGR); 	// BGR due to NN
		colorCam.setFps(FPS);
		colorCam.setInterleaved(false);
		colorCam.setPreviewNumFramesPool(1);

		StereoDepth depth = p.createStereoDepth();

		depth.setDefaultProfilePreset(PresetMode.HIGH_ACCURACY);
		StereoDepthConfig config = depth.initialConfig();

		config.setMedianFilter(MedianFilter.KERNEL_7x7);
		config.setConfidenceThreshold(DEPTH_CONFIDENCE);
		config.setDepthUnit(DepthUnit.MILLIMETER);
		
		// TODO: Compare effect and test parameters
		// Parameters set as in RERUN
		/*
		RawStereoDepthConfig stereo_config = depth.initialConfig().get();
		stereo_config.postProcessing().speckleFilter().enable(false);
		stereo_config.postProcessing().speckleFilter().speckleRange(50);
		stereo_config.postProcessing().temporalFilter().enable(true);
		stereo_config.postProcessing().spatialFilter().enable(true);
		stereo_config.postProcessing().spatialFilter().holeFillingRadius((byte)2);
		stereo_config.postProcessing().spatialFilter().numIterations(1);
		stereo_config.postProcessing().thresholdFilter().minRange(400);
		stereo_config.postProcessing().thresholdFilter().maxRange(20000);
		stereo_config.postProcessing().decimationFilter().decimationFactor(1);
		depth.initialConfig().set(stereo_config);
		*/

		depth.setLeftRightCheck(true);
		depth.setExtendedDisparity(false);
		depth.setSubpixel(true);
		depth.setRectification(true);
		depth.useHomographyRectification(false);
		depth.setNumFramesPool(3);
		depth.setDepthAlign(DepthAlign.CENTER);
		depth.setOutputSize(DEPTH_WIDTH, DEPTH_HEIGHT);

		System.out.println("...building Stereo pipeline at "+FPS_DEPTH+"Hz");
		MonoCamera monoLeft = p.createMonoCamera();
		monoLeft.setResolution(MonoCameraProperties.SensorResolution.THE_480_P);
		monoLeft.setBoardSocket(CameraBoardSocket.LEFT);
		monoLeft.setFps(FPS_DEPTH);

		MonoCamera monoRight = p.createMonoCamera();
		monoRight.setResolution(MonoCameraProperties.SensorResolution.THE_480_P);
		monoRight.setBoardSocket(CameraBoardSocket.RIGHT);
		monoRight.setFps(FPS_DEPTH);

		monoLeft.out().link(depth.left());
		monoRight.out().link(depth.right());

		if(use_nn) {
			System.out.println("...building NN pipeline");
			XLinkOut nnOut = p.createXLinkOut();
			nnOut.setStreamName("nn");
			ImageManip manip = p.createImageManip();
			
			// TODO map 640 pixels to 416 to ensure the total width is used. Keep ratio!
			// Picture should be resizd to 416*312 

//			float dx = (width - width_nn)/(2f*width);
//			float dy = (height - height_nn)/(2f*height);
//
//			manip.initialConfig().setCropRect(dx,dy,1-dx,1-dy);
    		manip.initialConfig().setResizeThumbnail(width_nn, height_nn);
			manip.inputConfig().setBlocking(false);


			YoloDetectionNetwork detectionNetwork = p.createYoloDetectionNetwork();

			detectionNetwork.setConfidenceThreshold(0.3f);
			detectionNetwork.setNumClasses(80);

			detectionNetwork.setCoordinateSize(4);

			StringIntVectorMap mask = new StringIntVectorMap();

			//
//			detectionNetwork.setAnchors(new float[]{10,14, 23,27, 37,58, 81,82, 135,169, 344,319});
//			mask.put(new BytePointer("side26"), new IntVector(0,1,2));
//			mask.put(new BytePointer("side13"), new IntVector(3,4,5));
			
			detectionNetwork.setAnchors(new float[]{10,13, 16,30, 33,23, 30,61, 62,45, 59,119, 116,90, 156,198, 373,326});
			mask.put(new BytePointer("side52"), new IntVector(0,1,2));
			mask.put(new BytePointer("side26"), new IntVector(3,4,5));
			mask.put(new BytePointer("side13"), new IntVector(6,7,8));

			detectionNetwork.setAnchorMasks(mask);
			detectionNetwork.setIouThreshold(0.3f);
		
			detectionNetwork.setBlobPath(new Path(nn_path));
			detectionNetwork.setNumInferenceThreads(2);
			detectionNetwork.out().link(nnOut.input());
			manip.out().link(detectionNetwork.input());
			colorCam.preview().link(manip.inputImage());	
		}


		if(rgb_mode)
			colorCam.preview().link(xlinkOut.input());
		else
			monoLeft.out().link(xlinkOut.input());
	    depth.depth().link(xlinkDepth.input());

		try {

			device = new Device(p,USE_USB2);
			device.deallocate(false);
			is_running = device.isPipelineRunning();
		} catch(RuntimeException e) {
			System.err.println(e.getMessage());
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

		System.out.println("OAK-D Lite depth pipeline started: "+device.isPipelineRunning());
	}

	private class OAKDImageCallback extends Callback { 
		
		ArrayList<YoloDetection> detections = new ArrayList<YoloDetection>();

		public void call() {

			try {

				ImgFrame imgFrame = rgb_queue.tryGetImgFrame();
				if(imgFrame!=null && !imgFrame.isNull() ) {
					switch(imgFrame.getInstanceNum()) {
					case RGB_FRAME:
						rgb_tms = System.currentTimeMillis();
						if(rgb_mode)
							bufferRgbToMsU8(imgFrame.getData().asByteBuffer(),rgb);
						
							imgFrame.close();
						break;
					case MONO_FRAME:
						rgb_tms = System.currentTimeMillis();
						if(!rgb_mode)
							bufferMonoToMsU8(imgFrame.getData().asByteBuffer(),rgb);
						
							imgFrame.close();  // Shouldn't this be called always??
						break;
					}
					
					if(listeners.size()>0 ) {
						for(IDepthCallback<List<YoloDetection>> listener : listeners)
							listener.process(rgb, depth, detections, rgb_tms, depth_tms);
					}

				} 
				
			} catch (Exception e) {	 e.printStackTrace();}
		}
	}
	
	private class OAKDDepthCallback extends Callback { 

		public void call() {

			try {

				ImgFrame imgFrame = depth_queue.tryGetImgFrame();
				if(imgFrame!=null && !imgFrame.isNull() ) {
					switch(imgFrame.getInstanceNum()) {
					case DEPTH_FRAME:
						depth_tms = System.currentTimeMillis();
						bufferDepthToU16(imgFrame.getData().asByteBuffer().asShortBuffer(),depth);
						imgFrame.close();
						break;
					}
				} 
				
			} catch (Exception e) {	 e.printStackTrace();}
		}
	}

	private class OAKDNNCallback extends Callback { 

		public void call() {
			try {
				ImgDetections imgDet = nn_queue.tryGetImgDetections();
				if(imgDet!=null && !imgDet.isNull()) {
					if(transfer_det.isEmpty())
						transfer_det.put(imgDet);
				}
			} catch (InterruptedException e) {	}
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
		// BGR -> RGB
		input.get(output.getBand(2).data);
		input.get(output.getBand(1).data);
		input.get(output.getBand(0).data);	
	}


	static long tms;
	public static void main(String[] args) throws InterruptedException  {

		System.out.println("OAKD-Test");	

		BufferedImage im = new BufferedImage(640, 480, BufferedImage.TYPE_3BYTE_BGR);

		IStreamDepthAIOakD oakd;
		try {
			//		oakd = StreamYoloDepthAIOakD.getInstance(im.getWidth(), im.getHeight(),"models/yolo-v3-tiny-tf_openvino_2021.4_6shave.blob", 416,416);
			oakd = StreamYoloDepthAIOakD.getInstance(im.getWidth(), im.getHeight(),null, 640,480);
			oakd.registerCallback((image,np,d,t1,t2) -> {
				System.out.println(oakd.getFrameCount()+" "+oakd.getRGBTms()+" "+oakd.getDepthTms()+" "+(1000/(System.currentTimeMillis()-tms)));
				tms = System.currentTimeMillis();
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


