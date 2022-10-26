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
import java.io.File;
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
import org.bytedeco.depthai.ImgDetection;
import org.bytedeco.depthai.ImgDetections;
import org.bytedeco.depthai.ImgFrame;
import org.bytedeco.depthai.IntVector;
import org.bytedeco.depthai.MonoCamera;
import org.bytedeco.depthai.MonoCameraProperties;
import org.bytedeco.depthai.Path;
import org.bytedeco.depthai.Pipeline;
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
import com.comino.mavodometry.concurrency.OdometryPool;
import com.comino.mavodometry.estimators.inference.YoloDetection;
import com.comino.mavutils.file.MSPFileUtils;

import boofcv.struct.calib.CameraPinholeBrown;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;


public class StreamYoloDepthAIOakD implements IStreamDepthAIOakD {

	private final static UsbSpeed  USE_USB2        = UsbSpeed.HIGH;
	private final static int      DEPTH_CONFIDENCE = 100;

	private static final int      RGB_FRAME   = 0;
	private static final int      MONO_FRAME  = 1;
	private static final int      DEPTH_FRAME = 2;


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

	private final BlockingQueue<ImgFrame> transfer_rgb      = new ArrayBlockingQueue<ImgFrame>(3);
	private final BlockingQueue<ImgFrame> transfer_mono     = new ArrayBlockingQueue<ImgFrame>(3);
	private final BlockingQueue<ImgFrame> transfer_depth    = new ArrayBlockingQueue<ImgFrame>(3);
	private final BlockingQueue<ImgDetections> transfer_det = new ArrayBlockingQueue<ImgDetections>(3);


	private int width;
	private int height;

	private int width_nn;
	private int height_nn;

	private boolean use_nn = false;
	private String nn_path;

	private DataOutputQueue queue; 
	private DataOutputQueue nn; 

	private boolean rgb_mode = true;

	private OAKDImageCallback imageCallback;
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
		this.depth      = new GrayU16(width,height);
		this.intrinsics = new CameraPinholeBrown();
		this.width      = width;
		this.height     = height;

		if(nn_path!=null && MSPFileUtils.exists(nn_path)) {
			this.use_nn = true;
			this.nn_path    = nn_path;
			this.width_nn   = width_nn;
			this.height_nn  = height_nn;
			System.out.println("NN enabled using network model:"+nn_path);
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
								if(!transfer_det.isEmpty()) {
									detections.clear();
									ImgDetections detects = transfer_det.take();
									if(detects!=null && !detects.isNull()) {

										ImgDetection det = detects.detections();
										if(det!=null && !det.isNull()) {
											for(int i = 0; i < det.capacity();i++) {
												ImgDetection o = det.getPointer(i);
												if(o.confidence()> 0.7) 
													detections.add(new YoloDetection(o.label(),o.confidence(),
															o.xmin(),o.xmax(),o.ymin(),o.ymax()));
											}
										}
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
								for(IDepthCallback<List<YoloDetection>> listener : listeners)
									listener.process(rgb, depth, detections, rgb_tms, depth_tms);
							}

							Thread.sleep(10);

						} catch (InterruptedException e) { }

					}

				}));

		buildPipeline();

		queue = device.getOutputQueue("preview", 3, true);
		queue.deallocate(false);
		imageCallback = new OAKDImageCallback();
		queue.addCallback(imageCallback);

		if(use_nn) {
			nn = device.getOutputQueue("nn", 3, false);
			nn.deallocate(false);
			nnCallback = new OAKDNNCallback();
			nn.addCallback(nnCallback);
		}

		if(!is_running)
			throw new Exception("No OAKD camera found");

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

		ColorCamera colorCam = p.createColorCamera();
		colorCam.deallocate(false);
		colorCam.setPreviewSize(width, height);
		//			colorCam.setBoardSocket(CameraBoardSocket.RGB);
		colorCam.setResolution(ColorCameraProperties.SensorResolution.THE_1080_P);

		colorCam.setColorOrder(ColorOrder.BGR); 	// BGR due to NN
		colorCam.setFps(15);
		colorCam.setInterleaved(false);

		StereoDepth depth = p.createStereoDepth();

		depth.setDefaultProfilePreset(PresetMode.HIGH_ACCURACY);
		StereoDepthConfig config = depth.initialConfig();

		config.setMedianFilter(MedianFilter.KERNEL_7x7);
		config.setConfidenceThreshold(DEPTH_CONFIDENCE);
		config.setDepthUnit(DepthUnit.MILLIMETER);

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
		monoLeft.setFps(15);

		MonoCamera monoRight = p.createMonoCamera();
		monoRight.setResolution(MonoCameraProperties.SensorResolution.THE_480_P);
		monoRight.setBoardSocket(CameraBoardSocket.RIGHT);
		monoRight.setFps(15);

		monoLeft.out().link(depth.left());
		monoRight.out().link(depth.right());

		if(use_nn) {
			XLinkOut nnOut = p.createXLinkOut();
			nnOut.setStreamName("nn");
			ImageManip manip = p.createImageManip();
			
			// TODO map 640 pixels to 416 to ensure the total width is used. Keep ratio!

			float dx = (width - width_nn)/(2f*width);
			float dy = (height - height_nn)/(2f*height);

			manip.initialConfig().setCropRect(dx,dy,1-dx,1-dy);
			manip.inputConfig().setBlocking(true);
			

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
			detectionNetwork.input().setBlocking(true);
			detectionNetwork.out().link(nnOut.input());
			manip.out().link(detectionNetwork.input());
			colorCam.preview().link(manip.inputImage());	
		}


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
			//e.printStackTrace();
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

	private class OAKDNNCallback extends Callback { 

		public void call() {
			try {
				ImgDetections imgDet = nn.tryGetImgDetections();
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
			oakd = StreamYoloDepthAIOakD.getInstance(im.getWidth(), im.getHeight(),"models/yolov7tiny_coco_416x416.blob", 416,416);
			oakd.registerCallback((image,np,d,t1,t2) -> {
				//System.out.println(oakd.getFrameCount()+" "+oakd.getRGBTms()+" "+oakd.getDepthTms()+" "+(1000/(System.currentTimeMillis()-tms)));
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


