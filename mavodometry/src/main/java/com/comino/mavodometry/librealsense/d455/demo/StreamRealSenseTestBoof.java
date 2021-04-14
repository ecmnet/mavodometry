//package com.comino.mavodometry.librealsense.d455.demo;
///****************************************************************************
// *
// *   Copyright (c) 2017 Eike Mansfeld ecm@gmx.de. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without
// * modification, are permitted provided that the following conditions
// * are met:
// *
// * 1. Redistributions of source code must retain the above copyright
// *    notice, this list of conditions and the following disclaimer.
// * 2. Redistributions in binary form must reproduce the above copyright
// *    notice, this list of conditions and the following disclaimer in
// *    the documentation and/or other materials provided with the
// *    distribution.
// * 3. Neither the name of the copyright holder nor the names of its
// *    contributors may be used to endorse or promote products derived
// *    from this software without specific prior written permission.
// *
// * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
// * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
// * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// * POSSIBILITY OF SUCH DAMAGE.
// *
// ****************************************************************************/
//
//
//import java.awt.image.BufferedImage;
//
//import com.comino.mavodometry.librealsense.d455.boofcv.IDepthCallback;
//import com.comino.mavodometry.librealsense.d455.boofcv.StreamRealSenseD455Depth;
//import com.comino.mavodometry.librealsense.utils.RealSenseInfo;
//
//import boofcv.abst.feature.detect.interest.ConfigGeneralDetector;
//import boofcv.abst.feature.tracker.PointTrackerTwoPass;
//import boofcv.abst.sfm.d3.DepthVisualOdometry;
//import boofcv.alg.sfm.DepthSparse3D;
//import boofcv.alg.tracker.klt.PkltConfig;
//import boofcv.concurrency.BoofConcurrency;
//import boofcv.factory.feature.tracker.FactoryPointTrackerTwoPass;
//import boofcv.factory.sfm.FactoryVisualOdometry;
//import boofcv.io.image.ConvertBufferedImage;
//import boofcv.struct.image.GrayS16;
//import boofcv.struct.image.GrayU16;
//import boofcv.struct.image.GrayU8;
//import boofcv.struct.image.Planar;
//import georegression.struct.se.Se3_F64;
//import javafx.application.Application;
//import javafx.application.Platform;
//import javafx.embed.swing.SwingFXUtils;
//import javafx.scene.Scene;
//import javafx.scene.image.ImageView;
//import javafx.scene.image.WritableImage;
//import javafx.scene.input.MouseEvent;
//import javafx.scene.layout.FlowPane;
//import javafx.stage.Stage;
//
//public class StreamRealSenseTestBoof extends Application  {
//
//	private static final float  INLIER_PIXEL_TOL    	= 1.5f;
//	private static final int    MAXTRACKS   			= 600;
//	private static final int    KLT_RADIUS          	= 3;
//	private static final float  KLT_THRESHOLD       	= 1f;
//	private static final int    RANSAC_ITERATIONS   	= 1350;
//	private static final int    RETIRE_THRESHOLD    	= 2;
//	private static final int    ADD_THRESHOLD       	= 120;
//	private static final int    REFINE_ITERATIONS   	= 50;
//
//	private BufferedImage o_rgb;
//	private final ImageView ivrgb = new ImageView();
//	private WritableImage wirgb;
//
//	private BufferedImage o_depth;
//	private final ImageView ivdepth = new ImageView();
//	private WritableImage widepth;
//
//	private StreamRealSenseD455Depth realsense;
//
//
//	private long oldTimeDepth=0;
//	private long tms = 0;
//
//	private int mouse_x;
//	private int mouse_y;
//
//	@Override
//	public void start(Stage primaryStage) {
//		primaryStage.setTitle("BoofCV RealSense Demo");
//
//		FlowPane root = new FlowPane();
//		
//		BoofConcurrency.setMaxThreads(4);
//
//		root.getChildren().add(ivrgb);
//		root.getChildren().add(ivdepth);
//
//
//		ivrgb.setOnMouseMoved(event -> {
//			MouseEvent ev = event;
//			mouse_x = (int)ev.getX();
//			mouse_y = (int)ev.getY();
//		});
//
//
//		//	RealSenseInfo info = new RealSenseInfo(640,480, RealSenseInfo.MODE_RGB);
//		RealSenseInfo info = new RealSenseInfo(640,480, RealSenseInfo.MODE_RGB);
//
//		try {
//
//			realsense = new StreamRealSenseD455Depth(0,info);
//
//		} catch(Exception e) {
//			System.out.println("REALSENSE D455: "+e.getMessage());
//			return;
//		}
//
//
//		mouse_x = info.width/2;
//		mouse_y = info.height/2;
//
//		primaryStage.setScene(new Scene(root, info.width*2,info.height));
//		primaryStage.show();
//
//		try {
//			Thread.sleep(100);
//		} catch (InterruptedException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		}
//
//
//		//	visualOdometry.setCalibration(realsense.getIntrinsics(),new DoNothing2Transform2_F32());
//
//		Runtime.getRuntime().addShutdownHook(new Thread() {
//			public void run() {
//				realsense.stop();
//			}
//		});
//
//
//		o_rgb = new BufferedImage(info.width, info.height, BufferedImage.TYPE_INT_RGB);
//		wirgb = new WritableImage(info.width, info.height);
//		ivrgb.setImage(wirgb);
//
//		o_depth = new BufferedImage(info.width, info.height, BufferedImage.TYPE_BYTE_GRAY);
//		widepth = new WritableImage(info.width, info.height);
//		ivdepth.setImage(widepth);
//
//		realsense.registerCallback(new IDepthCallback() {
//
//			int fps; float mouse_depth; float md; int mc; int mf=0; int fpm;
//
//			@Override
//			public void process(Planar<GrayU8> rgb, GrayU16 depth, long timeRgb, long timeDepth) {
//
//
//				if((System.currentTimeMillis() - tms) > 250) {
//					tms = System.currentTimeMillis();
//					if(mf>0)
//						fps = fpm/mf;
//					if(mc>0)
//						mouse_depth = md / mc;
//					mc = 0; md = 0; mf=0; fpm=0;
//				}
//				mf++;
//				fpm += (int)(1f/((timeRgb - oldTimeDepth)/1000f)+0.5f);
//				oldTimeDepth = timeRgb;
//
//
//				ConvertBufferedImage.convertTo_U8(rgb, o_rgb, true);
//				ConvertBufferedImage.convertTo(depth, o_depth, true);
//
//				Platform.runLater(() -> {
//					SwingFXUtils.toFXImage(o_rgb, wirgb);
//					SwingFXUtils.toFXImage(o_depth, widepth);
//				});
//			}
//
//		}).start();
//
//	}
//
//	@Override
//	public void stop() throws Exception {
//		realsense.stop();
//		super.stop();
//	}
//
//	public static void main(String[] args) {
//
//
//		launch(args);
//	}
//
//}
