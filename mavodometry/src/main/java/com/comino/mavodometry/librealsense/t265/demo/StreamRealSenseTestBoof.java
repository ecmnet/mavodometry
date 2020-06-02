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
//package com.comino.mavodometry.librealsense.t265.demo;
//
//import java.awt.image.BufferedImage;
//
//import com.comino.mavodometry.librealsense.t265.boofcv.StreamRealSenseT265Pose;
//
//import boofcv.abst.feature.disparity.StereoDisparity;
//import boofcv.alg.distort.ImageDistort;
//import boofcv.alg.distort.LensDistortionNarrowFOV;
//import boofcv.alg.distort.LensDistortionWideFOV;
//import boofcv.alg.distort.NarrowToWidePtoP_F32;
//import boofcv.alg.distort.PointToPixelTransform_F32;
//import boofcv.alg.distort.pinhole.LensDistortionPinhole;
//import boofcv.alg.distort.universal.LensDistortionUniversalOmni;
//import boofcv.alg.interpolate.InterpolatePixel;
//import boofcv.alg.interpolate.InterpolationType;
//import boofcv.factory.distort.FactoryDistort;
//import boofcv.factory.feature.disparity.ConfigDisparityBMBest5;
//import boofcv.factory.feature.disparity.DisparityError;
//import boofcv.factory.feature.disparity.FactoryStereoDisparity;
//import boofcv.factory.interpolate.FactoryInterpolation;
//import boofcv.io.image.ConvertBufferedImage;
//import boofcv.struct.border.BorderType;
//import boofcv.struct.calib.CameraPinhole;
//import boofcv.struct.calib.StereoParameters;
//import boofcv.struct.image.GrayF32;
//import boofcv.struct.image.GrayU8;
//import javafx.application.Application;
//import javafx.application.Platform;
//import javafx.embed.swing.SwingFXUtils;
//import javafx.scene.Scene;
//import javafx.scene.image.ImageView;
//import javafx.scene.image.WritableImage;
//import javafx.scene.layout.GridPane;
//import javafx.stage.Stage;
//
//public class StreamRealSenseTestBoof extends Application  {
//
//	private static final int WIDTH  = 320;
//	private static final int HEIGHT = 240;
//	private static final int FOV    = 10;
//
//	private BufferedImage leftb;
//	private BufferedImage rightb;
//	private BufferedImage dispb;
//	private final ImageView leftv = new ImageView();
//	private WritableImage   lefti;
//	private final ImageView rightv = new ImageView();
//	private WritableImage   righti;
//	private final ImageView dispv = new ImageView();
//	private WritableImage   dispi;
//
//
//	private StreamRealSenseT265Pose t265;
//
//	private ImageDistort<GrayU8,GrayU8> distorter_left;
//	private ImageDistort<GrayU8,GrayU8> distorter_right;
//
//	private GrayU8 left_u  = new GrayU8(WIDTH,HEIGHT);
//	private GrayU8 right_u = new GrayU8(WIDTH,HEIGHT);
//	private GrayU8 map     = new GrayU8(WIDTH,HEIGHT);
//
//	CameraPinhole pinholeModel;
//	StereoParameters param;
//
//
//	@Override
//	public void start(Stage primaryStage) {
//		primaryStage.setTitle("BoofCV RealSense Demo");
//
//		GridPane root = new GridPane();
//		root.add(leftv, 0, 0);
//		root.add(rightv, 1, 0);
//		root.add(dispv, 2, 0);
//
//
//		try {
//
//			// http://boofcv.org/index.php?title=Example_Stereo_Disparity
//			// http://boofcv.org/index.php?title=Example_Stereo_Disparity_3D
//
//
//
//		} catch(Exception e) {
//			System.out.println("REALSENSE:"+e.getMessage());
//			return;
//		}
//
//		t265 = new StreamRealSenseT265Pose(StreamRealSenseT265Pose.POS_FOREWARD, 848,800,(tms, raw, p , s, a, left, right) ->  {
//
//			if(distorter_left == null) {
//
//				pinholeModel = new CameraPinhole(t265.getLeftModel().fx/1.5f,t265.getLeftModel().fy/1.5f,0,WIDTH/2,HEIGHT/2,WIDTH,HEIGHT);
//				LensDistortionNarrowFOV pinholeDistort = new LensDistortionPinhole(pinholeModel);
//
//				LensDistortionWideFOV fisheyeDistort = new LensDistortionUniversalOmni(t265.getLeftModel());
//				NarrowToWidePtoP_F32 transform = new NarrowToWidePtoP_F32(pinholeDistort,fisheyeDistort);
//
//				InterpolatePixel<GrayU8> interp = FactoryInterpolation.
//						createPixel(0, 255, InterpolationType.BILINEAR, BorderType.ZERO,left.getImageType());
//
//				distorter_left = FactoryDistort.distort(false,interp,left.getImageType());
//				// Pass in the transform created above
//				distorter_left.setModel(new PointToPixelTransform_F32(transform));
//
//			}
//
//			if(distorter_right == null) {
//
//				CameraPinhole pinholeModel = new CameraPinhole(t265.getRightModel().fx/1.5f,t265.getRightModel().fy/1.5f,0,WIDTH/2,HEIGHT/2,WIDTH,HEIGHT);
//				LensDistortionNarrowFOV pinholeDistort = new LensDistortionPinhole(pinholeModel);
//
//				LensDistortionWideFOV fisheyeDistort = new LensDistortionUniversalOmni(t265.getRightModel());
//				NarrowToWidePtoP_F32 transform = new NarrowToWidePtoP_F32(pinholeDistort,fisheyeDistort);
//
//				InterpolatePixel<GrayU8> interp = FactoryInterpolation.
//						createPixel(0, 255, InterpolationType.BILINEAR, BorderType.ZERO,right.getImageType());
//
//				distorter_right = FactoryDistort.distort(false,interp,right.getImageType());
//				// Pass in the transform created above
//				distorter_right.setModel(new PointToPixelTransform_F32(transform));
//
//
//			}
//
//			if(param==null)
//				return;
//
//		//	rectify(left,right,param,left_u,right_u);
//
//			//
//						distorter_left.apply(left, left_u);
//						distorter_right.apply(right, right_u);
//
//			GrayU8 disparity = denseDisparity(left_u.subimage(0, HEIGHT/2-FOV, WIDTH-1, HEIGHT/2+FOV),
//					right_u.subimage(0, HEIGHT/2-FOV, WIDTH-1, HEIGHT/2+FOV),3,1,200);
////			GrayF32 disparity = denseDisparitySubpixel(left_u.subimage(0, HEIGHT/2-FOV, WIDTH-1, HEIGHT/2+FOV),
////					right_u.subimage(0, HEIGHT/2-FOV, WIDTH-1, HEIGHT/2+FOV),5,1,200);
//
//			double d = 0;
//			for(int x = 0; x < WIDTH; x++) {
//				d = 0;
//				for(int y=0;y<FOV;y++) {
//					if(disparity.unsafe_get(x,y)> d)
//						d = disparity.unsafe_get(x,y);
//				}
//				d = d + 10;
//
//				for(int k=0;k<HEIGHT;k++)
//					map.set(x, k, 255);
//
//				if( d < 60 && d > 0 ) {
//					double z = 120.0 * pinholeModel.fx/d / 4 - 100;
//					if(z<HEIGHT && z > 1)
//						map.set(x, HEIGHT-(int)z, 0);
//				}
//
//			}
//
//			ConvertBufferedImage.convertTo(left_u, leftb);
//			ConvertBufferedImage.convertTo(right_u, rightb);
//			ConvertBufferedImage.convertTo(map, dispb);
//
//
//
//			Platform.runLater(() -> {
//				SwingFXUtils.toFXImage(leftb, lefti);
//				SwingFXUtils.toFXImage(rightb, righti);
//				SwingFXUtils.toFXImage(dispb, dispi);
//			});
//
//		});
//
//		t265.printDeviceInfo();
//
//		t265.start();
//
//		primaryStage.setScene(new Scene(root, WIDTH*3,HEIGHT));
//		primaryStage.show();
//
//
//		Runtime.getRuntime().addShutdownHook(new Thread() {
//			public void run() {
//				t265.stop();
//			}
//		});
//
//
//		leftb = new BufferedImage(WIDTH, HEIGHT, BufferedImage.TYPE_BYTE_GRAY);
//		lefti = new WritableImage(WIDTH, HEIGHT);
//		leftv.setImage(lefti);
//
//		rightb = new BufferedImage(WIDTH, HEIGHT, BufferedImage.TYPE_BYTE_GRAY);
//		righti = new WritableImage(WIDTH, HEIGHT);
//		rightv.setImage(righti);
//
//		dispb = new BufferedImage(WIDTH, HEIGHT, BufferedImage.TYPE_BYTE_GRAY);
//		dispi = new WritableImage(WIDTH, HEIGHT);
//		dispv.setImage(dispi);
//
//
//
//	}
//
//	@Override
//	public void stop() throws Exception {
//		t265.stop();
//		super.stop();
//	}
//
//	public static void main(String[] args) {
//
//
//		launch(args);
//	}
//
//	public  GrayF32 denseDisparitySubpixel(GrayU8 rectLeft , GrayU8 rectRight ,
//			int regionSize ,
//			int minDisparity , int rangeDisparity )
//	{
//		// A slower but more accuracy algorithm is selected
//		// All of these parameters should be turned
//		ConfigDisparityBMBest5 config = new ConfigDisparityBMBest5();
//		config.errorType = DisparityError.CENSUS;
//		config.disparityMin = minDisparity;
//		config.disparityRange = rangeDisparity;
//		config.subpixel = true;
//		config.regionRadiusX  = config.regionRadiusY  = regionSize;
//		config.maxPerPixelError = 35;
//		config.validateRtoL = 1;
//		config.texture = 0.05;
//
//		StereoDisparity<GrayU8, GrayF32> disparityAlg =
//				FactoryStereoDisparity.blockMatchBest5(config, GrayU8.class, GrayF32.class);
//
//		// process and return the results
//		disparityAlg.process(rectLeft,rectRight);
//
//		return disparityAlg.getDisparity();
//	}
//
//	public  GrayU8 denseDisparity(GrayU8 rectLeft , GrayU8 rectRight ,
//			int regionSize,
//			int minDisparity , int rangeDisparity )
//	{
//		// A slower but more accuracy algorithm is selected
//		// All of these parameters should be turned
//		ConfigDisparityBMBest5 config = new ConfigDisparityBMBest5();
//		config.errorType = DisparityError.CENSUS;
//		config.disparityMin = minDisparity;
//		config.disparityRange = rangeDisparity;
//		config.subpixel = false;
//		config.regionRadiusX = config.regionRadiusY = regionSize;
//		config.maxPerPixelError = 35;
//		config.validateRtoL = 1;
//		config.texture = 0.05;
//
//		double maxError = (config.regionRadiusX*2+1)*(config.regionRadiusY*2+1)*config.maxPerPixelError;
//
//		StereoDisparity<GrayU8, GrayU8> disparityAlg =
//				FactoryStereoDisparity.blockMatchBest5(config, GrayU8.class, GrayU8.class);
//
//		// process and return the results
//		disparityAlg.process(rectLeft,rectRight);
//
//		return disparityAlg.getDisparity();
//	}
//
//
//
//
//	private float fromRad(double radians) {
//		float deg = (float)(radians * 180.0 / Math.PI  ) % 360;
//		if(deg<0) deg += 360;
//		return deg;
//
//	}
//
//}
