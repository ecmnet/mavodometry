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
//import com.comino.mavcom.struct.Attitude3D_F64;
//import com.comino.mavodometry.librealsense.t265.boofcv.StreamRealSenseT265Pose;
//import com.comino.mavutils.MSPMathUtils;
//
//import boofcv.io.image.ConvertBufferedImage;
//import boofcv.struct.image.GrayU8;
//import boofcv.struct.image.Planar;
//import javafx.application.Application;
//import javafx.application.Platform;
//import javafx.embed.swing.SwingFXUtils;
//import javafx.event.ActionEvent;
//import javafx.scene.Scene;
//import javafx.scene.control.Button;
//import javafx.scene.control.Label;
//import javafx.scene.image.ImageView;
//import javafx.scene.image.WritableImage;
//import javafx.scene.layout.GridPane;
//import javafx.scene.paint.Color;
//import javafx.stage.Stage;
//import javafx.stage.StageStyle;
//
//public class StreamRealSenseTestPose extends Application  {
//
//	private static final int WIDTH  = 640;
//	private static final int HEIGHT = 480;
//
//
//	private BufferedImage  leftb;
//	private final ImageView leftv = new ImageView();
//	private WritableImage   lefti;
//
//	private Label labelx = new Label("X");
//	private Label labely = new Label("Y");
//	private Label labelz = new Label("Z");
//	private Label labelr = new Label("R");
//	private Label labelp = new Label("P");
//	private Label labelw = new Label("W");
//	private Label labelq = new Label();
//
//	private float quality=0;
//
//
//	private StreamRealSenseT265Pose t265;
//
//	private Attitude3D_F64   att      = new Attitude3D_F64();
//
//
//	@Override
//	public void start(Stage primaryStage) {
//		primaryStage.setTitle("BoofCV RealSense Demo");
//
//
//		labelx.setMinWidth(80);
//		labely.setMinWidth(80);
//		labelz.setMinWidth(80);
//		labelw.setMinWidth(80);
//		labelp.setMinWidth(80);
//		labelr.setMinWidth(80);
//
//
//		GridPane root = new GridPane();
//		root.add(leftv, 0, 0);
//		GridPane label = new GridPane();
//		root.add(label, 1, 0);
//		label.add(new Label(), 1, 0);
//		label.add(labelx, 1, 1);
//		label.add(labely, 2, 1);
//		label.add(labelz, 3, 1);
//		label.add(labelr, 1, 2);
//		label.add(labelp, 2, 2);
//		label.add(labelw, 3, 2);
//		label.add(new Label(), 1, 3);
//		label.add(new Label("Quality:"), 1, 4);
//		label.add(labelq, 2, 4);
//		label.add(new Label(), 1, 5);
//		Button reset = new Button("Reset");
//		label.add(reset, 1,6);
//
//		reset.setOnAction((ActionEvent event)-> {
//			t265.reset();
//		});
//
//
//		try {
//
//		t265 = StreamRealSenseT265Pose.getInstance(StreamRealSenseT265Pose.POS_DOWNWARD,WIDTH,HEIGHT);
//		t265.registerCallback((tms, raw, p , s, a, img) ->  {
//
//			ConvertBufferedImage.convertTo_U8((Planar<GrayU8>)img, leftb, true);
//
//			if(raw.tracker_confidence == 0)
//				quality = 0f;
//			else if(raw.tracker_confidence == 1)
//				quality = 0.33f;
//			else if(raw.tracker_confidence == 2)
//				quality = 0.66f;
//			else if(raw.tracker_confidence == 3)
//				quality = 1f;
//		    else
//			    System.out.println("TrackerConfidence is "+raw.tracker_confidence);
//
//
//
//			att.setFromMatrix(p.R);
//
//			Platform.runLater(() -> {
//				labelx.setText(String.format(" X : %+3.2f     ", p.T.x));
//				labely.setText(String.format(" Y : %+3.2f     ", p.T.y));
//				labelz.setText(String.format(" Z : %+3.2f     ", p.T.z));
//				labelp.setText(String.format(" P : %+3.1f°    ", att.getInDegree(Attitude3D_F64.PITCH)));
//				labelr.setText(String.format(" R : %+3.1f°    ", att.getInDegree(Attitude3D_F64.ROLL)));
//				labelw.setText(String.format(" W : %+3.1f°    ", att.getInDegree(Attitude3D_F64.YAW)));
//				labelq.setText(String.format(" %1.2f          ", quality));
//
//				SwingFXUtils.toFXImage(leftb, lefti);
//			});
//
//		});
//
//		t265.printDeviceInfo();
//
//		t265.start();
//
//		} catch(Exception e) {
//			e.printStackTrace();
//		}
//
//		Scene scene = new Scene(root, WIDTH+280,HEIGHT);
//
//		primaryStage.setScene(scene);
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
//		leftb = new BufferedImage(WIDTH, HEIGHT, BufferedImage.TYPE_3BYTE_BGR);
//		lefti = new WritableImage(WIDTH, HEIGHT);
//		leftv.setImage(lefti);
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
//
//	private float fromRad(double radians) {
//		float deg = (float)(radians * 180.0 / Math.PI  ) % 360;
//		if(deg<0) deg += 360;
//		return deg;
//
//	}
//
//}
