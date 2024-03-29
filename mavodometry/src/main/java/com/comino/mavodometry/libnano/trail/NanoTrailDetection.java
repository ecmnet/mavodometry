package com.comino.mavodometry.libnano.trail;

import java.awt.Color;
import java.awt.Graphics;
import java.nio.ByteBuffer;

import com.comino.mavodometry.libnano.wrapper.JetsonNanoLibrary;
import com.comino.mavodometry.libnano.wrapper.JetsonNanoLibrary.Result;
import com.comino.mavodometry.video.IVisualStreamHandler;
import com.comino.mavutils.jna.NativeString;
import com.sun.jna.ptr.PointerByReference;

import boofcv.alg.distort.LensDistortionNarrowFOV;
import boofcv.struct.distort.Point2Transform2_F64;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import georegression.struct.point.Point2D_F64;
import georegression.struct.se.Se3_F64;

public class NanoTrailDetection  {

	private final static float DNN_TURN_ANGLE_DEG               = 10.0f;
	private final static float DNN_LATERAL_CORRECTION_ANGLE_DEG = 10.0f;

	private static final float direction_filter_innov_coeff_    = 0.05f;

	private final PointerByReference net;
	private final Result[] results;
	private int result_length;

	private final GrayU16 sub_depth   = new GrayU16(-1,-1);
	private final Point2D_F64 norm    = new Point2D_F64();

	private Point2Transform2_F64 p2n;

	private final Color	bgColor           = new Color(128,128,128,140);

	private float turn_angle = 0;


	public NanoTrailDetection(int width, int height, IVisualStreamHandler<Planar<GrayU8>> stream) {

		NativeString  proto = new NativeString("networks/TrailNet_SResNet18/TrailNet_SResNet-18.prototxt", false);
		NativeString  model = new NativeString("networks/TrailNet_SResNet18/TrailNet_SResNet-18.caffemodel", false);
		NativeString  label = new NativeString("networks/TrailNet_SResNet18/label_map.txt", false);
		NativeString  outl  = new NativeString("out", false);

		net = JetsonNanoLibrary.INSTANCE.createDetectNetCustom(proto.getPointer(), model.getPointer(), label.getPointer(),outl.getPointer(), 0.0f, width, height);

		Result result = new Result();
		this.results =  ((Result[])result.toArray(6));

		if(stream!=null) {
			stream.registerOverlayListener((ctx,n,tms) -> {
				overlayFeatures(ctx);
			});
		}
	}

	public void configure(LensDistortionNarrowFOV model) {
		this.p2n = model.undistort_F64(true,false);
	}


	public void process(ByteBuffer img, GrayU16 depth, Se3_F64 to_ned) {


		result_length = JetsonNanoLibrary.INSTANCE.detect(net, img, results[0], 0);

		float prob_sum = results[0].Confidence + results[1].Confidence + results[2].Confidence;

		//System.out.println(results[0].Confidence + " / "+ results[1].Confidence + " / "+results[2].Confidence);

		float left_view_p   = results[0].Confidence / prob_sum;
	    float right_view_p  = results[2].Confidence / prob_sum;

	    float left_side_p   = results[3].Confidence / prob_sum;
	    float right_side_p  = results[5].Confidence / prob_sum;

	    //System.out.println(left_view_p + " / "+ right_view_p );

	    float current_turn_angle_deg =  DNN_TURN_ANGLE_DEG*(right_view_p - left_view_p) + DNN_LATERAL_CORRECTION_ANGLE_DEG *(right_side_p - left_side_p);


	    turn_angle = turn_angle*(1-direction_filter_innov_coeff_) + current_turn_angle_deg*direction_filter_innov_coeff_;


	}



	private void overlayFeatures(Graphics ctx) {
		if(result_length> 0) {
			ctx.setColor(bgColor);
			ctx.fillRect(5, 30, 100, 16);
			ctx.setColor(Color.WHITE);
			ctx.drawString(String.format("TrailDir.: %.1f°", turn_angle),10,43);
		}

	}

}
