package com.comino.mavodometry.libnano.detetction;

import java.awt.Graphics;
import java.nio.ByteBuffer;

import com.comino.mavodometry.libnano.wrapper.JetsonNanoLibrary;
import com.comino.mavodometry.libnano.wrapper.JetsonNanoLibrary.Result;
import com.comino.mavodometry.utils.DepthUtils;
import com.comino.mavodometry.video.IVisualStreamHandler;
import com.sun.jna.Pointer;
import com.sun.jna.ptr.PointerByReference;

import boofcv.alg.distort.LensDistortionNarrowFOV;
import boofcv.struct.distort.Point2Transform2_F64;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import georegression.struct.point.Point2D_F64;
import georegression.struct.se.Se3_F64;

public class NanoObjectDetection  {

	public static final int  CLASS_ALL    = 0;
	public static final int  CLASS_PERSON = 1;
	public static final int  CLASS_CAT    = 17;

	private static final int MAX_OBJECTS = 20;

	private final PointerByReference net;
	private final Result[] results;
	private int result_length;

	private final GrayU16 sub_depth   = new GrayU16(-1,-1);
	private final Point2D_F64 norm    = new Point2D_F64();

	private Point2Transform2_F64 p2n;

	private NanoObjectIdentity obj = new NanoObjectIdentity();

	private int class_filter;


	public NanoObjectDetection(int width, int height, IVisualStreamHandler<Planar<GrayU8>> stream) {

		this.net = JetsonNanoLibrary.INSTANCE.createDetectNet(JetsonNanoLibrary.NETWORK_TYPE_MOBILENET_V2,0.6f, width, height);


//      For output and labelling of YOLO: https://github.com/NVIDIA-AI-IOT/redtail/wiki/ROS-Nodes#trailnet
//		NativeString  proto = new NativeString("networks/Yolo_Relu/yolo-relu.prototxt", false);
//		NativeString  model = new NativeString("networks/Yolo_Relu/yolo-relu.caffemodel", false);
//		NativeString  outl  = new NativeString("fc25", false);
//
//		net = JetsonNanoLibrary.INSTANCE.createDetectNetCustom(proto.getPointer(), model.getPointer(), null,outl.getPointer(), 0.0f, width, height);


		Result result = new Result();
		this.results =  ((Result[])result.toArray(MAX_OBJECTS));

		if(stream!=null) {
			stream.registerOverlayListener((ctx,n,tms) -> {
				overlayFeatures(ctx);
			});
		}
	}


	public void configure(LensDistortionNarrowFOV model , int class_filter ) {
		this.p2n = model.undistort_F64(true,false);
		this.class_filter = class_filter;
	}



	public void process(ByteBuffer img, GrayU16 depth, Se3_F64 to_ned) {

		for(int i =0;i<results.length;i++)
			results[i].clear();

		obj.clear();
		result_length = JetsonNanoLibrary.INSTANCE.detect(net, img, results[0], 0);

		for(int i=0;i<result_length && i < results.length;i++) {

			if(class_filter!=0 && results[i].ClassID != class_filter)
				continue;

			Pointer p = JetsonNanoLibrary.INSTANCE.getClassDescription(net, results[i].ClassID);

			obj.update(results[i].ClassID,results[i].Confidence, p.getString(0),
					   (int)results[i].Left, (int)results[i].Top, (int)results[i].Right, (int)results[i].Bottom);

			computeObjectPt(obj, (int)results[i].Left, (int)results[i].Top,
						depth.subimage((int)results[i].Left, (int)results[i].Top,
								(int)results[i].Right, (int)results[i].Bottom, sub_depth),
						to_ned);
			break;

		}
	}

	public boolean hasObjectsDetected() {
		return obj.isValid();
	}


	public NanoObjectIdentity getFirstObject() {
		return obj;
	}

	private void computeObjectPt(NanoObjectIdentity o, int x0, int y0, GrayU16 sub_depth, Se3_F64 to_ned) {

		// find appropriate distance
		int distance_mm = DepthUtils.process(sub_depth);

		if(distance_mm == Integer.MAX_VALUE)
			return;

		// convert visual pixel into normalized image coordinate
		p2n.compute(x0+sub_depth.width/2,y0+sub_depth.height/2 ,norm);

		// project into 3D space
		o.getPosBODY().x = distance_mm*1e-3f;
		o.getPosBODY().y = o.getPosBODY().x*norm.x;
		o.getPosBODY().z = o.getPosBODY().x*norm.y;

		// Rotate to NED if available
//		if(!to_ned.T.isNaN()) {
//			GeometryMath_F64.mult(to_ned.R, o.getPosBODY(), o.getPosNED() );
//			o.getPosNED().plusIP(to_ned.T);
//		}
//		else {
			o.getPosNED().setTo(o.getPosBODY());
//		}

	}

	private void overlayFeatures(Graphics ctx) {
		obj.draw(ctx);
	}


}
