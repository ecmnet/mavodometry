package com.comino.mavodometry.libnano.detetction;

import java.awt.Graphics;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import com.comino.mavodometry.libnano.detetction.helper.DistanceDetermination;
import com.comino.mavodometry.libnano.wrapper.JetsonNanoLibrary;
import com.comino.mavodometry.libnano.wrapper.JetsonNanoLibrary.Result;
import com.comino.mavodometry.video.IVisualStreamHandler;
import com.sun.jna.Pointer;
import com.sun.jna.ptr.PointerByReference;

import boofcv.alg.distort.LensDistortionNarrowFOV;
import boofcv.struct.distort.PixelTransform;
import boofcv.struct.distort.Point2Transform2_F64;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import georegression.geometry.GeometryMath_F64;
import georegression.struct.point.Point2D_F32;
import georegression.struct.point.Point2D_F64;
import georegression.struct.se.Se3_F64;

public class NanoObjectDetection {

	private static final int MAX_OBJECTS = 100;

	private final PointerByReference net;
	private final Result[] results;
	private int result_length;

	private final ByteBuffer buffer;

	private final GrayU16 sub_depth   = new GrayU16(-1,-1);
	private final Point2D_F64 norm    = new Point2D_F64();

	private Point2Transform2_F64 p2n;


	private  Map<Integer,ObjectIdentity> objects = new ConcurrentHashMap<Integer,ObjectIdentity>();
	private  List<Integer>               removes = new ArrayList<Integer>();

	private boolean onlyPersons;


	public NanoObjectDetection(int width, int height, IVisualStreamHandler<Planar<GrayU8>> stream ) {

		this.net = JetsonNanoLibrary.INSTANCE.instance(0,null, width, height);

		Result result = new Result();
		this.results =  ((Result[])result.toArray(MAX_OBJECTS));

		this.buffer = ByteBuffer.allocate(width*height*3);

		if(stream!=null) {
			stream.registerOverlayListener(ctx -> {
				overlayFeatures(ctx);
			});
		}
	}

	public void configure(LensDistortionNarrowFOV model , PixelTransform<Point2D_F32> visualToDepth, boolean onlyPersons ) {
		this.p2n = model.undistort_F64(true,false);
		this.onlyPersons = onlyPersons;
	}


	public void process(Planar<GrayU8> img, GrayU16 depth, Se3_F64 to_ned) {

		convertToByteBuffer(img, buffer);

//		ExecutorService.submit(() -> {

			ObjectIdentity  obj; int id;

			result_length = JetsonNanoLibrary.INSTANCE.detect(net, buffer, results[0], 0);

				removes.clear();
				objects.forEach((k,o) -> { if(o.isExpired()) removes.add(k); });
				removes.forEach((k) -> { objects.remove(k); });

				for(int i=0;i<result_length;i++) {

					if(onlyPersons && results[i].ClassID != 1)
						continue;

					Pointer p = JetsonNanoLibrary.INSTANCE.getClassDescription(net, results[i].ClassID);

					id = results[i].Instance;
					if(!objects.containsKey(id)) {
						obj = new ObjectIdentity(id,
								results[i].ClassID,results[i].Confidence, p.getString(0),
								(int)results[i].Left, (int)results[i].Top,
								(int)results[i].Right - (int)results[i].Left,
								(int)results[i].Bottom - (int)results[i].Top);
						computeObjectPt(obj, (int)results[i].Left, (int)results[i].Top,
								depth.subimage((int)results[i].Left, (int)results[i].Top,
										(int)results[i].Right, (int)results[i].Bottom, sub_depth),
								to_ned);
						objects.put(id,obj);
					}
					else {
						obj = objects.get(id);
						obj.update(results[i].ClassID,results[i].Confidence, p.getString(0),
								(int)results[i].Left, (int)results[i].Top,
								(int)results[i].Right - (int)results[i].Left,
								(int)results[i].Bottom - (int)results[i].Top);

						computeObjectPt(obj, (int)results[i].Left, (int)results[i].Top,
								depth.subimage((int)results[i].Left, (int)results[i].Top,
										(int)results[i].Right, (int)results[i].Bottom, sub_depth),
								to_ned);
					}
				}

//		});

	}

	public Map<Integer,ObjectIdentity> getObjects() {
		return objects;
	}

	private void computeObjectPt(ObjectIdentity o, int x0, int y0, GrayU16 sub_depth, Se3_F64 to_ned) {

		// find appropriate distance
		int distance_mm = DistanceDetermination.process(sub_depth);

		if(distance_mm == Integer.MAX_VALUE)
			return;

		// convert visual pixel into normalized image coordinate
		p2n.compute(x0+sub_depth.width/2,y0+sub_depth.height/2 ,norm);

		// project into 3D space
		o.getPosBODY().x = distance_mm*1e-3f;
		o.getPosBODY().y = o.getPosBODY().x*norm.x;
		o.getPosBODY().z = o.getPosBODY().x*norm.y;

		// Rotate to NED if available
		if(!to_ned.T.isNaN()) {
			GeometryMath_F64.mult(to_ned.R, o.getPosBODY(), o.getPosNED() );
			o.getPosNED().plusIP(to_ned.T);
		}
		else {
			o.getPosNED().set(o.getPosBODY());
		}

		//		// Now update depth
		//		// Todo: should be done in sorted order by distance
		//		for(int x = 0; x< sub_depth.width; x++) {
		//			for(int y = 0; y < sub_depth.height; y++) {
		//				if( sub_depth.get(x, y) == 0 || sub_depth.get(x, y) > 15000)
		//					sub_depth.set(x, y, distance_mm);
		//			}
		//		}
	}


	private void overlayFeatures(Graphics ctx) {
		objects.forEach((i,o)->{
			o.draw(ctx);
		});
	}

	private void convertToByteBuffer(Planar<GrayU8> img,ByteBuffer buffer) {
		int i = 0;
		for (int y = 0; y < img.height; y++) {
			for (int x = 0; x < img.width; x++) {
				buffer.array()[i++] = img.bands[0].data[y * img.width + x];
				buffer.array()[i++] = img.bands[1].data[y * img.width + x];
				buffer.array()[i++] = img.bands[2].data[y * img.width + x];
			}

		}

	}

}
