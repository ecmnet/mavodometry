package com.comino.mavodometry.libnano.detetction;

import java.awt.Color;
import java.awt.Graphics;
import java.nio.ByteBuffer;
import java.util.HashMap;
import java.util.Map;

import com.comino.mavodometry.libnano.wrapper.JetsonNanoLibrary;
import com.comino.mavodometry.libnano.wrapper.JetsonNanoLibrary.Result;
import com.comino.mavodometry.video.IVisualStreamHandler;
import com.comino.mavutils.legacy.ExecutorService;
import com.sun.jna.Pointer;
import com.sun.jna.ptr.PointerByReference;

import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import georegression.struct.se.Se3_F64;

public class NanoObjectDetection {

	private final PointerByReference net;
	private final Result[] results;
	private int result_length;

	private ByteBuffer buffer;

	private Map<Integer,ObjectIdentity> objects = new HashMap<Integer,ObjectIdentity>();

	public NanoObjectDetection(int width, int height, IVisualStreamHandler<Planar<GrayU8>> stream) {

		this.net = JetsonNanoLibrary.INSTANCE.instance(0,null, width, height);

		Result result = new Result();
		this.results =  ((Result[])result.toArray(100));

		this.buffer = ByteBuffer.allocate(width*height*3);

		if(stream!=null) {
			stream.registerOverlayListener(ctx -> {
				overlayFeatures(ctx);
			});
		}
	}

	long tms = 0;
	public void process(Planar<GrayU8> img, GrayU16 depth, Se3_F64 to_ned) {

		if((System.currentTimeMillis()-tms)<200)
			return;
		tms = System.currentTimeMillis();

		convertToByteBuffer(img, buffer);

		ExecutorService.get().execute(() -> {

			result_length = JetsonNanoLibrary.INSTANCE.detect(net, buffer, results[0], 0);

			objects.clear();
			for(int i=0;i<result_length;i++) {
			   Pointer p = JetsonNanoLibrary.INSTANCE.getClassDescription(net, results[i].ClassID);

			   ObjectIdentity  o = new ObjectIdentity(results[i].ClassID,results[i].Confidence, p.getString(0),
            		   (int)results[i].Left, (int)results[i].Top,
		               (int)results[i].Right - (int)results[i].Left,
		               (int)results[i].Bottom - (int)results[i].Top);

               // check Coordinates
               o.processPosition(depth.subimage((int)results[i].Left, (int)results[i].Top,
            		                            (int)results[i].Right, (int)results[i].Right),
            		             to_ned);

               objects.put(results[i].Instance, o);

			}
		});

	}

	public Map<Integer,ObjectIdentity> getObjects() {
		return objects;
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
