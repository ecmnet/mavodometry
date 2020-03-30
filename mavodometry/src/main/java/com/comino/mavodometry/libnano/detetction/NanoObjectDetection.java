package com.comino.mavodometry.libnano.detetction;

import java.awt.Color;
import java.awt.Graphics;
import java.nio.ByteBuffer;

import com.comino.mavcom.model.segment.Status;
import com.comino.mavodometry.libnano.wrapper.JetsonNanoLibrary;
import com.comino.mavodometry.libnano.wrapper.JetsonNanoLibrary.Result;
import com.comino.mavodometry.video.IVisualStreamHandler;
import com.comino.mavutils.legacy.ExecutorService;
import com.sun.jna.Pointer;
import com.sun.jna.ptr.PointerByReference;

import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;

public class NanoObjectDetection {

	private final PointerByReference net;
	private final Result[] results;
	private int result_length;

	private int width;
	private int height;

	private ByteBuffer buffer;

	public NanoObjectDetection(int width, int height, IVisualStreamHandler<Planar<GrayU8>> stream) {

		this.width = width;
		this.height = height;
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
	public void process(Planar<GrayU8> img, GrayU16 depth) {

		if((System.currentTimeMillis()-tms)<100)
			return;
		tms = System.currentTimeMillis();

		convertToByteBuffer(img, buffer);
		ExecutorService.get().execute(() -> {
			result_length = JetsonNanoLibrary.INSTANCE.detect(net, buffer, results[0], 0);
		});

	}
    // Idea: Built histogramm of 100 depth classes. Return depth of major class
//	private float getDepthOfObject(Result result,GrayU16 depth) {
//		int depth_z = Integer.MAX_VALUE; int raw_z;
//		if(result.Confidence < 0.1 || result.ClassID == 0)
//			return Float.MAX_VALUE;
//		for(int x = (int)result.Left; x < (int)result.Right;x++) {
//			for(int y = (int)result.Top; y < (int)result.Bottom;y++) {
//				raw_z = depth.get(x, y);
//				if(raw_z > 20 && raw_z < depth_z && raw_z < 15000) {
//					depth_z =  raw_z;
//				}
//			}
//		}
//		return depth_z/1000f;
//	}

	private void overlayFeatures(Graphics ctx) {

		ctx.setColor(Color.ORANGE);

		for(int i=0; i< result_length; i++) {
			if(results[i].Confidence < 0.1 || results[i].ClassID == 0)
				continue;
			ctx.drawLine((int)results[i].Left, (int)results[i].Top, (int)results[i].Right, (int)results[i].Top);
			ctx.drawLine((int)results[i].Left, (int)results[i].Bottom, (int)results[i].Right, (int)results[i].Bottom);
			ctx.drawLine((int)results[i].Left, (int)results[i].Top, (int)results[i].Left, (int)results[i].Bottom);
			ctx.drawLine((int)results[i].Right, (int)results[i].Top, (int)results[i].Right, (int)results[i].Bottom);

			Pointer p = JetsonNanoLibrary.INSTANCE.getClassDescription(net, results[i].ClassID);
			ctx.drawString(String.format("%s",p.getString(0)), (int)results[i].Left, (int)results[i].Top-5);
		}

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
