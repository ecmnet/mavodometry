package com.comino.mavodometry.estimators.test;

import java.awt.image.BufferedImage;
import java.nio.ByteBuffer;

import com.comino.mavodometry.libnano.wrapper.JetsonNanoLibrary;
import com.comino.mavodometry.libnano.wrapper.JetsonNanoLibrary.Result;
import com.sun.jna.Pointer;
import com.sun.jna.Structure;
import com.sun.jna.ptr.PointerByReference;

import boofcv.core.image.ConvertByteBufferImage;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.io.image.UtilImageIO;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;

public class JetsonNanoInferenceTest {

	private static final PointerByReference net = JetsonNanoLibrary.INSTANCE.instance(0,null, 481,640);

	public JetsonNanoInferenceTest() {
		performTest();
		System.out.println("JetsonNanoInferenceTest finalized. Exiting....");
		System.exit(0);
	}

	private void performTest() {
		BufferedImage image = UtilImageIO.loadImage("test.jpg");
		if(image==null) {
			System.out.println("Image not loaded..");
			return;
		}
		Planar<GrayU8> img = ConvertBufferedImage.convertFromPlanar(image, null,true, GrayU8.class);

		System.out.println("Image loaded: "+img.width+"x"+img.height);

		Result result = new Result();
		Result[] results = ((Result[])result.toArray(100));
		ByteBuffer buf = ByteBuffer.allocate(img.width*img.height*3);

		System.out.println("BufferSize: "+buf.capacity());


		convertToByteBuffer(img,buf);
		int count = JetsonNanoLibrary.INSTANCE.detect(net, buf, results[0], 0);
		for(int i=0;i<count;i++) {
			Pointer p = JetsonNanoLibrary.INSTANCE.getClassDescription(net, results[i].ClassID);
			System.out.println("Detected object: "+p.getString(0)+" with confidence of "+results[i].Confidence);
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