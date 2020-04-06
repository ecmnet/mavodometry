package com.comino.mavodometry.estimators.test;

import java.awt.image.BufferedImage;
import java.nio.ByteBuffer;

import com.comino.mavodometry.libnano.utils.ImageConversionUtil;
import com.comino.mavodometry.libnano.wrapper.JetsonNanoLibrary;
import com.comino.mavodometry.libnano.wrapper.JetsonNanoLibrary.Result;
import com.comino.mavutils.jna.NativeString;
import com.sun.jna.Pointer;
import com.sun.jna.Structure;
import com.sun.jna.ptr.PointerByReference;

import boofcv.core.image.ConvertByteBufferImage;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.io.image.UtilImageIO;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;

public class JetsonNanoInferenceTest {

	private PointerByReference net = null;

	public JetsonNanoInferenceTest() {
		ImageConversionUtil.getInstance(320,240);
		net = JetsonNanoLibrary.INSTANCE.createSegNet(JetsonNanoLibrary.FCN_RESNET18_DEEPSCENE_576x320, 320, 240);
		performTest();
		System.out.println("JetsonNanoInferenceTest finalized. Exiting....");
		System.exit(0);


	}

	private void performTest() {
		BufferedImage image = UtilImageIO.loadImage("IMG2.jpg");
		if(image==null) {
			System.out.println("Image not loaded..");
			return;
		}
		Planar<GrayU8> img = ConvertBufferedImage.convertFromPlanar(image, null,true, GrayU8.class);

		System.out.println("Image loaded: "+img.width+"x"+img.height);



		ImageConversionUtil.getInstance().convertToByteBuffer(img);
		JetsonNanoLibrary.INSTANCE.segmenting(net, ImageConversionUtil.getInstance().getImage());

		ImageConversionUtil.getInstance().convertToPlanar(img);

		UtilImageIO.saveImage(img, "img_out.jpg");

	}

	public static void main(String[] args)  {
		new JetsonNanoInferenceTest();
	}

}
