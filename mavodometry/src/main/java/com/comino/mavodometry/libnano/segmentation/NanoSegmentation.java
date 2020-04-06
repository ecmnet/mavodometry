package com.comino.mavodometry.libnano.segmentation;

import java.nio.ByteBuffer;

import com.comino.mavodometry.libnano.wrapper.JetsonNanoLibrary;
import com.comino.mavodometry.video.IVisualStreamHandler;
import com.sun.jna.ptr.PointerByReference;

import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import georegression.struct.se.Se3_F64;

public class NanoSegmentation  {


	private final PointerByReference net;



	public NanoSegmentation(int width, int height, IVisualStreamHandler<Planar<GrayU8>> stream) {

		net = JetsonNanoLibrary.INSTANCE.createSegNet(JetsonNanoLibrary.FCN_RESNET18_DEEPSCENE_576x320, width, height);

	}

	public void process(ByteBuffer img, GrayU16 depth, Se3_F64 to_ned) {


	   JetsonNanoLibrary.INSTANCE.segmenting(net, img);


	}


}
