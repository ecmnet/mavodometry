package com.comino.mavodometry.libnano.utils;

import java.nio.ByteBuffer;

import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;

public class ImageConversionUtil {

	private static ImageConversionUtil instance = null;

	private ByteBuffer buffer = null;

	public static ImageConversionUtil getInstance(int width, int height) {
		if(instance==null)
			instance = new ImageConversionUtil(width, height);
		return instance;
	}

	public static ImageConversionUtil getInstance() {
		return instance;
	}

	public ImageConversionUtil(int width, int height) {
		this.buffer = ByteBuffer.allocate(width*height*3);
	}

	public  ByteBuffer convertToByteBuffer(Planar<GrayU8> img) {
		int i = 0; int j=0; int k=0;
		for (int y = 0; y < img.height; y++) {
			j = y * img.width;
			for (int x = 0; x < img.width; x++) {
				k = j + x;
				buffer.array()[i++] = img.bands[0].data[k];
				buffer.array()[i++] = img.bands[1].data[k];
				buffer.array()[i++] = img.bands[2].data[k];
			}

		}
		return buffer;
	}


	public  void convertToPlanar(Planar<GrayU8> img) {
		int i = 0; int j= 0; int k=0;
		for (int y = 0; y < img.height; y++) {
			j = y * img.width;
			for (int x = 0; x < img.width; x++) {
				k = j + x;
				img.bands[0].data[k] = buffer.array()[i++];
				img.bands[1].data[k] = buffer.array()[i++];
				img.bands[2].data[k] = buffer.array()[i++];
			}

		}
	}

	public ByteBuffer getImage() {
		return buffer;
	}

}
