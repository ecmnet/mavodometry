package com.comino.mavodometry.libnano.detetction.helper;

import boofcv.struct.ImageRectangle;
import boofcv.struct.image.GrayU16;

// Histogram based distance determination of surrounding box
// 100 classes (0.3 to 9.9m)

public class NanoObjectUtils {

	public static int[] dist_classes  = new int[100];

	public static int process(GrayU16 sub_depth) {

		int raw_z = 0;
		int ymid = sub_depth.height / 2;
		int xmid = sub_depth.width / 2;

		int x0 = xmid - sub_depth.width / 4;
		int x1 = x0 + xmid;

		int y0 = ymid - sub_depth.height / 3;
		int y1 = ymid;

		for(int i=0; i< dist_classes.length;i++)
			dist_classes[i] = 0;

		for(int x = x0; x < x1; x++) {
			for(int y = y0; y < y1; y++) {
				raw_z = sub_depth.get(x, y) / 100 - 3;
				if(raw_z >= 0 && raw_z < 100) {
					dist_classes[raw_z]++;
				}
			}
		}

		int current_max = 0; int max_class = -1;
		for(int i=0; i< dist_classes.length;i++) {
			if(dist_classes[i] > current_max) {
				current_max = dist_classes[i];
				max_class = i;
			}
		}

		if(max_class != -1)
			return ( max_class + 2 ) * 100;
		return Integer.MAX_VALUE;

	}


}
