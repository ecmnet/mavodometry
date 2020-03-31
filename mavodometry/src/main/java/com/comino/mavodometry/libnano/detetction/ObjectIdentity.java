package com.comino.mavodometry.libnano.detetction;

import java.awt.Color;
import java.awt.Graphics;

import boofcv.struct.image.GrayU16;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;

public class ObjectIdentity {

	private int               classid = 0;						// class of object
	private float             confidence = 0f;                  // Confidence
	private String            name    = null;                   // object name
	private Vector3D_F64      pos_ned = new Vector3D_F64();     // local position

	private int x;
	private int y;
	private int width;
	private int height;

	private float distance;

	private final Color color = new Color(0.8f,0.5f,0.5f,0.5f);

	public ObjectIdentity(int classid, float confidence, String name, int x, int y, int w, int h) {
		this.classid    = classid;
		this.name       = name;
		this.confidence = confidence;
		this.x          = x;
		this.y          = y;
		this.width      = w;
		this.height     = h;
	};

	public void processPosition(GrayU16 sub_depth, Se3_F64 to_ned) {

	}

	public void draw(Graphics ctx) {
		ctx.drawString(name, x, y+10);
		ctx.setColor(color);
        ctx.fillRect(x, y, width, height);
	}

	public int getClassid() {
		return classid;
	}

	public String getName() {
		return name;
	}

	public Vector3D_F64 getPosistion() {
		return pos_ned;
	}

	public float getConfidence() {
	    return confidence;
	}

	public float getDistance() {
	    return distance;
	}

	   // Idea: Built histogramm of 100 depth classes. Return depth of major class
//		private float getDepthOfObject(Result result,GrayU16 depth) {
//			int depth_z = Integer.MAX_VALUE; int raw_z;
//			if(result.Confidence < 0.1 || result.ClassID == 0)
//				return Float.MAX_VALUE;
//			for(int x = (int)result.Left; x < (int)result.Right;x++) {
//				for(int y = (int)result.Top; y < (int)result.Bottom;y++) {
//					raw_z = depth.get(x, y);
//					if(raw_z > 20 && raw_z < depth_z && raw_z < 15000) {
//						depth_z =  raw_z;
//					}
//				}
//			}
//			return depth_z/1000f;
//		}


}
