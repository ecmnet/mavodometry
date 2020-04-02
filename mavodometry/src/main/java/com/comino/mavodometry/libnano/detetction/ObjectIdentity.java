package com.comino.mavodometry.libnano.detetction;

import java.awt.Color;
import java.awt.Graphics;

import com.comino.mavodometry.libnano.detetction.helper.DistanceDetermination;

import boofcv.alg.distort.LensDistortionNarrowFOV;
import boofcv.alg.sfm.DepthSparse3D;
import boofcv.struct.image.GrayU16;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;

public class ObjectIdentity {

	private static final int EXPIRE_MS  = 100;

	private int               id         = 0;
	private int               classid    = 0;					   // class of object
	private float             confidence = 0f;                     // Confidence
	private String            name       = null;                   // object name
	private Point3D_F64       pos_ned    = new Point3D_F64();      // local position NED
	private Point3D_F64       pos_body   = new Point3D_F64();      // local position

	private int x;
	private int y;
	private int width;
	private int height;

	private long tms = 0;

	private final Color color = new Color(0.8f,0.5f,0.5f,0.5f);

	public ObjectIdentity() {

	};

	public ObjectIdentity(int id, int classid, float confidence, String name, int x, int y, int w, int h) {
		this.id = id;
		this.update(classid, confidence, name, x, y, w, h);
	};

	public void update(int classid, float confidence, String name, int x, int y, int w, int h) {
		this.classid    = classid;
		this.name       = name;
		this.confidence = confidence;
		this.x          = x;
		this.y          = y;
		this.width      = w;
		this.height     = h;
		this.tms        = System.currentTimeMillis();
	};

	public void update() {
		this.tms        = System.currentTimeMillis();
	};

	public void draw(Graphics ctx) {

		ctx.setColor(color);
		ctx.fillRect(x, y, width, height);
		ctx.setColor(Color.WHITE);
		ctx.drawString(name, x, y+10);

		if(pos_body.x < 12.0f) {
			ctx.drawString(String.format("%.1fm", pos_body.x),x+width-25, y+10);
			ctx.drawLine(x+width/2-10, y+height/2, x+width/2+10, y+height/2);
			ctx.drawLine(x+width/2, y+height/2-10, x+width/2, y+height/2+10);
		}

	}

	public boolean isExpired() {
		return (System.currentTimeMillis()-tms) > EXPIRE_MS;
	}

	public int getClassid() {
		return classid;
	}

	public String getName() {
		return name;
	}

	public Point3D_F64 getPosNED() {
		return pos_ned;
	}

	public Point3D_F64 getPosBODY() {
		return pos_body;
	}

	public float getConfidence() {
		return confidence;
	}

	public boolean equals(ObjectIdentity c) {
		return false;
	}

}
