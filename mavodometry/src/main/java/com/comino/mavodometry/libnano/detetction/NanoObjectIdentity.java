package com.comino.mavodometry.libnano.detetction;

import java.awt.Color;
import java.awt.Graphics;

import com.comino.mavodometry.utils.DepthUtils;

import boofcv.alg.distort.LensDistortionNarrowFOV;
import boofcv.alg.sfm.DepthSparse3D;
import boofcv.struct.ImageRectangle;
import boofcv.struct.image.GrayU16;
import georegression.metric.Intersection2D_I32;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;

public class NanoObjectIdentity {

	private static final int EXPIRE_MS  = 500;

	private int               id         = 0;
	private int               classid    = 0;					   // class of object
	private float             confidence = 0f;                     // Confidence
	private String            name       = null;                   // object name
	private Point3D_F64       pos_ned    = new Point3D_F64();      // local position NED
	private Point3D_F64       pos_body   = new Point3D_F64();      // local position

	public ImageRectangle     r          = new ImageRectangle();   // 2D rectangle of detected area

	private long tms = 0;

	private static final Color color = new Color(0.8f,0.5f,0.5f,0.5f);

	public NanoObjectIdentity() {

	};

	public NanoObjectIdentity(int id, int classid, float confidence, String name, int x0, int y0, int x1, int y1) {
		this.id = id;
		this.update(classid, confidence, name, x0,y0,x1,y1);
	};

	public void update(int classid, float confidence, String name, int x0, int y0, int x1, int y1) {
		this.classid    = classid;
		this.name       = name;
		this.confidence = confidence;
		this.tms        = System.currentTimeMillis();

		this.r.set(x0, y0, x1, y1);
	};


	public void draw(Graphics ctx) {

		if(!isValid())
			return;

		ctx.setColor(color);
		ctx.fillRect(r.x0, r.y0, r.getWidth(), r.getHeight());
		ctx.setColor(Color.WHITE);
	//	ctx.drawString(name, r.x0+5, r.y0+r.getHeight()-3);
		ctx.drawString(String.format("%.2f", confidence), r.x0+5, r.y0+r.getHeight()-3);

		if(pos_body.x < 12.0f) {
			ctx.drawString(String.format("%.1fm", pos_body.x),r.x0+5, r.y0+10);
			ctx.drawLine(r.x0+r.getWidth()/2-10, r.y0+r.getHeight()/2, r.x0+r.getWidth()/2+10, r.y0+r.getHeight()/2);
			ctx.drawLine(r.x0+r.getWidth()/2, r.y0+r.getHeight()/2-10, r.x0+r.getWidth()/2, r.y0+r.getHeight()/2+10);
		}
	}

	public void clear() {
		this.classid = 0;
	}

	public boolean isValid() {
		return this.classid > 0;
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

	public String toString() {
		return id+": "+pos_ned.toString();
	}

}
