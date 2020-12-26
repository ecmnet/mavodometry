package com.comino.mavodometry.video.impl;

import java.awt.Color;
import java.awt.Graphics;

import org.mavlink.messages.MAV_SEVERITY;

import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavodometry.video.IOverlayListener;

public class DefaultOverlayListener implements IOverlayListener {
	
	private DataModel model;
	
	
	private final Color	bgColor_header    = new Color(128,128,128,130);


	private int width;


	private int height;
	
	public DefaultOverlayListener(int width, int height, DataModel model) {
		super();
		this.model = model;
		this.width = width;
		this.height = height;
	}



	@Override
	public void processOverlay(Graphics ctx, long tms) {
		
		ctx.setColor(bgColor_header);
		ctx.fillRect(5, 5, width-10, 21);
		
		ctx.drawLine(100, height/2, width/2-20, height/2); ctx.drawLine(width/2+20, height/2, width-100, height/2);
		ctx.drawLine(width/2, 100, width/2, height/2-20); ctx.drawLine(width/2, height/2+20, width/2, height-100);

		ctx.setPaintMode();
		ctx.setColor(Color.white);
		

		if(!Float.isNaN(model.sys.t_armed_ms) && model.sys.isStatus(Status.MSP_ARMED)) {
			ctx.drawString(String.format("%.1fs",model.sys.t_armed_ms/1000f), 20, 20);
		}


		if(model.msg.isNew(MAV_SEVERITY.MAV_SEVERITY_INFO,tms)) {
			ctx.setColor(bgColor_header);
			ctx.fillRect(5, height-21, width-10, 19);
			ctx.setColor(Color.white);
			ctx.drawString(model.msg.text, 10, height-8);
		}
		
	}

}
