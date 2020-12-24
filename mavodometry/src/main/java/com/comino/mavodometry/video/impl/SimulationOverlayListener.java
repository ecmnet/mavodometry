package com.comino.mavodometry.video.impl;

import java.awt.Color;
import java.awt.Graphics;

import org.mavlink.messages.MAV_SEVERITY;

import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavodometry.video.IOverlayListener;

public class SimulationOverlayListener implements IOverlayListener {
	
	private DataModel model;
	
	
	private final Color	bgColor_header    = new Color(128,128,128,130);


	private int width;


	private int height;
	
	public SimulationOverlayListener(int width, int height, DataModel model) {
		super();
		this.model = model;
		this.width = width;
		this.height = height;
	}



	@Override
	public void processOverlay(Graphics ctx, long tms) {
		
		ctx.setColor(bgColor_header);
		ctx.fillRect(5, 5, width-10, 21);
		
		ctx.setColor(Color.white);
		
		if(!Float.isNaN(model.sys.t_armed_ms) && model.sys.isStatus(Status.MSP_ARMED)) {
			ctx.drawString(String.format("%.1fsec",model.sys.t_armed_ms/1000f), 20, 20);
		}
		
		if(model.msg.isNew(MAV_SEVERITY.MAV_SEVERITY_INFO,tms))
			ctx.drawString(model.msg.text, 10, height-5);
		
	}

}
