/****************************************************************************
 *
 *   Copyright (c) 2021 Eike Mansfeld ecm@gmx.de. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

package com.comino.mavodometry.video.impl;

import java.awt.Color;
import java.awt.Graphics2D;


import org.mavlink.messages.MAV_SEVERITY;

import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.LogMessage;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavodometry.video.IOverlayListener;

public class DefaultOverlayListener extends AbstractOverlayListener implements IOverlayListener {
	
	private final int   LOWER_BOUND;
	private final int   UPPER_BOUND;
	private final int   TEXT_BOUND;

	private int width;
	private int height;

	public DefaultOverlayListener(int width, int height, DataModel model) {
		super(model);
		
		this.width   = width;
		this.height  = height;
		
		LOWER_BOUND  = height - 14;
		UPPER_BOUND  = height - 35;
		TEXT_BOUND   = height - 25;
	}



	@Override
	public void processOverlay(Graphics2D ctx, String stream_name, long tms) {

		ctx.setPaintMode();
		ctx.setColor(Color.white);
		
		// time
		ctx.setFont(big);
		ctx.drawLine(10,UPPER_BOUND,10, LOWER_BOUND);
		ctx.drawString(fminute.format(model.sys.t_armed_ms/60000)+fsecond.format(model.sys.t_armed_ms/1000f%60), 15 , TEXT_BOUND);
		ctx.setFont(small);
		ctx.drawString("armed",15,LOWER_BOUND);

		// altitude (currently LPOS-Z)
		ctx.drawLine(70,UPPER_BOUND,70, LOWER_BOUND);
		ctx.setFont(big);
		if(Float.isFinite(model.hud.ar))
		  ctx.drawString(onedecimal.format(model.hud.ar), 75, TEXT_BOUND);
		else
		  ctx.drawString("-", 75, TEXT_BOUND);
		ctx.setFont(small);
		ctx.drawString("altitude",75,LOWER_BOUND);

		// PX4 mode
		ctx.drawLine(120,UPPER_BOUND,120,LOWER_BOUND);
		ctx.setFont(big);
		if(model.sys.isStatus(Status.MSP_READY_FOR_FLIGHT)) {
			
			ctx.drawString(model.sys.getModeString(),125, TEXT_BOUND);
			ctx.setFont(small);
			ctx.drawString("mode",125,LOWER_BOUND);
		}
		else {
			ctx.drawString("NOT READY",125, TEXT_BOUND);
			ctx.setFont(small);
			ctx.drawString("status",125,LOWER_BOUND);
		}
		
		ctx.drawLine(200,UPPER_BOUND,200,LOWER_BOUND);
		ctx.setFont(big);
		if(Float.isFinite(model.battery.b0) )
		  ctx.drawString(onedecimal.format(model.battery.b0), 205, TEXT_BOUND);
		else
		  ctx.drawString("-", 205, TEXT_BOUND);
		ctx.setFont(small);
		ctx.drawString("battery",205,LOWER_BOUND);
		
		ctx.setFont(big);
		ctx.drawLine(245,UPPER_BOUND,245, LOWER_BOUND);
		if(model.msg.isNew(MAV_SEVERITY.MAV_SEVERITY_DEBUG,DataModel.getSynchronizedPX4Time_us())) {
			ctx.drawString(model.msg.text, 250, TEXT_BOUND);
			ctx.setFont(small);
			ctx.drawString(LogMessage.severity_texts[model.msg.severity].toLowerCase(),250,LOWER_BOUND);
		}  else {
			ctx.setFont(small);
			ctx.drawString("info",250,LOWER_BOUND);
		}

		// resize operator
		final int ln = 10;
		ctx.drawLine(width-25,height-ln,width-ln,height-ln);
		ctx.drawLine(width-ln,height-25,width-ln,height-ln);

		ctx.setFont(big);

	}

}
