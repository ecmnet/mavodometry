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
import java.awt.Graphics;
import java.text.DecimalFormat;

import org.mavlink.messages.MAV_SEVERITY;

import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavodometry.video.IOverlayListener;

public class DefaultOverlayListener implements IOverlayListener {
	
	private DataModel model;
	
	private final Color	bgColor_header    = new Color(128,128,128,130);
	private final DecimalFormat time      = new DecimalFormat("#0.0s");

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
			ctx.drawString(time.format(model.sys.t_armed_ms/1000f), 20, 20);
		}

		if(model.msg.isNew(MAV_SEVERITY.MAV_SEVERITY_INFO,tms)) {
			ctx.setColor(bgColor_header);
			ctx.fillRect(5, height-21, width-10, 19);
			ctx.setColor(Color.white);
			ctx.drawString(model.msg.text, 10, height-6);
		}
		
	}

}
