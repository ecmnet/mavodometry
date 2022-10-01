package com.comino.mavodometry.video.impl;

import java.awt.Font;
import java.awt.Graphics2D;
import java.text.DecimalFormat;

import com.comino.mavcom.model.DataModel;
import com.comino.mavodometry.video.IOverlayListener;

public abstract class AbstractOverlayListener implements IOverlayListener {
	
	protected static final DecimalFormat fsecond    = new DecimalFormat("00.0");
	protected static final DecimalFormat fminute    = new DecimalFormat("00:");
	protected static final DecimalFormat faltitude  = new DecimalFormat("#0.0;#0.0-");

	protected static final Font          big        = new Font ("TAHOMA", Font.BOLD, 11);
	protected static final Font          small      = new Font ("TAHOMA", Font.PLAIN, 9);

	protected final DataModel model;
	
	public AbstractOverlayListener(DataModel model) {
		super();
		this.model = model;
	}

	@Override
	public abstract void processOverlay(Graphics2D ctx, String stream_name, long tms_usec); 

}
