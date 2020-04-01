/****************************************************************************
 *
 *   Copyright (c) 2017 Eike Mansfeld ecm@gmx.de. All rights reserved.
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

import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.BufferedOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.List;

import javax.imageio.ImageIO;

import com.comino.mavcom.model.DataModel;
import com.comino.mavodometry.video.IOverlayListener;
import com.comino.mavodometry.video.IVisualStreamHandler;
import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;

import boofcv.io.image.ConvertBufferedImage;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;

public class HttpMJPEGHandler<T> implements HttpHandler, IVisualStreamHandler<T>  {

	private static final int MAX_VIDEO_RATE_MS = 50;

	private List<IOverlayListener> listeners = null;
	private BufferedImage image = null;
	private DataModel model = null;
	private Graphics2D ctx;

	private T input_image;
	private boolean is_running = false;

	private long last_image_tms = 0;

	public HttpMJPEGHandler(int width, int height, DataModel model) {
		this.model = model;
		this.listeners = new ArrayList<IOverlayListener>();
		this.image = new BufferedImage(width, height, BufferedImage.TYPE_3BYTE_BGR);
		this.ctx = image.createGraphics();
		this.ctx.setFont(new Font("Monospaced", Font.PLAIN, 10));

		ImageIO.setUseCache(false);

	}

	public void stop() {
		is_running = false;
	}

	@Override @SuppressWarnings("unchecked")
	public void handle(HttpExchange he) throws IOException {
        is_running = true;
		he.getResponseHeaders().add("content-type","multipart/x-mixed-replace; boundary=--BoundaryString");
		he.sendResponseHeaders(200, 0);
		OutputStream os = new BufferedOutputStream(he.getResponseBody());
		
		System.out.println("Start streaming vision data...");

		while(is_running) {

			try {

				synchronized(this) {
				  if(input_image==null)
						wait();
				}

				os.write(("--BoundaryString\r\nContent-type:image/jpeg content-length:1\r\n\r\n").getBytes());

				if(input_image instanceof Planar) {
					ConvertBufferedImage.convertTo_U8((Planar<GrayU8>)input_image, image, true);
				}
				else if(input_image instanceof GrayU8)
					ConvertBufferedImage.convertTo((GrayU8)input_image, image, true);

				if(listeners.size()>0) {
					for(IOverlayListener listener : listeners)
						listener.processOverlay(ctx);
				}

				ImageIO.write(image, "jpg", os );
				os.write("\r\n\r\n".getBytes());

				input_image = null;

			} catch (Exception e) { is_running = false; }
		}
        he.close();
	}

	@Override
	public void registerOverlayListener(IOverlayListener listener) {
		this.listeners.add(listener);
	}

	@Override
	public  void addToStream(T input, DataModel model, long tms_us) {

		if((System.currentTimeMillis()-last_image_tms)<MAX_VIDEO_RATE_MS )//|| !model.sys.isStatus(Status.MSP_GCL_CONNECTED))
			return;
		last_image_tms = System.currentTimeMillis();


		synchronized(this) {
			input_image = input;
			notify();
		}

	}
}
