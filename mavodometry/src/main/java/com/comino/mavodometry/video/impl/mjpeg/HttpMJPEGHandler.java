/****************************************************************************
 *
 *   Copyright (c) 2017,2021 Eike Mansfeld ecm@gmx.de. All rights reserved.
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


package com.comino.mavodometry.video.impl.mjpeg;

import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.BufferedOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.List;

import org.libjpegturbo.turbojpeg.TJ;
import org.libjpegturbo.turbojpeg.TJCompressor;
import org.libjpegturbo.turbojpeg.TJException;

import com.comino.mavcom.model.DataModel;
import com.comino.mavodometry.video.INoVideoListener;
import com.comino.mavodometry.video.IOverlayListener;
import com.comino.mavodometry.video.IVisualStreamHandler;
import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;

import boofcv.io.image.ConvertBufferedImage;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;


public class HttpMJPEGHandler<T> implements HttpHandler, IVisualStreamHandler<T>  {

	private static final int 		FRAME_DROP             = 2;
	private static final int		DEFAULT_VIDEO_QUALITY = 60;
	private static final int		LOW_VIDEO_QUALITY     = 10;

	private final List<IOverlayListener> listeners;
	private final BufferedImage image;
	private final Graphics2D ctx;

	private final byte[] header = "--BoundaryString\r\nContent-type:image/jpeg content-length:1\r\n\r\n".getBytes();

	private boolean isReady = false;

	private boolean is_running = false;

	private boolean no_video   = false;

	private long last_image_tms = 0;
	private float  fps = 0;

	private TJCompressor tj;
	private final byte[] buffer;

	private T input;
	private final DataModel model;

	private long frame_count;
	private int  quality;

	public HttpMJPEGHandler(int width, int height, DataModel model) {
		this.model = model;
		this.listeners = new ArrayList<IOverlayListener>();
		this.image = new BufferedImage(width, height, BufferedImage.TYPE_3BYTE_BGR);
		this.ctx = image.createGraphics();
		this.ctx.setFont(new Font("SansSerif", Font.PLAIN, 11));
		this.buffer = new byte[width*height*6];
		last_image_tms = System.currentTimeMillis();

		try {
			tj = new TJCompressor();
			tj.setSubsamp(TJ.SAMP_420);
			tj.setSourceImage(image, 0, 0, 0, 0);
		} catch (TJException e1) {
			// TODO Fallback tio ImageIO
			e1.printStackTrace();
		}


	}
	
	public void registerNoVideoListener(INoVideoListener no_video_handler) {
		
	}

	public void stop() {
		is_running = false;
	}

	@Override @SuppressWarnings("unchecked")
	public void handle(HttpExchange he) throws IOException {


		if(is_running) {
			return;
		}
		
		frame_count = 0;

		System.out.println("Videostreaming started");

		he.getResponseHeaders().add("content-type","multipart/x-mixed-replace; boundary=--BoundaryString");
		he.sendResponseHeaders(200, 0);

		is_running = true;

		OutputStream ios = new BufferedOutputStream(he.getResponseBody(),30000);

		long tms = 0; 
		while(is_running) {

			try {

				synchronized(this) {
					tms = System.currentTimeMillis();
					if(!isReady) {
						wait(2000);
					}	  
				}

				isReady = false;
				
				quality = LOW_VIDEO_QUALITY + (int)((DEFAULT_VIDEO_QUALITY - LOW_VIDEO_QUALITY) * model.sys.wifi_quality);

				tj.setJPEGQuality(quality);
				ios.write(header);

				if((System.currentTimeMillis()-tms) >1950 || quality == 10 ) {
					if(!no_video) {
						no_video = true;
						ctx.clearRect(0, 0, image.getWidth(), image.getHeight());
						ctx.drawString("No video available", image.getWidth()/2-40 , image.getHeight()/2);
						tj.compress(buffer, TJ.FLAG_PROGRESSIVE | TJ.FLAG_FASTDCT | TJ.FLAG_FASTUPSAMPLE);
						ios.write(buffer, 0, tj.getCompressedSize());
						ios.write("\r\n\r\n".getBytes());
						is_running = false;
					}
					continue;
				}


				synchronized(this) {
					if(input instanceof Planar) {
						ConvertBufferedImage.convertTo_U8((Planar<GrayU8>)input, image, true);
					}
					else if(input instanceof GrayU8)
						ConvertBufferedImage.convertTo((GrayU8)input, image, true);


					if(listeners.size()>0) {
						for(IOverlayListener listener : listeners)
							listener.processOverlay(ctx, DataModel.getSynchronizedPX4Time_us());
					}
				}

				no_video = false;

				tj.compress(buffer, TJ.FLAG_PROGRESSIVE | TJ.FLAG_FASTDCT | TJ.FLAG_FASTUPSAMPLE);
				ios.write(buffer, 0, tj.getCompressedSize());
				ios.write("\r\n\r\n".getBytes());
				
				fps = ((fps * 59) + ((float)(1000f / (System.currentTimeMillis()-last_image_tms)))) /60f;
				last_image_tms = System.currentTimeMillis();

			} catch (InterruptedException i) {  
				  System.out.println(i.getMessage());
			} catch (Exception e) { is_running = false; }
		}

		ios.flush();
		ios.close();
		he.close();
	}


	@Override
	public void registerOverlayListener(IOverlayListener listener) {
		this.listeners.add(listener);
	}

	@Override
	public void addToStream(T in, DataModel model, long tms_us) {

		if(++frame_count % FRAME_DROP == 0 || !is_running)
			return;

		input = in;
		synchronized(this) {
			isReady = true;
			notify();
		}
	}

	public float getFps() {
		return fps;
	}
	
	public String toString() {
		return "Frames: "+frame_count+" =>\t "+fps+"fps \t Quality: "+quality;
	}

}
