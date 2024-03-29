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


package com.comino.mavodometry.video.impl.h264;

import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.BufferedOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;

import javax.imageio.IIOImage;
import javax.imageio.ImageIO;
import javax.imageio.ImageWriteParam;
import javax.imageio.ImageWriter;
import javax.imageio.plugins.jpeg.JPEGImageWriteParam;
import javax.imageio.stream.ImageOutputStream;

import org.jcodec.common.model.Picture;

import com.comino.mavcom.model.DataModel;
import com.comino.mavodometry.video.INoVideoListener;
import com.comino.mavodometry.video.IOverlayListener;
import com.comino.mavodometry.video.IVisualStreamHandler;
import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;

import boofcv.io.image.ConvertBufferedImage;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;

public class HttpH264Handler<T> implements HttpHandler, IVisualStreamHandler<T>  {

	private static final int 		MAX_VIDEO_RATE_MS     = 25;
	private static final float		DEFAULT_VIDEO_QUALITY = 0.5f;

	private final List<IOverlayListener> listeners;
	private final BufferedImage image;
	private final Graphics2D ctx;

	private boolean isReady = false;

	private boolean is_running = false;

	private long last_image_tms = 0;
	private float quality = DEFAULT_VIDEO_QUALITY;
	private IIOImage ioimage;
	
	private Picture pic;

	public HttpH264Handler(int width, int height, DataModel model) {
		this.listeners = new ArrayList<IOverlayListener>();
		this.image = new BufferedImage(width, height, BufferedImage.TYPE_3BYTE_BGR);
		this.ctx = image.createGraphics();
		this.ctx.setFont(new Font("SansSerif", Font.PLAIN, 11));
		this.ioimage = new IIOImage(image, null, null);
		ImageIO.setUseCache(false);
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

		he.getResponseHeaders().add("content-type","multipart/x-mixed-replace; boundary=--BoundaryString");
		he.sendResponseHeaders(200, 0);
		OutputStream os = new BufferedOutputStream(he.getResponseBody());

		is_running = true;

		ImageWriter writer = null;
		Iterator<ImageWriter> iter = ImageIO.getImageWritersByFormatName("jpg");
		if (iter.hasNext())
			writer = (ImageWriter) iter.next();
		
		ImageOutputStream ios = ImageIO.createImageOutputStream(os);
		writer.setOutput(ios);
		ImageWriteParam iwparam = new JPEGImageWriteParam(Locale.getDefault());
		iwparam.setCompressionMode(ImageWriteParam.MODE_EXPLICIT);
		iwparam.setCompressionQuality(quality);
		iwparam.setProgressiveMode(ImageWriteParam.MODE_DEFAULT);
		
	

		long tms = 0;
		while(is_running) {
			
			try {

				synchronized(this) {
					tms = System.currentTimeMillis();
					if(!isReady) {
						wait(2000);
					}	  
				}
			
				
				os.write(("--BoundaryString\r\nContent-type:image/jpeg content-length:1\r\n\r\n").getBytes());

				if((System.currentTimeMillis()-tms) >1950) {
					ctx.clearRect(0, 0, image.getWidth(), image.getHeight());
					ctx.drawString("No video available", image.getWidth()/2-40 , image.getHeight()/2);
//					if(listeners.size()>0) {
//						for(IOverlayListener listener : listeners)
//							listener.processOverlay(ctx, DataModel.getSynchronizedPX4Time_us());
//					}
					
					writer.write(null, ioimage , iwparam);
					os.write("\r\n\r\n".getBytes());
					os.flush();
					//is_running = false;
					continue;
				}

				
				if(listeners.size()>0) {
					for(IOverlayListener listener : listeners)
						listener.processOverlay(ctx, "none", DataModel.getSynchronizedPX4Time_us());
				}
				ioimage.setRenderedImage(image);
				
				writer.write(null, ioimage, iwparam);
		//		writer.write(null, new IIOImage(image, null, null), iwparam);
				os.write("\r\n\r\n".getBytes());

				isReady = false;

			} catch (Exception e) { is_running = false; }
		}
		writer.dispose(); 
		ios.flush();
		ios.close();
		he.close();
	}


	@Override
	public void registerOverlayListener(IOverlayListener listener) {
		this.listeners.add(listener);
	}

	@SuppressWarnings("unchecked")
	@Override
	public  void addToStream(String source, T input, DataModel model, long tms_us) {


		if((System.currentTimeMillis()-last_image_tms)<MAX_VIDEO_RATE_MS )//|| !model.sys.isStatus(Status.MSP_GCL_CONNECTED))
			return;
		last_image_tms = System.currentTimeMillis();

		synchronized(this) {
			
			if(input instanceof Planar) {
				ConvertBufferedImage.convertTo_U8((Planar<GrayU8>)input, image, true);
			}
			else if(input instanceof GrayU8)
				ConvertBufferedImage.convertTo((GrayU8)input, image, true);
			
			isReady = true;
			notify();
		}
	}

	@Override
	public float getFps() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public void enableStream(String string) {
		// TODO Auto-generated method stub
		
	}

}
