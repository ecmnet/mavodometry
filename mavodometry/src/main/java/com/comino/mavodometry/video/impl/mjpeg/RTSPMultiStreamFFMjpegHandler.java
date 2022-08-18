//package com.comino.mavodometry.video.impl.mjpeg;
//
//
//import static org.bytedeco.ffmpeg.global.avcodec.avcodec_open2;
//import static org.bytedeco.ffmpeg.global.avcodec.AV_CODEC_ID_MJPEG;
//import static org.bytedeco.ffmpeg.global.avcodec.avcodec_find_encoder;
//import static org.bytedeco.ffmpeg.global.avcodec.*;
//import static org.bytedeco.ffmpeg.global.avutil.*;
//
//import java.awt.Color;
//import java.awt.Font;
//import java.awt.Graphics2D;
//import java.awt.image.BufferedImage;
//import java.awt.image.DataBufferByte;
//import java.io.BufferedReader;
//import java.io.BufferedWriter;
//import java.io.IOException;
//import java.io.InputStreamReader;
//import java.io.OutputStreamWriter;
//import java.io.StringWriter;
//import java.net.DatagramPacket;
//import java.net.DatagramSocket;
//import java.net.InetAddress;
//import java.net.ServerSocket;
//import java.net.Socket;
//import java.net.SocketException;
//import java.util.ArrayList;
//import java.util.HashMap;
//import java.util.List;
//import java.util.Map;
//import java.util.StringTokenizer;
//import java.util.UUID;
//import java.util.concurrent.ArrayBlockingQueue;
//import java.util.concurrent.BlockingQueue;
//import java.util.concurrent.TimeUnit;
//
//import org.bytedeco.ffmpeg.avcodec.AVCodec;
//import org.bytedeco.ffmpeg.avcodec.AVCodecContext;
//import org.bytedeco.ffmpeg.avcodec.AVPacket;
//import org.bytedeco.ffmpeg.avformat.AVFormatContext;
//import org.bytedeco.ffmpeg.avutil.AVFrame;
//import org.bytedeco.ffmpeg.avutil.AVRational;
//import org.bytedeco.javacpp.BytePointer;
//import org.bytedeco.javacpp.PointerPointer;
//import org.libjpegturbo.turbojpeg.TJ;
//import org.libjpegturbo.turbojpeg.TJCompressor;
//import org.libjpegturbo.turbojpeg.TJException;
//
//import com.comino.mavcom.model.DataModel;
//import com.comino.mavodometry.video.INoVideoListener;
//import com.comino.mavodometry.video.IOverlayListener;
//import com.comino.mavodometry.video.IVisualStreamHandler;
//import com.comino.mavutils.rtps.RTPpacket;
//
//import boofcv.io.image.ConvertBufferedImage;
//import boofcv.struct.image.GrayU8;
//import boofcv.struct.image.Planar;
//import georegression.struct.point.Point2D_I32;
//
//
//public class RTSPMultiStreamFFMjpegHandler<T> implements  IVisualStreamHandler<T>  {
//
//	// Note: Relies on https://libjpeg-turbo.org
//
//	private static final int		DEFAULT_VIDEO_QUALITY = 70;
//	private static final int		MAX_VIDEO_QUALITY     = 90;
//	private static final int		LOW_VIDEO_QUALITY     = 40;
//
//	private static final int        THUMBNAIL_WIDTH        = 64*2;
//	private static final int        THUMBNAIL_HEIGHT       = 48*2;
//
//	private static int MJPEG_TYPE = 26; //RTP payload type for MJPEG video
//
//	//rtsp states
//	private final static int INIT = 0;
//	private final static int READY = 1;
//	private final static int PLAYING = 2;
//	//rtsp message types
//	private final static int SETUP = 3;
//	private final static int PLAY = 4;
//	private final static int PAUSE = 5;
//	private final static int TEARDOWN = 6;
//	private final static int DESCRIBE = 7;
//
//	private final List<IOverlayListener> listeners;
//	private final BufferedImage          image;
//	private final BufferedImage          image_thumb;
//	private final Graphics2D             ctx;
//
//
//	private long       last_image_tms    = 0;
//	private float      fps = 0;
//	private boolean    is_running = false;
//
//	private T          input;
//
//	private DatagramSocket RTPsocket;         //socket to be used to send and receive UDP packets
//	private DatagramPacket senddp;            //UDP packet containing the video frames
//
//	private InetAddress ClientIPAddr;         //Client IP address
//	private int RTP_dest_port = 0;            //destination port for RTP packets  (given by the RTSP Client)
//	private int RTSP_dest_port = 0;
//
//
//	private static BufferedReader RTSPBufferedReader;
//	private static BufferedWriter RTSPBufferedWriter;
//
//	private Socket RTSPsocket;                //socket used to send/receive RTSP messages
//
//	private static int state;                 //RTSP Server state == INIT or READY or PLAY
//	private int RTSPSeqNb = 0;                //Sequence number of RTSP messages within the session
//	private int imagenb = 0;                  //image nb of the image currently transmitted
//
//	private static String RTSPid = UUID.randomUUID().toString(); 
//	private final static String CRLF = "\r\n";
//
//
//	private int RTSPport = 1051;
//
//	private boolean done = false;
//
//	private final byte[] buffer;
//	private final byte[] packet_bits;
//
//	private boolean no_video;
//	private int quality = 0;
//
//	private final Receiver receiver;
//	private DataModel model;
//	
//	private final AVPacket pkt;
//	private AVCodecContext codec_ctx;
//	
//	private AVFrame raw;
//
//	private INoVideoListener no_video_handler;
//
//	private final Map<String,BlockingQueue<T>> transfers = new HashMap<String,BlockingQueue<T>>();
//
//
//	public RTSPMultiStreamFFMjpegHandler(int width, int height, DataModel model) {
//
//		this.model = model;
//		this.receiver = new Receiver(width,height);
//		this.listeners = new ArrayList<IOverlayListener>();
//		this.image = new BufferedImage(width, height, BufferedImage.TYPE_3BYTE_BGR);
//		this.image_thumb = new BufferedImage(width, height, BufferedImage.TYPE_3BYTE_BGR);
//		this.pkt  = new AVPacket();
//
//		this.ctx = image.createGraphics();
//		this.ctx.setFont(new Font("SansSerif", Font.PLAIN, 10));
//
//		this.buffer      = new byte[width*height*6];
//		this.packet_bits = new byte[RTPpacket.MAX_PAYLOAD];
//
//		last_image_tms = System.currentTimeMillis();
//		
//		
//		AVCodec codec = avcodec_find_encoder(AV_CODEC_ID_MJPEG);
//		if (codec == null) {
//			System.err.println("Codec not found");
//			return;
//		}
//		
//		av_new_packet(pkt,640*480);
//		
//		codec_ctx = avcodec_alloc_context3(codec);
//		codec_ctx.codec_id(codec.id());
//		codec_ctx.codec_type(AVMEDIA_TYPE_VIDEO);
//		
//		codec_ctx.width(width);
//		codec_ctx.height(height);
//		AVRational base = new AVRational(); base.num(1); base.den(15);
//		codec_ctx.time_base(base);
////		AVRational rate = new AVRational(); base.num(15); base.den(1);
////		codec_ctx.framerate(rate);
//      // http://blog.allenworkspace.net/2020/06/ffmpeg-decode-and-then-encode-frames-to.html?m=1
//		
//		codec_ctx.pix_fmt(AV_PIX_FMT_YUVJ420P);
//		
//		if(avcodec_open2(codec_ctx, codec, (PointerPointer)null)<0) {
//			System.err.println("Cannot open codec");
//			return;
//		}
//		
//		
//		
//		raw = av_frame_alloc();
//		raw.width(width);
//		raw.height(height);
//		raw.format(AV_PIX_FMT_YUV420P);
//		
//		BytePointer b = new BytePointer(((DataBufferByte)image.getRaster().getDataBuffer()).getData());
//		av_image_fill_arrays(raw.data(), raw.linesize(),
//				b, AV_PIX_FMT_YUV420P, codec_ctx.width(), codec_ctx.height(), 1);
//	
//		
//		
//
////  RGB/BGR is not supported !!!
////
////  NOTE: !!!!!!!!!!!!!!!!!
////  Encoder mjpeg [MJPEG (Motion JPEG)]:
////    General capabilities: threads
////    Threading capabilities: frame and slice
////    Supported pixel formats: yuvj420p yuvj422p yuvj444p
////    ===> TODO: Copy BOOFCV pic directly into frame and convert to yuv420p
////		
//		
//	
//
//	}
//
//	public void registerNoVideoListener(INoVideoListener no_video_handler) {
//		this.no_video_handler = no_video_handler;
//	}
//
//	public void stop() {
//		//		rtcpReceiver.stopRcv();
//		System.out.println("Video stopped");
//		is_running = false;
//	}
//
//	public void start(int RTSPport) throws Exception {
//		this.RTSPport = RTSPport;
//		new Thread(new Controller()).start();
//	}
//
//	public float getFps() {
//		return fps;
//	}
//
//	public void enableStream(String stream_name) {
//		receiver.enableStream(stream_name);
//		fps = 0;
//	}
//
//	@Override
//	public void  addToStream(String source, T in, DataModel model, long tms) {
//
//		BlockingQueue<T>  queue = transfers.get(source);
//		if(queue==null) {
//			queue = new ArrayBlockingQueue<T>(1);
//			transfers.put(source, queue);
//			System.out.println(source+" videostream created..");
//			return;
//		}
//		queue.offer(in);
//	}
//
//	@Override
//	public void registerOverlayListener(IOverlayListener listener) {
//		this.listeners.add(listener);			
//	}
//
//
//	private class Receiver implements Runnable {
//
//		private final int          width;
//		private final int          height;
//
//		private final Point2D_I32  p0;
//		private final Point2D_I32  p1;
//
//		private String[]           streams;
//		private BlockingQueue<T>   queue;
//
//		public Receiver(int width, int height) {
//
//			this.p0 = new Point2D_I32(width-THUMBNAIL_WIDTH-20,height-THUMBNAIL_HEIGHT-20);
//			this.p1 = new Point2D_I32(width-20, height - 20);
//			this.width = width;
//			this.height = height;
//
//		}
//
//		@SuppressWarnings("unchecked")
//		public void run() {
//
//			no_video = false;
//
//			System.out.println("Video streaming started ");
//			while(is_running) {
//
//				try {
//
//					if(RTPsocket.isClosed() || transfers == null || streams == null)
//						return;
//
//					queue = transfers.get(streams[0]);
//
//					try {
//						if(queue == null || (input = queue.poll(100, TimeUnit.MILLISECONDS)) == null) {
//							sendNoVideo();
//							Thread.sleep(100);
//							continue;
//						} 
//					}
//					catch(InterruptedException e) {
//						System.out.println(" Queue timeout");
//						sendNoVideo();
//						continue;
//					}
//
//					no_video = false;
//					imagenb++;
//
//					fps = ((fps * 59) + ((float)(1000f / (System.currentTimeMillis()-last_image_tms)))) / 60f;
//					last_image_tms = System.currentTimeMillis();
//
//
//					if(input instanceof Planar) {
//						ConvertBufferedImage.convertTo_U8(((Planar<GrayU8>)input), image, true);
//					}
//					else if(input instanceof GrayU8)
//						ConvertBufferedImage.convertTo((GrayU8)input, image, true);
//
//
//					if(listeners.size()>0) {
//						for(IOverlayListener listener : listeners)
//							listener.processOverlay(ctx, streams[0], DataModel.getSynchronizedPX4Time_us());
//					}
//
//					if(streams.length > 1) {
//						overlayThumbnail(transfers.get(streams[1]));
//					}
//					
//					BytePointer b = new BytePointer(((DataBufferByte)image.getRaster().getDataBuffer()).getData());
//					av_image_fill_arrays(raw.data(), raw.linesize(),
//							b, AV_PIX_FMT_YUVJ420P, codec_ctx.width(), codec_ctx.height(), 1);
//					
//					
//					codec_ctx.bit_rate((long)(300000+(400000-300000)*model.sys.wifi_quality));
//					
//					if(avcodec_send_frame(codec_ctx, raw)<0) {
//						System.err.println("Encode error (send)");
//					}
//					
//					if(avcodec_receive_packet(codec_ctx, pkt)<0) {
//						System.err.println("Encode error (receive)");
//					}
//					
//					if(pkt.size()>RTPpacket.MAX_PAYLOAD) {
//						
//						codec_ctx.bit_rate(300000);
//						if(avcodec_send_frame(codec_ctx, raw)<0) {
//							System.err.println("Encode error (send)");
//						}
//						
//						if(avcodec_receive_packet(codec_ctx, pkt)<0) {
//							System.err.println("Encode error (receive)");
//						}
//					}
//					
//					
//					System.out.println(pkt.size());
//					pkt.data().get(buffer, 0, pkt.size());
//
//					RTPpacket rtp_packet = new RTPpacket(MJPEG_TYPE, imagenb, (int)(imagenb*fps),buffer,pkt.size());
//					int packet_length = rtp_packet.getpacket(packet_bits);
//
//					//send the packet as a DatagramPacket over the UDP socket 
//					if(!RTPsocket.isClosed() ) { //&& packet_length < 65535) {
//						senddp = new DatagramPacket(packet_bits, packet_length, ClientIPAddr, RTP_dest_port);
//						RTPsocket.send(senddp);
//					}
//
//					
//					av_packet_unref(pkt);
//					
//					//		rtp_packet.printheader();
//
//				}
//				catch(Exception ex) {
//				    ex.printStackTrace();
//					try {
//						sendNoVideo();
//					} catch(Exception k) { }
//					continue;
//				}
//			}
//			close();
//
//		}
//
//		private void sendNoVideo() throws IOException {
//			if(!no_video) {
//				if(no_video_handler!= null)
//					no_video_handler.trigger();
//				no_video = true;
//			}
//			ctx.clearRect(0, 0, image.getWidth(), image.getHeight());
//			ctx.drawString("No video available", image.getWidth()/2-40 , image.getHeight()/2);
////			tj.compress(buffer, TJ.FLAG_PROGRESSIVE | TJ.FLAG_FASTDCT | TJ.FLAG_FASTUPSAMPLE | TJ.CS_RGB );
//			int size = 100;
//			RTPpacket rtp_packet = new RTPpacket(MJPEG_TYPE, imagenb, (int)(imagenb*fps), buffer, size);
//
//			int packet_length = rtp_packet.getpacket(packet_bits);
//
//			//send the packet as a DatagramPacket over the UDP socket 
//			if(!RTPsocket.isClosed()) {
//				senddp = new DatagramPacket(packet_bits, packet_length, ClientIPAddr, RTP_dest_port);
//				RTPsocket.send(senddp);
//			}
//		}
//
//		private void overlayThumbnail(BlockingQueue<T> q) {
//
//			if(q==null)
//				return;
//
//			if(q.isEmpty()) {
//				ctx.drawImage(image_thumb,p0.x,p0.y,p1.x,p1.y,0,0,width,height,null);
//				return;
//			}
//
//			T overlay = null;
//			try {
//				overlay = q.poll(1, TimeUnit.MILLISECONDS);
//			} catch(InterruptedException e) { 
//				ctx.drawImage(image_thumb,p0.x,p0.y,p1.x,p1.y,0,0,width,height,null);
//				return;
//			}
//
//			//      Too slow:
//			//		AverageDownSampleOps.down((Planar<GrayU8>)overlay, ov);
//
//			ConvertBufferedImage.convertTo_U8((Planar<GrayU8>)overlay, image_thumb, true);
//			ctx.drawImage(image_thumb,p0.x,p0.y,p1.x,p1.y,0,0,width,height,null);
//
//		}
//
//		public void enableStream(String stream_name) {
//			this.streams = stream_name.split("\\+");
//			fps = 0;
//		}
//
//	}
//
//
//	private int parseRequest() {
//		int request_type = -1;
//		try {
//			//parse request line and extract the request_type:
//			String RequestLine = RTSPBufferedReader.readLine();
//
//			System.out.println("RTSP Server - Received from Client:");
//			System.out.println(RequestLine);
//
//			StringTokenizer tokens = new StringTokenizer(RequestLine);
//			String request_type_string = tokens.nextToken();
//
//			//convert to request_type structure:
//			if ((new String(request_type_string)).compareTo("SETUP") == 0)
//				request_type = SETUP;
//			else if ((new String(request_type_string)).compareTo("PLAY") == 0)
//				request_type = PLAY;
//			else if ((new String(request_type_string)).compareTo("PAUSE") == 0)
//				request_type = PAUSE;
//			else if ((new String(request_type_string)).compareTo("TEARDOWN") == 0)
//				request_type = TEARDOWN;
//			else if ((new String(request_type_string)).compareTo("DESCRIBE") == 0)
//				request_type = DESCRIBE;
//
//			//parse the SeqNumLine and extract CSeq field
//			String SeqNumLine = RTSPBufferedReader.readLine();
//			System.out.println(SeqNumLine);
//			tokens = new StringTokenizer(SeqNumLine);
//			tokens.nextToken();
//			RTSPSeqNb = Integer.parseInt(tokens.nextToken());
//
//			//get LastLine
//			String LastLine = RTSPBufferedReader.readLine();
//			System.out.println(LastLine);
//
//			tokens = new StringTokenizer(LastLine);
//			if (request_type == SETUP) {
//				//extract RTP_dest_port from LastLine
//				for (int i=0; i<3; i++)
//					tokens.nextToken(); //skip unused stuff
//				RTP_dest_port = Integer.parseInt(tokens.nextToken());
//			}
//			else if (request_type == DESCRIBE) {
//				tokens.nextToken();
//				String describeDataType = tokens.nextToken();
//			}
//			else {
//				//otherwise LastLine will be the SessionId line
//				tokens.nextToken(); //skip Session:
//				RTSPid = tokens.nextToken();
//			}
//		} catch(Exception ex) {
//			close();
//		}
//
//		return(request_type);
//	}
//
//	public void close() {
//
//		if(is_running)
//			System.out.println("Closing video stream");
//
//		//		rtcpReceiver.stopRcv();
//
//		try {
//			RTSPBufferedReader.close();
//			RTSPBufferedWriter.close();
//			RTSPsocket.close();
//		} catch (IOException e) {
//			
//			e.printStackTrace();
//		}
//
//		RTPsocket.close();
//		imagenb = 0;
//
//		done = false;
//		state = INIT;
//
//		try {
//			Thread.sleep(200);
//		} catch (InterruptedException e) { }
//
//	}
//
//	// Creates a DESCRIBE response string in SDP format for current media
//	private String describe() {
//		StringWriter writer1 = new StringWriter();
//		StringWriter writer2 = new StringWriter();
//
//		// Write the body first so we can get the size later
//		writer2.write("v=0" + CRLF);
//		writer2.write("m=video " + RTSP_dest_port + " RTP/AVP " + MJPEG_TYPE + CRLF);
//		writer2.write("a=control:streamid=" + RTSPid + CRLF);
//		writer2.write("a=mimetype:string;\"video/MJPEG\"" + CRLF);
//		String body = writer2.toString();
//
//		writer1.write("Content-Base: LQUAC"+ CRLF);
//		writer1.write("Content-Type: " + "application/sdp" + CRLF);
//		writer1.write("Content-Length: " + body.length() + CRLF);
//		writer1.write(body);
//
//		return writer1.toString();
//	}
//
//	//------------------------------------
//	//Send RTSP Response
//	//------------------------------------
//	private void sendResponse() {
//		try {
//			RTSPBufferedWriter.write("RTSP/1.0 200 OK"+CRLF);
//			RTSPBufferedWriter.write("CSeq: "+RTSPSeqNb+CRLF);
//			RTSPBufferedWriter.write("Session: "+RTSPid+CRLF);
//			RTSPBufferedWriter.flush();
//			System.out.println("RTSP Server - Sent response to Client.");
//		} catch(Exception ex) {
//			System.out.println("Exception caught: "+ex);
//		}
//	}
//
//	private void sendDescribe() {
//		String des = describe();
//		try {
//			RTSPBufferedWriter.write("RTSP/1.0 200 OK"+CRLF);
//			RTSPBufferedWriter.write("CSeq: "+RTSPSeqNb+CRLF);
//			RTSPBufferedWriter.write(des);
//			RTSPBufferedWriter.flush();
//			System.out.println("RTSP Server - Sent response to Client.");
//		} catch(Exception ex) {
//			System.out.println("Exception caught: "+ex);
//		}
//	}
//
//	private class Controller implements Runnable {
//
//		@Override
//		public void run() {
//
//			int request_type;
//
//			while(true) {
//
//				try {
//
//					//Initiate TCP connection with the client for the RTSP session
//					ServerSocket listenSocket = new ServerSocket(RTSPport);
//					RTSPsocket = listenSocket.accept();
//					listenSocket.close();
//
//					//Get Client IP address
//					ClientIPAddr = RTSPsocket.getInetAddress();
//
//					//Initiate RTSPstate
//					state = INIT;
//
//					//Set input and output stream filters:
//					RTSPBufferedReader = new BufferedReader(new InputStreamReader(RTSPsocket.getInputStream()) );
//					RTSPBufferedWriter = new BufferedWriter(new OutputStreamWriter(RTSPsocket.getOutputStream()) );
//
//					while(!done) {
//						request_type = parseRequest(); //blocking
//						System.err.println(request_type);
//
//						if (request_type == SETUP) {
//							done = true;
//
//							//update RTSP state
//							state = READY;
//							System.out.println("New RTSP state: READY");
//
//							//Send response
//							sendResponse();
//
//							//init RTP
//							try {
//								RTPsocket = new DatagramSocket();
//								RTPsocket.setSendBufferSize(512*1024);
//								RTPsocket.setTrafficClass(0x08);
//							} catch (SocketException e) {
//								e.printStackTrace();
//								done = false;
//							}
//						}
//					}
//
//					while(done) {
//
//						request_type = parseRequest(); //blocking
//
//						if ((request_type == PLAY) && (state == READY)) {
//							//send back response
//							sendResponse();
//							//start timer
//							//						rtcpReceiver.startRcv();
//							is_running = true;
//							new Thread(receiver).start();
//							state = PLAYING;
//							System.out.println("New RTSP state: PLAYING");
//						}
//						else if ((request_type == PAUSE) && (state == PLAYING)) {
//							//send back response
//							sendResponse();
//							//stop timer
//							//							rtcpReceiver.stopRcv();
//							//update state
//							state = READY;
//							System.out.println("New RTSP state: READY");
//						}
//						else if (request_type == TEARDOWN) {
//							//send back response
//							sendResponse();
//							close();
//							is_running = false;
//						}
//						else if (request_type == DESCRIBE) {
//							System.out.println("Received DESCRIBE request");
//							sendDescribe();
//						}
//					}
//				} catch(Exception e) {
//					try {
//						RTSPsocket.close();
//					} catch (IOException e1) {
//						
//						e1.printStackTrace();
//					}
//					e.printStackTrace();
//					is_running = false;
//				}
//			}
//
//		}
//
//	}	
//
//	public String toString() {
//		return "Frames: "+imagenb+" => "+fps+"fps => Quality: "+quality;
//	}
//
//
//	public static void main(String argv[]) throws Exception
//	{
//		//create a Server object
//		RTSPMultiStreamFFMjpegHandler<Planar<GrayU8>> server = new RTSPMultiStreamFFMjpegHandler<Planar<GrayU8>>(640,480, new DataModel());
//
//		Planar<GrayU8> test = new Planar<GrayU8>(GrayU8.class, 640,480,3);
//		DataModel model = new DataModel();
//
//
//		long tms = System.currentTimeMillis();
//
//		server.registerOverlayListener((ctx,n,t) -> {
//
//			ctx.setColor(Color.WHITE);
//			ctx.drawString("TMS="+t, 20, 20);
//
//			ctx.setColor(Color.getHSBColor((float)Math.random(), (float)Math.random(), (float)Math.random()));
//			ctx.drawString("TMS="+t, (int)(Math.random()*70)+80, (int)(Math.random()*70)+80);
//
//		});
//
//		server.start(1051);
//
//		while(true) {
//
//			Thread.sleep(33);
//			tms = System.currentTimeMillis();
//			server.addToStream("test",test, model, tms*1000);
//			if(server.is_running)
//				System.out.println(server.toString());
//
//
//		}
//
//
//	}
//
//
//
//}
