package com.comino.mavodometry.video.impl.mjpeg;

import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.InterruptedIOException;
import java.io.OutputStreamWriter;
import java.io.StringWriter;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;
import java.util.StringTokenizer;
import java.util.UUID;

import javax.imageio.IIOImage;
import javax.imageio.ImageIO;
import javax.imageio.ImageWriteParam;
import javax.imageio.ImageWriter;
import javax.imageio.plugins.jpeg.JPEGImageWriteParam;
import javax.imageio.stream.ImageOutputStream;
import javax.swing.Timer;

import com.comino.mavcom.model.DataModel;
import com.comino.mavodometry.video.IOverlayListener;
import com.comino.mavodometry.video.IVisualStreamHandler;
import com.comino.mavutils.rtps.RTCPpacket;
import com.comino.mavutils.rtps.RTPpacket;

import boofcv.io.image.ConvertBufferedImage;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;

public class RTPSMjpegHandler<T> implements  IVisualStreamHandler<T> , Runnable, ActionListener {


	private static final int 		VIDEO_RATE_MS         = 40;
	private static final float		DEFAULT_VIDEO_QUALITY = 0.6f;
	private static final float		LOW_VIDEO_QUALITY     = 0.2f;
	private static final float      LOW_VIDEO_THERSHOLD   = 0.50f;

	private static int MJPEG_TYPE = 26; //RTP payload type for MJPEG video

	//rtsp states
	private final static int INIT = 0;
	private final static int READY = 1;
	private final static int PLAYING = 2;
	//rtsp message types
	private final static int SETUP = 3;
	private final static int PLAY = 4;
	private final static int PAUSE = 5;
	private final static int TEARDOWN = 6;
	private final static int DESCRIBE = 7;

	private final List<IOverlayListener> listeners;
	private final BufferedImage          image;
	private final Graphics2D             ctx;

	private long       last_image_tms = 0;
	private float      fps = 0;
	private boolean    is_ready = false;
	private boolean    is_running = false;

	private IIOImage   ioimage;
	private T          input;
	
	private ImageWriteParam iwparam = new JPEGImageWriteParam(Locale.getDefault());


	private DatagramSocket RTPsocket;         //socket to be used to send and receive UDP packets
	private DatagramPacket senddp;            //UDP packet containing the video frames

	private InetAddress ClientIPAddr;         //Client IP address
	private int RTP_dest_port = 0;            //destination port for RTP packets  (given by the RTSP Client)
	private int RTSP_dest_port = 0;

	private static int RTCP_RCV_PORT = 19001; //port where the client will receive the RTP packets
	private static int RTCP_PERIOD   = 400;   //How often to check for control events
	private DatagramSocket RTCPsocket;
	private RtcpReceiver rtcpReceiver;
	private int congestionLevel;

	private static BufferedReader RTSPBufferedReader;
	private static BufferedWriter RTSPBufferedWriter;
	private Socket RTSPsocket; //socket used to send/receive RTSP messages

	private Timer timer;                      //timer used to send the images at the video frame rate
	private int sendDelay;                    //the delay to send images over the wire. Ideally should be
	//equal to the frame rate of the video file, but may be 
	//adjusted when congestion is detected.
	private static int state;                 //RTSP Server state == INIT or READY or PLAY
	private int RTSPSeqNb = 0;                //Sequence number of RTSP messages within the session
	private int imagenb = 0; //image nb of the image currently transmitted

	private static String RTSPid = UUID.randomUUID().toString(); 
	private final static String CRLF = "\r\n";
	
	private ImageWriter writer;
	private ByteArrayOutputStream baos ;
	private ImageOutputStream ios;
	
	private int RTSPport = 1051;
	
	private boolean done = false;
	
	private byte[] packet_bits = new byte[32768];

	public RTPSMjpegHandler(int width, int height) {

		this.listeners = new ArrayList<IOverlayListener>();
		this.image = new BufferedImage(width, height, BufferedImage.TYPE_3BYTE_BGR);
		this.ctx = image.createGraphics();
		this.ctx.setFont(new Font("SansSerif", Font.PLAIN, 11));
		this.ioimage = new IIOImage(image, null, null);
		ImageIO.setUseCache(false);

		last_image_tms = System.currentTimeMillis();
		
		iwparam.setCompressionMode(ImageWriteParam.MODE_EXPLICIT);
		iwparam.setCompressionQuality(DEFAULT_VIDEO_QUALITY);
		iwparam.setProgressiveMode(ImageWriteParam.MODE_DEFAULT);
		
		sendDelay = VIDEO_RATE_MS;
		timer = new Timer(sendDelay, this);
		timer.setInitialDelay(0);
		timer.setCoalesce(true);
		
		rtcpReceiver = new RtcpReceiver(RTCP_PERIOD);
		
        try {
        	Iterator<ImageWriter> iter = ImageIO.getImageWritersByFormatName("jpg");
    		if (iter.hasNext())
    			writer = (ImageWriter) iter.next();
    		else
    			writer = null;
    		
    	    baos =  new ByteArrayOutputStream(32768);
			ios = ImageIO.createImageOutputStream(baos);			
			writer.setOutput(ios);
			
		} catch (Exception e) {
			 ios = null;
			e.printStackTrace();
		}
	}

	public void stop() {
		timer.stop();
		rtcpReceiver.stopRcv();
		is_running = false;
	}

	public void start(int RTSPport) throws Exception {
		this.RTSPport = RTSPport;
		is_running = true;
		new Thread(this).start();

	}


	@Override
	public void addToStream(T in, DataModel model, long tms_us) {
		if((System.currentTimeMillis()-last_image_tms) < VIDEO_RATE_MS || !is_running)
			return;

		fps = (fps * 0.7f) + ((float)(1000f / (System.currentTimeMillis()-last_image_tms)) * 0.3f);

		synchronized(this) {
			input = in;
			is_ready = true;
			last_image_tms = System.currentTimeMillis();
		}
	}

	@Override
	public void registerOverlayListener(IOverlayListener listener) {
		this.listeners.add(listener);			
	}

	public void actionPerformed(ActionEvent e) {
		byte[] frame; int image_length;

		//if the current image nb is less than the length of the video
		imagenb++;

		try {
			
			if(input instanceof Planar) {
				ConvertBufferedImage.convertTo_U8((Planar<GrayU8>)input, image, true);
			}
			else if(input instanceof GrayU8)
				ConvertBufferedImage.convertTo((GrayU8)input, image, true);


			if(listeners.size()>0) {
				for(IOverlayListener listener : listeners)
					listener.processOverlay(ctx, DataModel.getSynchronizedPX4Time_us());
			}
			
		    baos.reset();

			//adjust quality of the image if there is congestion detected
			if (congestionLevel > 0) {
				iwparam.setCompressionQuality(DEFAULT_VIDEO_QUALITY - congestionLevel * 0.1f);
			} else {
				iwparam.setCompressionQuality(LOW_VIDEO_QUALITY);
			}
			
			writer.write(null, ioimage, iwparam);
			frame = baos.toByteArray();
			image_length = frame.length;

			//Builds an RTPpacket object containing the frame
			RTPpacket rtp_packet = new RTPpacket(MJPEG_TYPE, imagenb, imagenb*VIDEO_RATE_MS, frame, image_length);

			//get to total length of the full rtp packet to send
			int packet_length = rtp_packet.getlength();

			//retrieve the packet bitstream and store it in an array of bytes
//			byte[] packet_bits = new byte[packet_length];
			rtp_packet.getpacket(packet_bits);

			//send the packet as a DatagramPacket over the UDP socket 
			senddp = new DatagramPacket(packet_bits, packet_length, ClientIPAddr, RTP_dest_port);
			RTPsocket.send(senddp);

//			//print the header bitstreaSym
//			rtp_packet.printheader();

		}
		catch(Exception ex) {
			timer.stop();
			rtcpReceiver.stopRcv();

		}

	}

	private class CongestionController implements ActionListener {
		private Timer ccTimer;
		int interval;   //interval to check traffic stats
		int prevLevel;  //previously sampled congestion level

		public CongestionController(int interval) {
			this.interval = interval;
			ccTimer = new Timer(interval, this);
			ccTimer.start();
		}

		public void actionPerformed(ActionEvent e) {

			//adjust the send rate
			if (prevLevel != congestionLevel) {
				sendDelay = VIDEO_RATE_MS + congestionLevel * (int)(VIDEO_RATE_MS * 0.1);
				timer.setDelay(sendDelay);
				prevLevel = congestionLevel;
				System.out.println("Send delay changed to: " + sendDelay);
			}
		}
	}


	private class RtcpReceiver implements ActionListener {
		private Timer rtcpTimer;
		private byte[] rtcpBuf;
		int interval;

		public RtcpReceiver(int interval) {
			//set timer with interval for receiving packets
			this.interval = interval;
			rtcpTimer = new Timer(interval, this);
			rtcpTimer.setInitialDelay(0);
			rtcpTimer.setCoalesce(true);

			//allocate buffer for receiving RTCP packets
			rtcpBuf = new byte[512];
		}

		public void actionPerformed(ActionEvent e) {
			//Construct a DatagramPacket to receive data from the UDP socket
			DatagramPacket dp = new DatagramPacket(rtcpBuf, rtcpBuf.length);
			float fractionLost;

			try {
				RTCPsocket.receive(dp);   // Blocking
				RTCPpacket rtcpPkt = new RTCPpacket(dp.getData(), dp.getLength());
			//	System.out.println("[RTCP] " + rtcpPkt);

				//set congestion level between 0 to 4
				fractionLost = rtcpPkt.fractionLost;
				if (fractionLost >= 0 && fractionLost <= 0.01) {
					congestionLevel = 0;    //less than 0.01 assume negligible
				}
				else if (fractionLost > 0.01 && fractionLost <= 0.25) {
					congestionLevel = 1;
				}
				else if (fractionLost > 0.25 && fractionLost <= 0.5) {
					congestionLevel = 2;
				}
				else if (fractionLost > 0.5 && fractionLost <= 0.75) {
					congestionLevel = 3;
				}
				else {
					congestionLevel = 4;
				}
			}
			catch (InterruptedIOException iioe) {
				
			}
			catch (IOException ioe) {
				
			}
		}

		public void startRcv() {
			rtcpTimer.start();
		}

		public void stopRcv() {
			rtcpTimer.stop();
		}
	}

	private int parseRequest() {
		int request_type = -1;
		try { 
			//parse request line and extract the request_type:
			String RequestLine = RTSPBufferedReader.readLine();
			System.out.println("RTSP Server - Received from Client:");
			System.out.println(RequestLine);

			StringTokenizer tokens = new StringTokenizer(RequestLine);
			String request_type_string = tokens.nextToken();

			//convert to request_type structure:
			if ((new String(request_type_string)).compareTo("SETUP") == 0)
				request_type = SETUP;
			else if ((new String(request_type_string)).compareTo("PLAY") == 0)
				request_type = PLAY;
			else if ((new String(request_type_string)).compareTo("PAUSE") == 0)
				request_type = PAUSE;
			else if ((new String(request_type_string)).compareTo("TEARDOWN") == 0)
				request_type = TEARDOWN;
			else if ((new String(request_type_string)).compareTo("DESCRIBE") == 0)
				request_type = DESCRIBE;

			if (request_type == SETUP) {
				//extract VideoFileName from RequestLine
				//VideoFileName = tokens.nextToken();
			}

			//parse the SeqNumLine and extract CSeq field
			String SeqNumLine = RTSPBufferedReader.readLine();
			System.out.println(SeqNumLine);
			tokens = new StringTokenizer(SeqNumLine);
			tokens.nextToken();
			RTSPSeqNb = Integer.parseInt(tokens.nextToken());

			//get LastLine
			String LastLine = RTSPBufferedReader.readLine();
			System.out.println(LastLine);

			tokens = new StringTokenizer(LastLine);
			if (request_type == SETUP) {
				//extract RTP_dest_port from LastLine
				for (int i=0; i<3; i++)
					tokens.nextToken(); //skip unused stuff
				RTP_dest_port = Integer.parseInt(tokens.nextToken());
			}
			else if (request_type == DESCRIBE) {
				tokens.nextToken();
				String describeDataType = tokens.nextToken();
			}
			else {
				//otherwise LastLine will be the SessionId line
				tokens.nextToken(); //skip Session:
				RTSPid = tokens.nextToken();
			}
		} catch(Exception ex) {
			timer.stop();
	        rtcpReceiver.stopRcv();
	        
	        try {
				RTSPBufferedReader.close();
				RTSPBufferedWriter.close();
				RTSPsocket.close();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		
			RTPsocket.close();
			RTCPsocket.close();
			imagenb = 0;
		
			done = false;
			state = INIT;
		}

		return(request_type);
	}

	// Creates a DESCRIBE response string in SDP format for current media
	private String describe() {
		StringWriter writer1 = new StringWriter();
		StringWriter writer2 = new StringWriter();

		// Write the body first so we can get the size later
		writer2.write("v=0" + CRLF);
		writer2.write("m=video " + RTSP_dest_port + " RTP/AVP " + MJPEG_TYPE + CRLF);
		writer2.write("a=control:streamid=" + RTSPid + CRLF);
		writer2.write("a=mimetype:string;\"video/MJPEG\"" + CRLF);
		String body = writer2.toString();

		writer1.write("Content-Base: LQUAC"+ CRLF);
		writer1.write("Content-Type: " + "application/sdp" + CRLF);
		writer1.write("Content-Length: " + body.length() + CRLF);
		writer1.write(body);

		return writer1.toString();
	}

	//------------------------------------
	//Send RTSP Response
	//------------------------------------
	private void sendResponse() {
		try {
			RTSPBufferedWriter.write("RTSP/1.0 200 OK"+CRLF);
			RTSPBufferedWriter.write("CSeq: "+RTSPSeqNb+CRLF);
			RTSPBufferedWriter.write("Session: "+RTSPid+CRLF);
			RTSPBufferedWriter.flush();
			System.out.println("RTSP Server - Sent response to Client.");
		} catch(Exception ex) {
			System.out.println("Exception caught: "+ex);
			System.exit(0);
		}
	}

	private void sendDescribe() {
		String des = describe();
		try {
			RTSPBufferedWriter.write("RTSP/1.0 200 OK"+CRLF);
			RTSPBufferedWriter.write("CSeq: "+RTSPSeqNb+CRLF);
			RTSPBufferedWriter.write(des);
			RTSPBufferedWriter.flush();
			System.out.println("RTSP Server - Sent response to Client.");
		} catch(Exception ex) {
			System.out.println("Exception caught: "+ex);
		}
	}

	@Override
	public void run() {

		int request_type;
		
		try {

		while(is_running) {
			
			//Initiate TCP connection with the client for the RTSP session
			ServerSocket listenSocket = new ServerSocket(RTSPport);
			RTSPsocket = listenSocket.accept();
			listenSocket.close();

			//Get Client IP address
			ClientIPAddr = RTSPsocket.getInetAddress();

			//Initiate RTSPstate
			state = INIT;

			//Set input and output stream filters:
			RTSPBufferedReader = new BufferedReader(new InputStreamReader(RTSPsocket.getInputStream()) );
			RTSPBufferedWriter = new BufferedWriter(new OutputStreamWriter(RTSPsocket.getOutputStream()) );

			while(!done) {
				request_type = parseRequest(); //blocking
				System.err.println(request_type);
				
				if (request_type == SETUP) {
					done = true;

					//update RTSP state
					state = READY;
					System.out.println("New RTSP state: READY");

					//Send response
					sendResponse();

					//init RTP and RTCP sockets
					try {
						RTCPsocket = new DatagramSocket(RTCP_RCV_PORT);
						RTPsocket = new DatagramSocket();
					} catch (SocketException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
						done = false;
					}
				}
			}

			while(done) {

				request_type = parseRequest(); //blocking
				System.err.println(request_type);

				if ((request_type == PLAY) && (state == READY)) {
					//send back response
					sendResponse();
					//start timer
					timer.start();
					rtcpReceiver.startRcv();
					//update state
					state = PLAYING;
					System.out.println("New RTSP state: PLAYING");
				}
				else if ((request_type == PAUSE) && (state == PLAYING)) {
					//send back response
					sendResponse();
					//stop timer
					timer.stop();
					rtcpReceiver.stopRcv();
					//update state
					state = READY;
					System.out.println("New RTSP state: READY");
				}
				else if (request_type == TEARDOWN) {
					//send back response
					sendResponse();
					//stop timer
					timer.stop();
					rtcpReceiver.stopRcv();
					//             server.rtcpReceiver.stopRcv();
					//             //close sockets
					
					try {
						RTSPBufferedReader.close();
						RTSPBufferedWriter.close();
						RTSPsocket.close();
					} catch (IOException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				
					RTPsocket.close();
					RTCPsocket.close();
					imagenb = 0;
				
					done = false;
					state = INIT;
				}
				else if (request_type == DESCRIBE) {
					System.out.println("Received DESCRIBE request");
					sendDescribe();
				}
			}
		}
		} catch(Exception e) {
			is_running = false;
		}

	}	
	
	
	public static void main(String argv[]) throws Exception
	{
		//create a Server object
		RTPSMjpegHandler<Planar<GrayU8>> server = new RTPSMjpegHandler<Planar<GrayU8>>(640,480);
		
		Planar<GrayU8> test = new Planar<GrayU8>(GrayU8.class, 640,480,3);
		DataModel model = new DataModel();
		
		long tms = System.currentTimeMillis();
		
	    server.registerOverlayListener((ctx,t) -> {
	    	
	    	ctx.setColor(Color.WHITE);
	    	ctx.drawString("TMS="+t, 20, 20);
		
	    });
		
		server.start(1051);
		
		while(true) {
			
			Thread.sleep(20);
			tms = System.currentTimeMillis();
			server.addToStream(test, model, tms*1000);
			
			
		}
		
		
	}



}
