package frc.robot.auto;

import frc.robot.constants.Robot_Framework;
import java.net.*;
import java.io.*;

public class EchoServer extends Thread implements Robot_Framework
{
	private DatagramSocket socket;
	private boolean running;
	private byte[] buf = new byte[1024];

	private double throttle = 0.0;
	private double turn = 0.0;

	public EchoServer() {
		super("Server");
		try {
			socket = new DatagramSocket(5805);
		} catch(SocketException e) {
			e.printStackTrace();
		}
	}

	public void run() {
		running = true;

		while(running) {
			try {
				DatagramPacket packet = new DatagramPacket(buf, buf.length);
				socket.receive(packet);

				InetAddress address = packet.getAddress();
				int port = packet.getPort();
				packet = new DatagramPacket(buf, buf.length, address, port); 
				String received = 
					new String(packet.getData(), 0, packet.getLength());
				
				throttle = Double.parseDouble(received.substring(0, 4));
				turn = Double.parseDouble(received.substring(4, 8));

				if(received.equals("end")) {
					running = false;
					continue;
				}

				socket.send(packet);
			} catch(IOException e) {
				e.printStackTrace();
			}
		}
		socket.close();
	}

	public double getThrottle() {
		return throttle;
	}

	public double getTurn() { 
		return turn;
	}

}
