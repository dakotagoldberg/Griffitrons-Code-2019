package frc.robot.auto;

import java.net.*;
import java.io.*;

public class EchoServer extends Thread 
{
	private DatagramSocket socket;
	private boolean running;
	private byte[] buf = new byte[1024];

	private double throttle = 0.0;

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
				
				throttle = Double.parseDouble(received);
				System.out.println(throttle);

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

	public double getThrottle(){
		return throttle;
	}

}
