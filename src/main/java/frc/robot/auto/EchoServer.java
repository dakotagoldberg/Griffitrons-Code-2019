package frc.robot.auto;

import java.net.*;
import java.io.*;

public class EchoServer extends Thread 
{
	private DatagramSocket socket;
	private boolean running;
	private byte[] buf = new byte[256];

	public EchoServer()
	{
		try 
		{
			socket = new DatagramSocket(1661);
		}
		catch(SocketException e)
		{
			e.printStackTrace();
		}
	}

	public void run() 
	{
		running = true;

		while(running) 
		{
			try
			{
				DatagramPacket packet = new DatagramPacket(buf, buf.length);
				socket.receive(packet);

				InetAddress address = packet.getAddress();
				int port = packet.getPort();
				packet = new DatagramPacket(buf, buf.length, address, port); 
				String received = 
					new String(packet.getData(), 0, packet.getLength());
				
				System.out.println(received);

				if(received.equals("end"))
				{
					running = false;
					continue;
				}

				socket.send(packet);
			}
			catch(IOException e)
			{
				e.printStackTrace();
			}
		}
		socket.close();
	}
	
	public static void main(String[] args)
	{
		new EchoServer().run();
	}
}
