/*Copyright (c) 2014, University of Zurich, Department of Informatics, Artificial Intelligence Laboratory
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its contributors 
   may be used to endorse or promote products derived from this software without 
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/
package org.roboy.starmind;

import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.net.Socket;
import java.net.UnknownHostException;

import javax.imageio.ImageIO;

public class RoboyConnect {
	
	Socket s;
	String address;
	int port;
	
	public RoboyConnect(String address, int port){
		this.address = address;
		this.port = port;
	}
	
	public void connect() throws UnknownHostException, IOException{
		if(!this.isConnected()){
			s = new Socket(this.address, this.port);
		}
	}
	
	public void disconnect(){
		if(this.isConnected()){
			try {
				s.close();
				s = null;
			} catch (IOException e) { }
		}
	}
	
	public boolean isConnected(){
		return (s != null && s.isConnected());
	}
	
	public BufferedImage getImage() throws UnknownHostException, IOException{
		if(!this.isConnected()){
			this.connect();
		}
		byte[] b = new byte[]{'p','i','c'};
		s.getOutputStream().write(b);
		s.getOutputStream().flush();
		int bytesRead = 0;
		byte[] buffer = new byte[1024];
		ByteArrayOutputStream bs = new ByteArrayOutputStream();
		do {
			bytesRead = s.getInputStream().read(buffer, 0, buffer.length);
			bs.write(buffer, 0, bytesRead);
		} while(bytesRead == buffer.length);
		
		return ImageIO.read(new ByteArrayInputStream(bs.toByteArray()));
	}
	
	public void sendText(String text, String voice) throws UnknownHostException, IOException{
		if(!this.isConnected()){
			this.connect();
		}
		String str = (voice != null && !voice.isEmpty()) ? "text:voice_"+voice+"#"+text : "text:"+text;
		s.getOutputStream().write(str.getBytes());
	}
	
	public String getVoices() throws UnknownHostException, IOException{
		if(!this.isConnected()){
			this.connect();
		}
		byte[] b = new byte[]{'v','o','i','c','e','s'};
		s.getOutputStream().write(b);
		s.getOutputStream().flush();
		int bytesRead = 0;
		byte[] buffer = new byte[1024];
		ByteArrayOutputStream bs = new ByteArrayOutputStream();
		do {
			bytesRead = s.getInputStream().read(buffer, 0, buffer.length);
			bs.write(buffer, 0, bytesRead);
		} while(bytesRead == buffer.length);
		return bs.toString();
	}
	
	public void sendFacial(String facial) throws UnknownHostException, IOException{
		if(!this.isConnected()){
			this.connect();
		}
		String str = "facial:"+facial;
		s.getOutputStream().write(str.getBytes());
	}
	
	public void sendBody(String body) throws UnknownHostException, IOException{
		if(!this.isConnected()){
			this.connect();
		}
		String str = "body:"+body;
		s.getOutputStream().write(str.getBytes());
	}
	
	public void sendHead(int roll, int pitch, int yaw) throws UnknownHostException, IOException{
		if(!this.isConnected()){
			this.connect();
		}
		if(roll < -3){
			roll = -3;
		}
		if(roll > 3){
			roll = 3;
		}
		if(pitch < 0){
			pitch = 0;
		}
		if(pitch > 12){
			pitch = 12;
		}
		if(yaw < -12){
			yaw = -12;
		}
		if(yaw > 12){
			yaw = 12;
		}
		String str = "head:"+roll+":"+pitch+":"+yaw;
		s.getOutputStream().write(str.getBytes());
	}
	
	public void sendMode(char mode) throws UnknownHostException, IOException{
		if(!this.isConnected()){
			this.connect();
		}
		String str = "mode:"+mode;
		s.getOutputStream().write(str.getBytes());
	}

}
