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
import java.io.File;
import java.io.IOException;
import java.net.UnknownHostException;

import javax.imageio.ImageIO;

public class RoboyStarmindModel {
	
	private String[] bodyMovements = {"DrawingBack_5","GazingBothHands_3a","GazingBothHands_3b","GazingBothHands_3c",
										"GivingHand_5","Introduction","ShakingHand_5",
										"TheatreGiveLeftHand","TheatreRemoveCloth","TheatreStretchHands",
										"TheatreTakeBackLeftHand","TheatreWave","WavingHand"};
	private String[] facialExpressions = {"normal","speak","sleep","smile","smileblink","shy","surprise","kiss","neutral","angry","sweat"};
	
	private RoboyConnect connector;
	//private boolean connected;
	private int delayMillis;
	private ImagePanel imagePanel;
	private RoboyStarmindState state;
	private String roboyText;
	private String roboyVoice;
	private char roboyMode;
	private int roboyHeadRoll;
	private int roboyHeadPitch;
	private int roboyHeadYaw;
	private String roboyFacial;
	private String roboyBody;
	private String[] voices;

	public RoboyStarmindModel(RoboyStarmindState state){
		//this.connected = false;
		this.state = state;
	}

	public void connect(String address, int port, int delay, ImagePanel imgPanel){
		connector = new RoboyConnect(address, port);
		this.delayMillis = delay*1;
		this.imagePanel = imgPanel;
		new Thread(new Runnable() {
			
			@Override
			public void run() {
				try {
					connector.connect();
					if(state != null){
						state.roboyStarmindStateChanged(RoboyStarmindState.CONNECTED);
					}
					
					if(connector.isConnected()){
						String voiceStr = connector.getVoices();
						voices = voiceStr.split("\n");
						if(state != null){
							state.roboyStarmindStateChanged(RoboyStarmindState.VOICES_RECEIVED);
						}
					}
					
					while(connector.isConnected()){
						BufferedImage bi = connector.getImage();
						//File outputfile = new File(System.currentTimeMillis()+".png");
					    //ImageIO.write(bi, "png", outputfile);
						imagePanel.setImage(bi);
						Thread.sleep(delayMillis);
					}
					
				} catch (UnknownHostException e) {
					// TODO Auto-generated catch block
					if(state != null){
						state.roboyStarmindFailed(RoboyStarmindState.ERROR_CONNECTION, e);
					}
				} catch (IOException e) {
					// TODO Auto-generated catch block
					if(state != null){
						state.roboyStarmindFailed(RoboyStarmindState.ERROR_CONNECTION, e);
					}
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					if(state != null){
						state.roboyStarmindFailed(RoboyStarmindState.ERROR_RUNNER, e);
					}
				} catch(Exception e) {
					if(state != null){
						state.roboyStarmindFailed(RoboyStarmindState.ERROR_RUNNER, e);
					}
				}
				
			}
		}).start();
	}
	
	public boolean isConnected(){
		return connector != null && connector.isConnected();
	}
	
	public void disconnect(){
		if(connector != null){
			connector.disconnect();
			if(state != null){
				state.roboyStarmindStateChanged(RoboyStarmindState.DISCONNECTED);
			}
		}
	}
	
	public boolean sendText(String text, String voice){
		this.roboyText = text;
		this.roboyVoice = voice;
		if(connector != null && connector.isConnected()){
			new Thread(new Runnable() {
				
				@Override
				public void run() {
					try {
						connector.sendText(roboyText, roboyVoice);
						if(state != null){
							state.roboyStarmindStateChanged(RoboyStarmindState.TEXT_SENT);
						}
					} catch (UnknownHostException e) {
						// TODO Auto-generated catch block
						if(state != null){
							state.roboyStarmindFailed(RoboyStarmindState.ERROR_CONNECTION, e);
						}
					} catch (IOException e) {
						// TODO Auto-generated catch block
						if(state != null){
							state.roboyStarmindFailed(RoboyStarmindState.ERROR_CONNECTION, e);
						}
					}
				}
			}).start();
			return true;
		}
		return false;
	}
	
	public boolean sendMode(char mode){
		this.roboyMode = mode;
		if(connector != null && connector.isConnected()){
			new Thread(new Runnable() {
				
				@Override
				public void run() {
					try {
						connector.sendMode(roboyMode);
						if(state != null){
							state.roboyStarmindStateChanged(RoboyStarmindState.MODE_SENT);
						}
					} catch (UnknownHostException e) {
						// TODO Auto-generated catch block
						if(state != null){
							state.roboyStarmindFailed(RoboyStarmindState.ERROR_CONNECTION, e);
						}
					} catch (IOException e) {
						// TODO Auto-generated catch block
						if(state != null){
							state.roboyStarmindFailed(RoboyStarmindState.ERROR_CONNECTION, e);
						}
					}
				}
			}).start();
			return true;
		}
		return false;
	}
	
	public boolean sendHead(int roll, int pitch, int yaw){
		this.roboyHeadRoll = roll;
		this.roboyHeadPitch = pitch;
		this.roboyHeadYaw = yaw;
		if(connector != null && connector.isConnected()){
			new Thread(new Runnable() {
				
				@Override
				public void run() {
					try {
						connector.sendHead(roboyHeadRoll, roboyHeadPitch, roboyHeadYaw);
						if(state != null){
							state.roboyStarmindStateChanged(RoboyStarmindState.HEAD_SENT);
						}
					} catch (UnknownHostException e) {
						if(state != null){
							state.roboyStarmindFailed(RoboyStarmindState.ERROR_CONNECTION, e);
						}
					} catch (IOException e) {
						if(state != null){
							state.roboyStarmindFailed(RoboyStarmindState.ERROR_CONNECTION, e);
						}
					}
				}
			}).start();
			return true;
		}
		return false;
	}
	
	public boolean sendFacial(String facial){
		this.roboyFacial = facial;
		if(connector != null && connector.isConnected()){
			new Thread(new Runnable() {
				
				@Override
				public void run() {
					try {
						connector.sendFacial(roboyFacial);
						if(state != null){
							state.roboyStarmindStateChanged(RoboyStarmindState.FACIAL_SENT);
						}
					} catch (UnknownHostException e) {
						if(state != null){
							state.roboyStarmindFailed(RoboyStarmindState.ERROR_CONNECTION, e);
						}
					} catch (IOException e) {
						if(state != null){
							state.roboyStarmindFailed(RoboyStarmindState.ERROR_CONNECTION, e);
						}
					}
				}
			}).start();
			return true;
		}
		return false;
	}
	
	public boolean sendBody(String body){
		this.roboyBody = body;
		if(connector != null && connector.isConnected()){
			new Thread(new Runnable() {
				
				@Override
				public void run() {
					try {
						connector.sendBody(roboyBody);
						if(state != null){
							state.roboyStarmindStateChanged(RoboyStarmindState.BODY_SENT);
						}
					} catch (UnknownHostException e) {
						if(state != null){
							state.roboyStarmindFailed(RoboyStarmindState.ERROR_CONNECTION, e);
						}
					} catch (IOException e) {
						if(state != null){
							state.roboyStarmindFailed(RoboyStarmindState.ERROR_CONNECTION, e);
						}
					}
				}
			}).start();
			return true;
		}
		return false;
	}
	
	public String[] getVoices() {
		return voices;
	}

	public void setVoices(String[] voices) {
		this.voices = voices;
	}

	public String[] getBodyMovements() {
		return bodyMovements;
	}

	public void setBodyMovements(String[] bodyMovements) {
		this.bodyMovements = bodyMovements;
	}

	public String[] getFacialExpressions() {
		return facialExpressions;
	}

	public void setFacialExpressions(String[] facialExpressions) {
		this.facialExpressions = facialExpressions;
	}
	
	
	
}
