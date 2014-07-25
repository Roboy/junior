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

import java.awt.EventQueue;

import javax.imageio.ImageIO;
import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.Rectangle;

import javax.swing.JTextArea;
import javax.swing.JButton;
import java.awt.event.ActionListener;
import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.image.BufferedImage;
import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.Socket;
import java.util.prefs.BackingStoreException;
import java.util.prefs.Preferences;
import java.awt.FlowLayout;
import javax.swing.JPanel;
import javax.swing.JSplitPane;
import javax.swing.JTextField;
import java.awt.Component;
import javax.swing.Box;
import javax.swing.JSlider;
import javax.swing.JToggleButton;
import javax.swing.JComboBox;
import javax.swing.SwingConstants;
import javax.swing.BoxLayout;
import javax.swing.border.Border;
import java.awt.CardLayout;
import java.awt.GridLayout;
import javax.swing.JTabbedPane;
import javax.swing.JCheckBox;
import javax.swing.event.ChangeListener;
import javax.swing.event.ChangeEvent;
import java.awt.event.ItemListener;
import java.awt.event.ItemEvent;
import java.awt.event.InputMethodListener;
import java.awt.event.InputMethodEvent;

public class RoboyStarmindGUI implements RoboyStarmindState {

	private JFrame frame;
	private JTextField addressTextField;
	private JTextField portTextField;
	private JTextField delayTextField;
	private JTextField textField;
	private JButton btnSpeech;
	private JButton btnConnect;
	private JComboBox speechComboBox;
	private ImagePanel imagePanel;
	private RoboyStarmindModel model;
	private Font subTitleFont;
	private JSlider rollSlider;
	private JSlider pitchSlider;
	private JSlider yawSlider;
	private JTextField modeTextField;
	private JButton btnChangeMode;
	private JButton btnResetHeadPosition;
	private JCheckBox chckbxContinuesSendingHead;
	private JButton btnSendHeadPosition;
	private JComboBox expressionComboBox;
	private JButton btnRunFacialExpression;
	private JComboBox movementComboBox;
	private JButton btnRunBodyMovement;
	private Preferences prefs;
	

	/**
	 * Launch the application.
	 */
	public static void main(String[] args) {
		EventQueue.invokeLater(new Runnable() {
			public void run() {
				try {
					RoboyStarmindGUI window = new RoboyStarmindGUI();
					window.frame.setVisible(true);
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
		});
	}

	/**
	 * Create the application.
	 */
	public RoboyStarmindGUI() {
		initialize();
		model = new RoboyStarmindModel(this);
		this.guiConnected(model.isConnected());
		assignItems();
		prefs = Preferences.userNodeForPackage(this.getClass());
		assignAddress();
	}

	/**
	 * Initialize the contents of the frame.
	 */
	private void initialize() {
		subTitleFont = new Font("Ubuntu", Font.BOLD, 16);
		
		frame = new JFrame();
		frame.setBounds(0, 0, 630, 850);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.getContentPane().setLayout(null);
		
		// TITLE
		JLabel lblRoboyStarmind = new JLabel("Roboy Cockpit");
		lblRoboyStarmind.setBounds(12, 0, 558, 35);
		lblRoboyStarmind.setHorizontalTextPosition(JLabel.LEFT);
		lblRoboyStarmind.setFont(new Font("Ubuntu", Font.BOLD, 30));
		frame.getContentPane().add(lblRoboyStarmind);
		
		// CONNECTION PANEL
		JPanel connectPanel = new JPanel();
		connectPanel.setBounds(12, 36, 606, 40);
		frame.getContentPane().add(connectPanel);
		connectPanel.setLayout(new FlowLayout(FlowLayout.LEFT, 3, 5));
		
		JLabel lblAddress = new JLabel("Address:");
		connectPanel.add(lblAddress);
		
		addressTextField = new JTextField();
		connectPanel.add(addressTextField);
		addressTextField.setColumns(10);
		
		Component horizontalGlue = Box.createHorizontalGlue();
		connectPanel.add(horizontalGlue);
		
		JLabel lblPort = new JLabel("Port:");
		connectPanel.add(lblPort);
		
		portTextField = new JTextField("30000");
		connectPanel.add(portTextField);
		portTextField.setColumns(4);
		
		Component horizontalGlue_2 = Box.createHorizontalGlue();
		connectPanel.add(horizontalGlue_2);
		
		JLabel lblIntervalsec = new JLabel("Interval (millis):");
		connectPanel.add(lblIntervalsec);
		
		delayTextField = new JTextField("500");
		connectPanel.add(delayTextField);
		delayTextField.setColumns(4);
		
		Component horizontalGlue_1 = Box.createHorizontalGlue();
		connectPanel.add(horizontalGlue_1);
		
		btnConnect = new JButton("Connect");
		btnConnect.setActionCommand("connect");
		btnConnect.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				if("connect".compareTo(arg0.getActionCommand()) == 0){
					int p = Integer.parseInt(portTextField.getText());
					int d = Integer.parseInt(delayTextField.getText());
					prefs.put("roboy_address", addressTextField.getText());
					prefs.putInt("roboy_port", p);
					prefs.putInt("roboy_delay", d);
					try {
						prefs.flush();
					} catch (BackingStoreException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
					model.connect(addressTextField.getText(), 
							p, 
							d, imagePanel);
				}else{
					model.disconnect();
				}
			}
		});
		connectPanel.add(btnConnect);
		
		// VIDEO
		imagePanel = new ImagePanel();
		imagePanel.setBounds(12, 85, 606, 349);
		imagePanel.setBackground(Color.BLACK);
		frame.getContentPane().add(imagePanel);
		
		// TEXT TO SPEECH
		JLabel lblTextToSpeech = new JLabel("Text to Speech");
		lblTextToSpeech.setBounds(12, 446, 266, 24);
		lblTextToSpeech.setHorizontalAlignment(SwingConstants.LEFT);
		lblTextToSpeech.setFont(subTitleFont);
		frame.getContentPane().add(lblTextToSpeech);
		
		JPanel languagePanel = new JPanel();
		languagePanel.setBounds(12, 510, 322, 38);
		frame.getContentPane().add(languagePanel);
		languagePanel.setLayout(new FlowLayout(FlowLayout.LEFT, 3, 5));
		
		JLabel lblSpeech = new JLabel("Voice:");
		languagePanel.add(lblSpeech);
		
		speechComboBox = new JComboBox();
		languagePanel.add(speechComboBox);
		
		btnSpeech = new JButton("Speak");
		languagePanel.add(btnSpeech);
		btnSpeech.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				sendText();
			}
		});
		
		textField = new JTextField();
		textField.setBounds(12, 470, 606, 40);
		textField.addKeyListener(new KeyListener() {
			
			@Override
			public void keyTyped(KeyEvent arg0) {
				// TODO Auto-generated method stub
				if(arg0.getKeyChar() == '\n'){
					sendText();
				}
			}
			
			@Override
			public void keyReleased(KeyEvent arg0) {
				// TODO Auto-generated method stub
				
			}
			
			@Override
			public void keyPressed(KeyEvent arg0) {
				// TODO Auto-generated method stub
				
			}
		});
		frame.getContentPane().add(textField);
		
		// TABS
		JTabbedPane tabbedPane = new JTabbedPane(JTabbedPane.TOP);
		tabbedPane.setBounds(12, 569, 606, 240);
		frame.getContentPane().add(tabbedPane);
		
		JPanel modeTabPanel = new JPanel();
		tabbedPane.addTab("Swith Mode", null, modeTabPanel, null);
		modeTabPanel.setLayout(null);
		
		JLabel lblMode = new JLabel("Mode:");
		lblMode.setBounds(12, 32, 70, 18);
		modeTabPanel.add(lblMode);
		
		modeTextField = new JTextField();
		modeTextField.setBounds(63, 27, 86, 28);
		modeTabPanel.add(modeTextField);
		modeTextField.setColumns(10);
		
		btnChangeMode = new JButton("Change Mode");
		btnChangeMode.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				String mode = modeTextField.getText().trim();
				if(!mode.isEmpty()){
					if(model.sendMode(mode.charAt(0))){
						modeTextField.setEnabled(false);
						btnChangeMode.setEnabled(false);
					}
				}
			}
		});
		btnChangeMode.setBounds(12, 70, 137, 30);
		modeTabPanel.add(btnChangeMode);
		
		JPanel headTabpanel = new JPanel();
		tabbedPane.addTab("Head", null, headTabpanel, null);
		headTabpanel.setLayout(null);
		
		JPanel headPanel = new JPanel();
		headPanel.setBounds(286, 12, 300, 190);
		headTabpanel.add(headPanel);
		GridBagLayout gbl_headPanel = new GridBagLayout();
		gbl_headPanel.columnWidths = new int[] {50, 250, 0};
		gbl_headPanel.rowHeights = new int[]{30, 30, 30, 0};
		gbl_headPanel.columnWeights = new double[]{0.0, 0.0, Double.MIN_VALUE};
		gbl_headPanel.rowWeights = new double[]{0.0, 0.0, 0.0, Double.MIN_VALUE};
		headPanel.setLayout(gbl_headPanel);
		
		JLabel lblRoll = new JLabel("Roll:");
		GridBagConstraints gbc_lblRoll = new GridBagConstraints();
		gbc_lblRoll.fill = GridBagConstraints.BOTH;
		gbc_lblRoll.insets = new Insets(0, 0, 5, 5);
		gbc_lblRoll.gridx = 0;
		gbc_lblRoll.gridy = 0;
		headPanel.add(lblRoll, gbc_lblRoll);
		rollSlider = new JSlider();
		rollSlider.addChangeListener(new ChangeListener() {
			@Override
			public void stateChanged(ChangeEvent e) {
				JSlider slider = (JSlider)e.getSource();
				if(!slider.getValueIsAdjusting() && chckbxContinuesSendingHead != null && chckbxContinuesSendingHead.isSelected()){
					model.sendHead(rollSlider.getValue(), pitchSlider.getValue(), yawSlider.getValue());
				}
			}
		});
		rollSlider.setSnapToTicks(true);
		rollSlider.setPaintTicks(true);
		rollSlider.setPaintLabels(true);
		rollSlider.setMinorTickSpacing(1);
		rollSlider.setMajorTickSpacing(1);
		rollSlider.setMinimum(-3);
		rollSlider.setMaximum(3);
		rollSlider.setValue(0);
		GridBagConstraints gbc_rollSlider = new GridBagConstraints();
		gbc_rollSlider.fill = GridBagConstraints.BOTH;
		gbc_rollSlider.insets = new Insets(0, 0, 5, 0);
		gbc_rollSlider.gridx = 1;
		gbc_rollSlider.gridy = 0;
		headPanel.add(rollSlider, gbc_rollSlider);
		
		JLabel lblPitch = new JLabel("Pitch:");
		GridBagConstraints gbc_lblPitch = new GridBagConstraints();
		gbc_lblPitch.fill = GridBagConstraints.BOTH;
		gbc_lblPitch.insets = new Insets(0, 0, 5, 5);
		gbc_lblPitch.gridx = 0;
		gbc_lblPitch.gridy = 1;
		headPanel.add(lblPitch, gbc_lblPitch);
		pitchSlider = new JSlider();
		pitchSlider.addChangeListener(new ChangeListener() {
			@Override
			public void stateChanged(ChangeEvent e) {
				JSlider slider = (JSlider)e.getSource();
				if(!slider.getValueIsAdjusting() && chckbxContinuesSendingHead != null && chckbxContinuesSendingHead.isSelected()){
					model.sendHead(rollSlider.getValue(), pitchSlider.getValue(), yawSlider.getValue());
				}
			}
		});
		pitchSlider.setMinorTickSpacing(1);
		pitchSlider.setMajorTickSpacing(1);
		pitchSlider.setPaintLabels(true);
		pitchSlider.setPaintTicks(true);
		pitchSlider.setSnapToTicks(true);
		pitchSlider.setMinimum(0);
		pitchSlider.setMaximum(12);
		pitchSlider.setValue(0);
		GridBagConstraints gbc_pitchSlider = new GridBagConstraints();
		gbc_pitchSlider.fill = GridBagConstraints.BOTH;
		gbc_pitchSlider.insets = new Insets(0, 0, 5, 0);
		gbc_pitchSlider.gridx = 1;
		gbc_pitchSlider.gridy = 1;
		headPanel.add(pitchSlider, gbc_pitchSlider);
		
		JLabel lblYaw = new JLabel("Yaw:");
		GridBagConstraints gbc_lblYaw = new GridBagConstraints();
		gbc_lblYaw.fill = GridBagConstraints.BOTH;
		gbc_lblYaw.insets = new Insets(0, 0, 0, 5);
		gbc_lblYaw.gridx = 0;
		gbc_lblYaw.gridy = 2;
		headPanel.add(lblYaw, gbc_lblYaw);
		yawSlider = new JSlider();
		yawSlider.addChangeListener(new ChangeListener() {
			@Override
			public void stateChanged(ChangeEvent e) {
				JSlider slider = (JSlider)e.getSource();
				if(!slider.getValueIsAdjusting() && chckbxContinuesSendingHead != null && chckbxContinuesSendingHead.isSelected()){
					model.sendHead(rollSlider.getValue(), pitchSlider.getValue(), yawSlider.getValue());
				}
			}
		});
		yawSlider.setMinorTickSpacing(1);
		yawSlider.setMajorTickSpacing(3);
		yawSlider.setSnapToTicks(true);
		yawSlider.setPaintTicks(true);
		yawSlider.setPaintLabels(true);
		yawSlider.setMinimum(-12);
		yawSlider.setMaximum(12);
		yawSlider.setValue(0);
		GridBagConstraints gbc_yawSlider = new GridBagConstraints();
		gbc_yawSlider.fill = GridBagConstraints.BOTH;
		gbc_yawSlider.gridx = 1;
		gbc_yawSlider.gridy = 2;
		headPanel.add(yawSlider, gbc_yawSlider);
		
		btnResetHeadPosition = new JButton("Reset Head Position");
		btnResetHeadPosition.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				model.sendHead(0, 0, 0);
				resetSliders();
			}
		});
		btnResetHeadPosition.setBounds(12, 26, 231, 30);
		headTabpanel.add(btnResetHeadPosition);
		
		btnSendHeadPosition = new JButton("Send Head Position");
		btnSendHeadPosition.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				model.sendHead(rollSlider.getValue(), pitchSlider.getValue(), yawSlider.getValue());
			}
		});
		btnSendHeadPosition.setBounds(12, 106, 231, 30);
		headTabpanel.add(btnSendHeadPosition);
		
		chckbxContinuesSendingHead = new JCheckBox("Continues sending head position");
		chckbxContinuesSendingHead.setBounds(12, 70, 262, 24);
		headTabpanel.add(chckbxContinuesSendingHead);
		
		JPanel facialTabPanel = new JPanel();
		tabbedPane.addTab("Facial", null, facialTabPanel, null);
		facialTabPanel.setLayout(null);
		
		JLabel lblExpression = new JLabel("Expression:");
		lblExpression.setBounds(12, 32, 103, 18);
		facialTabPanel.add(lblExpression);
		
		expressionComboBox = new JComboBox();
		expressionComboBox.setBounds(104, 27, 250, 28);
		facialTabPanel.add(expressionComboBox);
		
		btnRunFacialExpression = new JButton("Run Facial Expression");
		btnRunFacialExpression.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				String facial = expressionComboBox.getSelectedItem().toString();
				if(!facial.isEmpty()){
					model.sendFacial(facial);
				}
			}
		});
		btnRunFacialExpression.setBounds(12, 70, 245, 30);
		facialTabPanel.add(btnRunFacialExpression);
		
		JPanel bodyTabPanel = new JPanel();
		tabbedPane.addTab("Body", null, bodyTabPanel, null);
		bodyTabPanel.setLayout(null);
		
		JLabel lblMovement = new JLabel("Movement:");
		lblMovement.setBounds(12, 32, 102, 18);
		bodyTabPanel.add(lblMovement);
		
		movementComboBox = new JComboBox();
		movementComboBox.setBounds(104, 27, 250, 28);
		bodyTabPanel.add(movementComboBox);
		
		btnRunBodyMovement = new JButton("Run Body Movement");
		btnRunBodyMovement.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				String body = movementComboBox.getSelectedItem().toString();
				if(!body.isEmpty()){
					model.sendBody(body);
				}
			}
		});
		btnRunBodyMovement.setBounds(12, 70, 245, 30);
		bodyTabPanel.add(btnRunBodyMovement);
		
		
	}
	
	private void assignItems(){
		this.expressionComboBox.removeAllItems();
		for(String t : model.getFacialExpressions()){
			this.expressionComboBox.addItem(t);
		}
		this.movementComboBox.removeAllItems();
		for(String t : model.getBodyMovements()){
			this.movementComboBox.addItem(t);
		}
	}
	
	private void assignAddress(){
		addressTextField.setText(prefs.get("roboy_address", "<ip or host>"));
		portTextField.setText(Integer.toString(prefs.getInt("roboy_port", 30000)));
		delayTextField.setText(Integer.toString(prefs.getInt("roboy_delay", 500)));
	}
	
	private void guiConnected(boolean connected){
		this.addressTextField.setEnabled(!connected);
		this.portTextField.setEnabled(!connected);
		this.delayTextField.setEnabled(!connected);
		
		this.textField.setEnabled(connected);
		this.speechComboBox.setEnabled(connected);
		this.btnSpeech.setEnabled(connected);
		this.rollSlider.setEnabled(connected);
		this.pitchSlider.setEnabled(connected);
		this.yawSlider.setEnabled(connected);
		this.modeTextField.setEnabled(connected);
		this.btnChangeMode.setEnabled(connected);
		this.btnResetHeadPosition.setEnabled(connected);
		this.chckbxContinuesSendingHead.setEnabled(connected);
		this.btnSendHeadPosition.setEnabled(connected);
		this.expressionComboBox.setEnabled(connected);
		this.btnRunFacialExpression.setEnabled(connected);
		this.movementComboBox.setEnabled(connected);
		this.btnRunBodyMovement.setEnabled(connected);
		
		this.btnConnect.setText(connected ? "Disconnect" : "Connect");
		this.btnConnect.setActionCommand(connected ? "disconnect" : "connect");
		
		if(!connected){
			this.resetSliders();
		}
	}
	
	private void resetTextArea(){
		btnSpeech.setEnabled(true);
		textField.setText("");
		textField.setEnabled(true);
	}
	
	private void resetSliders(){
		this.rollSlider.setValue(0);
		this.pitchSlider.setValue(0);
		this.yawSlider.setValue(0);
	}
	
	private void resetMode(){
		modeTextField.setText("");
		modeTextField.setEnabled(true);
		btnChangeMode.setEnabled(true);
	}
	
	private void sendText(){
		String text = textField.getText().trim();
		String voice = (String)speechComboBox.getSelectedItem();
		if(text != null && !text.isEmpty()){
			prefs.put("roboy_voice", voice);
			try {
				prefs.flush();
			} catch (BackingStoreException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			if(model.sendText(text, voice)){
				//btnSpeech.setEnabled(false);
				//textField.setEnabled(false);
			}
		}
	}

	@Override
	public void roboyStarmindStateChanged(int state) {
		switch(state){
		case RoboyStarmindState.CONNECTED:
			this.guiConnected(true);
			break;
		case RoboyStarmindState.DISCONNECTED:
			imagePanel.setImage(null);
			this.guiConnected(false);
			break;
		case RoboyStarmindState.TEXT_SENT:
			resetTextArea();
			break;
		case RoboyStarmindState.VOICES_RECEIVED:
			speechComboBox.removeAllItems();
			for (String t : model.getVoices()){
				speechComboBox.addItem(t);
			}
			if(model.getVoices().length > 0){
				String voice = prefs.get("roboy_voice", model.getVoices()[0]);
				speechComboBox.setSelectedItem(voice);
			}
			break;
		case RoboyStarmindState.MODE_SENT:
			resetMode();
			break;
		case RoboyStarmindState.HEAD_SENT:
			break;
		case RoboyStarmindState.FACIAL_SENT:
			break;
		case RoboyStarmindState.BODY_SENT:
			break;
		}
	}

	@Override
	public void roboyStarmindFailed(int error, Exception e) {
		//roboyStarmindStateChanged(model.isConnected() ? CONNECTED : DISCONNECTED);
		roboyStarmindStateChanged(DISCONNECTED);
	}
}
