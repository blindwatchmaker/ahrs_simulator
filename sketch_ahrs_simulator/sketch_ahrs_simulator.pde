/*
 *  2015.09.01 ('c')void 
 *  
 *  Main 
 *
 */

import processing.serial.*;
import controlP5.*;

ControlP5 cp5 = null;

/************************************************************************************************
 * 
 * 	Camera manipulation
 * 
 ************************************************************************************************/
// zoom out factor 
int zoom_out= 600;

// rotion camera angle   
float rot_cam_x = 67 * PI/180; // 0
float rot_cam_y = 0;
float rot_cam_z = 38 * PI/180; // PI/2

/************************************************************************************************
 * 
 * 	 Status controls 
 * 
 ************************************************************************************************/
class StatusControl extends Listener
{
	Textlabel cp5_txt_algorithm;
	Textlabel cp5_txt_roll;
	Textlabel cp5_txt_pitch;
	Textlabel cp5_txt_yaw;
	
	final int text_margin_x = 90;
	final int text_margin_y = 15;
	
	public StatusControl(int pos_x, int pos_y) {   
		cp5_txt_algorithm = cp5.addTextlabel("algorithm_name")
				.setColorValue(0xffffff00)
				.setFont(createFont("", 12))
				;
		
		cp5_txt_roll = cp5.addTextlabel("atti_label_roll")
				.setColorValue(0xffffff00)
				.setFont(createFont("", 12))
				;

		cp5_txt_pitch = cp5.addTextlabel("atti_label_pitch")
				.setColorValue(0xffffff00)
				.setFont(createFont("", 12))
				;

		cp5_txt_yaw = cp5.addTextlabel("atti_label_yaw")
				.setColorValue(0xffffff00)
				.setFont(createFont("", 12))
				;
		
		move(pos_x, pos_y);
	}
	
	public void move(int text_x, int text_y) {
		cp5_txt_algorithm.setPosition(text_x, text_y);
		text_y += text_margin_y;
		
		cp5_txt_roll.setPosition(text_x, text_y);
		text_x += text_margin_x;
		cp5_txt_pitch.setPosition(text_x, text_y);
		text_x += text_margin_x;
		cp5_txt_yaw.setPosition(text_x, text_y);
	}
	
	public void set_algorithm_name(String algorithm_name) {
		cp5_txt_algorithm.setText(String.format("Algorithm : %s", algorithm_name));
	}
	
	public void notify(EularAngle eular_angle, ImuData imu_data) {
		cp5_txt_roll.setText(String.format("Roll: %.2f", eular_angle.roll));
		cp5_txt_pitch.setText(String.format("Pitch: %.2f", eular_angle.pitch));
		cp5_txt_yaw.setText(String.format("Yaw: %.2f", eular_angle.yaw));
	}
}

StatusControl status_control = null;

/************************************************************************************************
 * 
 * 	 
 * 
 ************************************************************************************************/
class AlgorithmControl
{
	ScrollableList sll_algorithm_list = null;
			
	public AlgorithmControl(int pos_x, int pos_y) {
				
		sll_algorithm_list = cp5.addScrollableList("algorithm")  
		          .setPosition(pos_x, pos_y)
		          .setSize(100,500)
		          .setBarHeight(20)
		          .setItemHeight(20)
		          .setOpen(false)
		          .addItems(attitude_estimation_algorithm_list)
		          ;
	}
	
	private String get_algorithm() { 
		return attitude_estimation_algorithm_list[int(sll_algorithm_list.getValue())];
	}
}

AlgorithmControl algorithm_control = null;

/************************************************************************************************
 * 
 * 	 Data source manager 
 *    - manage data sources 
 *    - read imu data from an imu sensor or a log file.
 * 
 ************************************************************************************************/
class DataSourceManager
{ 	
	private DataSource data_soruce = null;
	
	Listener[] listeners = null;
	
	public DataSourceManager(Listener [] l) {
		listeners = l;
	}
	
	public boolean start(String filename) {
		stop();
		data_soruce = new DataSourceLogFile(algorithm_control.get_algorithm(), filename);
		return start(); 
	}
	
	public boolean start(Serial serial_port) { 
		stop();
		data_soruce = new DataSourceSerialPort(algorithm_control.get_algorithm(), serial_port);
		return start(); 
	}
	
	public void stop() {
		if(data_soruce != null) {
			data_soruce.quit();
		}
		data_soruce = null;
	}
	
	private boolean start() {
		for(Listener o : listeners) {
			data_soruce.register_listener(o);
		}
		data_soruce.start();
		status_control.set_algorithm_name(data_soruce.get_algorithm_name());
		return true;
	}
}

DataSourceManager data_source_manager = null;

/************************************************************************************************
 * 
 * 	 
 * 
 ************************************************************************************************/
class FilePathControl
{
	Textfield tf_filename = null;
	Button btn_select_file;
	
	public FilePathControl(int pos_x, int pos_y) {
		btn_select_file = cp5.addButton("button_select_file")
		.setBroadcast(false)
		.setValue(0)
		.setLabel("select file")
		.setPosition(pos_x, pos_y)
		.setSize(55,20)
		.setBroadcast(true)
		;
		
		pos_x += 60;
		
		tf_filename = cp5.addTextfield("tf_filename")
		.setPosition(pos_x,10)
		.setSize(240,20)
		.setLabel("")
		.setText("")
		.setAutoClear(false)
		;	
	}
	
	public void set(String name) {
		tf_filename.setText(name);
	}
	
	public String get() {
		return tf_filename.getText();
	}
	
	public void show(boolean s) {
		if(s) {
			tf_filename.show();
			btn_select_file.show();
		}
		else {
			tf_filename.hide();
			btn_select_file.hide();
		}
	}
}

FilePathControl filepath_control;

void button_select_file(int theValue) {
	selectInput("Select a file to process:", "fileSelected");
}

void fileSelected(File selection) {
	if (selection == null) {
		println("Window was closed or the user hit cancel.");
	} else {
		println("User selected " + selection.getAbsolutePath());
		filepath_control.set(selection.getAbsolutePath());
	}
}

/************************************************************************************************
 * 
 * 	 
 * 
 ************************************************************************************************/
class SerialPortControl
{
	PApplet owner = null;
	
	Serial serial_port = null;
	
	ScrollableList sll_serial_port_list = null;
	ScrollableList sll_baudrate_list = null;
		
	String[] str_serial_port_list = null;
	String[] str_baudrate_list = null;
	
	int start_scan_time = 0;
	final int SCAN_PERIOD_MSEC = 500;
	
	public SerialPortControl(PApplet _owner, int pos_x, int pos_y) {
		
		owner = _owner;
		
		sll_serial_port_list = cp5.addScrollableList("serial ports") // addDropdownList
		          .setPosition(pos_x, pos_y)
		          .setSize(200,500)
		          .setBarHeight(20)
		          .setItemHeight(20)
		          .setOpen(false)
		          ;
		
		pos_x += 205;
		
		sll_baudrate_list = cp5.addScrollableList("baudrate")
		          .setPosition(pos_x, pos_y)
		          .setSize(95,500)
		          .setBarHeight(20)
		          .setItemHeight(20)
		          .setOpen(false)
		          ;
		
		String[] s = {"9600", "38400", "57600", "115200"};
		str_baudrate_list = s;
		sll_baudrate_list.addItems(str_baudrate_list);
		
		scan();
	}
	
	public void scan() {
		
		if(millis() - start_scan_time < SCAN_PERIOD_MSEC) {
			return;
		}
		
		start_scan_time = millis();
		
		str_serial_port_list = Serial.list();
		sll_serial_port_list.clear();
		sll_serial_port_list.addItems(str_serial_port_list);
	}
	
	public Serial open() {
		close();
		
		try {
			String port = get_port();
			int baudrate = get_baudrate();

			println(String.format("Open Serial %s(%d)", port, baudrate));

			serial_port = new Serial(owner, port, baudrate);
		} 
		catch (Exception e) {
			println("Exception " + e.toString());
		}
		
		return serial_port;
	}
	
	public void close() {
		if(serial_port != null) {
			serial_port.stop();
			serial_port = null;
		}
	}
	
	private String get_port() {
		return str_serial_port_list[int(sll_serial_port_list.getValue())];
	}
	
	private int get_baudrate() {
		return int(str_baudrate_list[int(sll_baudrate_list.getValue())]);
	}
	
	public void show(boolean s) {
		if(s) {
			sll_serial_port_list.show();
			sll_baudrate_list.show();
		}
		else {
			sll_serial_port_list.hide();
			sll_baudrate_list.hide();
		}
	}
}

SerialPortControl serial_port_control = null;

/************************************************************************************************
 * 
 * 	 
 * 
 ************************************************************************************************/

Imu3dModel imu_3d_model = null;

/************************************************************************************************
 * 
 * 	 
 * 
 ************************************************************************************************/

enum Input{FILE, SERIAL};
Input selected_input = Input.FILE;

Toggle bnt_start;

/************************************************************************************************
 * 
 * 	 
 * 
 ************************************************************************************************/

void setup() {
	size(640, 480, P3D);
	
	textureMode(NORMAL);
	frameRate(30);
	smooth();
	fill(255);
	stroke(color(0,0,0));
	
	//
	//
	//
	imu_3d_model = new Imu3dModel();
	imu_3d_model.setVisible(true);
	
	
	//
	// create controls 
	//
	cp5 = new ControlP5(this);
 
	status_control = new StatusControl(10, height-50);
	
	filepath_control = new FilePathControl(120,10);
	
	serial_port_control = new SerialPortControl(this, 120, 10);
	
	algorithm_control = new AlgorithmControl(425, 10);

	bnt_start = cp5.addToggle("toggle_start")
	.setBroadcast(false)
	.setLabel("start")
	.setPosition(10,10)
	.setSize(50,35)
	.setValue(false)
	.setBroadcast(true)
	;
	
	RadioButton rbtn = cp5.addRadioButton("radioButtonSelectInput")
	.setPosition(65,10)
	.setSize(20,17)
	.setColorForeground(color(120))
	.setColorActive(color(255))
	.setColorLabel(color(255))
	.setItemsPerRow(1)
	.setSpacingColumn(50)
	.addItem("FILE",1)
	.addItem("SERIAL",2) 
	;
	
	switch(selected_input) {
	case FILE: 
		radioButtonSelectInput(1);
		rbtn.activate(0);
		break;
		
	case SERIAL:
		radioButtonSelectInput(2);
		rbtn.activate(1);
		break;
		
	default:
		break;
	}

	
	Listener[] listeners = {imu_3d_model, status_control};
	data_source_manager = new DataSourceManager(listeners);
}


void draw() { 
	background(0);
	noStroke();
	
	pushMatrix();  
	{
		// camera manifulation 
		beginCamera();
		camera(0, 0, zoom_out, 
			   0, 0, 0, 
			   0.0, 1.0, 0.0);
		rotateX(rot_cam_x);
		rotateY(rot_cam_y);
		rotateZ(rot_cam_z);
		endCamera();

		// draw object 		
		imu_3d_model.display();
	}
	popMatrix();
	
	serial_port_control.scan();
	
	status_control.move(10, height-50);
}
 
/*
 * 	controlP5 event 
 */
void toggle_start(boolean theFlag) {
	if(theFlag==true) {
		bnt_start.setLabel("STOP"); 
		
		switch(selected_input) {
		case FILE:  
			{
				String f = filepath_control.get();				
				if(f.equals("") == false) {
					data_source_manager.start(f);
				} 
			}
			break;
			
		case SERIAL: 
			{
				Serial s = serial_port_control.open();
				if(s != null) {
					data_source_manager.start(s);
				}
			}
			break;
			
		default:
			break;
		}		
		
	} else {
		bnt_start.setLabel("START");
		serial_port_control.close();
		data_source_manager.stop();
	}
}


void radioButtonSelectInput(int a) {
	println("a radio Button event: "+a);
	
	switch(a) {
	case 1: 
		filepath_control.show(true);
		serial_port_control.show(false);
		selected_input = Input.FILE;
		break;
		
	case 2:
		filepath_control.show(false);
		serial_port_control.show(true);
		selected_input = Input.SERIAL;
		break;
		
	default:
		break;
	}
}


void mouseDragged() { 
	float rate = 0.01;
	rot_cam_x += (pmouseY-mouseY) * rate;
	rot_cam_z += (pmouseX-mouseX) * rate;
}


void mouseWheel(MouseEvent event) { 
	float e = event.getCount();
	zoom_out += e*5;

	final int ZOOM_MIN = 400;
	final int ZOOM_MAX = 2000;

	if(zoom_out < ZOOM_MIN) {
		zoom_out = ZOOM_MIN;
	}

	if(zoom_out > ZOOM_MAX) {
		zoom_out = ZOOM_MAX;
	} 
}