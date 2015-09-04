/*
 *  2015.09.01 ('c')void 
 *  
 *  Data source  
 *
 */

import java.util.ArrayList;

import processing.serial.*;


/************************************************************************************************
 * 
 * 	Interface of Listener
 * 
 ************************************************************************************************/

abstract class Listener
{
	public abstract void notify(EularAngle eular_angle, ImuData imu_data);
}

/************************************************************************************************
 * 
 * 	Interface of data sorce
 *   - serial port or log file 
 * 
 ************************************************************************************************/
abstract class DataSource extends Thread {
	protected boolean running; 
	private ArrayList<Listener> listener_list = null;
	
	// algorithm 
	protected AttitudeEstimation attitude_estimation;
	
	// IMU message parser 
	protected ImuProtocol imu_protocol;
	
	protected float sampling_rate_hz;
	
	public DataSource(String algorithm_name) { 
		running = false;
		listener_list = new ArrayList<Listener>();
		
		imu_protocol = create_imu_protocol();
		
		sampling_rate_hz = imu_protocol.get_sampling_rate_hz();
		
		attitude_estimation = create_attitude_estimation_object(algorithm_name, sampling_rate_hz);
	}
	
	public void finalize() {
		quit();
	}

//	public void start() {}  

	void run() { 
		try {
			running = true;
			thread_proc();
		} 
		catch (Exception e) {
		}
	}

	public void quit() {
		running = false;   
		interrupt();
	}
	
	public String get_algorithm_name() {
		String name = attitude_estimation.getClass().getName();
		String [] s = splitTokens(name, "$");
		
		if(s.length == 1) {
			return name;
		}
		else {
			return s[s.length-1]; 
		} 
	}
	
	public void register_listener(Listener o) {
		listener_list.add(o);
	}
	
	protected void feed(ImuData imu_data) {  
		if(imu_data == null) {
			return;
		}
		
		// attitude estimation
		attitude_estimation.update(imu_data);
		
		// notification
		for(int i=0; i<listener_list.size(); i++) {
			listener_list.get(i).notify(attitude_estimation.get_eular_angle(), imu_data);
		}
	}
	
	protected abstract void thread_proc();
}


/************************************************************************************************
 * 
 * 	File reader 
 * 
 ************************************************************************************************/
class DataSourceLogFile extends DataSource {
	String lines[];
	int period_msec;
	
	public DataSourceLogFile(String algorithm_name, String filename) {
		super(algorithm_name);
		
		period_msec = int(1000.0/sampling_rate_hz);
		lines = loadStrings(filename);  
		
		println(String.format("Starting DataSourceLogFile(%s)", filename));
	}
	
	void thread_proc() {
		
		println(String.format("Starting DataSourceLogFile. %s", String.valueOf(running)));
		
		for(int i=0; i<lines.length; i++) {
			if(running == false) {
				break;
			}
			
			delay(period_msec); 
						
			imu_protocol.push(lines[i]); 
			ImuData data = imu_protocol.pop();
			if(data != null) {
				feed(data);
			}
		}
		
		println("end of file");
	}
}


/************************************************************************************************
 * 
 * 	Serial port reader 
 * 
 ************************************************************************************************/

static ArrayList<DataSourceSerialPort> serial_source_list;

/*
 *  Serial event handler 
 */
void serialEvent(Serial port) {  
	try {
		for (DataSourceSerialPort s : serial_source_list) {
			if(s.serial_port == port) {
				s.read();
			}
		}
	}
	catch(Exception e) {
		println("serialEvent exception " + e);
	}
}


class DataSourceSerialPort extends DataSource {
	Serial serial_port;
	
	public DataSourceSerialPort(String algorithm_name, Serial port) {		
		super(algorithm_name);
		
		serial_port = port;
		
		if (serial_source_list == null) {
			serial_source_list = new ArrayList<DataSourceSerialPort>();
		}

		serial_source_list.add(this);
		
		// initialize imu 
		imu_protocol.init_imu(serial_port);
	}
	
	public void finalize() {
		serial_source_list.remove(this);
	}
	
	public void read() {
		String msg = serial_port.readString();
		imu_protocol.push(msg); 
	}
	
	void thread_proc() {
		println("Starting DataSourceSerialPort");
		
	    serial_port.clear();
	    serial_port.bufferUntil('\n');
	    
		while(true) { 
			ImuData data = imu_protocol.pop();
			if(data != null) {
				feed(data);
			}
			else {
				// waiting 
				delay(1);
				continue;
			} 
		}
	}
}

