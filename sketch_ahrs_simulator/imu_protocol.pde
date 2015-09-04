/*
 *  2015.09.02 ('c')void 
 *  
 *  IMU sensor protocol 
 *
 */

/************************************************************************************************
 * 
 *   Interface of IMU Protocol object 
 * 
 ************************************************************************************************/
abstract class ImuProtocol
{ 
	private ArrayList<ImuData> imu_data_queue;
	
	public ImuProtocol() {
		imu_data_queue = new ArrayList<ImuData>();
	}
	
	public int size() {
		return imu_data_queue.size();
	}
		
	public void push(String msg) {
		ImuData d = parse(msg);
		if(d != null) {
			imu_data_queue.add(d);
		}
	}
	
	public ImuData pop() {
		if(imu_data_queue.size() == 0) {
			return null;
		}
		else {
			ImuData d = imu_data_queue.get(0);
			imu_data_queue.remove(d);
			return d;
		}		
	}
	
	// 
	// interface for derived class 
	// 

	public abstract float get_sampling_rate_hz();
	
	// parse message from imu module or file 
	protected abstract ImuData parse(String message);
	
	// initialize imu module. optional
	public boolean init_imu(Serial port) {
		return true;
	} 
}


ImuProtocol create_imu_protocol() {
	return new ImuProtocolMyAhrsPlus();
}


/************************************************************************************************
 * 
 *   myAHRS+ protocol  
 * 
 ************************************************************************************************/

class ImuProtocolMyAhrsPlus extends ImuProtocol
{	
	float get_sampling_rate_hz() {
		return 100; // Hz 
	}
	
	// initialize myahrs+ 
	public boolean init_imu(Serial port) { 
		port.write("@asc_out,RPYIMU*26\n");
		delay(10);
		port.write("@mode,A*2E\n");
		delay(10);
		port.write("@divider,1*3C\n"); // 100 Hz 
		delay(10);
		
		return true;
	}
	
	public ImuData parse(String message) {
		String [] fields = splitTokens(message, ",");
				
		if(fields.length != 15 || "$RPYIMU".equals(fields[0]) == false) {
			println(String.format("error unknown message(%s) ", message));
			return null;
		} 
		
	    final int SEQ   = 1;
	    final int ROLL  = 2;
	    final int PITCH = 3;
	    final int YAW   = 4;
	    final int AX    = 5;
	    final int AY    = 6;
	    final int AZ    = 7;
	    final int GX    = 8;
	    final int GY    = 9;
	    final int GZ    = 10;
	    final int MX    = 11;
	    final int MY    = 12;
	    final int MZ    = 13;
	    
	    final float deg2rad = PI/180;
	    
	    ImuData imu_data = new ImuData();
	    
	    imu_data.sequence = int(fields[1]);
	    
		// g 	
	    imu_data.ax = float(fields[AX]);
	    imu_data.ay = float(fields[AY]);
	    imu_data.az = float(fields[AZ]);

	    // dps -> rad/sec
	    imu_data.gx = float(fields[GX])*deg2rad;
	    imu_data.gy = float(fields[GY])*deg2rad;
	    imu_data.gz = float(fields[GZ])*deg2rad;

	    // no unit 
	    imu_data.mx = float(fields[MX]);
	    imu_data.my = float(fields[MY]);
	    imu_data.mz = float(fields[MZ]);
		
		return imu_data;
	}
}
