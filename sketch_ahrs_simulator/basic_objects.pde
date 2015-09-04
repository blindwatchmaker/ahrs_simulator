/*
 *  2015.09.01 ('c')void 
 *  
 *  Basic object 
 *
 */

class ImuData {
	public int sequence;
	public float ax, ay, az; // g
	public float gx, gy, gz; // radian/sec 
	public float mx, my, mz; // none  
	
	ImuData() {
		sequence = 0;
		ax = ay = az = 0;
		gx = gy = gz = 0;
		mx = my = mz = 0;
	}
	
	String to_string() {
		return String.format("S(%d) A(%.4f,%.4f,%.4f) G(%.4f,%.4f,%.4f) M(%.4f,%.4f,%.4f)", sequence, ax, ay, az, gx, gy, gz, mx, my, mz);
	}
}


class EularAngle {
	public float roll, pitch, yaw; // degree 
	
	EularAngle() {
		roll = 0;
		pitch = 0;
		yaw = 0;
	}
	
	EularAngle(float r, float p, float y) {
		roll = r;
		pitch = p;
		yaw = y;
	}
}
