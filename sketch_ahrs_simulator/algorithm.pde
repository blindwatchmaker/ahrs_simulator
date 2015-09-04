/*
 *  2015.09.01 ('c')void 
 *  
 *  Attitude Estimation Algorithm
 *  
 */


/************************************************************************************************
 * 
 * 	Interface of attitude estimation algorithm   
 * 
 ************************************************************************************************/
abstract class AttitudeEstimation extends Object
{
	protected float sampling_rate_hz;
	
	public AttitudeEstimation(float hz) {
		sampling_rate_hz = hz;
	}
	
	public abstract void update(ImuData imu_data);
	public abstract EularAngle get_eular_angle();
}


/************************************************************************************************
 * 
 * 	AttitudeEstimation Factory 
 * 
 ************************************************************************************************/

String [] attitude_estimation_algorithm_list = {"DCM", "USER", "TEST"};

AttitudeEstimation create_attitude_estimation_object(String algorithm_name, float sampling_rate_hz) { 
	if(algorithm_name.equals("DCM")) {
		return new AttitudeEstimationDCM(sampling_rate_hz);
	}
	else if(algorithm_name.equals("TEST")) {
		return new AttitudeEstimationTest(sampling_rate_hz);
	}
	else if(algorithm_name.equals("USER")) {
		return new AttitudeEstimationUser(sampling_rate_hz);
	}
	else {
		// default
		return new AttitudeEstimationDCM(sampling_rate_hz);
	}
}


/************************************************************************************************
 * 
 * 	Basic operation 
 * 
 ************************************************************************************************/
static class Mat3x3 
{	
	static void multiply(float[][] a, float[][] b,float[][] mat) {
		float[] op = new float[3];
		for(int x=0; x<3; x++) {
			for(int y=0; y<3; y++) {
				for(int w=0; w<3; w++) {
					op[w]=a[x][w]*b[w][y];
				}
				mat[x][y]=0;
				mat[x][y]=op[0]+op[1]+op[2];
			}
		}
	}	

	static void vector_multiply(float[][] a, float[] b, float[] out) {
		for(int x = 0; x < 3; x++) {
			out[x] = a[x][0] * b[0] + a[x][1] * b[1] + a[x][2] * b[2];
		}
	}

	static float [] get_row_vector(float[][] m, int row) {
		float[] row_vec = new float[3];
		for(int x = 0; x < 3; x++) {
			row_vec[x] = m[row][x];
		}
		return row_vec;
	}

	static void set_row_vector(float[][] m, int row, float[] row_vec) {
		for(int x = 0; x < 3; x++) {
			m[row][x] = row_vec[x] ;
		}
	}
}
 

static class Vec3
{
	static float dot_product(float[] vector1, float[] vector2) {
		float op=0;
		for(int c=0; c<3; c++) {
			op+=vector1[c]*vector2[c];
		}
		return op;
	}

	static double dot_product(double[] vector1, double[] vector2) {
		double op=0;
		for(int c=0; c<3; c++) {
			op+=vector1[c]*vector2[c];
		}
		return op;
	}

	static void cross_product(float[] vectorOut, float[] v1, float[] v2) {
		vectorOut[0]= (v1[1]*v2[2]) - (v1[2]*v2[1]);
		vectorOut[1]= (v1[2]*v2[0]) - (v1[0]*v2[2]);
		vectorOut[2]= (v1[0]*v2[1]) - (v1[1]*v2[0]);
	}

	static void scale(float[] vectorOut, float[] vectorIn, float scale) {
		for(int c=0; c<3; c++) {
			vectorOut[c]=vectorIn[c]*scale;
		}
	}

	static void add(float[] vectorOut, float[] vectorIn1, float[] vectorIn2) {
		for(int c=0; c<3; c++) {
			vectorOut[c] = vectorIn1[c] + vectorIn2[c];
		}
	}

	static void sub(float[] vectorOut, float[] vectorIn1, float[] vectorIn2) {
		for(int c=0; c<3; c++) {
			vectorOut[c] = vectorIn1[c] - vectorIn2[c];
		}
	}

	static float norm(float[] vector) {
		return sqrt(vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2]);
	}
}


/************************************************************************************************
 * 
 * 	AttitudeEstimationTest
 * 
 ************************************************************************************************/

class AttitudeEstimationTest extends AttitudeEstimation
{
	float count;
	
	public AttitudeEstimationTest(float hz) {
		super(hz);
		count = 0;
	}
	
	public void update(ImuData imu_data) {
		count += 0.01;
		//println("AttitudeEstimationTest.update() "+ imu_data.to_string());
	}
	
	public EularAngle get_eular_angle() {
		float roll = PI*sin(count * 0.1);
		float pitch = PI/2*sin(count * 0.2);
		float yaw = PI*sin(count * 0.3) + PI;	
		
		float rad2deg = 180/PI;
		
		return new EularAngle(roll*rad2deg, pitch*rad2deg, yaw*rad2deg);
	}
}


/************************************************************************************************
 * 
 * 	AttitudeEstimationDCM
 * 
 *  reference :
 *    - http://gentlenav.googlecode.com/files/DCMDraft2.pdf
 *    - https://github.com/ptrbrtz/razor-9dof-ahrs/tree/master/Arduino/Razor_AHRS
 *    
 ************************************************************************************************/

class AttitudeEstimationDCM extends AttitudeEstimation
{ 
	// Constants
	final float Kp_ROLLPITCH = 2.0;
	final float Ki_ROLLPITCH = 0.0002;

	final float Kp_YAW = 1.2;
	final float Ki_YAW = 0.00002;
	
	// DCM variables
	float MAG_Heading;
	float[] Accel_Vector; // Store the acceleration in a vector
	float[] Gyro_Vector;  // Store the gyros turn rate in a vector
	float[] Omega_Vector; // Corrected Gyro_Vector data

	float[] Omega_Prp;    // Omega Proportional correction (roll, pitch)
	float[] Omega_Py;     // Omega Proportional correction (yaw)

	float[] Omega_Irp;    // Omega Integrator (roll, pitch)
	float[] Omega_Iy;     // Omega Integrator (yaw)

	float[] Omega;  
	float[][] DCM_Matrix;  
	float[][] Update_Matrix;  
	float[][] Temporary_Matrix;  

	float Accel_magnitude;

	// Euler angles
	float yaw;
	float pitch;
	float roll;

	float G_Dt;
	
	public AttitudeEstimationDCM(float hz) {
		super(hz);
		
		Accel_Vector = new float[3]; 
		Gyro_Vector = new float[3];
		Omega_Vector = new float[3];
		
		Omega_Prp = new float[3];
		Omega_Py = new float[3];
		
		Omega_Irp = new float[3];
		Omega_Iy = new float[3];
		
		Omega = new float[3];
		
		DCM_Matrix = new float[3][3];
		Update_Matrix = new float[3][3];
		Temporary_Matrix = new float[3][3];
				
		for(int c=0; c<3; c++) {
			for(int r=0; r<3; r++) {
				DCM_Matrix[c][r] = 0;
				Update_Matrix[c][r] = 0;
				Temporary_Matrix[c][r] = 0;
			}
		}	
		
		DCM_Matrix[0][0] = 1;
		DCM_Matrix[1][1] = 1;
		DCM_Matrix[2][2] = 1;
		
		MAG_Heading = 0;
		roll = pitch = yaw = 0;
		Accel_magnitude = 0;
		G_Dt = 1.0/hz;
	}
	
	public void update(ImuData imu_data) {
		update(imu_data.gx, imu_data.gy, imu_data.gz, imu_data.ax, imu_data.ay, imu_data.az, imu_data.mx, imu_data.my, imu_data.mz);
	}
	
	public EularAngle get_eular_angle() {
		float rad2deg = 180/PI;
		EularAngle e = new EularAngle(roll*rad2deg, pitch*rad2deg, yaw*rad2deg);
		return e;
	}
	
	/*
	 *  센서좌표계로 표현한 가속도,각속도,지자기
	 * 	 - accel unit : g
	 * 	 - gyro  unit : rad/sec
	 * 	 - mag   unit : gauss
	 */
	private boolean update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {

		compass_heading(mx, my, mz);
		matrix_update(ax, ay, az, gx, gy, gz);
		normalize();
		drift_correction();
		euler_angles();

		return true;
	}
	
	private void matrix_update(float ax, float ay, float az, float gx, float gy, float gz) {
		float[] Omega_rp = new float[3];// {0, 0, 0}; // Omega Integrator
		float[] Omega_y = new float[3];// {0, 0, 0};

		Gyro_Vector[0] = gx; //gyro x roll
		Gyro_Vector[1] = gy; //gyro y pitch
		Gyro_Vector[2] = gz; //gyro z yaw

		// gravity 
		Accel_Vector[0] = -ax;
		Accel_Vector[1] = -ay;
		Accel_Vector[2] = -az;

		Vec3.add(Omega_rp, Omega_Prp, Omega_Irp);
		Vec3.add(Omega_y, Omega_Py, Omega_Iy);

		Vec3.add(Omega_Vector, Gyro_Vector, Omega_rp);
		Vec3.add(Omega_Vector, Omega_Vector, Omega_y);

		Update_Matrix[0][0] =  0;
		Update_Matrix[0][1] = -G_Dt*Omega_Vector[2]; //-z
		Update_Matrix[0][2] =  G_Dt*Omega_Vector[1]; // y

		Update_Matrix[1][0] =  G_Dt*Omega_Vector[2]; // z
		Update_Matrix[1][1] =  0;
		Update_Matrix[1][2] = -G_Dt*Omega_Vector[0]; //-x

		Update_Matrix[2][0] = -G_Dt*Omega_Vector[1]; //-y
		Update_Matrix[2][1] =  G_Dt*Omega_Vector[0]; // x
		Update_Matrix[2][2] =  0;

		Mat3x3.multiply(DCM_Matrix,Update_Matrix,Temporary_Matrix); //a*b=c

		for(int x=0; x<3; x++) { //Matrix Addition (update)
			for(int y=0; y<3; y++) {
				DCM_Matrix[x][y] += Temporary_Matrix[x][y];
			}
		}
	}

	private void normalize() {
		float error=0;
		float renorm=0;
		
		float[] temp_row_0 = new float[3];
		float[] temp_row_1 = new float[3];
		float[] temp_row_2 = new float[3];
		float[] temp = new float[3];
		
		error= -Vec3.dot_product(Mat3x3.get_row_vector(DCM_Matrix, 0), Mat3x3.get_row_vector(DCM_Matrix, 1))*0.5; //eq.19
		
		Vec3.scale(temp_row_0, Mat3x3.get_row_vector(DCM_Matrix, 1), error); //eq.19
		Vec3.scale(temp_row_1, Mat3x3.get_row_vector(DCM_Matrix, 0), error); //eq.19
		
		Vec3.add(temp_row_0, temp_row_0, Mat3x3.get_row_vector(DCM_Matrix, 0));//eq.19
		Vec3.add(temp_row_1, temp_row_1, Mat3x3.get_row_vector(DCM_Matrix, 1));//eq.19
		
		Vec3.cross_product(temp_row_2, temp_row_0, temp_row_1); // c= a x b //eq.20
		
		renorm= 0.5 *(3 - Vec3.dot_product(temp_row_0, temp_row_0)); //eq.21
		Vec3.scale(temp, temp_row_0, renorm);
		Mat3x3.set_row_vector(DCM_Matrix, 0, temp);

		renorm= 0.5 *(3 - Vec3.dot_product(temp_row_1, temp_row_1)); //eq.21
		Vec3.scale(temp, temp_row_1, renorm);
		Mat3x3.set_row_vector(DCM_Matrix, 1, temp);

		renorm= 0.5 *(3 - Vec3.dot_product(temp_row_2, temp_row_2)); //eq.21
		Vec3.scale(temp, temp_row_2, renorm);
		Mat3x3.set_row_vector(DCM_Matrix, 2, temp);
	}

	private void drift_correction() {
		float mag_heading_x;
		float mag_heading_y;
		float errorCourse;
		
		//Compensation the Roll, Pitch and Yaw drift.
		float[] Scaled_Omega_I = new float [3];

		float[] errorRollPitch = new float [3];
		float[] errorYaw = new float [3];

		float Accel_weight;
		boolean bound = false;

		// Calculate the magnitude of the accelerometer vector
		Accel_magnitude = Vec3.norm(Accel_Vector);

		//*****Roll and Pitch***************
		// Dynamic weighting of accelerometer info (reliability filter)
		// Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)

		Accel_weight = constrain(1 - 2*abs(1 - Accel_magnitude), 0, 1, bound);
		
		Vec3.cross_product(errorRollPitch, Accel_Vector, Mat3x3.get_row_vector(DCM_Matrix, 2)); //adjust the ground of reference
		
		Vec3.scale(Omega_Prp, errorRollPitch, Kp_ROLLPITCH*Accel_weight);

		Vec3.scale(Scaled_Omega_I, errorRollPitch, Ki_ROLLPITCH*Accel_weight);
		Vec3.add(Omega_Irp, Omega_Irp, Scaled_Omega_I); // integration

		//*****YAW***************
		// We make the gyro YAW drift correction based on compass magnetic heading

		mag_heading_x = cos(MAG_Heading);
		mag_heading_y = sin(MAG_Heading);

		errorCourse=(DCM_Matrix[0][0]*mag_heading_y) - (DCM_Matrix[1][0]*mag_heading_x);  //Calculating YAW error
		Vec3.scale(errorYaw, Mat3x3.get_row_vector(DCM_Matrix, 2),errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.

		Vec3.scale(Omega_Py, errorYaw, Kp_YAW); // proportional of YAW.

		Vec3.scale(Scaled_Omega_I, errorYaw, Ki_YAW); // Integrator
		Vec3.add(Omega_Iy, Omega_Iy, Scaled_Omega_I); // integration
	}

	private void euler_angles() {
		// radian
		pitch = -asin(DCM_Matrix[2][0]); // -pi/2 ~ pi/2
		roll  =  atan2(DCM_Matrix[2][1], DCM_Matrix[2][2]); // -pi ~ pi
		yaw   =  atan2(DCM_Matrix[1][0], DCM_Matrix[0][0]); // -pi ~ pi
		if(yaw < 0) {
			yaw += 2*PI; // 0 ~ 2pi
		}
	}

//	void Accel_adjust(void) {
//		Accel_Vector[1] += Accel_Scale(speed_3d*Omega[2]);  // Centrifugal force on Acc_y = GPS_speed*GyroZ
//		Accel_Vector[2] -= Accel_Scale(speed_3d*Omega[1]);  // Centrifugal force on Acc_z = GPS_speed*GyroY
//	}

	private void compass_heading(float magnetom_x, float magnetom_y, float magnetom_z) {
		float mag_x;
		float mag_y;
		float cos_roll;
		float sin_roll;
		float cos_pitch;
		float sin_pitch;

		cos_roll = cos(roll);
		sin_roll = sin(roll);
		cos_pitch = cos(pitch);
		sin_pitch = sin(pitch);

		// Tilt compensated magnetic field X
		mag_x = magnetom_x * cos_pitch + magnetom_y * sin_roll * sin_pitch + magnetom_z * cos_roll * sin_pitch;

		// Tilt compensated magnetic field Y
		mag_y = magnetom_y * cos_roll - magnetom_z * sin_roll;

		// Magnetic Heading
		MAG_Heading = atan2(-mag_y, mag_x);
	}

	private float constrain(float x, float min, float max, boolean bound) {
		if(x < min) {
			bound = true;
			return min;
		}

		if(x > max) {
			bound = true;
			return max;
		}

		bound = false;
		return x;
	}
} // AttitudeEstimationDCM

/************************************************************************************************
 * 
 * 	AttitudeEstimationUser
 * 
 *   - 알고리즘 추가 예 
 * 
 ************************************************************************************************/

class AttitudeEstimationUser extends AttitudeEstimation
{
	float count;
	
	public AttitudeEstimationUser(float hz) {
		super(hz);
		count = 0;
	}
	
	public void update(ImuData imu_data) {
		count += 0.01;
		//println("AttitudeEstimationUser.update() "+ imu_data.to_string());
	}
	
	public EularAngle get_eular_angle() {
		float roll = PI*sin(count * 0.1);
		float pitch = PI/2*sin(count * 0.2);
		float yaw = PI*sin(count * 0.3) + PI;	
		
		float rad2deg = 180/PI;
		
		return new EularAngle(roll*rad2deg, pitch*rad2deg, yaw*rad2deg);
	}
}

 






