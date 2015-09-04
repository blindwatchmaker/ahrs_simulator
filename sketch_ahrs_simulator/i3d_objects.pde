/*
 *  2015.09.01 ('c')void 
 *  
 *  IMU 3D Model 
 *
 */

class Imu3dModel extends Listener
{
	//
	//  attitude stuff 
	// 
	EularAngle attitude = null; 
	
	PImage texture_image_pz = null;
	PImage texture_image_mz = null;
	PImage texture_image_px = null;
	PImage texture_image_mx = null;
	PImage texture_image_py = null;
	PImage texture_image_my = null;

	//
	final float deg2rad = PI/180;
	
	boolean visible = false;

	public Imu3dModel() {
		texture_image_pz = loadImage("texture/texture_+z.png"); // +z 
		texture_image_mz = loadImage("texture/texture_-z.png"); // -z 
		texture_image_px = loadImage("texture/texture_+x.png"); 
		texture_image_mx = loadImage("texture/texture_-x.png"); 
		texture_image_py = loadImage("texture/texture_+y.png");  
		texture_image_my = loadImage("texture/texture_-y.png"); 
		attitude = new EularAngle();
	}
	
	void notify(EularAngle eular_angle, ImuData imu_data) {
		attitude = eular_angle;
	}
	
	public void setVisible(boolean s) {
		visible = s;
	}

	public void display() {
		if(visible == false) {
			return;
		}
		
		colorMode(RGB, 255, 255, 255);

		int z_base_offset = -80;

		// draw Axis 
		pushMatrix();
		{ 
			translate(0, 0, z_base_offset + 4);
			stroke(255, 255, 255, 150);
			strokeWeight(2);
			noFill();

			for(int i=0; i<5; i++) {
				ellipse(0, 0, 400 - i*80, 400 - i*80);
			}

			rotateZ(-PI/2);
			textSize(50); 
			noStroke();

			// Axis X
			pushMatrix();
			{ 
				fill(255, 0, 0);   
				translate(0, -200, 0);
				draw_cylinder(0, 10, 30, 16); // arrow
				translate(0, 20, 0);
				draw_cylinder(3, 3, 180, 16);
			}
			popMatrix();

			// Axis Y
			pushMatrix();
			{ 
				fill(0, 255, 0);
				rotateZ(PI/2);
				translate(0, -200, 0);
				draw_cylinder(0, 10, 30, 16); // arrow
				translate(0, 20, 0);
				draw_cylinder(3, 3, 180, 16);
			}
			popMatrix();

			// Axis Z
			pushMatrix();
			{ 
				fill(0, 0, 255);
				rotateX(PI/2);
				translate(0, -100, 0);
				draw_cylinder(0, 10, 30, 16); // arrow
				translate(0, 20, 0);
				draw_cylinder(3, 3, 80, 16);
			}
			popMatrix();    

		}
		popMatrix();

		// draw ahrs model 
		pushMatrix();
		{    
			rotateZ(attitude.yaw * deg2rad);
			rotateY(attitude.pitch * deg2rad);
			rotateX(attitude.roll * deg2rad);

			pushMatrix();
			{// draw cube 
				scale(100, 80, 25);
				draw_textured_cube(texture_image_px, texture_image_mx,						
						texture_image_py, texture_image_my,
						texture_image_pz, texture_image_mz);
			}
			popMatrix();
		}
		popMatrix();		
	}

	private void draw_textured_cube(PImage tex_px, PImage tex_mx, PImage tex_py, PImage tex_my, PImage tex_pz, PImage tex_mz) {
		beginShape(QUADS);
		texture(tex_mz);
		// +Z face
		vertex(-1, -1, 1, 0, 0);
		vertex( 1, -1, 1, 1, 0);
		vertex( 1, 1, 1, 1, 1);
		vertex(-1, 1, 1, 0, 1);
		endShape();

		beginShape(QUADS);
		texture(tex_pz);
		// -Z face
		vertex( 1, -1, -1, 0, 0);
		vertex(-1, -1, -1, 1, 0);
		vertex(-1, 1, -1, 1, 1);
		vertex( 1, 1, -1, 0, 1);
		endShape();

		beginShape(QUADS);
		texture(tex_py);
		// +Y face
		vertex(-1, 1, 1, 0, 0);
		vertex( 1, 1, 1, 1, 0);
		vertex( 1, 1, -1, 1, 1);
		vertex(-1, 1, -1, 0, 1);
		endShape();
		
		beginShape(QUADS);
		texture(tex_my);		
		// -Y face
		vertex(-1, -1, -1, 0, 0);
		vertex( 1, -1, -1, 1, 0);
		vertex( 1, -1, 1, 1, 1);
		vertex(-1, -1, 1, 0, 1);
		endShape();
		
		beginShape(QUADS);
		texture(tex_px);
		// +X face
		vertex( 1, -1, 1, 0, 0);
		vertex( 1, -1, -1, 1, 0);
		vertex( 1, 1, -1, 1, 1);
		vertex( 1, 1, 1, 0, 1);
		endShape();

		beginShape(QUADS);
		texture(tex_mx);
		// -X face
		vertex(-1, -1, -1, 0, 0);
		vertex(-1, -1, 1, 1, 0);
		vertex(-1, 1, 1, 1, 1);
		vertex(-1, 1, -1, 0, 1);
		endShape();
	}

	private void draw_cylinder(float topRadius, float bottomRadius, float tall, int sides) {
		float angle = 0;
		float angleIncrement = TWO_PI / sides;
		beginShape(QUAD_STRIP);
		for (int i = 0; i < sides + 1; ++i) {
			vertex(topRadius*cos(angle), 0, topRadius*sin(angle));
			vertex(bottomRadius*cos(angle), tall, bottomRadius*sin(angle));
			angle += angleIncrement;
		}
		endShape();

		// If it is not a cone, draw the circular top cap
		if (topRadius != 0) {
			angle = 0;
			beginShape(TRIANGLE_FAN);

			// Center point
			vertex(0, 0, 0);
			for (int i = 0; i < sides + 1; i++) {
				vertex(topRadius * cos(angle), 0, topRadius * sin(angle));
				angle += angleIncrement;
			}
			endShape();
		}

		// If it is not a cone, draw the circular bottom cap
		if (bottomRadius != 0) {
			angle = 0;
			beginShape(TRIANGLE_FAN);

			// Center point
			vertex(0, tall, 0);
			for (int i = 0; i < sides + 1; i++) {
				vertex(bottomRadius * cos(angle), tall, bottomRadius * sin(angle));
				angle += angleIncrement;
			}
			endShape();
		}
	}
} // Imu3dModel

 

