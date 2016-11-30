/****************************************
 * 
 *	THOMAS 2
 *	@author CyberCoyotes
 *
 ****************************************/

package org.usfirst.frc.team3603.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends IterativeRobot {
	
	Joystick xbox = new Joystick(1);
	Victor left1 = new Victor(1);
	Victor right1 = new Victor(2);
	Victor left2 = new Victor(3);
	Victor right2 = new Victor(4);
	
	AnalogGyro gyro = new AnalogGyro(0);
	
	RobotDrive mainDrive = new RobotDrive(left1, right1);
	RobotDrive mainDrive2 = new RobotDrive(left2, right2);
	RobotDrive mainDrive3 = new RobotDrive(left1, right1, left2, right2);
	
	Timer timer = new Timer();
	
	Encoder jerrie = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
	
    public void robotInit() {
    	gyro.reset();
    }
    
	public void autonomousInit() {
    }
	
    public void autonomousPeriodic() {
    }

    public void teleopPeriodic() {
    	timer.start();
    	jerrie.reset();
    	while (isOperatorControl() && isEnabled()) {
    		double magnitude1 = xbox.getRawAxis(1);
    		double magnitude2 = xbox.getRawAxis(5);
    		double all = magnitude1 + magnitude2;
    		
    		if(all>=0.25||all<=0.25)
		    	mainDrive.tankDrive(-magnitude1, -magnitude2);
		    	mainDrive2.tankDrive(-magnitude1, -magnitude2);
    		}
    		while(xbox.getRawButton(4)) {
    			mainDrive.tankDrive(0.5, 0.5);
    			mainDrive2.tankDrive(0.5, 0.5);
    		}
    		while(xbox.getRawButton(1)) {
    			mainDrive.tankDrive(-0.5,-0.5);
    			mainDrive2.tankDrive(-0.5,-0.5);
    		}
    		while(xbox.getRawButton(3)) {
    			mainDrive.tankDrive(0.75, -0.75);
    			mainDrive2.tankDrive(-0.75, 0.75);
    		}
    		while(xbox.getRawButton(2)) {
    			mainDrive.tankDrive(-0.75, 0.75);
    			mainDrive2.tankDrive(0.75, -0.75);
    		}
    		if(xbox.getRawButton(8)) {
    			double start = gyro.getAngle();
    			while(gyro.getAngle()<=start+90) {
    				mainDrive.tankDrive(0.75, -0.75);
    				mainDrive2.tankDrive(0.75, -0.75);
    			}
    			start = 0;
    		}
    		if(xbox.getRawButton(7)) {
    			double start = gyro.getAngle();
    			while(gyro.getAngle()>=start-90) {
    				mainDrive.tankDrive(-0.75, 0.75);
    				mainDrive2.tankDrive(-0.75, 0.75);
    			}
    			start = 0;
    		
    		}
    		int count = jerrie.get();
    		double distance = jerrie.getRaw();
    		double distance2 = jerrie.getDistance();
    		double rate = jerrie.getRate();
    		
    		
    		SmartDashboard.putNumber("distance in inches", rate);
    		
    		
    		
    		
    		
    		
    }
    
    public void testPeriodic() {
    
    }
    
}
