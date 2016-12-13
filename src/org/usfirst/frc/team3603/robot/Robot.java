/****************************************
 * 
 *	THOMAS 2
 *	@author CyberCoyotes
 *
 ****************************************/

//
package org.usfirst.frc.team3603.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	
	Joystick xbox1 = new Joystick(0);
	Joystick xbox2 = new Joystick(1);
	Victor left1 = new Victor(1);
	Victor right1 = new Victor(2);
	Victor left2 = new Victor(3);
	Victor right2 = new Victor(4);
	
	AnalogGyro gyro = new AnalogGyro(0);
	RobotDrive mainDrive = new RobotDrive(left2, left1, right2, right1); 
	                             //(frontLeft, rearLeft, frontRight rearRight)
	
	Encoder jerrie = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
	DoubleSolenoid thrower = new DoubleSolenoid(0, 1);
	Value out = DoubleSolenoid.Value.kForward;
	Value in = DoubleSolenoid.Value.kForward;
	Timer timer = new Timer();
	
	AnalogInput right = new AnalogInput(1);
	AnalogInput mid = new AnalogInput(2);
	AnalogInput left = new AnalogInput(3);
	
    public void robotInit() {
    	gyro.reset();
    	gyro.setSensitivity(0.5);
    	jerrie.reset();
    	timer.start();
    	
    	jerrie.setMaxPeriod(.1);
    	jerrie.setMinRate(10);
    	jerrie.setDistancePerPulse(0.25);
    	jerrie.setSamplesToAverage(7);
    }
    
	public void autonomousInit() {
    }
	
    public void autonomousPeriodic() {
    }

    public void teleopPeriodic() {
    	jerrie.reset();
    	while (isOperatorControl() && isEnabled()) {
	    	if(xbox1.getRawButton(1) || xbox1.getRawButton(2) || xbox1.getRawButton(3) || xbox1.getRawButton(4) || xbox1.getRawButton(5) || xbox1.getRawButton(6) || xbox1.getRawButton(7) || xbox1.getRawButton(8) || xbox1.getRawButton(9) || xbox1.getRawButton(10) || 
	    			xbox2.getRawButton(1) || xbox2.getRawButton(2) || xbox2.getRawButton(3) || xbox2.getRawButton(4) || xbox2.getRawButton(5) || xbox2.getRawButton(6) || xbox2.getRawButton(7) || xbox2.getRawButton(8) || xbox2.getRawButton(9) || xbox2.getRawButton(10) ||
	    			xbox1.getRawAxis(1) >= 0.05 || xbox1.getRawAxis(2) >= 0.05 || xbox1.getRawAxis(3) >= 0.05 || xbox1.getRawAxis(4) >= 0.05 || xbox1.getRawAxis(5) >= 0.05 || xbox1.getRawAxis(6) >= 0.05 ||
	    			xbox2.getRawAxis(1) >= 0.05 || xbox2.getRawAxis(2) >= 0.05 || xbox2.getRawAxis(3) >= 0.05 || xbox2.getRawAxis(4) >= 0.05 || xbox2.getRawAxis(5) >= 0.05 || xbox2.getRawAxis(6) >= 0.05 ||
	    			xbox1.getRawAxis(1) <= -0.05 || xbox1.getRawAxis(2) <= -0.05 || xbox1.getRawAxis(3) <= -0.05 || xbox1.getRawAxis(4) <= -0.05 || xbox1.getRawAxis(5) <= -0.05 || xbox1.getRawAxis(6) <= -0.05 ||
	    			xbox2.getRawAxis(1) <= -0.05 || xbox2.getRawAxis(2) <= -0.05 || xbox2.getRawAxis(3) <= -0.05 || xbox2.getRawAxis(4) <= -0.05 || xbox2.getRawAxis(5) <= -0.05 || xbox2.getRawAxis(6) <= -0.05) {
	    		/**********************
	    		*** DRIVER CONTROLS ***
	    		**********************/
	    		double mag1 = Math.pow(xbox1.getRawAxis(1), 3);
	    		double mag2 = Math.pow(xbox1.getRawAxis(5), 3);
	    		//
	    		if(Math.abs(mag1)>=0.1 || Math.abs(mag2)>=0.1) {
	    			mainDrive.tankDrive(mag1, mag2);
	    		}
	    		
	    		while(xbox1.getRawButton(1)) {
	    			mainDrive.tankDrive(0.75, 0.75);
	    		}
	    		double max = gyro.getAngle()+5;
				double min = gyro.getAngle()-5;
	    		while(xbox1.getRawButton(2)) {
	    			if(gyro.getAngle()>=min&&gyro.getAngle()<=max) {
	    				left1.set(-0.75);
	    				left2.set(0.75);
	    				right1.set(-0.75);
	    				right2.set(0.75);
	    			}
	    			if(gyro.getAngle()>max) {
	    				left1.set(-0.6);
	    				left2.set(0.9);
	    				right1.set(-0.9);
	    				right2.set(0.6);
	    			}
	    			if(gyro.getAngle()<min) {
	    				left1.set(-0.9);
	    				left2.set(0.6);
	    				right1.set(-0.6);
	    				right2.set(0.9);
	    			}
	    		}
	    		while(xbox1.getRawButton(3)) {
	    			if(gyro.getAngle()>=min && gyro.getAngle()<=max) {
	    				left1.set(0.75);
	    				left2.set(-0.75);
	    				right1.set(0.75);
	    				right2.set(-0.75);
	    			}
	    			if(gyro.getAngle()>max) {
	    				left1.set(0.6);
	    				left2.set(-0.9);
	    				right1.set(0.9);
	    				right2.set(-0.6);
	    			}
	    			if(gyro.getAngle()<min) {
	    				left1.set(0.9);
	    				left2.set(-0.6);
	    				right1.set(0.6);
	    				right2.set(-0.9);
	    			}
	    		}
	    		while(xbox1.getRawButton(4)) {
	    			mainDrive.tankDrive(-0.75, -0.75);
	    		}
	    		while(xbox1.getRawAxis(2)>=0.2) {
	    			mainDrive.tankDrive(xbox1.getRawAxis(2), -xbox1.getRawAxis(2));
	    		}
	    		while(xbox1.getRawAxis(3)>=0.2) {
	    			mainDrive.tankDrive(-xbox1.getRawAxis(3), xbox1.getRawAxis(3));
	    		}
	    		
	    		/****************************
	    		 *** MANIPULATOR CONTROLS *** Reserved for testing at the moment
	    		 ****************************/
	    		double x = Math.pow(xbox2.getRawAxis(2), 3);
	    		double y = Math.pow(xbox2.getRawAxis(1), 3);
	    		double rot = Math.pow(xbox2.getRawAxis(4), 3);
	    		
	    		if(Math.abs(x)>=0.1 || Math.abs(y)>=0.1 || Math.abs(rot)>=0.1) {
	    			mainDrive.mecanumDrive_Cartesian(x, y, rot, 0);
	    		}
	    		
	    		if(xbox2.getRawButton(5)) {
	    			thrower.set(out);
	    		} else {
	    			thrower.set(in);
	    		}
	    		
	    		if(xbox2.getRawButton(1)) {
	    			left1.set(0.75);
	    		}
	    		if(xbox2.getRawButton(2)) {
	    			left2.set(0.75);
	    		}
	    		if(xbox2.getRawButton(3)) {
	    			right1.set(0.75);
	    		}
	    		if(xbox2.getRawButton(4)) {
	    			right2.set(0.75);
	    		}
	    		/**Sensor reading section**/
	    		while(xbox2.getRawButton(6)) {
		    		int r = right.getValue();
		    		int m = mid.getValue();
		    		int l = left.getValue();
		    		if(r>=4000) {
		    			mainDrive.tankDrive(0.5, -0.5);
		    		}
		    		if(m>=4000) {
		    			mainDrive.tankDrive(0, 0);
		    		}
		    		if(l>=4000) {
		    			mainDrive.tankDrive(-0.5, 0.5);
		    		}
	    		}
	    		
	    		if(gyro.getAngle()>=360) {
	    			gyro.reset();
	    		}
	    		if(gyro.getAngle()<=-360) {
	    			gyro.reset();
	    		}
	    		
	    		SmartDashboard.putNumber("Rate", jerrie.getRate());
	    		SmartDashboard.putNumber("Distance", jerrie.getDistance());
	    		SmartDashboard.putNumber("Gyro Value", gyro.getAngle());
	    		SmartDashboard.putNumber("Time", timer.get());
    	
	    	} else {
	    		mainDrive.tankDrive(0, 0);
	            left1.stopMotor();
	            left2.stopMotor();
	            right1.stopMotor();
	            right2.stopMotor();
	    	}
    	}
    }
    public void testPeriodic() {
    }
}
//cut my slice into pieces
//this is my plastic fork