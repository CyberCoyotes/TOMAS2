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
	
	Joystick joy1 = new Joystick(2);
	Joystick joy2 = new Joystick(3);

	AnalogGyro gyro = new AnalogGyro(0);
	
	Victor backLeftMotor = new Victor(1);
	Victor backRightMotor = new Victor(2);
	Victor frontLeftMotor = new Victor(3);
	Victor frontRightMotor = new Victor(4);
	
	RobotDrive mainDrive = new RobotDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor); 
	                             //(frontLeft, rearLeft, frontRight rearRight);
	
	Encoder jerrie = new Encoder(0, 1, true, Encoder.EncodingType.k4X);
								//(Pin1, Pin2, invert read, EncodingType);
	Timer timer = new Timer();
	
    public void robotInit() {
    	gyro.initGyro();
    	gyro.calibrate();
    	gyro.reset();
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
	    	if(joy1.getRawButton(1) || joy1.getRawButton(2) || joy1.getRawButton(3) || joy1.getRawButton(4) || joy1.getRawButton(5) || joy1.getRawButton(6) || joy1.getRawButton(7) || joy1.getRawButton(8) || joy1.getRawButton(9) || joy1.getRawButton(10) || 
	    			joy2.getRawButton(1) || joy2.getRawButton(2) || joy2.getRawButton(3) || joy2.getRawButton(4) || joy2.getRawButton(5) || joy2.getRawButton(6) || joy2.getRawButton(7) || joy2.getRawButton(8) || joy2.getRawButton(9) || joy2.getRawButton(10) ||
	    			joy1.getRawAxis(0) >= 0.05 || joy1.getRawAxis(1) >= 0.05 || joy1.getRawAxis(2) >= 0.05 || joy1.getRawAxis(3) >= 0.05 || joy1.getRawAxis(4) >= 0.05 || joy1.getRawAxis(5) >= 0.05 || joy1.getRawAxis(6) >= 0.05 ||
	    			joy2.getRawAxis(0) >= 0.05 || joy2.getRawAxis(1) >= 0.05 || joy2.getRawAxis(2) >= 0.05 || joy2.getRawAxis(3) >= 0.05 || joy2.getRawAxis(4) >= 0.05 || joy2.getRawAxis(5) >= 0.05 || joy2.getRawAxis(6) >= 0.05 ||
	    			joy1.getRawAxis(0) <= -0.05 || joy1.getRawAxis(1) <= -0.05 || joy1.getRawAxis(2) <= -0.05 || joy1.getRawAxis(3) <= -0.05 || joy1.getRawAxis(4) <= -0.05 || joy1.getRawAxis(5) <= -0.05 || joy1.getRawAxis(6) <= -0.05 ||
	    			joy2.getRawAxis(0) <= -0.05 || joy2.getRawAxis(1) <= -0.05 || joy2.getRawAxis(2) <= -0.05 || joy2.getRawAxis(3) <= -0.05 || joy2.getRawAxis(4) <= -0.05 || joy2.getRawAxis(5) <= -0.05 || joy2.getRawAxis(6) <= -0.05) {
	    		
	    		/**********************
	    		*** DRIVER CONTROLS ***
	    		**********************/
	    		
	    		double x = -Math.pow(joy1.getRawAxis(0), 3);
	    		double y = -Math.pow(joy1.getRawAxis(1), 3);
	    		double rot = -Math.pow(joy2.getRawAxis(0), 3);
	    		
	    		if(Math.abs(x)>=0.1 || Math.abs(y)>=0.1 || Math.abs(rot)>=0.1) {
	    			mainDrive.mecanumDrive_Cartesian(x, rot, y, gyro.getAngle());
	    		}
    	
	    	} else {
	    		backLeftMotor.set(0);
	    		frontLeftMotor.set(0);
	    		backRightMotor.set(0);
	    		frontRightMotor.set(0);
	    	}
	    	
	    	try {
				Thread.sleep(10);
			} catch (InterruptedException e) {//This is to protect from over sampling
				e.printStackTrace();
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
    	}
    }
    public void testPeriodic() {
    }
}
//cut my slice into pieces
//this is my plastic fork