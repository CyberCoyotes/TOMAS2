/****************************************
 * 
 *	TOMAS 2
 *	@author CyberCoyotes
 *
 ****************************************/

package org.usfirst.frc.team3603.robot;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	
	Joystick joy1 = new Joystick(2);
	Joystick joy2 = new Joystick(3);

	ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	
	Victor backLeftMotor = new Victor(1);
	Victor backRightMotor = new Victor(2);
	Victor frontLeftMotor = new Victor(3);
	Victor frontRightMotor = new Victor(4);
	RobotDrive mainDrive = new RobotDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor); 
	                             //(frontLeft, rearLeft, frontRight rearRight);
	
	Encoder enc = new Encoder(0, 1, true, Encoder.EncodingType.k4X);
								//(Pin1, Pin2, invert read, EncodingType);
	
	ADXL362 accel = new ADXL362(Range.k8G);
	Timer timer = new Timer();
	
	CameraServer cam = CameraServer.getInstance();
	
	AnalogOutput pullGear = new AnalogOutput(0);
	
    public void robotInit() {
    	backRightMotor.setInverted(true);
    	frontRightMotor.setInverted(true);
    	gyro.calibrate();
    	gyro.reset();
    	enc.reset();
    	
    	enc.setMaxPeriod(.1);//Find ideal number
    	enc.setMinRate(0.1); //Find ideal rate
    	enc.setDistancePerPulse(5.5/12*Math.PI/4);
    	enc.setSamplesToAverage(7); //Find ideal number
    }
	public void autonomousInit() {
    }
	
    public void autonomousPeriodic() {
    	while(isAutonomous()) {
    		timer.start();
    		if(timer.get() <= 3 && enc.getDistance() <= 3) {
    			mainDrive.mecanumDrive_Cartesian(0, 0.75, 0, gyro.getAngle()); //Drive forwards for 3 seconds or three feet
    		}
    		if(timer.get() <= 6 && enc.getDistance() <= 6) {
    			mainDrive.mecanumDrive_Cartesian(0.75, 0, 0, gyro.getAngle()); //Drive right for 3 seconds or three feet
    		}
    		if(timer.get() <= 9 && enc.getDistance() <= 9) {
    			mainDrive.mecanumDrive_Cartesian(0, -0.75, 0, gyro.getAngle());//Drive back for 3 seconds or three feet
    		}
    		if(timer.get() <= 12 && enc.getDistance() <= 12) {
    			mainDrive.mecanumDrive_Cartesian(-0.75, 0, 0, gyro.getAngle());//Drive left for 3 seconds or three feet
    		}
    		if(timer.get() <= 15 && gyro.getAngle() < 90) {
    			mainDrive.mecanumDrive_Cartesian(0.75, 0, 0.75, 0);//Make an arc for 3 seconds or 90 degrees
    		}
    		if(timer.get() <= 20) {
    			mainDrive.mecanumDrive_Cartesian(0, 0, -1, gyro.getAngle());//Spin left really fast for 5 seconds
    		}
    	}
    }

    public void teleopPeriodic() {
    	enc.reset();
    	while (isOperatorControl() && isEnabled()) {
	    	if(joy1.getRawButton(1) || joy1.getRawButton(2) || joy1.getRawButton(3) || joy1.getRawButton(4) || joy1.getRawButton(5) || joy1.getRawButton(6) || joy1.getRawButton(7) || joy1.getRawButton(8) || joy1.getRawButton(9) || joy1.getRawButton(10) ||  joy2.getRawButton(1) || joy2.getRawButton(2) || joy2.getRawButton(3) || joy2.getRawButton(4) || joy2.getRawButton(5) || joy2.getRawButton(6) || joy2.getRawButton(7) || joy2.getRawButton(8) || joy2.getRawButton(9) || joy2.getRawButton(10) || joy1.getRawAxis(0) >= 0.05 || joy1.getRawAxis(1) >= 0.05 || joy1.getRawAxis(2) >= 0.05 || joy1.getRawAxis(3) >= 0.05 || joy1.getRawAxis(4) >= 0.05 || joy1.getRawAxis(5) >= 0.05 || joy1.getRawAxis(6) >= 0.05 || joy2.getRawAxis(0) >= 0.05 || joy2.getRawAxis(1) >= 0.05 || joy2.getRawAxis(2) >= 0.05 || joy2.getRawAxis(3) >= 0.05 || joy2.getRawAxis(4) >= 0.05 || joy2.getRawAxis(5) >= 0.05 || joy2.getRawAxis(6) >= 0.05 || joy1.getRawAxis(0) <= -0.05 || joy1.getRawAxis(1) <= -0.05 || joy1.getRawAxis(2) <= -0.05 || joy1.getRawAxis(3) <= -0.05 || joy1.getRawAxis(4) <= -0.05 || joy1.getRawAxis(5) <= -0.05 || joy1.getRawAxis(6) <= -0.05 || joy2.getRawAxis(0) <= -0.05 || joy2.getRawAxis(1) <= -0.05 || joy2.getRawAxis(2) <= -0.05 || joy2.getRawAxis(3) <= -0.05 || joy2.getRawAxis(4) <= -0.05 || joy2.getRawAxis(5) <= -0.05 || joy2.getRawAxis(6) <= -0.05) {
	    		/**********************
	    		*** DRIVER CONTROLS ***
	    		**********************/
	    		double x = Math.pow(joy1.getRawAxis(0), 3);
	    		double y = Math.pow(joy1.getRawAxis(1), 3);
	    		double rot = Math.pow(joy2.getRawAxis(0), 3);
	    		
	    		if(Math.abs(x)>=0.1 || Math.abs(y)>=0.1 || Math.abs(rot)>=0.1) {
	    			mainDrive.mecanumDrive_Cartesian(x, y, rot, 0);
	    		}
	    		
	    	} else {
	    		//Brake if the controllers don't read anything
	    		backLeftMotor.set(0);
	    		frontLeftMotor.set(0);
	    		backRightMotor.set(0);
	    		frontRightMotor.set(0);
	    	}
	    	
	    	try {
				Thread.sleep(5);
			} catch (InterruptedException e) {//This is to protect from over sampling
				e.printStackTrace();
			}

    		if(gyro.getAngle()>=360) {
    			gyro.reset();
    		}
    		if(gyro.getAngle()<=-360) {
    			gyro.reset();
    		}
    		SmartDashboard.putNumber("X-Axis", accel.getX()*32.174049);
    		SmartDashboard.putNumber("Y-Axis", accel.getY()*32.174049);
    		SmartDashboard.putNumber("Z-Axis", accel.getZ()*32.174049); //Gs to feet/s^2
    		SmartDashboard.putNumber("Rate", enc.getRate());
    		SmartDashboard.putNumber("Distance", enc.getDistance());
    		SmartDashboard.putNumber("Gyro Value", gyro.getAngle());
    		SmartDashboard.putNumber("Time", timer.get());
    	}
    }

	public void testPeriodic() {
    }
}
//cut my slice into pieces
//this is my plastic fork












