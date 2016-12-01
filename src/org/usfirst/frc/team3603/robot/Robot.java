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
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends IterativeRobot {
	
	Joystick xbox = new Joystick(0);
	Victor left1 = new Victor(1);
	Victor right1 = new Victor(2);
	Victor left2 = new Victor(3);
	Victor right2 = new Victor(4);
	
	AnalogGyro gyro = new AnalogGyro(0);
	
	RobotDrive mainDrive = new RobotDrive(left2, left1, right2, right1); 
	                                    //(frontLeft, rearLeft, frontRight rearRight)
	
	Encoder jerrie = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
	
    public void robotInit() {
    	gyro.reset();
    }
    
	public void autonomousInit() {
    }
	
    public void autonomousPeriodic() {
    }

    public void teleopPeriodic() {
    	jerrie.reset();
    	while (isOperatorControl() && isEnabled()) {
    		double mag1 = xbox.getRawAxis(1);
    		double mag2 = xbox.getRawAxis(5);
    		double all = mag1 + mag2;
    		
    		if(all>=0.25||all<=0.25) {
    			mainDrive.tankDrive(-mag1, -mag2);
    		}
    		if(xbox.getRawButton(1)) {
    			mainDrive.tankDrive(0.75, 0.75);
    		}
    		
    		double rate = gyro.getAngle();
    		SmartDashboard.putNumber("ratethingy", rate);
    	}
    }
    
    public void testPeriodic() {
    
    }
}
