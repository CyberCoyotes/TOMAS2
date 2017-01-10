package org.usfirst.frc.team3603.robot;

import edu.wpi.first.wpilibj.vision.AxisCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.image.BinaryImage;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.NIVisionException;
import edu.wpi.first.wpilibj.image.ParticleAnalysisReport;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc3603.RobotMap;

public class Camera extends Subsystem{

	public double getAngle(int target){
		return getVectorToImage(target)[1];
	}
	public double getDistance(int target){
		return getVectorToImage(target)[0];
	}

	@SuppressWarnings("deprecation")
	private double[] getVectorToImage(int target){
		double[] vector = {0.0, 0.0};//0 is magnitude, 1 is angle (positive is left negative is right)
		try {
			ColorImage colorImage = AxisCamera.getInstance("10.21.68.11").getImage();
			BinaryImage thresholdImage = colorImage.thresholdHSV(61, 191, 201, 255, 152, 255);//must change thresholding values
			colorImage.free();
			//			thresholdImage.write("thresholdImage0.png");
			BinaryImage convexHullImage = thresholdImage.convexHull(true);
			thresholdImage.free();
			BinaryImage noSmallObjectImage = convexHullImage.removeSmallObjects(true, 13);
			convexHullImage.free();
			ParticleAnalysisReport[] blobs = noSmallObjectImage.getOrderedParticleAnalysisReports();
			int numberOfBlobs = blobs.length;
			if(numberOfBlobs>0){
				SmartDashboard.putInt("Blobs", numberOfBlobs);
				int targetIndex = 0;
				switch(target){
				case RobotMap.kTop:
					int topMost = Integer.MAX_VALUE;
					for(int i=0; i<numberOfBlobs; i++){
						if(blobs[i].center_mass_y<topMost){
							topMost = blobs[i].center_mass_y;
							targetIndex = i;
						}
					}
				 	vector[0] = RobotMap.centerDistances[blobs[targetIndex].boundingRectHeight+1];
					break;

				case RobotMap.kBottom:
					int bottomMost = 0;
					for(int i=0; i<numberOfBlobs; i++){
						if(blobs[i].center_mass_y>bottomMost){
							bottomMost = blobs[i].center_mass_y;
							targetIndex = i;
						}
					}
					vector[0] = RobotMap.centerDistances[blobs[targetIndex].boundingRectHeight];
					break;

				case RobotMap.kLeft:
					int leftMost = Integer.MAX_VALUE;
					for(int i=0; i<numberOfBlobs; i++){
						if(blobs[i].center_mass_x<leftMost){
							leftMost = blobs[i].center_mass_x;
							targetIndex = i;
						}
					}
					vector[0] = RobotMap.middleDistances[blobs[targetIndex].boundingRectHeight+1];
					break;

				case RobotMap.kRight:
					int rightMost = 0;
					for(int i=0; i<numberOfBlobs; i++){
						if(blobs[i].center_mass_x>rightMost){
							rightMost = blobs[i].center_mass_x;
							targetIndex = i;
						}
					}
					vector[0] = RobotMap.middleDistances[blobs[targetIndex].boundingRectHeight+1];
					break;
				}
				vector[1] = 23.5 - 0.146875*blobs[targetIndex].center_mass_x;//angle in degrees between image center and normal from camera
				System.out.println("height "+blobs[targetIndex].boundingRectHeight);
				System.out.println("Image vector: magnitude="+vector[0]+" angle="+vector[1]);
				noSmallObjectImage.free();
			}
			else{
				SmartDashboard.putString("No blobs", "No blobs");
			}
		}
		catch (NIVisionException e) {
			e.printStackTrace();
		}
		return vector;
	}
	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
}