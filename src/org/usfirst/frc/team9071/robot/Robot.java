package org.usfirst.frc.team9071.robot;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends SampleRobot {
	RobotDrive myRobot = new RobotDrive(0,2,1,3);
	Joystick stick = new Joystick(0);
	Joystick s2 = new Joystick(1);
	SpeedController sp = new Talon(4);
	SpeedController sp1 = new Talon(5);
	SpeedController sp2 = new Talon(6);
	SpeedController sp3 = new Talon(7);
	double st = 0.5;
	boolean fl = false;
	ADXRS450_Gyro myGyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
	Thread visionThread;

	public Robot() {
		myRobot.setExpiration(0.1);
	}
	@Override
	public void robotInit() {
		myRobot.setInvertedMotor(MotorType.kFrontRight, true);
		myRobot.setInvertedMotor(MotorType.kRearRight, true);
		//CameraServer.getInstance().addServer("server");
		//CameraServer.getInstance().addServer("c1", 1);
		sp2.set(.3);
		visionThread = new Thread(() -> {
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
			camera.setResolution(640,480);
			camera.setFPS(15);
			CvSink cvSink = CameraServer.getInstance().getVideo();
			CvSource outputStream = CameraServer.getInstance().putVideo("Rectangle", 640, 480);
			Mat mat = new Mat();
			while (!Thread.interrupted()) {
				if (cvSink.grabFrame(mat) == 0) {
					outputStream.notifyError(cvSink.getError());
					continue;
				}
				//Imgproc.rectangle(mat, new Point(100, 100), new Point(400, 400),
					//	new Scalar(255, 255, 255), 5);
				outputStream.putFrame(mat);
			}
		});
		visionThread.setDaemon(true);
		visionThread.start();
	}
	@Override
	public void autonomous() {
	}
	
	/*
     * This function helps you to get the magnitude and 
     * degree of a triangle
     *  @param x magnitude in X direction
     *  @param y magnitude in Y direction
     *  @param retDegree let the value returned be a angle is true
     */
    private double mathHelper(double x, double y, boolean retDegree) {
    	double hyp = Math.abs(Math.sqrt(x*x+y*y));
    	double deg = Math.toDegrees(Math.cos(y/hyp));
    	return retDegree ? deg : hyp;
    }
    
    private int joystickHelper(Joystick js,Joystick js1) {
    	if (js.getRawButton(1) || js1.getRawButton(1))
    		return 1;
    	if (js.getRawButton(2) || js1.getRawButton(2))
    		return 2;
    	if (js.getRawButton(3) || js1.getRawButton(3))
    		return 3;
    	if (js.getRawButton(4) || js1.getRawButton(4))
    		return 4;
    	if (js.getRawButton(5) || js1.getRawButton(5))
    		return 5;
    	if (js.getRawButton(6) || js1.getRawButton(6))
    		return 6;
    	if (js.getRawButton(7) || js1.getRawButton(7))
    		return 7;
    	if (js.getRawButton(8) || js1.getRawButton(8))
    		return 8;
    	if (js.getRawButton(9) || js1.getRawButton(9))
    		return 9;
    	if (js.getRawButton(10) || js1.getRawButton(10))
    		return 10;
    	if (js.getRawButton(11) || js1.getRawButton(11))
    		return 11;
    	if (js.getRawButton(12) || js1.getRawButton(12))
    		return 12;
    	return 0;
    }
    
	@Override
	public void disabled() {
		myRobot.stopMotor();
		sp2.stopMotor();
		sp1.set(-.3);
	}
	@Override
	public void operatorControl() {
		myRobot.setSafetyEnabled(true);
		while (isOperatorControl() && isEnabled()) {
			switch(joystickHelper(stick,s2)) {
			case 1:
				sp.set(.3);
				break;
			case 2:
				sp.set(-.5);
				break;
			case 3:
				sp1.set(-1);
				break;
			case 4:
				sp1.set(-.5);
				break;
			case 5:
				if(st < 1.0 && !fl) {
					st += .2;
					fl = true;
				}
				break;
			case 6:
				if(st > 0.0 && !fl) {
					fl = true;
					st -= .2;
				}
				break;
			default:
				sp3.stopMotor();
				fl = false;
				sp.stopMotor();
				sp1.stopMotor();
				break;
			}
			//myRobot.mecanumDrive_Cartesian(st*.5*-stick.getY(), st*.5*-stick.getX(), st*.5*stick.getZ(), 0);//myGyro.getAngle());
			myRobot.mecanumDrive_Polar(st*.5*-mathHelper(-stick.getY(),-stick.getX(),false), mathHelper(-stick.getY(),-stick.getX(),true), st*.5*stick.getZ());
			Timer.delay(0.005);
		}
	}
	@Override
	public void test() {
		myRobot.setSafetyEnabled(true);
		while (isOperatorControl() && isEnabled()) {
			myRobot.mecanumDrive_Cartesian(stick.getX(), stick.getY(), stick.getZ(), myGyro.getAngle());
			Timer.delay(0.005);
		}
	}
}

/*if(stick.getRawButton(1) || s2.getRawButton(1)) {
sp.set(.3);
}else if(stick.getRawButton(3) || s2.getRawButton(3)) {
sp1.set(-1);
}else if(stick.getRawButton(2) || s2.getRawButton(2)) {
sp.set(-.5);
}else if(stick.getRawButton(4) || s2.getRawButton(4)) {
sp1.set(-.5);
}else if(stick.getRawButton(5) || s2.getRawButton(5)){
if(st < 1.0 && !fl) {
	st += .2;
	fl = true;
}
}else if(stick.getRawButton(6) || s2.getRawButton(6)){
if(st > 0.0 && !fl) {
	fl = true;
	st -= .2;
}
}else{
sp3.stopMotor();
fl = false;
sp.stopMotor();
sp1.stopMotor();
}*/
