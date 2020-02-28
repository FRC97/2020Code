/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.opencv.core.KeyPoint;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.GripBallContourPipeline;
import frc.robot.subsystems.GripBallPipeline;
import frc.robot.subsystems.GripBallTwoPipeline;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  WPI_TalonSRX fr = new WPI_TalonSRX(0);
  WPI_TalonSRX fl  = new WPI_TalonSRX(1);
  WPI_TalonSRX br = new WPI_TalonSRX(2);
  WPI_TalonSRX bl = new WPI_TalonSRX(3);
  SpeedControllerGroup left = new SpeedControllerGroup(fl, bl);
  SpeedControllerGroup right = new SpeedControllerGroup(fr, br);
  DifferentialDrive drive = new DifferentialDrive(left, right);
  NetworkTable table;
  double[] areas;
  double[] defaultValue = new double[0];
  private static final int IMG_WIDTH = 320;
  private static final int IMG_HEIGHT = 240;
  VisionThread visionThread;
  private final Object imgLock = new Object();
  SmartDashboard board;
  CameraServer cserver;
  public double centerX;
  public double centerY;
  KeyPoint[] blobs;
  String aa;
  @Override
  public void robotInit() {
    table = NetworkTableInstance.getDefault().getTable("GRIP/mycontoursReport");
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setResolution(IMG_WIDTH, IMG_HEIGHT);

    visionThread = new VisionThread(camera, new GripBallTwoPipeline(), pipeline -> {
        
           blobs = pipeline.findBlobsOutput().toArray();
            
            synchronized (imgLock) {
              for(int i=0; i<blobs.length; i++){
              
                centerX = Math.round(blobs[i].pt.x);
                centerY = Math.round(blobs[i].pt.y);
              
            }
            }  
            
        
        
    });
    visionThread.start();
  }

  @Override
  public void teleopPeriodic() {
    double[] areas = table.getEntry("area").getDoubleArray(defaultValue);
    SmartDashboard.putNumber("bX", centerX);
    SmartDashboard.putNumber("bY", centerY);
    System.out.print("areas: " + centerX + "   " + centerY  );

    for (double area : areas) {
      System.out.print(area + " ");
    }
    if(centerX-140>0){
      drive.arcadeDrive(0.01, 0.5);
      
    }
    else if(centerX-60>0){
      drive.arcadeDrive(0,0);
    }
    else{
      drive.arcadeDrive(0.01,-0.5);
    }

    System.out.println();
  }
  @Override
  public void robotPeriodic() {
    
   
    //SmartDashboard.putNumber("NEO Position", testMotor.getEncoder().getPosition());
    //SmartDashboard.putNumber("NEO Velocity", testMotor.getEncoder().getVelocity());
    //SmartDashboard.putNumber("Motor Controller", TestController.get());
  }
  
  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

 
}
