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

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.GathererSub;
import frc.robot.subsystems.ShooterController;
import edu.wpi.first.cameraserver.CameraServer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private CANSparkMax FR = new CANSparkMax(RobotMap.FR, MotorType.kBrushless);
  private CANSparkMax FL = new CANSparkMax(RobotMap.FL, MotorType.kBrushless);
  private CANSparkMax BR = new CANSparkMax(RobotMap.BR, MotorType.kBrushless);
  private CANSparkMax BL = new CANSparkMax(RobotMap.BL, MotorType.kBrushless);
  //private CANSparkMax gatherer = new CANSparkMax(RobotMap.Gatherer, MotorType.kBrushless);
  private static final CANSparkMax topShooterMotor = new CANSparkMax(RobotMap.TS, MotorType.kBrushless);
  private static final CANSparkMax bottomShooterMotor =  new CANSparkMax(RobotMap.BS, MotorType.kBrushless);
  private static final WPI_VictorSPX indexMotor =  new WPI_VictorSPX(RobotMap.Indexer);
  private SpeedControllerGroup right;
  private SpeedControllerGroup left;
  private DifferentialDrive Drive;
  private Joystick joy = JoystickMap.joyStick;
  private AnalogGyro gyro;
  private CameraServer cameraServer = CameraServer.getInstance();
  private int reverseDrive = 1;
  private boolean gathering = false;
 // private CANSparkMax testMotor = new CANSparkMax(, MotorType.kBrushless);
  //public static final Ultrasonic.Unit kMillimeters;
  private AnalogPotentiometer ultrasonic = new AnalogPotentiometer(0);
  
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Speed Controller Group
    right = new SpeedControllerGroup(FR, BR);
    left = new SpeedControllerGroup(FL, BL);

    // Differential Driv Deadband percentage
    //Drive = new DifferentialDrive(left, right);
    //Drive.setDeadband(0.05);
    
    // init encocder  

    //init Server Camera
    //cameraServer.startAutomaticCapture(0);
    //init Gyro
    gyro = new AnalogGyro(1);
  }

  /**
   * Displays values into SmartDashboard
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Joy Stick X", joy.getRawAxis(JoystickMap.Xval));
    SmartDashboard.putNumber("Joy Stick Y", joy.getRawAxis(JoystickMap.Yval));
    SmartDashboard.putNumber("Joy Stick Slider", joy.getRawAxis(JoystickMap.slider));
    SmartDashboard.putNumber("Slider %-Value", sliderContrain());
    // SmartDashboard.putNumber("NEO BACK RIGHT", BR.get());
    // SmartDashboard.putNumber("NEO FRONT RIGHT", FR.get());
    // SmartDashboard.putNumber("NEO FRONT LEFT", FL.get());
    // SmartDashboard.putNumber("NEO BACK LEFT", BL.get());
    SmartDashboard.putNumber("Shooter RPM", shooterRPM());
    // SmartDashboard.putNumber("Gyro", gyro.getAngle());
    SmartDashboard.putNumber("Ultrasonic", ultrasonic.get());
    //SmartDashboard.putNumber("NEO Position", testMotor.getEncoder().getPosition());
    //SmartDashboard.putNumber("NEO Velocity", testMotor.getEncoder().getVelocity());
    //SmartDashboard.putNumber("Motor Controller", TestController.get());
  }
  
  @Override
  public void autonomousInit() {
    gyro.reset();
    //0.02s loop
    //gyro.setSensitivity();
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {

  }
  boolean shooting = false;
  @Override

  public void teleopPeriodic() {
    double speed = -contrain(joy.getRawAxis(JoystickMap.Yval)) * reverseDrive;
    double turn = contrain(joy.getRawAxis(JoystickMap.Xval)) * reverseDrive;

    //Drive.arcadeDrive(speed*0, turn*0);
    //double d = ultrasonic.getRangeMM() * 0.001

    if (joy.getRawButtonPressed(JoystickMap.button9P)) {
      reverseDrive*= -1;
    }
    if (joy.getRawButtonPressed(JoystickMap.button4P)) {
      gathering = !gathering;
      GathererSub.active(gathering);
    }
    if (joy.getRawButtonPressed(JoystickMap.triggerP)) {
      shooting = !shooting;
    }

    if (shooting){
      double shooterSpeed = ShooterController.Calculate(3);
      System.out.println(shooterSpeed);
      topShooterMotor.set(shooterSpeed);
      bottomShooterMotor.set(shooterSpeed);
      indexMotor.set(1);
    }
    else {
      topShooterMotor.set(0);
      bottomShooterMotor.set(0);
      indexMotor.set(0);
    }
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }
  private double contrain(double value){
    if (value > 1){
      return 1;
    }
    else if (value < -1){
      return -1;
    }
    else{
      return value;
    }
  }
  private double sliderContrain(){
    double v = joy.getRawAxis(JoystickMap.slider) - 1; //-1 to 1 turns to -2 to 0
    return Math.round((Math.abs(v/2)*100)); // reutrns 0 - 100 
  }

  private double shooterRPM(){
    //getVelocity returns motor speed in Rotations per minute
    double avgRpm = topShooterMotor.getEncoder().getVelocity();
    return avgRpm;
  }
}
