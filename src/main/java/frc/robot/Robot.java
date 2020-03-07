/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SPI.Port;
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
  private static final CANSparkMax topShooterMotor = new CANSparkMax(RobotMap.TS, MotorType.kBrushless);
  private static final CANSparkMax bottomShooterMotor =  new CANSparkMax(RobotMap.BS, MotorType.kBrushless);
  private static final WPI_VictorSPX indexMotor =  new WPI_VictorSPX(RobotMap.Indexer);
  private CANSparkMax paraCord = new CANSparkMax(RobotMap.Cord, MotorType.kBrushed);
  private CANSparkMax hookLift = new CANSparkMax(RobotMap.Hook, MotorType.kBrushed);
  private Servo rampClean = new Servo(0);
  private final Timer m_timer = new Timer();
  private SpeedControllerGroup right;
  private SpeedControllerGroup left;
  private DifferentialDrive Drive;
  private Joystick joy = JoystickMap.joyStick;
  private ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  private CameraServer cameraServer1 = CameraServer.getInstance();
  //private CameraServer cameraServer2 = CameraServer.getInstance();
  private int reverseDrive = 1;
  private boolean gathering = false;
  private boolean optimalDistanceMet = false;
  private boolean turnDone = false;
  private boolean distanceToTargetMet = false;
  private double initialGyro = 0.0;
 // private CANSparkMax testMotor = new CANSparkMax(, MotorType.kBrushless);
  //public static final Ultrasonic.Unit kMillimeters;. 
  private AnalogPotentiometer ultrasonic = new AnalogPotentiometer(0, 1023, 0);
  private final double voltsPerMilimeter = (5/1024);
  
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
    Drive = new DifferentialDrive(left, right);
    Drive.setDeadband(0.05);
    
    // init encocder  

    //init Server Camera
    cameraServer1.startAutomaticCapture("Shooter Cam", 0);
    //cameraServer2.startAutomaticCapture("Gatherer Cam", 1);
    //init Gyro
    gyro.calibrate();
    
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
    SmartDashboard.putNumber("Gyro", gyro.getAngle());
    SmartDashboard.putNumber("Ultrasonic", (5*(ultrasonic.get()/voltsPerMilimeter))*0.001);
    //SmartDashboard.putNumber("NEO Position", testMotor.getEncoder().getPosition());
    //SmartDashboard.putNumber("NEO Velocity", testMotor.getEncoder().getVelocity());
    //SmartDashboard.putNumber("Motor Controller", TestController.get());
  }
  
  @Override
  public void autonomousInit() {
    //gyro.reset();
    //initialGyro = gyro.getAngle();
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void autonomousPeriodic() {
    double dist = 5*(ultrasonic.get()/voltsPerMilimeter)*0.001;
    double currentGyro = gyro.getAngle();
    double time = 0.0;
    System.out.println(dist);
    // if (!optimalDistanceMet) {
    //   if (dist < 2.15) {
    //     Drive.arcadeDrive(-0.4, 0);
    //   } 
    //   if (dist > 2.55) {
    //     Drive.arcadeDrive(0.4, 0);
    //   }
    //   if (dist > 2.15 && dist < 2.55) {
    //     optimalDistanceMet = true;
    //   }
    // }
    // if (optimalDistanceMet) {
    //   if (currentGyro < initialGyro - 92) {
    //     Drive.tankDrive(-0.4, 0.4);
    //   }
    //   if (currentGyro > initialGyro - 88) {
    //     Drive.tankDrive(0.4, -0.4);
    //   }
    //   if (currentGyro > (initialGyro - 95) && currentGyro < (initialGyro - 85)) {
    //     turnDone = true;
    //   }
    // }
    if (!turnDone) {
      if (dist < 2.8){
        Drive.arcadeDrive(-0.4, 0);
      }
      if (dist > 3.2){
        Drive.arcadeDrive(0.4, 0);
      }
      if (dist >= 2.8 && dist <= 3.2){
        distanceToTargetMet = true;
      }
    }
    if (distanceToTargetMet){
      time = m_timer.get();
      double shooterSpeed = ShooterController.Calculate(3);
      System.out.printf("The Shooter Speed: %s",shooterSpeed);
      topShooterMotor.set(shooterSpeed);
      bottomShooterMotor.set(shooterSpeed);
      rampClean.set(-Math.abs(Math.cos(m_timer.get())));
      indexMotor.set(1);
    }
    if (m_timer.get() > time + 5) {
      topShooterMotor.set(0);
      bottomShooterMotor.set(0);
      rampClean.stopMotor();
      indexMotor.set(0);
    }
  } 

  @Override
  public void teleopInit() {
    //topShooterMotor.getEncoder().setVelocityConversionFactor(Math.PI/30);
  }
  boolean shooting = false;

  @Override
  public void teleopPeriodic() {
    double speed = -contrain(joy.getRawAxis(JoystickMap.Yval)) * reverseDrive;
    double turn = contrain(joy.getRawAxis(JoystickMap.Xval)) * reverseDrive;

    Drive.arcadeDrive(speed*sliderContrain()/100, turn*0.4);
    double distance = 5*(ultrasonic.get()/voltsPerMilimeter)*0.001;

    if (joy.getRawButtonPressed(JoystickMap.triggerP)) {
      shooting = !shooting;
    }
    if (joy.getRawButtonPressed(JoystickMap.button2P)) {
      reverseDrive*= -1;
    }
    if (joy.getRawButton(JoystickMap.button3P)) {
      indexMotor.set(1);
      rampClean.set(-Math.abs(Math.cos(m_timer.get())));
    }
    else{
      indexMotor.set(0);
      rampClean.stopMotor();
    }
    if (joy.getRawButtonPressed(JoystickMap.button4P)) {
      gathering = !gathering;
      GathererSub.active(gathering);
    }

    if (joy.getRawButton(JoystickMap.button5P)) {
      paraCord.set(1);
    }
    else if (joy.getRawButton(JoystickMap.button6P)) {
      paraCord.set(-1);
    }
    else{
      paraCord.set(0);
    }

    if (joy.getRawButton(JoystickMap.button7P)){
      hookLift.set(0.2);
    }
    else if (joy.getRawButton(JoystickMap.button8P)) {
      hookLift.set(-0.2);
      paraCord.set(-1);

    }
    else{
      hookLift.set(0);
    }

    if (shooting){
      double shooterSpeed = ShooterController.Calculate(3);
      System.out.printf("The Shooter Speed: %s\nDistance: %s",shooterSpeed, distance);
      topShooterMotor.set(shooterSpeed);
      bottomShooterMotor.set(shooterSpeed);
      rampClean.set(-Math.abs(Math.cos(m_timer.get())));
      indexMotor.set(1);
    }
    else {
      topShooterMotor.set(0);
      bottomShooterMotor.set(0);
      rampClean.stopMotor();
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
