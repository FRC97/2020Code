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
import edu.wpi.first.wpilibj.AnalogInput;
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
  private AnalogInput ultrasonic = new AnalogInput(0);
  private long voltsPerMilimeter = (5/1024);
  
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
    SmartDashboard.putNumber("Slider %-Value", sliderContrain());
    SmartDashboard.putBoolean("Gathering", gathering);
    SmartDashboard.putBoolean("Shooting", shooting);
    SmartDashboard.putNumber("Shooter RPM", shooterRPM());
    SmartDashboard.putNumber("Gyro", gyro.getAngle());
    SmartDashboard.putNumber("Ultrasonic", (ultrasonic.getAverageVoltage()/512) * 2.54);
  }
  
  @Override
  public void autonomousInit() {
    gyro.reset();
    initialGyro = gyro.getAngle();
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void autonomousPeriodic() {
    double currentGyro = gyro.getAngle();
    double dist = 5*(ultrasonic.getValue()/voltsPerMilimeter)*0.001;

    if (m_timer.get() < 2) {

      Drive.arcadeDrive(0.4, 0);

    } else if (m_timer.get() > 2.2 && m_timer.get() < 5) {

      double shooterSpeed = ShooterController.Calculate(3);
      System.out.printf("The Shooter Speed: %s",shooterSpeed);
      topShooterMotor.set(shooterSpeed);
      bottomShooterMotor.set(shooterSpeed);

      if (m_timer.get() > 3.3) {

        rampClean.set(-Math.abs(Math.cos(m_timer.get())));
        indexMotor.set(1);

      }
    } else if (!turnDone) {

      if (currentGyro > initialGyro - 88) {
        Drive.tankDrive(0.3, -0.3);
      } else if (currentGyro < initialGyro - 92) {
        Drive.tankDrive(-0.3, 0.3);
      }
      
      if (currentGyro <= initialGyro - 88 && currentGyro >= initialGyro - 92) {
        turnDone = true;
      }

    } else if (turnDone && !distanceToTargetMet) {

      if (dist < 30) {
        Drive.arcadeDrive(0.2, 0);
      } else if (dist > 50) {
        Drive.arcadeDrive(-0.2, 0);
      } else {
        distanceToTargetMet = true;
      }

    } 
    
    if (distanceToTargetMet) {

      if (currentGyro > initialGyro + 2) {
        Drive.tankDrive(0.3, -0.3);
      } else if (currentGyro < initialGyro - 2) {
        Drive.tankDrive(-0.3, 0.3);
      }

    } else {

      Drive.arcadeDrive(0, 0);
      topShooterMotor.set(0);
      bottomShooterMotor.set(0);
      rampClean.stopMotor();;
      indexMotor.set(0);

    }
  } 

  @Override
  public void teleopInit() {
    reverseDrive *= -1;
    //topShooterMotor.getEncoder().setVelocityConversionFactor(Math.PI/30);
  }
  boolean shooting = false;

  @Override
  public void teleopPeriodic() {
    double speed = -contrain(joy.getRawAxis(JoystickMap.Yval)) * reverseDrive;
    double turn = contrain(joy.getRawAxis(JoystickMap.Xval)) * reverseDrive;
    double zTurn = contrain(joy.getRawAxis(JoystickMap.Zval)) * reverseDrive;

    Drive.arcadeDrive((speed*sliderContrain()/100), turn*0.4);
    double distance = 5*(ultrasonic.getValue());

    if (joy.getRawAxis(JoystickMap.Zval) > 0 || joy.getRawAxis(JoystickMap.Zval) < 0) {
      Drive.tankDrive(-zTurn * sliderContrain()/100, zTurn * sliderContrain()/100);
    }
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
      GathererSub.active(gathering, "For");
    }

    if (joy.getRawButton(JoystickMap.button5P)) {
      paraCord.set(1.0);
    } else if (joy.getRawButton(JoystickMap.button6P)) {
      paraCord.set(-1.0);
    }
    else{
      paraCord.set(0);
    }

    if (joy.getRawButton(JoystickMap.button7P)){
      hookLift.set(0.4);
    }
    else if (joy.getRawButton(JoystickMap.button8P)) {
      hookLift.set(-0.4);

    }
    else{
      hookLift.set(0);
    }
    if (joy.getRawButton(JoystickMap.button10P)) {
      gathering = !gathering;
      GathererSub.active(gathering, "rev");
    }
    if (shooting){
      double shooterSpeed = ShooterController.Calculate(4);
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
