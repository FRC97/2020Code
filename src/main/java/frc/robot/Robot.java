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

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


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
  private CANSparkMax FR = new CANSparkMax(RobotMap.FR, MotorType.kBrushless);
  private CANSparkMax FL = new CANSparkMax(RobotMap.FL, MotorType.kBrushless);
  private CANSparkMax BR = new CANSparkMax(RobotMap.BR, MotorType.kBrushless);
  private CANSparkMax BL = new CANSparkMax(RobotMap.BL, MotorType.kBrushless);
  
  
  // private WPI_TalonSRX TopShooterMotor = new WPI_TalonSRX(RobotMap.TS);
  // private WPI_TalonSRX BottomShooterMotor = new WPI_TalonSRX(RobotMap.BS);

  // private WPI_VictorSPX DeployClimbMotor = new WPI_VictorSPX(RobotMap.Dc);
  // private WPI_VictorSPX RetractClimbMotor = new WPI_VictorSPX(RobotMap.Rc);

  private WPI_VictorSPX GathererMotor = new WPI_VictorSPX(RobotMap.Gatherer);
  private WPI_VictorSPX IndexerMotor = new WPI_VictorSPX(RobotMap.Indexer);
  
  //private [[TYPE_MOTORCONTROLLER]] TestController = new [[TYPE_MOTORCONTROLLER]](RobotMap.Test);



  private SpeedControllerGroup right;
  private SpeedControllerGroup left;

  private DifferentialDrive Drive;
  
  private boolean testMode = false;

  @Override
  public void robotInit() {
    // Inverted settings
    // FR.setInverted(false);
    // BR.setInverted(false);
    // FL.setInverted(false);
    // BL.setInverted(false);

    right = new SpeedControllerGroup(FR, BR);
    left = new SpeedControllerGroup(FL, BL);

    // Differential Driv Deadband percentage
    Drive = new DifferentialDrive(left, right);
    Drive.setDeadband(0.05);

    // init encocder  
    /*
    RightMaster.setSensorPhase(true);
    RightMaster.setSensorPhase(false);

    RightMaster.setSelectedSensorPosition(0, 0, 10);
    LeftMaster.setSelectedSensorPosition(0, 0, 10);
    */
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Joy Stick X", joy.getRawAxis(0));
    SmartDashboard.putNumber("Joy Stick Y", joy.getRawAxis(1));
    SmartDashboard.putNumber("Joy Stick Slider", joy.getRawAxis(3));
    SmartDashboard.putNumber("Slider %-Value", sliderContrain());
    SmartDashboard.putNumber("NEO BACK RIGHT", BR.get());
    SmartDashboard.putNumber("NEO FRONT RIGHT", FR.get());
    SmartDashboard.putNumber("NEO FRONT LEFT", FL.get());
    SmartDashboard.putNumber("NEO BACK LEFT", BL.get());
    SmartDashboard.putBoolean("TEST MOTOR TOGGLE", testMode);
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

  @Override
  public void teleopPeriodic() {
    double speed = -contrain(joy.getRawAxis(1));
    double turn = contrain(joy.getRawAxis(0));
    Drive.arcadeDrive(speed*sliderContrain(), turn);

    if (joy.getRawButton(2)){ 
      GathererMotor.set(joy.getRawAxis(3));
    }
    else{
      GathererMotor.set(0);
    }
    if (joy.getRawButton(4)){ 
      IndexerMotor.set(-.5);
    }
    else{
      IndexerMotor.set(0);
    }
    
    if (testMode){
      if (joy.getRawButton(5)){
        FR.set(0.5);
        FL.set(0);
        BR.set(0);
        BL.set(0);
      }
      if (joy.getRawButton(6)){ 
        FR.set(0);
        FL.set(0.5);
        BR.set(0);
        BL.set(0);
      }
      if (joy.getRawButton(7)){ 
        FR.set(0);
        FL.set(0);
        BR.set(0.5);
        BL.set(0);
      }
      if (joy.getRawButton(8)){
        FR.set(0);
        FL.set(0);
        BR.set(0);
        BL.set(0.5);
      }
    }
    if (joy.getRawButtonPressed(9)) {
      testMode = !testMode;
    }
    //testMotor.set(speed);
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
    double v = joy.getRawAxis(3) - 1; //-1 to 1 turns to -2 to 0
    return Math.abs(v/2);
  }
}
