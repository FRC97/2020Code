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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


//import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.command.Scheduler;
//import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.DriveTrain;
//import frc.robot.subsystems.JoystickController;
//import frc.robot.subsystems.PID_Drive;
import frc.robot.subsystems.Shooter;
/*import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;*/
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static DriveTrain driveTrain;
  //public static OI m_oi;
  public static Joystick m_stick;
  public static Shooter shooter;
  public RobotMap map;
  //public ArcadeDrive m_ADrive;
  //private final DifferentialDrive m_robotDrive = new DifferentialDrive(Left, Right);
  //private final Timer m_timer = new Timer();
  //public PID_Drive piddrive = new PID_Drive(1, 1, 1);
  NetworkTable table;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  private WPI_VictorSPX FR = new WPI_VictorSPX(RobotMap.FR);
  private WPI_VictorSPX FL = new WPI_VictorSPX(RobotMap.FL);
  private CANSparkMax testMotor = new CANSparkMax(10, MotorType.kBrushless);;
  //private WPI_TalonSRX TestController = new WPI_TalonSRX(0);
  private WPI_TalonSRX BR = new WPI_TalonSRX(RobotMap.BR);
  private WPI_TalonSRX BL = new WPI_TalonSRX(RobotMap.BL);

  private Joystick joy = new Joystick(0);

  private SpeedControllerGroup right;
  private SpeedControllerGroup left;

  private DifferentialDrive Drive;

  @Override
  public void robotInit() {
    // Inverted settings
    FR.setInverted(false);
    BR.setInverted(false);
    FL.setInverted(true);
    BL.setInverted(true);

    right = new SpeedControllerGroup(FR, BR);
    left = new SpeedControllerGroup(BR, BL);

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
    SmartDashboard.putNumber("NEO Position", testMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("NEO Velocity", testMotor.getEncoder().getVelocity());
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
    
    Drive.arcadeDrive(speed, turn);
    testMotor.set(speed);
    //Scheduler.getInstance().run();
    //piddrive.setSetpoint(getAngle(m_stick.getX(),m_stick.getY()));
    //DriveTrain.m_drive.arcadeDrive(m_stick.getY()-0.1,constrain(getAngle(m_stick.getX(),m_stick.getY())));
    //piddrive.execute(m_stick.getY()-0.1);
    shooter.shoot(joy.getRawButtonPressed(0));
    /*
    if (m_stick.getX() < 0.1 && m_stick.getX() > -0.1) {
      x_value = 0;
    } else {
      x_value = m_stick.getX();
    }
    */
    //m_ADrive.execute();
    //m_robotDrive.arcadeDrive(m_stick.getY(), x_value);
    //front_Left.set(ControlMode.PercentOutput, m_stick.getX());
    //back_Left.set(ControlMode.PercentOutput, m_stick.getY()+0.1);
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
}
