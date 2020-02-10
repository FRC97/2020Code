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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ShooterController;
import edu.wpi.cscore.UsbCamera;
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
  //private final ShooterController shooterController = new ShooterController();
  //private [[TYPE_MOTORCONTROLLER]] TestController = new [[TYPE_MOTORCONTROLLER]](RobotMap.Test);
  private Joystick joy = JoystickMap.joyStick;
  private SpeedControllerGroup right;
  private SpeedControllerGroup left;
  private DifferentialDrive Drive;
  private AnalogGyro gyro;
  private CameraServer cameraServer = CameraServer.getInstance();
  private int reverseDrive = 1;
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
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
    //init Server Camera
    cameraServer.startAutomaticCapture(0);
    //init Gyro
    gyro = new AnalogGyro(0);
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
    SmartDashboard.putNumber("NEO BACK RIGHT", BR.get());
    SmartDashboard.putNumber("NEO FRONT RIGHT", FR.get());
    SmartDashboard.putNumber("NEO FRONT LEFT", FL.get());
    SmartDashboard.putNumber("NEO BACK LEFT", BL.get());
    //SmartDashboard.putNumberArray("Motor Shaft Speed and RPM", speedAndRPM());
    SmartDashboard.putNumber("Gyro", gyro.getAngle());
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

    // Scheduler.getInstance().add();
    // Scheduler.getInstance().add();
  }

  @Override
  public void teleopPeriodic() {
    double speed = -contrain(joy.getRawAxis(JoystickMap.Yval)) * reverseDrive;
    double turn = contrain(joy.getRawAxis(JoystickMap.Xval)) * reverseDrive;
    Drive.arcadeDrive(speed*sliderContrain(), turn*(0.4));

   //shooterController.shoot(JoystickMap.joyStick.getRawButtonPressed(JoystickMap.triggerP));
    if (joy.getRawButtonPressed(JoystickMap.button9P)) {
      reverseDrive*= -1;
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
    double v = joy.getRawAxis(JoystickMap.slider) - 1; //-1 to 1 turns to -2 to 0
    return Math.abs(v/2);
  }
  
  private double[] speedAndRPM(){
    double[] thing = new double[2];

    /**
     * Averages all 4 motor RPM
     */
    double avgRpm = 
    (
    FR.getEncoder().getVelocity() + 
    FL.getEncoder().getVelocity() + 
    BR.getEncoder().getVelocity() + 
    BL.getEncoder().getVelocity()
    )/4;
    /**
     * RPM to m/s formula
     * Shaft radius: 0.004m
     * Link:
     * https://www.humblix.com/i6dea32/how-to-convert-angular-velocity-expressed-in-revolutions-per-second-rpm-to-linear-speed-expressed-in-meters-per-second-m-s
     */
    double shaftSpeed = ((Math.PI*2 * 0.004)/60)*avgRpm;

    thing[0] = shaftSpeed;
    thing[1] = avgRpm;

    return thing;
  }
}
