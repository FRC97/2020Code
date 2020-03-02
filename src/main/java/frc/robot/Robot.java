/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.Autonomous;
import frc.robot.commands.Shooter;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.JoystickController;

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
  private ArcadeDrive ADrive;
  private Autonomous auto;
  private JoystickController joystickController = new JoystickController();
  private Gyro gyro;
  
  //private [[TYPE_MOTORCONTROLLER]] TestController = new [[TYPE_MOTORCONTROLLER]](RobotMap.Test);
  
  private boolean testMode = false;

  @Override
  public void robotInit() {

  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Joy Stick X", joystickController.getX());
    SmartDashboard.putNumber("Joy Stick Y", joystickController.getY());
    SmartDashboard.putNumber("Joy Stick Slider", joystickController.getSlider());
    SmartDashboard.putNumber("Slider %-Value", ArcadeDrive.constrain(joystickController.getY()));
    SmartDashboard.putNumber("NEO BACK RIGHT", DriveTrain.BR.get());
    SmartDashboard.putNumber("NEO FRONT RIGHT", DriveTrain.FR.get());
    SmartDashboard.putNumber("NEO FRONT LEFT", DriveTrain.FL.get());
    SmartDashboard.putNumber("NEO BACK LEFT", DriveTrain.BL.get());
    SmartDashboard.putBoolean("TEST MOTOR TOGGLE", testMode);
    SmartDashboard.putNumber("Ultrasonic Distance from Wall", Autonomous.m_ultrasonic.getValue());
    //SmartDashboard.putNumber("NEO Position", testMotor.getEncoder().getPosition());
    //SmartDashboard.putNumber("NEO Velocity", testMotor.getEncoder().getVelocity());
    //SmartDashboard.putNumber("Motor Controller", TestController.get());
  }
  
  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {

    auto.execute();
    Shooter.shoot(Autonomous.canshoot);

  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {

    ADrive.execute();

    //testMotor.set(speed);
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }
  
}
