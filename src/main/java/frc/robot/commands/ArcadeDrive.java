/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.JoystickController;

public class ArcadeDrive extends Command {
  public DriveTrain m_DriveTrain = new DriveTrain();

  public final static double speed = constrain(JoystickController.Y_Value);

  public ArcadeDrive() {
    // Use requires() here to declare subsystem dependencies
    requires(m_DriveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  double Rotation;

  public boolean calcRotateVal(double targetAngle, Gyro gyro) {
    double error = targetAngle - gyro.getAngle();
    if (error > 2) {
      this.Rotation = error * 0.5;
      return false;
    } else {
      this.Rotation = 0;
      return true;
    }
  }

  public double getAngle(double x, double y) {
    return Math.atan(x/y)/Math.PI;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {

    DriveTrain.m_drive.arcadeDrive(speed, getAngle(JoystickController.X_Value, JoystickController.Y_Value));

  }

  private static double constrain(double num) {

    double joy = JoystickController.slider / 2;

    if (num > 1) {
      num = 1;
    } else if (num < -1) {
      num = -1;
    }

    double val = num * joy;

    return val;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {

    DriveTrain.m_drive.arcadeDrive(0, 0);

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
