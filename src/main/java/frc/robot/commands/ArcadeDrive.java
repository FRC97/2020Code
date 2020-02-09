/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.JoystickMap; //Joy stick Subsystem class

/**
 * Class file resembles functions that of in Robot.java.
 * File non-functional as no tests to be run using this for drive train
 */
public class ArcadeDrive extends Command {
  private Joystick joystick;
  private DriveTrain m_DriveTrain;
  private double speed;
  private double Rotation;

  //Constructor method 
  public ArcadeDrive() {
    //init all field values
    initialize();
    // Use requires() here to declare subsystem dependencies
    requires(m_DriveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
   m_DriveTrain = new DriveTrain();

    //Init Joystick from USB 0
    joystick = new Joystick(0);
    
    //calling joystick class method Returns Double of Y-Axis
    speed = constrain(JoystickMap.joyStick.getRawAxis(JoystickMap.Yval)); 

  }

  //pass in target angle in deg, and Gyro
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

  public double Angle(double x, double y) { //Custom gyro Angle calculation method
    return Math.atan(x/y)/Math.PI;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {

    DriveTrain.m_drive.arcadeDrive(speed, Angle(1.0, 1.0)); //This is the same controls as in Robot
    //DriveTrain.arcadeDriver(SPEED, TURN);
  }

  /**
   * Method does not adjust percent toggle without negative values
   */
  private static double constrain(double num) {

    double joy = JoystickMap.joyStick.getRawAxis(JoystickMap.Yval) / 2; //makes -1 to 1 --> to -0.5 to 0.5

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

    DriveTrain.m_drive.arcadeDrive(0, 0); //

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
