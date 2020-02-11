/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.Ultrasonic.Unit;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.DriveTrain;

public class Autonomous extends CommandBase {
  /**
   * Creates a new Autonomous.
   */

  public DriveTrain m_DriveTrain = new DriveTrain();
  public Gyro gyro;
  public Ultrasonic ultra1 = new Ultrasonic(RobotMap.pingChannel1, RobotMap.echoChannel1, Unit.kMillimeters);
  public Ultrasonic ultra2 = new Ultrasonic(RobotMap.pingChannel2, RobotMap.echoChannel2, Unit.kMillimeters);
  public double desiredvalue1 = 1;
  public double desiredvalue2 = 2;
  public static boolean canshoot = false;
  double initial;

  public Autonomous() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    initial = gyro.getAngle();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double ul1 = ultra1.pidGet(); //forward
    double ul2 = ultra2.pidGet(); //side

    double zRotation = Math.atan((ul1 - desiredvalue1) / (ul2 - desiredvalue2)) / Math.PI;

    if (ul1 < desiredvalue1 - 1 || ul1 > desiredvalue1 + 1 || 
    ul2 < desiredvalue2 - 1 || ul2 > desiredvalue2 + 1) {

      DriveTrain.m_drive.arcadeDrive(0.5, zRotation);

    } else {

      if (gyro.getAngle() != initial) {
        DriveTrain.m_drive.tankDrive(0.1, -0.1);
      } else {
        canshoot = true;
      }

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
