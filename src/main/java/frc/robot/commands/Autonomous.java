/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PID_Drive;

public class Autonomous extends CommandBase {
  /**
   * Creates a new Autonomous.
   */

  // distance in inches the robot wants to stay from an object
  private static final double kHoldDistance = 109.0;
  private static final double kDistanceFromWallToTrench = 28.0;

  // factor to convert sensor values to a distance in inches
  private static final double kValueToInches = 0.125;

  // proportional speed constant
  private static final int kUltrasonicPort = 0;

  public final static AnalogInput m_ultrasonic = new AnalogInput(kUltrasonicPort);

  public DriveTrain m_DriveTrain = new DriveTrain();
  public Gyro gyro;
  public static boolean canshoot = false;
  double initial = 0;

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

    int count = 0;
    double currentDistance = m_ultrasonic.getValue() * kValueToInches;
    double zRotation = gyro.getAngle();

    PID_Drive.setpoint(initial);

    while (!PID_Drive.vision()) {

      PID_Drive.turnInPlace(0.2);

    }

    if (currentDistance > kHoldDistance + 1 || currentDistance < kHoldDistance - 1) {

      PID_Drive.drive(0.2, kHoldDistance, currentDistance);
      

    } else {

      canshoot = true;
      count = 1;

    }

    if (count == 1) {

      while (zRotation < initial - 89 || zRotation > initial - 91) { 

        PID_Drive.executeTurn(0.2, -5);

      }

      if (currentDistance > kDistanceFromWallToTrench + 1 || currentDistance < kDistanceFromWallToTrench -1) {

        PID_Drive.drive(0.5, kDistanceFromWallToTrench, currentDistance);

      }

      

      // if (PID_Drive.vision()) {
         //Go drive towards trench
      //   PID_Drive.PID_DriveFirst(vision.get());

      // }

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
