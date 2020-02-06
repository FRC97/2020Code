/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.JoystickController;
import frc.robot.subsystems.ShooterSub;

public class Shooter extends CommandBase {

  static double topspeed = 0;
  static double bottomspeed = 0;

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    ShooterSub.top.set(ControlMode.PercentOutput, 0);
    ShooterSub.bottom.set(ControlMode.PercentOutput, 0);

  }

  public void setSpeeds(double topspeed, double bottomspeed) {

    Shooter.topspeed = topspeed;
    Shooter.bottomspeed = bottomspeed;

  }

  public static void shoot(boolean pressed) {
    if (pressed) {

      ShooterSub.top.set(ControlMode.PercentOutput, topspeed);
      ShooterSub.top.set(ControlMode.PercentOutput, bottomspeed);

    } else {

      ShooterSub.top.set(ControlMode.PercentOutput, 0);
      ShooterSub.bottom.set(ControlMode.PercentOutput, 0);

    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {

    shoot(JoystickController.triggerP);

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
