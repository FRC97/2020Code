/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ShooterSub extends SubsystemBase {

  public static final WPI_TalonSRX top = new WPI_TalonSRX(RobotMap.topShooter_ID);
  public static final WPI_TalonSRX bottom = new WPI_TalonSRX(RobotMap.bottomShooter_ID);
  /**
   * Creates a new Shooter.
   */
  public ShooterSub() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
