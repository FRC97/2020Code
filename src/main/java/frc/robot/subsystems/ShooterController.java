/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterController extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  private WPI_TalonSRX topShooterMotor;
  private WPI_TalonSRX bottomShooterMotor;

  public ShooterController(){
    topShooterMotor = new WPI_TalonSRX(RobotMap.TS);
    bottomShooterMotor = new WPI_TalonSRX(RobotMap.BS);

    topShooterMotor.set(0);
    bottomShooterMotor.set(0);
  }

  /**
   * This Method sets the top and bottom motors to their adjusted speeds
   * From commands Shooter.java will send in to Params:
   * @param topAmount
   * @param bottomAmount
   */
  public void shoot(double topAmount, double bottomAmount){
    topShooterMotor.set(topAmount);
    bottomShooterMotor.set(bottomAmount);
  }
}
