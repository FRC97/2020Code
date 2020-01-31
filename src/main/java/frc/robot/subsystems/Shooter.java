/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  WPI_TalonSRX top = new WPI_TalonSRX(RobotMap.topshooter_ID);
  WPI_TalonSRX bot = new WPI_TalonSRX(RobotMap.bottomshooter_ID);
  double topspeed=0;
  double botspeed=0;
  public Shooter() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setSpeeds(double topspeed, double botspeed){
    this.topspeed = topspeed;
    this.botspeed = botspeed;
  }
  public void shoot(boolean pressed){
    if(pressed){
      top.set(ControlMode.PercentOutput, topspeed);
      bot.set(ControlMode.PercentOutput, -botspeed);
    }
    else{
      top.set(ControlMode.PercentOutput, 0);
      bot.set(ControlMode.PercentOutput, 0);
    }

  }
}
