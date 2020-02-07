/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class GathererSub extends SubsystemBase {
  
  public static final WPI_VictorSPX gatherer = new WPI_VictorSPX(RobotMap.gatherer_ID);
  /**
   * Creates a new Gatherer.
   */
  public GathererSub() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
