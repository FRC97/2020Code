/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.ButtonMonitor;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class IndexerSub extends SubsystemBase {

  public static final WPI_VictorSPX indexer = new WPI_VictorSPX(RobotMap.Indexer);
  //public static final ButtonMonitor button = 
    //new ButtonMonitor(controller, RobotMap.indexButton, ButtonPressEventHandler)

  /**
   * Creates a new IndexerSub.
   */
  public IndexerSub() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
