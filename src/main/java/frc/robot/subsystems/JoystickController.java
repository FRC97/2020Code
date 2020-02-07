/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class JoystickController extends SubsystemBase {
  /**
   * Creates a new JoystickController.
   */
  public static final double Y_Value = Robot.m_stick.getRawAxis(1);
  public static final double X_Value = Robot.m_stick.getRawAxis(0);
  public static final double Z_Value = Robot.m_stick.getRawAxis(2);
  public static final double slider = Robot.m_stick.getRawAxis(3);

  public static final boolean triggerP = Robot.m_stick.getRawButtonPressed(1);
  public static final boolean button4P = Robot.m_stick.getRawButtonPressed(4);



  public JoystickController() {
/**
 * Add your docs here.
 */

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
 }
}