/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class JoystickController extends SubsystemBase {
  /**
   * Creates a new JoystickController.
   */
  private static Joystick joy = new Joystick(0);
  public static final double Yval = joy.getRawAxis(1);
  public static final double Xval = joy.getRawAxis(0);
  public static final double Zval = joy.getRawAxis(2);
  public static final double slider = joy.getRawAxis(3);

  public static final boolean triggerP = joy.getRawButtonPressed(1);
  public static final boolean button4P = joy.getRawButtonPressed(4);
  public static final boolean button5P = joy.getRawButtonPressed(5);
  public static final boolean button6P = joy.getRawButtonPressed(6);
  public static final boolean button7P = joy.getRawButtonPressed(7);



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