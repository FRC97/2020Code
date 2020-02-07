/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class PID_Sub extends PIDSubsystem {
  /**
   * Creates a new PID_Sub.
   */

  static double P, I, D = 1;
  static double previous_error;

  public PID_Sub() {
    super(
        // The PIDController used by the subsystem
        new PIDController(P, I, D));
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here

    double error = setpoint - output;
    P *= error;
    I += error*0.02;
    D = (error - previous_error)/2;
    previous_error = error;

  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }
}
