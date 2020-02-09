/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;

import com.fasterxml.jackson.annotation.JacksonInject.Value;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.interfaces.Gyro;
//import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;;
/**
 * Add your docs here.
 */
public class PID_Drive extends PIDSubsystem {

  //public final PIDController turnController;
  /**
   * Add your docs here.
   */
    double speed;
    double P, I, D = 1;
    double integral, previous_error, setpoint = 0;
    Gyro gyro;
    double derivative;
    double error;
    DifferentialDrive robotDrive;
    double rcw;


    public PID_Drive(double P, double I, double D, Gyro gyro){

      super("SubsystemName", P, I, D);  
      this.P = P;
      this.I = I;
      this.D = D;
      this.gyro = gyro;
    }
  
    public void setSetpoint(double setpoint)
    {
        this.setpoint = setpoint;
    }
    //public double linear_Error()

    public void PID(){
        error = setpoint - gyro.getAngle(); // Error = Target - Actual
        if (error> setpoint*0.005){
          this.integral += (error*.02); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
          derivative = (error - this.previous_error) / .02;
          this.rcw = P*error + I*this.integral + D*derivative;
        }
        previous_error = error;
    }

    public void executeTurn(double speed, double angle)
    {
      setSetpoint(angle);
      PID();
      DriveTrain.m_drive.arcadeDrive(speed, constrain(rcw*Math.PI/180));
    }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    //setDefaultCommand(new MySpecialCommand());
  }

  @Override
  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // yourPot.getAverageVoltage() / kYourMaxVoltage;
    return 0.0;
  }

  @Override
  protected void usePIDOutput(double output) {
    // Use output to drive your system, like a motor
    // e.g. yourMotor.set(output);
  }
  public double constrain(double num) {
    if (num > 1) {
      num = 1; 
    }
    else if (num < -1) {
      num = -1;
    }
    return num;
  }
}
