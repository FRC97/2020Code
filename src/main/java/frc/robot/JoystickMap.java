/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class JoystickMap extends SubsystemBase {
  /**
   * Creates a new JoystickController.
   */
  public static final Joystick joyStick = new Joystick(0);

  public static final int Xval = 0;
  public static final int Yval = 1;
  public static final int Zval = 2;
  public static final int slider = 3;

  public static final int triggerP = 1;//Shooter
  public static final int button2P = 2;//Change Direction
  public static final int button3P = 3;//Indexer
  public static final int button4P = 4;//Gatherer
  public static final int button5P = 5;//Wind Cord
  public static final int button6P = 6;//Unwide Cord
  public static final int button7P = 7;//Lift Parascope
  public static final int button8P = 8;//Down Parascope
  public static final int button9P = 9;//Drive to 3m (optimal shooting distance)
  public static final int button10P = 10;

}