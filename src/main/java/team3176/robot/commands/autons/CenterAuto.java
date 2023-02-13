// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/** 
 * Center Auto:
 *  1. actuate out intake 
 *  2. Extend Arm and spit cone 
 *  3. Retract Arm
 *  4. Reverse about a foot or 2 
 *  5. Spin 180 
 *  6. Drive forward slowly to bring down ramp and drive up 
 *  7. Balance 
 *  8. Wait for auto to end
 */

package team3176.robot.commands.autons;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CenterAuto extends SequentialCommandGroup {
  /** Creates a new CenterAuto. */
  public CenterAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();
  }
}
