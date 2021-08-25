/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
//import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.AutoDriveStraightTime;
import frc.robot.commands.AutoDriveStraightToBall;
import frc.robot.commands.AutoSpinToAnglePID;
import frc.robot.subsystems.DriveTrain;

/**
 * A complex autonomous command that drives the Red path in the FRC At Home Challenge
 */
public class AutoGoRedPath2 extends SequentialCommandGroup {

  /**
   * Creates a new SequentialCommandGroup
   */
  public AutoGoRedPath2(DriveTrain driveTrain) {

    // positive angles turn right, negative angles turn left
    addCommands(
        new AutoSpinToAnglePID(45, 0.45), //turn 45 degrees to the right
        //new AutoDriveStraightTime( -0.5, 0.8),
        new AutoDriveStraightToBall( -0.5, 0.8, 0.6),
        new AutoSpinToAnglePID(-80, 0.45),
        //new AutoDriveStraightTime( -0.5, 1.0),
        new AutoDriveStraightToBall( -0.5, 1.0, 0.75),
        new AutoSpinToAnglePID(-75, 0.45),
        new WaitCommand(1.0)
    );

      /*
      // additional complex commands
      //new ParallelCommandGroup(new AutoDriveStraightTime( 0.45, 2.0), new WaitCommand(3.0)),
      //new ParallelDeadlineGroup(new AutoTurnToAngle(45.0, 0.5), new WaitCommand(3.0)),
      //new AutoDriveStraightTime( 0.45, 2.0),
      //new SequentialCommandGroup(new AutoSpinToAngle( -45.0, 2.0), new WaitCommand(2.0)),
      //new InstantCommand(driveTrain::stop, driveTrain)
      */
  }
}
