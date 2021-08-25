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
import frc.robot.commands.AutoSpinToAnglePID;
import frc.robot.subsystems.DriveTrain;

/**
 * A complex autonomous command that drives the Red path in the FRC At Home Challenge
 */
public class AutoGoRedPath1 extends SequentialCommandGroup {

  /**
   * Creates a new SequentialCommandGroup
   */
  public AutoGoRedPath1(DriveTrain driveTrain) {

    // positive angles turn right, negative angles turn left
    addCommands(
        new WaitCommand(1.0),
        new AutoSpinToAnglePID(-150, 0.45),
        new AutoDriveStraightTime(-1.0, 2.0),
        new AutoSpinToAnglePID(110, 0.45),
        new AutoDriveStraightTime( -1.0, 2.0),
        new AutoSpinToAnglePID(45, 0.45),
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