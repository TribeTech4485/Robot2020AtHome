/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
//import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.AutoDriveStraightTime;
import frc.robot.commands.AutoSpinToAnglePID;
import frc.robot.subsystems.DriveTrain;

/**
 * A complex autonomous command that drives the Bounce path of FRC At Home Challenge
 */
public class AutoGoBouncePath extends SequentialCommandGroup {

  /**
   * Creates a new SequentialCommandGroup
   *
   * @param driveTrain The DriveTrain subsystem this command will run on
   */
  public AutoGoBouncePath(DriveTrain driveTrain) {
    // +0.1=-1 time
    addCommands(
      new InstantCommand(() -> driveTrain.stop(), driveTrain),
      new AutoDriveStraightTime( -0.6, 2.3), //-0.5, 3.3
      new AutoSpinToAnglePID(-82.0, 0.4),
      new AutoDriveStraightTime(-0.5, 2.0),
      new AutoDriveStraightTime(0.5, 2.0),
      new AutoSpinToAnglePID(-38.0, 0.4), // Conflict Spot D4
      new AutoDriveStraightTime(0.5, 6.0),
      new AutoSpinToAnglePID(60.0, 0.4),
      new AutoDriveStraightTime(-0.5, 2.5),
      new AutoSpinToAnglePID(-12.0, 0.4), // Conflict Spot D6
      new AutoDriveStraightTime(-0.5, 4.5),
      new AutoDriveStraightTime(0.5, 6.0),
      new AutoSpinToAnglePID(70, 0.4),
      new AutoDriveStraightTime(-0.5, 6),
      new AutoSpinToAnglePID(-85.0, 0.4), // Conflict Spot B8
      new AutoDriveStraightTime(-0.5, 6),
      new AutoDriveStraightTime(0.5, 2),
      new AutoSpinToAnglePID(80.0, 0.4),
      new AutoDriveStraightTime(-0.5, 4.5),
      new InstantCommand(driveTrain::stop, driveTrain)
    ); //Positive is Right , Negative is Leftt
    
    //super(new InstantCommand(() -> driveTrain.stop(), driveTrain),
    //new ParallelCommandGroup(new AutoDriveStraightTime( 0.45, 2.0), new WaitCommand(3.0)),
    //new ParallelDeadlineGroup(new AutoTurnToAngle(45.0, 0.5), new WaitCommand(3.0)),
    //new AutoDriveStraightTime( 0.45, 2.0),
    //new SequentialCommandGroup(new AutoSpinToAngle( -45.0, 2.0), new WaitCommand(2.0)),
    //new InstantCommand(driveTrain::stop, driveTrain)
  }
}
