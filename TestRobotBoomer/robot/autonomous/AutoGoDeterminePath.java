/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
//import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.AutoDriveStraightTime;
import frc.robot.commands.AutoSpinToAnglePID;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.Timer;

/**
 * A complex autonomous command that drives forward, and then drives backward.
 */
public class AutoGoDeterminePath extends SequentialCommandGroup {

  /**
   * Creates a new SequentialCommandGroup
   *
   * @param driveTrain The DriveTrain subsystem this command will run on
   */
  public AutoGoDeterminePath(DriveTrain driveTrain) {
    boolean buttpress = false;
    double tiempoInit = Timer.getFPGATimestamp();
    double tiempo = Timer.getFPGATimestamp();
    tiempo = Timer.getFPGATimestamp();
    // positive angles turn right, negative angles turn left
    addCommands(
          new InstantCommand(() -> driveTrain.stop(), driveTrain),    // make sure stopped
          new AutoDriveStraightTime( -1.0, 0.6),
          new WaitCommand(1.0),
          new InstantCommand(driveTrain::stop, driveTrain)    // make sure stopped
    );
    //while (!buttpress) {
    //    tiempo = Timer.getFPGATimestamp() - tiempoInit;
    //} 
    //stop driving
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
