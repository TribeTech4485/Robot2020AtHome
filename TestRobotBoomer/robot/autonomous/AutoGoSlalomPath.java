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
import frc.robot.commands.AutoDriveStraightUnits;
import frc.robot.commands.AutoSpinToAnglePID;
import frc.robot.subsystems.DriveTrain;

/**
 * A complex autonomous command that drives the Slalom path of FRC At Home Challenge
 */
public class AutoGoSlalomPath extends SequentialCommandGroup {

   /**
   * Creates a new SequentialCommandGroup
   *
   * @param driveTrain The DriveTrain subsystem this command will run on
   */
  public AutoGoSlalomPath(DriveTrain driveTrain) {
    
    // positive angles turn right, negative angles turn left
    addCommands(
	    new InstantCommand(() -> driveTrain.stop(), driveTrain),    // make sure stopped
      new AutoDriveStraightUnits(-0.5, 30),
      new WaitCommand(0.1), 
      new AutoSpinToAnglePID(-55.0, 0.50),
      new WaitCommand(0.1), 
      new AutoDriveStraightUnits(-0.50, 85), // 90
      new WaitCommand(0.1),  
      new AutoSpinToAnglePID( 50.0, 0.5), 
      new WaitCommand(0.1),
      new AutoDriveStraightUnits(-0.6, 170), // past D4-D8
      new WaitCommand(0.1),
      new AutoSpinToAnglePID(80.0, 0.40),
      new WaitCommand(0.1), 
      new AutoDriveStraightUnits(-0.50, 90), // perpindicular with D4-D8 (near)
      new WaitCommand(0.1),  
      new AutoSpinToAnglePID(-85.0, 0.40),
      new WaitCommand(0.1),
      new AutoDriveStraightUnits(-0.50, 65), // paralel with D4-D8
      new WaitCommand(0.1),
      new AutoSpinToAnglePID(-90,0.4),
      new WaitCommand(0.1),
      new AutoDriveStraightUnits(-.5,65), // perpindicular with D4-D8 (far)
      new WaitCommand(0.1),  
      new AutoSpinToAnglePID(-90.0,.4),
      new WaitCommand(0.1),
      new AutoDriveStraightUnits(-.5,60),   // paralel with D4-D8 
      new WaitCommand(0.1),
      new AutoSpinToAnglePID(-90.0,.4),
      new WaitCommand(0.1),
      new AutoDriveStraightUnits(-.5,80),   // perpindicular with D4-D8 (near)
      new WaitCommand(0.1),  
      new AutoSpinToAnglePID(90.0,.4),
      new WaitCommand(0.1),
      new AutoDriveStraightUnits(-0.6, 177), // past D4-D8
      new WaitCommand(0.1),  
      new AutoSpinToAnglePID(85, .4),
      new WaitCommand(0.1),
      new AutoDriveStraightUnits(-.5, 75),
      new WaitCommand(0.1),  
      new AutoSpinToAnglePID(-85, .4),
      new WaitCommand(0.1),
      new AutoDriveStraightUnits(-.5, 60),
  
      /* Temp
      // forward but up a bit 
      new AutoSpinToAnglePID(45.0, 0.50), // turn left 
      new AutoSpinToAnglePID(30.0, 0.50), // turn down  
      new AutoDriveStraightTime(0.50, 2.0), // go down  
      new AutoSpinToAnglePID(-30.0, 0.50), // turn right a bit 
      new AutoDriveStraightTime(0.50, 2.0), // up 
      new AutoSpinToAnglePID(45.0, 0.50), // turn left */ 
      
      new InstantCommand(driveTrain::stop, driveTrain)       // make sure stopped
    );
  }
}
