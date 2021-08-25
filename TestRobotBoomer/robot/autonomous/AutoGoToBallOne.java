/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import java.text.MessageFormat;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class AutoGoToBallOne extends CommandBase {

    private final DriveTrain m_driveTrain;
    private double speed;
    
    private double travelDistance = 0.0;
    private double startTime = 0.0;
    private double travelTime = 0.0;    // in seconds

    private AutoGoBluePath1 autoGoBluePath1;  // commands to execute after this one
    private AutoGoRedPath1 autoGoRedPath1;
    private AutoGoBluePath2 autoGoBluePath2;
    private AutoGoRedPath2 autoGoRedPath2;

  /**
   * Creates a new AutoDriveToBallOne.
   */
  public AutoGoToBallOne(double speedIn) {

    this.m_driveTrain = RobotContainer.m_driveTrain;    // get driveTrain object from RobotContainer
    this.speed = speedIn;

    this.autoGoBluePath1 = new AutoGoBluePath1(this.m_driveTrain);  // create instances of next commands to do
    this.autoGoRedPath1 = new AutoGoRedPath1(this.m_driveTrain);
    this.autoGoBluePath2 = new AutoGoBluePath2(this.m_driveTrain);  // create instances of next commands to do
    this.autoGoRedPath2 = new AutoGoRedPath2(this.m_driveTrain);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_driveTrain);  // means no other command can use subsystem when this command is running.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_driveTrain.resetEncoders();   // reset the encoders to 0 position
    startTime = Timer.getFPGATimestamp();   // get start time
    travelTime = 0.0;
    m_driveTrain.turnOnCollector();     // turn on collector
    m_driveTrain.turnOnConveyor();     // turn on conveyor

    System.out.println(MessageFormat.format("**Started {0}  start time: {1}", this.getName(), String.format("%.3f", startTime)));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_driveTrain.doTankDrive(speed, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    travelTime = Timer.getFPGATimestamp() - startTime;
    m_driveTrain.turnOffConveyor();     // turn off conveyor
    if (travelTime < 7.0) {
        autoGoRedPath2.schedule();
        System.out.println("**scheduling AutoGoRedderPath command " + String.format("%.3f",travelTime));
    } else if (travelTime < 10.0) {
        autoGoRedPath1.schedule();
        System.out.println("**scheduling AutoGoRedPath command " + String.format("%.3f",travelTime));
    } else if (travelTime < 13.0) {
        autoGoBluePath1.schedule();
        System.out.println("**scheduling AutoGoBluePath command " + String.format("%.3f",travelTime));
    } else {
        autoGoBluePath2.schedule();
        System.out.println("**scheduling AutoGoBluerPath command " + String.format("%.3f",travelTime));
    }

    //if (travelDistance < 12.0) {
    //    autoGoBluePath1.schedule();  // based on distance to 1st ball, do either Blue or Red path
    //    System.out.println("**scheduling AutoGoBluePath command");
    //} else {
    //    autoGoRedPath1.schedule();
    //    System.out.println("**scheduling AutoGoRedPath command");
    //}
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    //System.out.println("ball sensor: " + m_driveTrain.checkBallSensor());

      if (!m_driveTrain.checkBallSensor()) {         // check if limit switch tripped, means have ball
        travelTime = Timer.getFPGATimestamp() - startTime;    // get elapsed time
        travelDistance = m_driveTrain.getLeftDistanceInch();
        //m_driveTrain.turnOffCollector();     // turn off collector
        m_driveTrain.turnOffConveyor();     // turn off conveyor
        System.out.println(MessageFormat.format("**Ending {0}  travel time: {1}  distance: {2}", this.getName(),String.format("%.3f",travelTime),String.format("%.3f",travelDistance)));
        return true;
      } else {
        return false;       // if limit switch not tripped means not at ball, so return false
      }
  }
}
