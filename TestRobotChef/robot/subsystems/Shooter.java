/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Shooter extends SubsystemBase { 

  //// ----- Talon SRX Spinner Motors ----- /////
  // There are 2 separate motors spinning in opposite directions
  private final WPI_TalonSRX spinnerLeft1 = new WPI_TalonSRX(DriveConstants.leftSpinner1Id);
  private final WPI_TalonSRX spinnerRight1 = new WPI_TalonSRX(DriveConstants.rightSpinner1Id);

  private static int counter = 4; // for limiting display

  /**
   * Creates a new Shooter.
   */
  public Shooter() {

    spinnerLeft1.configFactoryDefault(); // Clear any non default configuration/settings
    spinnerRight1.configFactoryDefault();

    spinnerLeft1.setInverted(true); // Invert 1 side of robot so will spin opposite
    spinnerRight1.setInverted(false);
    spinnerLeft1.setNeutralMode(NeutralMode.Coast); // set neutral mode
    spinnerRight1.setNeutralMode(NeutralMode.Coast);

    SupplyCurrentLimitConfiguration supplyLimit = new SupplyCurrentLimitConfiguration(true, 30, 35, 1.0);
    spinnerLeft1.configSupplyCurrentLimit(supplyLimit);
    spinnerRight1.configSupplyCurrentLimit(supplyLimit);	// set current limits
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * set spinner motor speeds
   *
   * @param leftMotorPercent  Speed in range [-1,1]
   * @param rightMotorPercent Speed in range [-1,1]
   */
  public void setSpeed(double leftMotorPercent, double rightMotorPercent) {

    if (counter++ % 5 == 0) { System.out.println("**spinner power L-R: "+String.format("%.3f  ", leftMotorPercent)+" ~ "+String.format("%.3f  ", rightMotorPercent)); }

    //leftMotorPercent = leftMotorPercent * 0.75;
    //rightMotorPercent = rightMotorPercent * 0.75;

    spinnerLeft1.set(leftMotorPercent);
    spinnerRight1.set(rightMotorPercent);
  }

  public void stop() {
    System.out.println("**in shooter stop");
    setSpeed(0.0, 0.0);
  }
}
