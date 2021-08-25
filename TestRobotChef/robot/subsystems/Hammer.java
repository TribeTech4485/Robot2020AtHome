/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hammer extends SubsystemBase {

  // Pneumatic solenoids for Hammer
  private Solenoid solenoidHammerRaise;   // The solenoids we use have two channels, one for each output
  private Solenoid solenoidHammerDown;
  private boolean hammerExtended = false;  // Boolean for Hammer piston control (true = piston extended, so start false)

  /**
   * Creates a new Hammer.
   */
  public Hammer() {

    // Initialize the solenoids
    solenoidHammerRaise = new Solenoid(0);
    solenoidHammerDown = new Solenoid(1);

    /*
    // compressor instantiation only needed if want to turn off compressor or check pressure
    Compressor c = new Compressor(0);
    //c.setClosedLoopControl(true);	// is closed loop by default

    boolean enabled = c.enabled();
    boolean pressureSwitch = c.getPressureSwitchValue();
    //double current = c.getCompressorCurrent();
    System.out.println("**Compressor is on: " + enabled + "  pressure switch: " + pressureSwitch);
    */
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Function to set the solenoids
  public void doHammerAction(final boolean extended) {

    hammerExtended = extended;
    // Make sure the solenoids are set to opposite values
    solenoidHammerRaise.set(!hammerExtended); // solenoid controls output that pulls piston in, so set it to ! hammerExtended
    solenoidHammerDown.set(hammerExtended); // solenoid controls output that pushes piston out, so set it to hammerExtended
  }

  // set solenoid 0
  public void setHammerRaise(final boolean extended) {
    solenoidHammerRaise.set(extended);
  }

  // set solenoid 1
  public void setHammerDown(final boolean extended) {
    solenoidHammerDown.set(extended);
  }
}
