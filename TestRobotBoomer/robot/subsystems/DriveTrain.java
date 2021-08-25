/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.revrobotics.CANDigitalInput;
//import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {
  
   //// ----- Motor Controllers ----- /////
  // for Neo motors - 6 separate motor controllers with 1 pwm channel per controller
  private final CANSparkMax motorDriveLeft1 = new CANSparkMax(DriveConstants.leftDrive1Id, MotorType.kBrushless);
  private final CANSparkMax motorDriveLeft2 = new CANSparkMax(DriveConstants.leftDrive2Id, MotorType.kBrushless);
  private final CANSparkMax motorDriveLeft3 = new CANSparkMax(DriveConstants.leftDrive3Id, MotorType.kBrushless);
  private final CANSparkMax motorDriveRight1 = new CANSparkMax(DriveConstants.rightDrive1Id, MotorType.kBrushless);
  private final CANSparkMax motorDriveRight2 = new CANSparkMax(DriveConstants.rightDrive2Id, MotorType.kBrushless);
  private final CANSparkMax motorDriveRight3 = new CANSparkMax(DriveConstants.rightDrive3Id, MotorType.kBrushless);

  // define Speed Controller Groups and Differential Drive for use in drive train
  private final SpeedControllerGroup driveGroupLeft = new SpeedControllerGroup(motorDriveLeft1, motorDriveLeft2, motorDriveLeft3);
  private final SpeedControllerGroup driveGroupRight = new SpeedControllerGroup(motorDriveRight1, motorDriveRight2, motorDriveRight3);
  private final DifferentialDrive differentialDrive = new DifferentialDrive(driveGroupLeft, driveGroupRight);

  // define encoders for tracking distance
  private CANEncoder leftDriveEncoder; // Declare the left encoder
  private CANEncoder rightDriveEncoder;   // Declare the right encoder

  private double leftEncoderZeroValue = 0;
  private double rightEncoderZeroValue = 0;

  // motors for conveyor and collector
  private final CANSparkMax collectorMotor = new CANSparkMax(DriveConstants.collectorMotorId, MotorType.kBrushless);
  private final CANSparkMax conveyorMotor = new CANSparkMax(DriveConstants.conveyorMotorId, MotorType.kBrushless);

  // initialize digital input on Roborio DIO 0
  private final DigitalInput ballSensor = new DigitalInput(0);

  // navX Gyro on RoboRio
  private AHRS m_Gyro;

  private static final boolean kSquareInputs = true;
  private static final boolean kSkipGyro = false;
  private static final boolean kSkipEncoder = false;
  private static int counter = 49; // for limiting display
  private static int counter2 = 4;

  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {

    // for Neo motors
    motorDriveLeft1.restoreFactoryDefaults();
    motorDriveLeft2.restoreFactoryDefaults();
    motorDriveLeft3.restoreFactoryDefaults();
    motorDriveRight1.restoreFactoryDefaults();
    motorDriveRight2.restoreFactoryDefaults();
    motorDriveRight3.restoreFactoryDefaults();

    motorDriveLeft1.setInverted(false); // set direction
    motorDriveLeft2.setInverted(false);
    motorDriveLeft3.setInverted(false);
    motorDriveRight1.setInverted(false);
    motorDriveRight2.setInverted(false);
    motorDriveRight3.setInverted(false);
    //DifferentialDrive inverts right side by default, so no need to setInvert() here
    //differentialDrive.setRightSideInverted(true);

    differentialDrive.setDeadband(0.03);
    differentialDrive.setSafetyEnabled(false);    // ***to avoid error 'differentialDrive not fed often enough'

    motorDriveLeft1.setIdleMode(IdleMode.kBrake); // set idle mode
    motorDriveLeft2.setIdleMode(IdleMode.kBrake);
    motorDriveLeft3.setIdleMode(IdleMode.kBrake);
    motorDriveRight1.setIdleMode(IdleMode.kBrake);
    motorDriveRight2.setIdleMode(IdleMode.kBrake);
    motorDriveRight3.setIdleMode(IdleMode.kBrake);

    motorDriveLeft1.setSmartCurrentLimit(DriveConstants.kAmpsMax);  // set current limit
    motorDriveLeft2.setSmartCurrentLimit(DriveConstants.kAmpsMax);
    motorDriveLeft3.setSmartCurrentLimit(DriveConstants.kAmpsMax);
    motorDriveRight1.setSmartCurrentLimit(DriveConstants.kAmpsMax);
    motorDriveRight2.setSmartCurrentLimit(DriveConstants.kAmpsMax);
    motorDriveRight3.setSmartCurrentLimit(DriveConstants.kAmpsMax);
    // end for Neo motors 
 
    if (kSkipEncoder) {
      leftDriveEncoder = null;
      rightDriveEncoder = null;

    } else {
      //https://github.com/REVrobotics/SPARK-MAX-Examples/tree/master/Java
      leftDriveEncoder = motorDriveLeft1.getEncoder(); // Define the left encoder
      rightDriveEncoder = motorDriveRight1.getEncoder();   // Declare the right encoder

      leftDriveEncoder.setPositionConversionFactor(DriveConstants.kEncoderTicksPerRevolution); // set encoder constants
      rightDriveEncoder.setPositionConversionFactor(DriveConstants.kEncoderTicksPerRevolution);

      //leftDriveEncoder.setPositionConversionFactor((DriveConstants.kWheelDiameter * Math.PI) / (DriveConstants.kDriveMotorGearRatio * DriveConstants.kEncoderTicksPerRevolution)); // set encoder constants
      //rightDriveEncoder.setPositionConversionFactor((DriveConstants.kWheelDiameter * Math.PI) / (DriveConstants.kDriveMotorGearRatio * DriveConstants.kEncoderTicksPerRevolution)); 

      //leftDriveEncoder.setReverseDirection(true);
      //rightDriveEncoder.setReverseDirection(true);
      // resetEncoders(); // set encoders to 0

      // set limit switches - can be 1 of 2 polarities: normally open or normally closed dipending on direction to limit 
      //m_forwardLimit = motorDriveLeft1.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyClosed);
      //m_reverseLimit = motorDriveRight1.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyClosed);
      //m_forwardLimit.enableLimitSwitch(false);
      //m_reverseLimit.enableLimitSwitch(false);
      //boolean limitPressed = m_forwardLimit.get();  // returns true if switch is pressed (or not connected), false when released
    }

    if (kSkipGyro) {
      m_Gyro = null;

    } else {
      // navX-MXP Gyro instantiation
      try {
        // Instantiate Gyro - communicate w/navX-MXP via the MXP SPI Bus
        // Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB
        // See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details
        m_Gyro = new AHRS(SPI.Port.kMXP);

      } catch (RuntimeException ex) {
        DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
      }
      while (m_Gyro.isCalibrating()) {
        try { Thread.sleep(500); } catch (Exception e) { System.out.println(e); } //sleep in milliseconds
        System.out.println("**gyro isCalibrating . . .");
      }
    //SmartDashboard.putBoolean("gyro connected", m_Gyro.isConnected());
    System.out.println("**gyro connected: " + m_Gyro.isConnected());
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Tank style driving for the DriveTrain.
   *
   * @param left  Speed in range [-1,1]
   * @param right Speed in range [-1,1]
   */
  public void doTankDrive(double leftDrivePercent, double rightDrivePercent) {

    if (counter++ % 50 == 0) { System.out.println("**driveTrain power L-R: "+String.format("%.3f", leftDrivePercent)+" ~ "+String.format("%.3f", rightDrivePercent)); }
 
    //leftDrivePercent = leftDrivePercent * 0.75;
    //rightDrivePercent = rightDrivePercent * 0.75;
    // SquareInputs adjust inputs at low speeds so better control - note 1.0 * 1.0 is still 1
    differentialDrive.tankDrive(leftDrivePercent, rightDrivePercent, kSquareInputs); // send output to drive train
  
    // use this instead if get differentialDrive not updated often enough
    //motorDriveLeft1.set(leftDrivePercent);
    //motorDriveLeft2.set(leftDrivePercent);
    //motorDriveLeft3.set(leftDrivePercent);
    //motorDriveRight1.set(rightDrivePercent);
    //motorDriveRight2.set(rightDrivePercent);
    //motorDriveRight3.set(rightDrivePercent); 
  }

  /**
   * Arcade style driving for the DriveTrain.
   *
   * @param speed    Speed in range [-1,1]
   * @param rotation Rotation in range [-1,1]
   */
  public void doArcadeDrive(double speed, double rotation) {

    if (counter2++ % 5 == 0) { System.out.println("**arcade driveTrain speed: "+String.format("%.3f", speed)+"  rotation: "+String.format("%.3f", rotation)); }
    speed = speed * 0.75;
    // SquareInputs adjust inputs at low speeds so better control - note 1.0 * 1.0 is still 1
    differentialDrive.arcadeDrive(speed, rotation, kSquareInputs);
  }

  // http://pdocs.kauailabs.com/navx-mxp/guidance/terminology (for pitch, roll, yaw, IMU terminology)
  public double getHeadingAngle() {
    //if (counter3++ % 5 == 0) {
      //System.out.println("gyropitch " + String.format("%.3f", m_Gyro.getPitch()));
      //System.out.println("gyroroll  " + String.format("%.3f", m_Gyro.getRoll()));
      System.out.println("gyroyaw   " + String.format("%.3f", m_Gyro.getYaw()));
      //System.out.println("gyrorawZ  " + String.format("%.3f", m_Gyro.getRawGyroZ()));
      //System.out.println("gyrorawY  " + String.format("%.3f", m_Gyro.getRawGyroY()));
      //System.out.println("gyrorawX  " + String.format("%.3f", m_Gyro.getRawGyroX()));
      //System.out.println("gyroangle " + String.format("%.3f", m_Gyro.getAngle()));
      //System.out.println("gyrorelat angle " + String.format("%.3f", getRelativeAngle()));
    //}
    //return Math.IEEEremainder(m_Gyro.getAngle(), 360.0) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    //return Math.IEEEremainder(m_Gyro.getYaw(), 360.0) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    return Math.IEEEremainder(m_Gyro.getYaw(), 360.0);
  }

  public double getYaw() {
    return m_Gyro.getYaw(); // get rotation around Z axis for current heading
  }

  public void resetGyro() {
      //m_Gyro.reset();
      // "Zero" yaw (whatever direction sensor is pointing now becomes new "Zero" degrees
      m_Gyro.zeroYaw();   // yaw is only thing that can be reset, pitch and roll can't (see docs)
  }

  // public void resetEncoder() {
  public void resetEncoders() {
    double ticks = 0.0;
    float  tempVal = 0f;
    CANError setPosError = leftDriveEncoder.setPosition(tempVal);    // this does not work ??
    if (setPosError == CANError.kOk) {
        leftEncoderZeroValue = 0.0;
        System.out.println("** reset left encoder successful");
    } else {
        ticks = leftDriveEncoder.getPosition();         // get current position
        ticks = Math.floor(ticks * 100.0) / 100.0;      // round to 2 decimal places
        leftEncoderZeroValue = ticks >= 0 ? ticks : 0;  // only allow positive numbers to substracted later
        System.out.println("** ERROR reset left encoder");
    }
    setPosError = rightDriveEncoder.setPosition(tempVal);
    if (setPosError == CANError.kOk) {
        rightEncoderZeroValue = 0.0;
        System.out.println("** reset right encoder successful");
    } else {
        ticks = rightDriveEncoder.getPosition();
        ticks = Math.floor(ticks * 100.0) / 100.0;    // round to 2 decimal places
        rightEncoderZeroValue = ticks >= 0 ? ticks : 0;  // only allow positive numbers to substracted later
        System.out.println("** ERROR reset right encoder");
    }
    //try { Thread.sleep(250); } catch (Exception e) { System.out.println(e); } //sleep in milliseconds while encoders to reset
    System.out.println("** resetting encoders to initial value: L/R " + leftEncoderZeroValue +" ~ "+ rightEncoderZeroValue);
  }

  public double getLeftEncoderCount() {
    double currentValue = leftDriveEncoder.getPosition() - leftEncoderZeroValue;
    //double currentValue = leftDriveEncoder.getPosition();
    System.out.println("**left encoder count: " + String.format("%.4f", currentValue));
    return currentValue;  // return position of encoder
  }

  public double getRightEncoderCount() {
    double currentValue = rightDriveEncoder.getPosition() - rightEncoderZeroValue;
    //double currentValue = rightDriveEncoder.getPosition();
    //System.out.println("**right encoder count: " + currentValue);
    return currentValue;  // return position of encoder
  }

  public double getLeftDistanceInch() {
    // = Math.PI * DriveConstants.kWheelDiameter * (getLeftEncoderCount() / DriveConstants.kEncoderTicksPerRevolution);
    return DriveConstants.kInchesPerPulse * getLeftEncoderCount();
  }

  public double getRightDistanceInch() {
    // = Math.PI * DriveConstants.kWheelDiameter * (getRightEncoderCount() / DriveConstants.kEncoderTicksPerRevolution);
    return DriveConstants.kInchesPerPulse * getRightEncoderCount();
  }

  public double getAveDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }

  public void turnOnConveyor() {
    conveyorMotor.set( 1.0 ); 
  }

  public void turnOffConveyor() {
    conveyorMotor.set( 0.0 ); 
  }

  public void turnOnCollector() {
    collectorMotor.set( -0.30 );
  }

  public void turnOffCollector() {
    collectorMotor.set( 0.0 ); 
  }

  public boolean checkBallSensor() {
    return ballSensor.get();      // get value of digital input
    /*
    if (bogusStartTime == 0.0) {
        bogusStartTime =  Timer.getFPGATimestamp();
    }
    double bogusTime =  Timer.getFPGATimestamp() - bogusStartTime;
    if (bogusTime >= 8.0) {
       return true;
    } else {
      return false;
    }
    */
  }

  public void stop() {
    System.out.println("in drivetrain stop");
    doTankDrive(0.0, 0.0);
  }
}
