/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class Robot extends TimedRobot {
  // constants
  private int k_liftUpLimit = 0;
  private int K_liftDownLimit = 0;
  private int k_ballDetection = 0;
  private int k_lineDetection = 0;
  private int k_extendOutLimit = 0;
  private int K_extendInLimit = 0;

  private int k_liftUpButton = 4;
  private int k_liftDownButton = 5;
  private int k_extendOutButton = 6;
  private int K_extendInButton = 7;
  private int k_rampDeployUpButton = 8;
  private int k_rampDeployDownButton = 9;

  private boolean driveModeTank = false; // Change this to true to select tank steering

  private int lock = 0;

  // motors
  private WPI_TalonSRX m_intakemotor;
  private WPI_TalonSRX m_lifter;
  private WPI_TalonSRX m_extender;
  private WPI_TalonSRX m_rampDeploy;

  // This is the real drive train
  WPI_TalonSRX m_frontLeft = new WPI_TalonSRX(7);
  WPI_TalonSRX m_rearLeft = new WPI_TalonSRX(6);
  SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontLeft, m_rearLeft);
  WPI_TalonSRX m_frontRight = new WPI_TalonSRX(9);
  WPI_TalonSRX m_rearRight = new WPI_TalonSRX(8);
  SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_rearRight);
  DifferentialDrive m_4motorDrive = new DifferentialDrive(m_left, m_right);
  // Note this is only used so that the old chassis can be driven by Spark
  private DifferentialDrive m_myRobot;

  // controlers
  private Joystick m_manipStick;
  private XboxController m_driver;

  // pnuematics
  private Compressor m_c;
  private DoubleSolenoid m_ballGrab = new DoubleSolenoid(1, 2);

  // detectors
  // switches to detect ball handler extension limits
  private Counter m_extenderOutDetection = new Counter(0);
  private Counter m_extenderInDetection = new Counter(1);
  // Using optical sensors as digital inputs
  private Counter m_liftUpLimitOpticalSwitch = new Counter(2);
  private Counter m_liftDownOpticalLimit = new Counter(3);
  private Counter m_ballDetectionOpticalSwitch = new Counter(4);
  private Counter m_lineDetectionOpticalSwitch = new Counter(5);

  private AnalogInput m_liftUpLimit = new AnalogInput(0);
  private AnalogInput m_liftDownLimit = new AnalogInput(1);
  private AnalogInput m_ballDetection = new AnalogInput(2);
  private AnalogInput m_lineDetection = new AnalogInput(3);

  // class data
  double slowfast = 1;
  int slowFastLock = 0;
  private int lastUpLimitReading = 0;
  private int lastDownLimitReading = 0;
  private int lastBalldetectionReading = 0;
  private int lastLineDetectionReading = 0;

  // method definitions //\\//\\//\\//\\
  /******************************************************************************************* */
  // ballGrabber
  private void ballGrabber() {

    if (m_manipStick.getRawButton(1)) {
      m_ballGrab.set(DoubleSolenoid.Value.kReverse);
    } else if (m_manipStick.getRawButton(2)) {
      m_ballGrab.set(DoubleSolenoid.Value.kForward);
    } else {
      m_ballGrab.set(DoubleSolenoid.Value.kOff);
    }
  }

  /******************************************************************************************* */
  // lifter
  private void lifter() {
    int moveUp = 1;
    int moveDown = 1;

    // see if we have hit travel limits
    if (m_liftUpLimitOpticalSwitch.get() > k_liftUpLimit) {
      moveUp = 0; // the max extension struck at least once since last loop - so stop the motors
      m_liftUpLimitOpticalSwitch.reset();
    }
    if (m_liftDownOpticalLimit.get() > K_liftDownLimit) {
      moveDown = 0; // the minimum extension was struck at least once since last loop - so stop the
      // motors
      m_liftDownOpticalLimit.reset();
    }

    if (m_manipStick.getRawButton(k_liftUpButton)) {
      m_lifter.set(.8 * moveUp);
    } else if (m_manipStick.getRawButton(k_liftDownButton)) {
      m_lifter.set(-.8 * moveDown);
    } else {
      m_lifter.set(0);
    }
  }

  /******************************************************************************************* */
  // extender
  private void extender() {
    int moveOut = 1;
    int moveIn = 1;

    // see if we have hit travel limits
    if (m_extenderOutDetection.get() > k_extendOutLimit) {
      moveOut = 0; // the max extension struck at least once since last loop - so stop the motors
      m_extenderOutDetection.reset();
    }
    if (m_extenderInDetection.get() > K_extendInLimit) {
      moveIn = 0; // the minimum extension was struck at least once since last loop - so stop the
                  // motors
      m_extenderInDetection.reset();
    }

    if (m_manipStick.getRawButton(k_extendOutButton)) {
      m_extender.set(.8 * moveOut);
    } else if (m_manipStick.getRawButton(K_extendInButton)) {
      m_extender.set(-.8 * moveIn);
    } else {
      m_extender.set(0);
    }
  }

  /******************************************************************************************* */
  // rampDeploy
  private void rampDeploy() {

    if (m_manipStick.getRawButton(k_rampDeployUpButton)) {
      m_rampDeploy.set(.8);
    } else if (m_manipStick.getRawButton(k_rampDeployDownButton)) {
      m_rampDeploy.set(-.8);
    } else {
      m_rampDeploy.set(0);
    }
  }

  /******************************************************************************************* */
  // slowfast
  private void slowFast() {
    if (m_driver.getAButton()) {
      slowfast = 1;
    }
    if (m_driver.getBButton()) {
      slowfast = .5;
    }
  }

  /*
   * if (slowfast == .5) { slowfast = 1; } else { slowfast = .5; }
   */
  /******************************************************************************************* */
  // ball handler
  private void ballhandle() {
    int move = 1;

    if (m_ballDetectionOpticalSwitch.get() > k_ballDetection) {
      move = 0; // the intake was struck at least once since last loop - so stop the motors
      m_ballDetectionOpticalSwitch.reset();
    }
    if (m_manipStick.getButton(ButtonType.kTrigger)) {
      m_intakemotor.set(.8 * move);
    } else if (m_manipStick.getButton(ButtonType.kTop)) {
      m_intakemotor.set(-.8);
    } else {
      m_intakemotor.set(0);
    }
  }

  /******************************************************************************************* */
  private void driveStationUpdate() {
    // report limit detections
    SmartDashboard.putNumber("Ball Detection", m_ballDetection.getValue()); // - lastBalldetectionReading);
    SmartDashboard.putNumber("Line Detection", m_lineDetection.getValue()); // - lastLineDetectionReading);
    // mini loop to detect line and print stuff
    if ((m_ballDetection.getValue() == 1) && (lock == 0)) {
      if ((findSpeedTank(Hand.kRight) < findSpeedTank(Hand.kLeft))
          || (findSpeedJoystick(Hand.kRight) < findSpeedJoystick(Hand.kLeft))) {
        SmartDashboard.putNumber("You need to go right", findSpeedTank(Hand.kLeft));
      } else {
        SmartDashboard.putNumber("You need to go left", findSpeedTank(Hand.kLeft));
      }
      lock = 1;
    } else {
      lock = 0;
    }
    // may not have correct values

    SmartDashboard.putNumber("Lift Up Limit", m_liftUpLimit.getValue()); // - lastUpLimitReading);
    SmartDashboard.putNumber("Lift Down Limit", m_liftDownLimit.getValue()); // - lastDownLimitReading);

    SmartDashboard.putNumber("Extend Out Limit", m_extenderOutDetection.get());
    SmartDashboard.putNumber("Extend In Limit", m_extenderInDetection.get());

    // Set to appropriate speed function
    if (driveModeTank == true) {
      SmartDashboard.putNumber("Left Speed", findSpeedTank(Hand.kLeft));
      SmartDashboard.putNumber("Right Speed", findSpeedTank(Hand.kRight));
    } else {
      SmartDashboard.putNumber("Left Speed", findSpeedJoystick(Hand.kLeft));
      SmartDashboard.putNumber("Right Speed", findSpeedJoystick(Hand.kRight));
    }
  }

  /******************************************************************************************* */
  // Use left/right triggers to control left/right wheel speed
  // Use left/right bumper to reverse/forward
  private double findSpeedTank(Hand hand) {
    double triggerValue;
    if (m_driver.getBumper(hand)) {
      triggerValue = -1;
    } else {
      triggerValue = 1;
    }
    return m_driver.getTriggerAxis(hand) * slowfast * triggerValue;
  }

  /******************************************************************************************* */
  private double findSpeedJoystick(Hand hand) {
    return (m_driver.getY(hand) * slowfast) * -1;
  }

  // classes
  /******************************************************************************************* */
  @Override
  public void robotInit() {
    // Note this is only used so that the old chassis can be driven by Sparks
    m_myRobot = new DifferentialDrive(new Spark(0), new Spark(2));

    // create robot components
    m_driver = new XboxController(0);
    m_manipStick = new Joystick(2);

    m_intakemotor = new WPI_TalonSRX(4);
    m_extender = new WPI_TalonSRX(10);
    m_lifter = new WPI_TalonSRX(3);
    m_rampDeploy = new WPI_TalonSRX(11);

    m_c = new Compressor(1);

    // initialize subsystems and sensors
    m_c.setClosedLoopControl(true);

    // Initialize all dio counters
    m_extenderOutDetection.reset();
    m_extenderInDetection.reset();
    m_liftUpLimitOpticalSwitch.reset();
    m_liftDownOpticalLimit.reset();
    m_ballDetectionOpticalSwitch.reset();
    m_lineDetectionOpticalSwitch.reset();

    m_liftUpLimit.setOversampleBits(4);
    m_liftUpLimit.setAverageBits(2);
    m_liftDownLimit.setOversampleBits(4);
    m_liftDownLimit.setAverageBits(2);

    m_ballDetection.setOversampleBits(4);
    m_ballDetection.setAverageBits(2);

    m_lineDetection.setOversampleBits(4);
    m_lineDetection.setAverageBits(2);

    m_liftDownLimit.setGlobalSampleRate(62500);

  }

  // drive/manip
  /******************************************************************************************* */
  @Override
  public void teleopPeriodic() {

    // findSpeedJoystick to use joystick
    // findSpeedTank to use triggers

    if (driveModeTank == true) {
      // Note this is only used so that the old chassis can be driven by Sparks
      m_myRobot.tankDrive(findSpeedTank(Hand.kLeft), findSpeedTank((Hand.kRight)));
      // This is the real drive train
      m_4motorDrive.tankDrive(findSpeedTank(Hand.kLeft), findSpeedTank((Hand.kRight)));

    } else {
      // Note this is only used so that the old chassis can be driven by Sparks
      m_myRobot.tankDrive(findSpeedJoystick(Hand.kLeft), findSpeedJoystick((Hand.kRight)));
      // This is the real drive train
      m_4motorDrive.tankDrive(findSpeedJoystick(Hand.kLeft), findSpeedJoystick((Hand.kRight)));
    }
    // report status to driver
    driveStationUpdate();

    // Operate the manipulator parts
    ballGrabber();
    ballhandle();
    lifter();
    extender();
    rampDeploy();

    // Set scaling factor for drive
    slowFast();

    // if we are on the line - reset the counter so we can check if we are still on
    // the line
    if (m_lineDetectionOpticalSwitch.get() > k_lineDetection) {
      m_lineDetectionOpticalSwitch.reset();
    }

    lastUpLimitReading = m_liftUpLimit.getValue();
    lastDownLimitReading = m_liftDownLimit.getValue();
    lastBalldetectionReading = m_ballDetection.getValue();
    lastLineDetectionReading = m_lineDetection.getValue();
  }

  /******************************************************************************************* */
  @Override
  public void autonomousPeriodic() {
    teleopPeriodic();
  }
}
