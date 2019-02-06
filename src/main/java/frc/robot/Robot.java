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

public class Robot extends TimedRobot {

  // motors
  private DifferentialDrive m_myRobot; // Note this is only used so that the old chassis can be driven by Spark
  private WPI_TalonSRX m_intakemotor;
  private WPI_TalonSRX m_lifter;
  private WPI_TalonSRX m_extender;

  // controlers
  private Joystick m_manipStick;
  private XboxController m_driver;

  // pnuematics
  private Compressor m_c;
  private DoubleSolenoid m_ballGrab = new DoubleSolenoid(1, 2);

  // detectors
  private Counter m_ballInDetection = new Counter(0);
  private Counter m_extenderOutDetection = new Counter(1);
  private Counter m_extenderInDetection = new Counter(2);
  private AnalogInput m_distanceLight = new AnalogInput(1);

  double slowfast = 1;
  int slowFastLock = 0;
  // This is the real drive train
  WPI_TalonSRX m_frontLeft = new WPI_TalonSRX(8);
  WPI_TalonSRX m_rearLeft = new WPI_TalonSRX(9);
  SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontLeft, m_rearLeft);
  WPI_TalonSRX m_frontRight = new WPI_TalonSRX(6);
  WPI_TalonSRX m_rearRight = new WPI_TalonSRX(7);
  SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_rearRight);
  DifferentialDrive m_4motorDrive = new DifferentialDrive(m_left, m_right);

  // method definitions //\\//\\//\\//\\

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

  // lifter
  private void lifter() {
    if (m_manipStick.getRawButton(4)) {
      m_lifter.set(.8);
    } else if (m_manipStick.getRawButton(5)) {
      m_lifter.set(-.8);
    } else {
      m_lifter.set(0);
    }
  }

  // extender
  private void extender() {
    int moveOut = 1;
    int moveIn = 1;

    // see if we have hit travel limits
    if (m_extenderOutDetection.get() > 0) {
      moveOut = 0; // the max extension struck at least once since last loop - so stop the motors
      m_extenderOutDetection.reset();
    }
    if (m_extenderInDetection.get() > 0) {
      moveIn = 0; // the minimum extension was struck at least once since last loop - so stop the
                  // motors
      m_extenderInDetection.reset();
    }

    if (m_manipStick.getRawButton(6)) {
      m_extender.set(.8 * moveOut);
    } else if (m_manipStick.getRawButton(7)) {
      m_extender.set(-.8 * moveIn);
    } else {
      m_extender.set(0);
    }
  }

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
  // ball handler
  private void ballhandle() {
    int move = 1;

    if (m_ballInDetection.get() > 0) {
      move = 0; // the intake was struck at least once since last loop - so stop the motors
      m_ballInDetection.reset();
    }
    if (m_manipStick.getButton(ButtonType.kTrigger)) {
      m_intakemotor.set(.8 * move);
    } else if (m_manipStick.getButton(ButtonType.kTop)) {
      m_intakemotor.set(-.8);
    } else {
      m_intakemotor.set(0);
    }
  }

  // classes
  @Override
  public void robotInit() {
    // Note this is only used so that the old chassis can be driven by Sparks
    m_myRobot = new DifferentialDrive(new Spark(0), new Spark(2));

    // create robot components
    m_driver = new XboxController(0);
    m_manipStick = new Joystick(2);

    m_intakemotor = new WPI_TalonSRX(5);
    m_extender = new WPI_TalonSRX(10);
    m_lifter = new WPI_TalonSRX(11);

    m_c = new Compressor(1);

    // initialize subsystems and sensors
    m_c.setClosedLoopControl(true);

    m_distanceLight.setOversampleBits(4);
    m_distanceLight.setAverageBits(2);
    m_distanceLight.setGlobalSampleRate(62500);
  }

  // drive/manip
  @Override
  public void teleopPeriodic() {
    // Note this is only used so that the old chassis can be driven by Sparkd
    m_myRobot.tankDrive(m_driver.getY(Hand.kLeft) * slowfast, m_driver.getY(Hand.kRight) * slowfast);
    // This is the real drive train
    m_4motorDrive.tankDrive(m_driver.getY(Hand.kLeft) * slowfast, m_driver.getY(Hand.kRight) * slowfast);

    // Operate the manipulator parts
    ballGrabber();
    ballhandle();
    lifter();
    extender();

    // Set scaling factor for drive
    slowFast();

    boolean enabled = m_c.enabled();
    boolean pressureSwitch = m_c.getPressureSwitchValue();
    double current = m_c.getCompressorCurrent();

    int raw = m_distanceLight.getValue();
    System.out.print(raw);
    System.out.print("* *");

  }
}