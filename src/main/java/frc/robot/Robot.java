// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/* Imports */
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
// import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

// import edu.wpi.first.wpilibj.Compressor;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;

/** This is a demo program showing how to use Mecanum control with the MecanumDrive class. */
public class Robot extends TimedRobot {
  
  private static final int kJoystickChannelLeft = 0;

  private static final int kMotor0 = 0;
  private static final int kMotor1 = 1;

  private static final int kMotor2 = 2;
  private static final int kMotor3 = 3;

  private static final int kMotor4 = 4;
  private static final int kMotor5 = 5;

  private static final int kMotor6 = 6;
  private static final int kMotor7 = 7;


  private MecanumDrive m_robotDrive1;
  private MecanumDrive m_robotDrive2;

  private Joystick m_stick;

  private final XboxController xbox =  new XboxController(0);

  PWMSparkMax motor0 = new PWMSparkMax(kMotor0);
  PWMSparkMax motor1 = new PWMSparkMax(kMotor1);
  // MotorControllerGroup motorGroupBackLeft = new MotorControllerGroup(motor0, motor1);

  PWMSparkMax motor2 = new PWMSparkMax(kMotor2);
  PWMSparkMax motor3 = new PWMSparkMax(kMotor3);
  // MotorControllerGroup motorGroupFrontLeft = new MotorControllerGroup(motor2, motor3);

  PWMSparkMax motor4 = new PWMSparkMax(kMotor4);
  PWMSparkMax motor5 = new PWMSparkMax(kMotor5);
  // MotorControllerGroup motorGroupBackRight = new MotorControllerGroup(motor4, motor5);
  // motorGroupBackRight.setInverted(true);

  PWMSparkMax motor6 = new PWMSparkMax(kMotor6);
  PWMSparkMax motor7 = new PWMSparkMax(kMotor7);
  // MotorControllerGroup motorGroupFrontRight = new MotorControllerGroup(motor6, motor7);
  // motorGroupFrontRight.setInverted(true);

  DoubleSolenoid exampleDoublePCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

  {
   // exampleDoublePCM.set(kOff);
    // exampleDoublePCM.set(kForward);
    // exampleDoublePCM.set(kReverse);
  }


  @Override
  public void robotInit() {

    // Order of: front left, rear left, front right, rear right
    m_robotDrive1 = new MecanumDrive(motor3, motor1, motor7, motor5);
    m_robotDrive2 = new MecanumDrive(motor2, motor0, motor6, motor4);

    m_stick = new Joystick(kJoystickChannelLeft);

  }

  @Override
  public void teleopPeriodic() {
    // Use the joystick X axis for forward movement, Y axis for lateral
    // movement, and Z axis for rotation.
    m_robotDrive1.driveCartesian(-m_stick.getX(), m_stick.getY(), -m_stick.getZ());
    m_robotDrive2.driveCartesian(-m_stick.getX(), m_stick.getY(), -m_stick.getZ());

    if (xbox.getLeftBumper()) {
      exampleDoublePCM.set(kForward);
      // exampleDoublePCM.set(kOff);
      // Using buttons
    }
    if (xbox.getRightBumper()) {
      exampleDoublePCM.set(kReverse);
      // exampleDoublePCM.set(kOff);
      motor0.set(-0.50);
    }




  }
}
