// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.security.Timestamp;
import java.util.concurrent.DelayQueue;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveTrain;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public RobotContainer m_robotContainer;
  private DriveTrain driveTrain = m_robotContainer.driveTrain;

  public static final WPI_TalonFX leftbackmotor = Constants.leftbackmotor;
  public static final WPI_TalonFX leftfrontmotor = Constants.leftfrontmotor;
  public static final WPI_TalonFX rightbackmotor = Constants.rightbackmotor;
  public static final WPI_TalonFX rightfrontmotor = Constants.rightfrontmotor;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    leftfrontmotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    rightfrontmotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    leftbackmotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    rightbackmotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
   
    rightfrontmotor.setInverted(true);
    leftbackmotor.setInverted(false);
    rightbackmotor.setInverted(true);
    leftfrontmotor.setInverted(false);
   
    leftfrontmotor.configNominalOutputForward(0, 10);
    leftfrontmotor.configNominalOutputReverse(0, 10);
    leftfrontmotor.configPeakOutputForward(1, 10);
    leftfrontmotor.configPeakOutputReverse(-1, 10);
    leftfrontmotor.configNeutralDeadband(0.001, 10);

    rightfrontmotor.configNominalOutputForward(0, 10);
    rightfrontmotor.configNominalOutputReverse(0, 10);
    rightfrontmotor.configPeakOutputForward(1, 10);
    rightfrontmotor.configPeakOutputReverse(-1, 10);
    rightfrontmotor.configNeutralDeadband(0.001, 10);

    leftbackmotor.configNominalOutputForward(0, 10);
    leftbackmotor.configNominalOutputReverse(0, 10);
    leftbackmotor.configPeakOutputForward(1, 10);
    leftbackmotor.configPeakOutputReverse(-1, 10);
    leftbackmotor.configNeutralDeadband(0.001, 10);

    rightbackmotor.configNominalOutputForward(0, 10);
    rightbackmotor.configNominalOutputReverse(0, 10);
    rightbackmotor.configPeakOutputForward(1, 10);
    rightbackmotor.configPeakOutputReverse(-1, 10);
    rightbackmotor.configNeutralDeadband(0.001, 10);

    // Sets how much error is allowed
    leftfrontmotor.configAllowableClosedloopError(0, 0, 10);
    leftbackmotor.configAllowableClosedloopError(0, 0, 10);
    rightfrontmotor.configAllowableClosedloopError(0, 0, 10);
    rightbackmotor.configAllowableClosedloopError(0, 0, 10);

    resetEncoders();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  public void resetEncoders() {
    leftbackmotor.setSelectedSensorPosition(0);
    rightbackmotor.setSelectedSensorPosition(0);
    leftfrontmotor.setSelectedSensorPosition(0);
    rightfrontmotor.setSelectedSensorPosition(0);
  }
  public static double getRightBackEncoderPosition(){
    return rightbackmotor.getSelectedSensorPosition();
  }
  public static double getLeftBackEncoderPosition(){
    return leftbackmotor.getSelectedSensorPosition();
  }
  
  public static double distanceTravelledTicks(){
    return(getLeftBackEncoderPosition() + getRightBackEncoderPosition()) / 2;
  }

  public static double distanceTravelledMetter(){
    double dist =distanceTravelledTicks() *Meters_Per_Ticks;
    return (dist);
  }
  private static final double In_To_M=.0254;
  private static final int Motor_Encoder_Codes_Per_Rev=2048;
  private static final double Diameter_Inches=5.0;
  private static final double Wheel_Diameter= Diameter_Inches * In_To_M;
  private static final double Wheel_Circumference= Wheel_Diameter * Math.PI;
  private static final double Gear_Ratio=12.75;
  private static final double Ticks_Per_Meter= ( Motor_Encoder_Codes_Per_Rev * Gear_Ratio)/(Wheel_Circumference);
  private static final double Meters_Per_Ticks= 1/Ticks_Per_Meter;

  public static double kP=1;
  public static double kI =0;
  public static double kD =0;
  public static double lastimestamp = Timer.getFPGATimestamp();
  public static double dt = Timer.getFPGATimestamp() - lastimestamp;
  public static double setpoint= 2; //meter
  public static double error;

  // @Override
  public void autonomousPeriodic() {
  
  double currrentpos = distanceTravelledMetter();
  error = setpoint - currrentpos;
  double errorsum = dt * error;
  double lasterror = error;
  double errorrate= error - lasterror;

    double output = ((error *kP)) +(errorsum * kI) + (errorrate *kD);

    SmartDashboard.putNumber("output value", output);
   

    if(output >= 1){
      output = 1;
    }
    else{output = output;}
    leftbackmotor.set(output);
    rightbackmotor.set(output);
    rightfrontmotor.set(output);
    leftfrontmotor.set(output);


    
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
