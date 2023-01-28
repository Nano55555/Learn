// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.ModuleLayer.Controller;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  private static final double In_To_M=.0254;
  private static final int Motor_Encoder_Codes_Per_Rev=2048;
  private static final double Diameter_Inches=5.0;
  private static final double Wheel_Diameter= Diameter_Inches * In_To_M;
  private static final double Wheel_Circumference= Wheel_Diameter * Math.PI;
  private static final double Gear_Ratio=12.75;
  private static final double Ticks_Per_Meter= ( Motor_Encoder_Codes_Per_Rev * Gear_Ratio)/(Wheel_Circumference);
  private static final double Meters_Per_Ticks= 1/Ticks_Per_Meter;


  /** Creates a new DriveTrain. */

  public DriveTrain() {
    Constants.leftfrontmotor.set(ControlMode.Follower, Constants.leftbackmotor.getDeviceID());
    Constants.rightfrontmotor.set(ControlMode.Follower, Constants.rightbackmotor.getDeviceID());

    Constants.leftbackmotor.setNeutralMode(NeutralMode.Coast);
    Constants.rightbackmotor.setNeutralMode(NeutralMode.Coast);
    Constants.leftfrontmotor.setNeutralMode(NeutralMode.Coast);
    Constants.rightfrontmotor.setNeutralMode(NeutralMode.Coast);

    Constants.rightfrontmotor.setInverted(true);
    Constants.leftbackmotor.setInverted(false);
    Constants.rightbackmotor.setInverted(true);
    Constants.leftfrontmotor.setInverted(false);

    Constants.leftfrontmotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    Constants.rightfrontmotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    Constants.leftbackmotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    Constants.rightbackmotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
   
    Constants.rightfrontmotor.setInverted(true);
    Constants.leftbackmotor.setInverted(false);
    Constants.rightbackmotor.setInverted(true);
    Constants.leftfrontmotor.setInverted(false);

  }
  public void drive(double throttle, double rotate){
    Constants.leftbackmotor.set(throttle + rotate);   
    Constants.rightbackmotor.set(throttle - rotate);
    Constants.leftfrontmotor.set(throttle + rotate);
    Constants.rightfrontmotor.set(throttle - rotate);
 
   }
  public void stop(){
  drive(0,0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
