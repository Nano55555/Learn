// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class Joystickdrive extends CommandBase {
  /** Creates a new Joystickdrive. */
  private static final XboxController drivecontroller = RobotContainer.drivercontroller;
  private static final DriveTrain drivetrain = new DriveTrain();

  public Joystickdrive(DriveTrain drivesubsystem ) {
    drivesubsystem = drivetrain;
    addRequirements(drivesubsystem);
    
    // Use addRequirements() here to declare subsystem dependencies.
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double drive = drivecontroller.getLeftY();
    double rotate = drivecontroller.getRightX();
    if ((drive <0.25 && drive >0)|| (drive <0 && drive >-0.25)){
      drive =0;
    }
    else {
      drive= drive*0.5;
    }
    if ((rotate>0 &&rotate<0.25)|| (rotate<0 &&rotate >-0.25)){
      rotate =0;
    }
    else {
      rotate = rotate *2;
    }
    DriveTrain.drive(drive,rotate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
