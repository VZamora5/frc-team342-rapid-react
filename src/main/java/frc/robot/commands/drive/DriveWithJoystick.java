// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;

public class DriveWithJoystick extends CommandBase {
  
  DriveSystem drive;
  Joystick leftJ;
  Joystick rightJ;
  
  /** Creates a new DriveWithJoystick. */
  public DriveWithJoystick(DriveSystem drive, Joystick leftJ, Joystick rightJ) {

    addRequirements(drive);
    this.drive = drive;
    this.leftJ = leftJ;
    this.rightJ = rightJ;
    // Use addRequirements() here to declare subsystem dependencies.
  }
  
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Checks whether joystick is within a deadzone and returns val
    double deadBandLeft = MathUtil.applyDeadband(leftJ.getY(), 0.15);
    double deadBandRight = MathUtil.applyDeadband(rightJ.getY(), 0.15);

    drive.drive(xVelocity, yVelocity, rotationVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    driveSystem.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
