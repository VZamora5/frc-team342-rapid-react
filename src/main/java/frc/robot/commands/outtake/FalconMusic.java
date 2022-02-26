// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.outtake;

import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OuttakeSubsystem;

public class FalconMusic extends CommandBase {
  private OuttakeSubsystem outtake;
  private Orchestra orchestra;

  private boolean hasStarted = false;
  private String song;

  /** Creates a new FalconMusic. */
  public FalconMusic(OuttakeSubsystem outtake, String song) {
    this.outtake = outtake;
    this.orchestra = this.outtake.getOrchestra();
    this.song = song;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(outtake);
  }

  public FalconMusic(OuttakeSubsystem outtake) {
    this(outtake, "cara-mi-addio.chrp");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // load music files
    orchestra.loadMusic(song); 

    // Play music
    orchestra.play();
    hasStarted = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    orchestra.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // isPlaying can return false before song has started so an extra check is needed
    return !orchestra.isPlaying() && hasStarted;
  }
}
