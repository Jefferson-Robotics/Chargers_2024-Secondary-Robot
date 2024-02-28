// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NoteSubsystem;

public class AutoPickup extends Command {
  /** Creates a new NoteAuto. */
  private final Camera camera;
  private DriveSubsystem drive;
  private final NoteSubsystem NoteSystem;
  private int x;
  private int y;
  private boolean done = false;
  public AutoPickup(DriveSubsystem drive,NoteSubsystem noteSystem,Camera cameraSystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    NoteSystem = noteSystem;
    camera = cameraSystem;
    this.drive=drive;
    addRequirements(cameraSystem,drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done=false;
    x=0;
    y=0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    camera.readDataStream();
    x=camera.getX();
    y=camera.getY();
    System.out.println(x);
    System.out.println(y);
    if(y<240){
      if(x<-25){
        drive.arcadeDrive(.2, -.05);
      }else if(25<x){
        drive.arcadeDrive(.2, .05);
      }else{
        drive.arcadeDrive(.2, 0);
      }
    }else{
      drive.arcadeDrive(0, 0);
      done=true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.arcadeDrive(0, 0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
