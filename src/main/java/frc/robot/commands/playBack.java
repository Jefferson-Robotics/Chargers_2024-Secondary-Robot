// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.ObjectInputStream;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class playBack extends Command {
  /** Creates a new playBack. */
  private final SendableChooser<File> recSelector;
  private FileInputStream fis;
  private BufferedInputStream bif;
  private ObjectInputStream ois;
  private double[] data;
  private boolean isDone;

  private DriveSubsystem driveSubsystem;

  public playBack(SendableChooser<File> RecSelector,DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.recSelector=RecSelector;
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isDone=false;
    try {
      fis = new FileInputStream(recSelector.getSelected());
      bif = new BufferedInputStream(fis);
      ois = new ObjectInputStream(bif);
    } catch (Exception e) {
     System.out.println("Failed to start input stream.");
      e.printStackTrace();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    try{
      data = (double[]) ois.readObject();
      driveSubsystem.arcadeDrive(data[0],data[1]);
    }catch(Exception e){
      isDone = true;
      e.printStackTrace();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    try{
      ois.close();
      bif.close();
      fis.close();
    }catch(Exception e){
      System.out.println("Failed to stop input stream.");
      e.printStackTrace();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}