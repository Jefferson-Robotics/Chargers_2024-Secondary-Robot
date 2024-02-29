// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.ObjectOutputStream;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class rec extends Command {
  /** Creates a new rec. */
  private final File recDir;
  private int FileNum=0;
  public String FileType = ".txt";
  private File recFile;
  private FileOutputStream fos;
  private BufferedOutputStream bos;
  private ObjectOutputStream oos;

  private final SendableChooser<File> RecSelector;
  private final GenericEntry FileName;
  private final DriveSubsystem Drive;

  public rec(File recDir,SendableChooser<File> RecSelector, GenericEntry FileName, DriveSubsystem DSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.recDir = recDir;
    this.RecSelector = RecSelector;
    this.FileName = FileName;
    this.Drive = DSub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try {
      recFile = new File(recDir,FileName.getString("rec"+FileNum)+FileType);
      fos = new FileOutputStream(recFile);
      bos = new BufferedOutputStream(fos);
      oos = new ObjectOutputStream(bos);
    } catch (Exception e) {
      System.out.println("Record failed to start output stream.");
      e.printStackTrace();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] data = {Drive.forwardSpeed,Drive.rotationSpeed};
    try {
      oos.writeObject(data);
    } catch (Exception e) {
     System.out.println("Record failed writing to file.");
      e.printStackTrace();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    try {
      oos.close();
      bos.close();
      fos.close();
    } catch (Exception e) {
     System.out.println("Record failed to start output stream.");
      e.printStackTrace();
    }
    RecSelector.addOption(recFile.getName(), recFile);
    FileNum++;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
