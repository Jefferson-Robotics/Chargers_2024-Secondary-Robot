// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoPickup;
import frc.robot.commands.playBack;
import frc.robot.commands.rec;
import frc.robot.subsystems.Camera;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.subsystems.DriveSubsystem;
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
import frc.robot.subsystems.NoteSubsystem;
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final NoteSubsystem noteSystem = new NoteSubsystem(0);
  private final Camera camera = new Camera(new SerialPort(115200, SerialPort.Port.kUSB1));
  private final AutoPickup notePickup = new AutoPickup(m_driveSubsystem, noteSystem, camera);
  private final ShuffleboardTab tab = Shuffleboard.getTab("RobotData");
  
  private final File recDir = new File("/home/lvuser/Recordings"); 
  private final SendableChooser<File> m_Chooser = new SendableChooser<>();
  private final GenericEntry fileNaming = tab.add("FileName", "")
   .withWidget(BuiltInWidgets.kTextView).getEntry();
  private final rec recordCommand = new rec(recDir,m_Chooser,fileNaming,m_driveSubsystem);
  private playBack playB = new playBack(m_Chooser,m_driveSubsystem);

  
  private final CommandXboxController m_joystick = new CommandXboxController(0);

  /* invert the joystick Y because forward Y is negative */
  private final Command m_teleopDrive = new RunCommand(() -> {
      m_driveSubsystem.arcadeDrive(-m_joystick.getLeftY(), m_joystick.getRightX());
    },
    m_driveSubsystem);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_driveSubsystem.setDefaultCommand(m_teleopDrive);
    File[] files = recDir.listFiles();
    for (int i = 0; i < files.length; i++) {
      m_Chooser.addOption(files[i].getName(), files[i]);
    }
    tab.add("Autonomous Mode", m_Chooser)
    .withWidget(BuiltInWidgets.kComboBoxChooser)
    .withPosition(0, 0)
    .withSize(2, 1);
    m_driveSubsystem.setDefaultCommand(m_teleopDrive);
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* If the upper left shoulder button is pressed, drive straight */
    m_joystick.b().toggleOnTrue(recordCommand);
    m_joystick.y().toggleOnTrue(playB);
    m_joystick.x().toggleOnTrue(notePickup);
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return playB;
  }
}
