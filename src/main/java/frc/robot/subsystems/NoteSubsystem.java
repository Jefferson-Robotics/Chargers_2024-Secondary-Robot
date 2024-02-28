// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NoteSubsystem extends SubsystemBase {
  /** Creates a new NoteSubsystem. */
  private final DigitalInput Beam;
  public NoteSubsystem(int BeamDIONum) {
    Beam = new DigitalInput(BeamDIONum);
  }
  public boolean noteColleted(){
    return !Beam.get();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
