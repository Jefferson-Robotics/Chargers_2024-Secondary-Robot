// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.DifferentialMechanism;
import com.ctre.phoenix6.controls.DifferentialFollower;
import com.ctre.phoenix6.controls.DutyCycleOut;

public class DriveTrain extends SubsystemBase {
  /** Creates a new falcon. */
  private final TalonFX Lmaster = new TalonFX(10);
  private final TalonFX Lslave = new TalonFX(12);

  private final TalonFX Rmaster = new TalonFX(11);
  private final TalonFX Rslave = new TalonFX(13);

  private final DutyCycleOut

  private
  

  public DriveTrain() {
    
    
  }
  public void drive(double xboxleft,double xboxright){
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
