// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
  public enum Drivetrain{
    WEST_COAST,
    SWERVE
  }

  private Drivetrain m_drivetrain;
  /** Creates a new Drive. */
  public Drive(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
