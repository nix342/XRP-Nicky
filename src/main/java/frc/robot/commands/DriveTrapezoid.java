// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveTrapezoid extends Command {
  private double m_distance;
  private Drivetrain m_drive;
  private Timer m_timer  = new Timer();
  private TrapezoidProfile m_profile;

  private static final double kMaxVelocity = 24.0; // inches per second
  private static final double kMaxAcceleration = 48.0; // inches per second squared
  private static final TrapezoidProfile.Constraints kConstraints =
      new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);

      TrapezoidProfile.State initialState;
      TrapezoidProfile.State goalState;


  /** Creates a new DriveTrapezoid. */
  public DriveTrapezoid(double distance, Drivetrain drivetrain) {

    m_distance = distance;
    m_drive = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.arcadeDrive(0, 0);
    m_drive.resetEncoders();
    m_timer.reset();
    m_timer.start();

    initialState =
        new TrapezoidProfile.State(0.0, 0.0);
    goalState = new TrapezoidProfile.State(m_distance, 0.0);
    
    m_profile = new TrapezoidProfile(kConstraints);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.arcadeDrivePIDVelocity(m_profile.calculate(m_timer.get(), initialState, goalState).velocity,0);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() > m_profile.totalTime();
  }
}
