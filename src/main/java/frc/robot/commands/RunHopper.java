// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;

public class RunHopper extends Command {
  private final Hopper hopperSubsystem;

  public RunHopper(Hopper hSubsystem) {
    hopperSubsystem = hSubsystem;
    addRequirements(hopperSubsystem);
  }

  public Command in(){
    return hopperSubsystem.runOnce(() -> hopperSubsystem.runLeft(hopperSubsystem.hopperSpeed))
                          .andThen(() -> hopperSubsystem.runRight(hopperSubsystem.hopperSpeed))
                          .andThen(() -> System.out.println("hopper in"));
  }

  public Command out(){
    return hopperSubsystem.runOnce(() -> hopperSubsystem.reverseLeft(hopperSubsystem.reverseHopperSpeed))
                          .andThen(() -> hopperSubsystem.reverseRight(hopperSubsystem.reverseHopperSpeed))
                          .andThen(() -> System.out.println("hopper out"));
  }

  public Command stop(){
    return hopperSubsystem.runOnce(() -> hopperSubsystem.runLeft(0))
                          .andThen(() -> hopperSubsystem.runRight(0))
                          .andThen(() -> System.out.println("hopper stop"));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
