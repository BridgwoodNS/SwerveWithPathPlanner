// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intakerrrr extends SubsystemBase {


private final TalonFX intakeMotor = new TalonFX(Constants.intakeMotor1);
private final TalonFX intakeMotor2 = new TalonFX(Constants.intakeMotor2); 


  /** Creates a new Intakerrrr. */
  public Intakerrrr() {



  }

public Command runIntake(DoubleSupplier speed) {
  return new InstantCommand(() -> {
    intakeMotor.set(speed.getAsDouble());
    intakeMotor2.set(speed.getAsDouble());
  }, this);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
