// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;


public class Robot extends TimedRobot {
  private Arm arm;
  private Drivetrain drive;
  private CommandXboxController xb;
  private Intake intake;

  @Override
  public void robotInit() {
    this.xb = new CommandXboxController(0);
    this.intake = Intake.getInstance();
    arm = Arm.getInstance();


    this.xb.x().onTrue(arm.setArmPosition(0.0));
    this.xb.y().onTrue(arm.setArmPosition(4.0*Math.PI / 5.0));
    this.xb.b().onTrue(arm.setArmPosition((7.0*Math.PI) / 6.0));
    this.xb.a().onTrue(arm.setArmPosition(4.90));

    this.xb.rightTrigger().onTrue(intake.setIntake());
    this.xb.rightTrigger().onFalse(intake.stopIntake());
    this.xb.rightBumper().onTrue(intake.setOuttake());
    this.xb.rightBumper().onFalse(intake.stopIntake());


    drive = new Drivetrain();
    drive.setDefaultCommand(
            drive.doTheDriveThingy(xb::getLeftY, xb::getLeftX, xb.getHID()::getLeftBumper)
    );
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
