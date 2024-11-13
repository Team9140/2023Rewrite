// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;


public class Robot extends TimedRobot {
  private Arm arm;
  private Drivetrain drive;
  private CommandXboxController xb;
  private Joystick joy;
  private Intake intake;

  //just to add mechanism to 3d view in AdvantagesScope for now
  Pose3d poseA = new Pose3d();
  StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose3d.struct).publish();

  @Override
  public void robotInit() {
    this.xb = new CommandXboxController(0);
    joy = new Joystick(0);
    this.intake = Intake.getInstance();
    arm = Arm.getInstance();

    this.xb.a().onTrue(intake.setIntake()).onFalse(intake.stopIntake());
    this.xb.x().onTrue(intake.setOuttake()).onFalse(intake.stopIntake());
    this.xb.b().toggleOnTrue(arm.setArmPosition(Math.PI));
    this.xb.y().onTrue(arm.setArmPosition(0));

    new Trigger(() -> joy.getX() > 0.5)
                .onTrue(this.arm.setArmPosition(Math.PI));

    new Trigger(() -> joy.getX() < -0.5)
                .onTrue(this.arm.setArmPosition(0));

    drive = new Drivetrain();
    drive.setDefaultCommand(
            drive.doTheDriveThingy(xb::getLeftY, xb::getRightX, xb.getHID()::getLeftBumper)
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

  @Override
  public void simulationPeriodic() {
    publisher.set(poseA);
    SmartDashboard.putNumber("Joystick X Value", joy.getX());
  }
}
