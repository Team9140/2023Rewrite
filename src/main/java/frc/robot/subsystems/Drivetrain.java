package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

public class Drivetrain extends SubsystemBase {

    // make motor controllers
    private final TalonFX m_leftMotor = new TalonFX(1);
    private final TalonFX m_rightMotor = new TalonFX(2);

    private final DifferentialDrive m_RobotDrive = new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);


    public Command doTheDriveThingy(Supplier<Double> stickForward, Supplier<Double> stickAngle, Supplier<Boolean>pressInPlace ) {
        return this.run(() -> {
            double forward = stickForward.get() / 10;
            double angle = stickAngle.get();
            boolean allowTurnInPlace = pressInPlace.get();
            m_RobotDrive.curvatureDrive(forward, angle, allowTurnInPlace);
        });
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}

