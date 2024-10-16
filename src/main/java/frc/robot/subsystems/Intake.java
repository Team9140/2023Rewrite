package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private static Intake instance;

    private TalonSRX motor;

    private Intake() {
        this.motor = new TalonSRX(3);
    }

    public static Intake getInstance() {
        return instance == null ? (instance = new Intake()) : instance;
    }

    public Command setIntake() {
        return this.runOnce(() -> this.motor.set(TalonSRXControlMode.PercentOutput, 0.5));
    }

    public Command stopIntake() {
        return this.runOnce(() -> this.motor.set(TalonSRXControlMode.PercentOutput, 0));
    }

    public Command setOuttake() {
        return this.runOnce(() -> this.motor.set(TalonSRXControlMode.PercentOutput, -0.5));
    }
}
