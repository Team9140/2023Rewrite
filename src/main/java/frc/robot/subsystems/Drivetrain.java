package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Drivetrain extends SubsystemBase {
    private static Drivetrain instance;


    // make motor controllers
    private final TalonFX m_leftMotor = new TalonFX(1);
    private final TalonFX m_rightMotor = new TalonFX(2);
    private final ADIS16470_IMU gyro = new ADIS16470_IMU();
    private final DifferentialDrive m_RobotDrive = new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);
    private SlewRateLimiter forwardSlewLimiter = new SlewRateLimiter(2.0);
    private double forwardSlewLimiterValue = 1;
    private SlewRateLimiter turnSlewLimiter = new SlewRateLimiter(1.0);
    private double wheelGain = 1;
    PIDController teleopTurnController = new PIDController(0.91628, 0.0, 0.0, Constants.Units.SECONDS_PER_LOOP);
    SimpleMotorFeedforward teleopTurnFeedforward = new SimpleMotorFeedforward(0.45277, 0.21407, 0.020394);
    private SimpleMotorFeedforward left_feedforward;
    private SimpleMotorFeedforward right_feedforward;
    private Drivetrain() {
        this.gyro.setGyroAngleY(0.0);
        this.left_feedforward = new SimpleMotorFeedforward(Constants.Drivetrain.Feedforward.Left.S,
                Constants.Drivetrain.Feedforward.Left.V, Constants.Drivetrain.Feedforward.Left.A);
        this.right_feedforward = new SimpleMotorFeedforward(Constants.Drivetrain.Feedforward.Right.S,
                Constants.Drivetrain.Feedforward.Right.V, Constants.Drivetrain.Feedforward.Right.A);
    }

    public static Drivetrain getInstance() {
        return instance != null ? instance : (instance = new Drivetrain());
    }

    public Command doTheDriveThingy(DoubleSupplier leftStickY, DoubleSupplier rightStickY, BooleanSupplier spinInPlace ) {
        return this.run(() -> {
            double speed = leftStickY.getAsDouble();
            double rotation = rightStickY.getAsDouble();
            boolean allowSpin = spinInPlace.getAsBoolean();

            curvatureDrive( speed, rotation, allowSpin);


        });

    }

    public void curvatureDrive(double throttle, double wheel, boolean quickTurn) {
        throttle = MathUtil.applyDeadband(throttle, Constants.Drivetrain.DEADBAND);
        wheel = MathUtil.applyDeadband(wheel, Constants.Drivetrain.DEADBAND);
        wheel = wheel * wheel * ((wheel < 0) ? -1 : 1);

        double vx = throttle * Constants.Drivetrain.DRIVE_MAX_MPS
                * this.forwardSlewLimiter.calculate(this.forwardSlewLimiterValue);

        double omega;

        this.wheelGain = Constants.Drivetrain.SLOW_WHEEL_TURN_GAIN
                - ((Constants.Drivetrain.SLOW_WHEEL_TURN_GAIN - Constants.Drivetrain.FAST_WHEEL_TURN_GAIN))
                * (vx / Constants.Drivetrain.DRIVE_MAX_MPS);

        // omega = wheel * Math.abs(throttle) * this.turnSlewLimiter.calculate(this.wheelGain);
        omega = wheel * Math.abs(throttle) * this.wheelGain;

        double turnVolts = (teleopTurnController.calculate(this.getYawRate(), omega)
                + teleopTurnFeedforward.calculate(omega));
        double leftForwardVolts = this.left_feedforward.calculate(vx);
        double rightForwardVolts = this.right_feedforward.calculate(vx);

        this.setDriveVolts(leftForwardVolts - turnVolts, rightForwardVolts + turnVolts);
    }

    public void setDriveVolts(double leftVolts, double rightVolts) {
        this.m_leftMotor.setVoltage(leftVolts);
        this.m_rightMotor.setVoltage(rightVolts);
    }

    public double getYawRate() {
        return Math.toRadians(-this.gyro.getRate());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}

