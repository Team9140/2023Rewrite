package frc.robot;

public class Constants {

    public static final double INTAKE_VOLTAGE = 0.5;

    public static class Ports {
        public static final double INTAKE_MOTOR = 3;
        public static final int ARM_MOTOR = 1;
        public static final String CTRE_CANBUS = "";
    }

    public static class Positions {
        public static final double OVERHAND = 0.1;
        public static final double INTAKE = 0.1;
        public static final double UNDERHAND = 0.1;
        public static final double AMP = 0.1;
    }

    public static class Arm {
        public static final double P = 0.1;
        public static final double I = 0.1;
        public static final double D = 0.1;
        public static final double S = 0.1;
        public static final double V = 0.1;
        public static final double A = 0.1;

        public static final double CRUISE_VELOCITY = 0.1;
        public static final double ACCELERATION = 0.1;
        public static final double SENSOR_TO_MECHANISM_RATIO = 0.1;

        public static final double  MAX_CURRENT = 0.1;
    }



}
