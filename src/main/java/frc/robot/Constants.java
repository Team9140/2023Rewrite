package frc.robot;

public class Constants {

    public static class Drivetrain {
        public static final double DEADBAND = 0.15;
        public static double SLOW_WHEEL_TURN_GAIN = 8.0;
        public static double FAST_WHEEL_TURN_GAIN = 1.0;

        public static double ARM_EXTENDED_QUICKTURN_WHEEL_GAIN = 1.5;
        public static double ARM_STOW_QUICKTURN_WHEEL_GAIN = 2.0;
        public static double ARM_EXTENDED_WHEEL_GAIN = 0.75;

        public static final double WHEEL_NONLINEARITY = 0.07;
        public static final double TRACK_WIDTH_INCHES = 17.5;
        public static final double TRACK_WIDTH_METERS = TRACK_WIDTH_INCHES * Units.METERS_PER_INCH;

        public static final double DRIVE_RATIO = 8.45;

        public static final double MOTOR_RPM = 5820.0;

        public static final double WHEEL_DIAMETER = 6.0;

        public static final double DRIVE_MAX_MPS = (MOTOR_RPM / DRIVE_RATIO / Units.SECONDS_PER_MINUTE) * WHEEL_DIAMETER * Math.PI * Units.METERS_PER_INCH;
        public static final double POSITION_CONVERSION = (1 / DRIVE_RATIO) * WHEEL_DIAMETER * Math.PI * Units.METERS_PER_INCH;
        public static final double VELOCITY_CONVERSION = POSITION_CONVERSION / Units.SECONDS_PER_MINUTE;

        // Set the current limit, an integer in amps, for the drivetrain.
        public static boolean ENABLE_CURRENT_LIMIT = true;
        public static int CURRENT_LIMIT = 40;

        public static double ARM_EXTENDED_FORWARD_MULTIPLIER = 0.75;
        public static double ARM_STOW_FORWARD_MULTIPLIER = 1.0;
        public static double ARM_EXTENDED_TURN_MULTIPLIER = 0.75;
        public static double ARM_STOW_TURN_MULTIPLIER = 1.0;
        public static double LIMITER = 0.80;

        public static double CHARGE_STATION_PITCH = 13;
        public static final double LEVEL_PITCH = 11.0;

        public static class Feedforward {
            public static class Left {
                public static final double S = 0.15863;
                public static final double V = 2.2191;
                public static final double A = 0.33996;
            }

            public static class Right {
                public static final double S = 0.13256;
                public static final double V = 2.1868;
                public static final double A = 0.14549;
            }

            public static class Avg {
                public static final double S = 0.145595;
                public static final double V = 2.20295;
                public static final double A = 0.242725;
            }
        }

        public static class PIDLoop {
            public static final double LeftP = 3.2313;
            public static final double RightP = 2.6093;
        }
    }

    public static class Units {
        public static final double SECONDS_PER_MINUTE = 60.0;

        public static final double METERS_PER_INCH = 0.0254;

        public static final double RADIANS_PER_ROTATION = 2.0 * Math.PI;
        public static final double SECONDS_PER_LOOP = 0.020;
    }
}
