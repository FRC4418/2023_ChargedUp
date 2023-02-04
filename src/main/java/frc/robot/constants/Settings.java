package frc.robot.constants;


// import com.stuypulse.stuylib.network.SmartBoolean;
// import com.stuypulse.stuylib.network.SmartNumber;
// import com.stuypulse.stuylib.network.SmartString;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;


/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {

    public interface Drivetrain {
        // If speed is below this, use quick turn
        double BASE_TURNING_SPEED = .25;
        double INVERT_ANGLE_THREASHOLD = 0.15;

        // Low Pass Filter and deadband for Driver Controls
        double SPEED_DEADBAND = 0.00;
        double ANGLE_DEADBAND = 0.10;
        
        double MAX_SPEED_ANGLE = 0.85;
        double MAX_SPEED = 0.7;

        double SPEED_POWER = 2.0;
        double ANGLE_POWER = 1.0;

        double SPEED_FILTER = 0.25;
        double ANGLE_FILTER = 0.005;

        double DISPLACEMENT_METERS = 0.0;
        double TIME_MOVING = 0.0;

        // Width of the robot
        double TRACK_WIDTH = Units.inchesToMeters(26.9); 

        public interface Motion {

            DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH);

            public interface FeedForward {
                double kS = 0.20094;
                double kV = 1.6658;
                double kA = 0.4515;
            }

            public interface PID {
                int kSlot = 0;
                double kF = 0;
                double kP = 0;
                double kI = 0;
                double kD = 0;
                double kTimeoutMs = 50;
            }
        }

        // Encoder Constants
        public interface Encoders {

            public interface GearRatio {

                public interface Stages {
                    double FIRST_STAGE = (8.0 / 60.0);

                    double SECOND_STAGE = (1.0 / 1.0);
                }

                double ENCODER_TO_WHEEL = Stages.FIRST_STAGE * Stages.SECOND_STAGE;
            }

            double WHEEL_DIAMETER = Units.inchesToMeters(4);
            double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

            double ENCODER_PULSES_PER_REVOLUTION = 2048;
            double ENCODER_DISTANCE_PER_PULSE =
                    (WHEEL_CIRCUMFERENCE / ENCODER_PULSES_PER_REVOLUTION)
                            * GearRatio.ENCODER_TO_WHEEL;
        }
    }

}
