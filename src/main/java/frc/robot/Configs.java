package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import frc.robot.Constants.PID;

public class Configs {
    public static class ArmExtension {
        public static EncoderConfig encoderConfig = new EncoderConfig();

        public static ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
        public static MAXMotionConfig motionConfig = new MAXMotionConfig();

        static {
            encoderConfig
                .positionConversionFactor(Constants.Encoders.extenderPosition)
                .velocityConversionFactor(Constants.Encoders.extenderVelocity);

            closedLoopConfig
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(
                    PID.ArmExtension.kP,
                    PID.ArmExtension.kI,
                    PID.ArmExtension.kD
                )
                .outputRange(
                    -Constants.maxExtenderOutput,
                    Constants.maxExtenderOutput
                );

            motionConfig
                .allowedClosedLoopError(PID.ArmExtension.tolerance)
                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
                .maxVelocity(PID.ArmExtension.maxVelocity)
                .maxAcceleration(PID.ArmExtension.maxAcceleration);
        }
    }

    public static class ArmRotation {
        public static EncoderConfig encoderConfig = new EncoderConfig();

        public static ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
        public static MAXMotionConfig motionConfig = new MAXMotionConfig();

        static {
            encoderConfig
                .positionConversionFactor(Constants.Encoders.rotatorPosition)
                .velocityConversionFactor(Constants.Encoders.rotatorVelocity);

            closedLoopConfig
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(
                    PID.ArmRotation.kP,
                    PID.ArmRotation.kI,
                    PID.ArmRotation.kD
                )
                .outputRange(
                    -Constants.maxRotatorOutput,
                    Constants.maxRotatorOutput
                )
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, 360);

            motionConfig
                .allowedClosedLoopError(PID.ArmRotation.tolerance)
                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
                .maxVelocity(PID.ArmRotation.maxVelocity)
                .maxAcceleration(PID.ArmRotation.maxAcceleration);
        }
    }
}
