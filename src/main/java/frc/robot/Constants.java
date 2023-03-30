// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    public static class DrivetainMotors{
        public static int kDriveMotorFrontLeft = 2;
        public static int kRotationMotorFrontLeft = 1;
        public static int kDriveMotorBackLeft = 3;
        public static int kRotationMotorBackLeft = 4;
        public static int kRotationMotorBackRight = 12;
        public static int kDriveMotorBackRight = 13;
        public static int kRotationMotorFrontRight = 14;
        public static int kDriveMotorFrontRight = 15;
    }
    public static class AnalogEncoders {
        public static int encoderBackRight = 0;
        public static int encoderFrontLeft = 1;
        public static int encoderBackLeft = 2;
        public static int encoderFrontRight = 3;
    }

    public static class otherVars {
        public static double kDriveEncoderDistance = 0.5; // this is a temp value needs tuning
        public static double kMaxSpeedMetersPerSecond = 4.233672; // From WCP themselves
    }
}
