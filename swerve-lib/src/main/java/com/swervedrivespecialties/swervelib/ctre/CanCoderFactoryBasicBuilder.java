package com.swervedrivespecialties.swervelib.ctre;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.swervedrivespecialties.swervelib.AbsoluteEncoder;
import com.swervedrivespecialties.swervelib.AbsoluteEncoderFactory;
import com.ctre.phoenix.sensors.CANCoder;

public class CanCoderFactoryBasicBuilder {

    public AbsoluteEncoderFactory<CanCoderAbsoluteConfiguration> build() {
        return configuration -> {

            CANCoder encoder = new CANCoder(configuration.getId(), configuration.getCanbus());

            return new EncoderImplementation(encoder);
        };
    }

    private static class EncoderImplementation implements AbsoluteEncoder {
        private final int ATTEMPTS = 3; // TODO: Allow changing number of tries for getting correct position

        private final CANCoder encoder;

        private EncoderImplementation(CANCoder encoder) {
            this.encoder = encoder;
        }

        @Override
        public double getAbsoluteAngle() {
            double angle = Math.toRadians(encoder.getPosition());

            ErrorCode code = encoder.getLastError();

            for (int i = 0; i < ATTEMPTS; i++) {
                if (code == ErrorCode.OK) break;
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) { }
                angle = Math.toRadians(encoder.getPosition());
                code = encoder.getLastError();
            }

            CtreUtils.checkCtreError(code, "Failed to retrieve CANcoder "+encoder.getDeviceID()+" absolute position after "+ATTEMPTS+" tries");

            angle %= 2.0 * Math.PI;
            if (angle < 0.0) {
                angle += 2.0 * Math.PI;
            }

            return angle;
        }

        @Override
        public Object getInternal() {
            return this.encoder;
        }
    }

    public enum Direction {
        CLOCKWISE,
        COUNTER_CLOCKWISE
    }
}
