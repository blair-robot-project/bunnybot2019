package org.usfirst.frc.team449.robot.components;

import edu.wpi.first.wpilibj.drive.Vector2d;

/**
 * A component to determine the 4 motor outputs for a mecanum drive from an x vector, y vector, and theta.
 */
public class MecanumComponent {

	/**
	 * Desired velocities on [-1, 1]. (Rotation is in CCW+ rotations/sec)
	 */
	double desiredLongitudinalVelocity, desiredLateralVelocity, desiredRotationalVelocity;

	/**
	 * Gyro angle in degrees
	 */
	double gyroAngularDisplacement;

	/**
	 * Default constructor
	 *
	 * @param desiredLongitudinalVelocity The desired forward-backward velocity, on [-1, 1].
	 * @param desiredLateralVelocity      The desired left-right velocity, on [-1, 1].
	 * @param desiredRotationalVelocity   The desired counter-clockwise positive rotations/sec, on [-1, 1].
	 * @param gyroAngularDisplacement     The gyro's angular displacement in degrees.
	 */
	public MecanumComponent(double desiredLongitudinalVelocity, double desiredLateralVelocity, double desiredRotationalVelocity,
	                        double gyroAngularDisplacement) {
		this.desiredLongitudinalVelocity = desiredLongitudinalVelocity;
		this.desiredLateralVelocity = desiredLateralVelocity;
		this.desiredRotationalVelocity = desiredRotationalVelocity;
		this.gyroAngularDisplacement = gyroAngularDisplacement;
	}

	/**
	 * Initial constructor for command to use.
	 */
	public MecanumComponent() {}

	/**
	 * Update the desired velocities for this component.
	 *
	 * @param desiredLongitudinalVelocity The desired forward-backward velocity, on [-1, 1].
	 * @param desiredLateralVelocity      The desired left-right velocity, on [-1, 1].
	 * @param desiredRotationalVelocity   The desired counter-clockwise positive rotations/sec, on [-1, 1].
	 * @param gyroAngularDisplacement     The gyro's angular displacement in degrees.
	 */
	public void updateDesiredVelocities(double desiredLongitudinalVelocity, double desiredLateralVelocity, double desiredRotationalVelocity,
	                                    double gyroAngularDisplacement) {
		this.desiredLongitudinalVelocity = desiredLongitudinalVelocity;
		this.desiredLateralVelocity = desiredLateralVelocity;
		this.desiredRotationalVelocity = desiredRotationalVelocity;
		this.gyroAngularDisplacement = gyroAngularDisplacement;
	}

	/**
	 * Determines wheel velocities based on desired robot velocities.<br>
	 * See {@link edu.wpi.first.wpilibj.drive.MecanumDrive#driveCartesian(double, double, double, double)} for math.
	 * @return A double array containing the four motor velocities to command (in the order of front right, front left,
	 * rear left, rear right).
	 */
	public double[] calculateMotorOutputs() {
		// Compensate for gyro angle.
		Vector2d input = new Vector2d(desiredLateralVelocity, desiredLongitudinalVelocity);
		input.rotate(-gyroAngularDisplacement);

		//Compute wheel speeds
		double[] motorOutputs = new double[4];
		//Front right
		motorOutputs[0] = -input.x + input.y + desiredRotationalVelocity;
		//Front left
		motorOutputs[1] = input.x + input.y - desiredRotationalVelocity;
		//Rear left
		motorOutputs[2] = -input.x + input.y + desiredRotationalVelocity;
		//Rear right
		motorOutputs[3] = input.x + input.y - desiredRotationalVelocity;

		//Normalize wheel speeds
		double maxMagnitude = Math.abs(motorOutputs[0]);
		for (int i = 1; i < motorOutputs.length; i++) {
			double temp = Math.abs(motorOutputs[i]);
			if (maxMagnitude < temp) {
				maxMagnitude = temp;
			}
		}
		if (maxMagnitude > 1.0) {
			for (int i = 0; i < motorOutputs.length; i++) {
				motorOutputs[i] = motorOutputs[i] / maxMagnitude;
			}
		}

		return motorOutputs;
	}

}
