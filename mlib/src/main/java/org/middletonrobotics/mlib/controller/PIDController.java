package org.middletonrobotics.mlib.controller;

import org.middletonrobotics.mlib.utilities.MathUtilities;

public class PIDController {
	private double kP;
	private double kI;
	private double kD;

	private double period;

	private double iZone = Double.POSITIVE_INFINITY;
	private double integral = 0.0;
	private double minimumIntegral = -1e6;
	private double maximumIntegral = 1e6;

	private double setpoint = 0.0;
	private double error = 0.0;
	private double errorDerivative = 0.0;

	private double previousMeasurement = 0.0;
	private boolean hasPreviousMeasurement = false;

	private double positionTolerence = 0.05;
	private double velocityTolerence = Double.POSITIVE_INFINITY;

	private boolean continuous = false;
	private double minimumInput = 0.0;
	private double maximumInput = 0.0;

	public PIDController(double kP, double kI, double kD, double period) {
		if (kP < 0.0 || kI < 0.0 | kD < 0.0) {
			throw new IllegalArgumentException("All coefficients must be an unsigned number.");
		}

		if (period <= 0.0) {
			throw new IllegalArgumentException("Time period must be a positive number");
		}

		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.period = period;
	}

	public PIDController(double kP, double kI, double kD) {
		this(kP, kI, kD, 0.01);
	}

	public PIDController(double kP, double kD) {
		this(kP, 0.0, kD, 0.01);
	}

	public double calculate(double measurement, double setpoint, double period) {
		if (period < 0.0) {
			throw new IllegalArgumentException("Time period must be an unsigned number");
		}

		return computeOutput(measurement, setpoint, period);

	}

	public double calculate(double measurement, double setpoint) {
		return computeOutput(measurement, setpoint, period);	
	}

	private double computeOutput(double measurement, double setpoint, double period) {
		this.setpoint = setpoint;
		error = setpoint - measurement;

		if (continuous) {
			double errorBound = (maximumInput - minimumInput) / 2;
			error = MathUtilities.inputModulus(setpoint - measurement, -errorBound, errorBound);
		} else {
			error = setpoint - measurement;
		}

		if (hasPreviousMeasurement) {
			double positionDelta = measurement - previousMeasurement;

			if (continuous) {
				MathUtilities.inputModulus(positionDelta, minimumInput, maximumInput);
			}
			
			errorDerivative = -positionDelta / period;
		} else {
			errorDerivative = 0.0;
		}

		previousMeasurement = measurement;
		hasPreviousMeasurement = true;

		if (Math.abs(error) > iZone) {
			integral = 0.0;
		} else if (kI != 0.0) {
			integral += error * period;
			integral = MathUtilities.clamp(integral, minimumIntegral / kI, maximumIntegral / kI);
		}

		return (kP * error) + (kI * integral) + (kD * errorDerivative);
	}

	public void reset() {
		integral = 0.0;
		error = 0.0;
		errorDerivative = 0.0;
		hasPreviousMeasurement = false;
	}

	public boolean atSetpoint() {
		return hasPreviousMeasurement && Math.abs(error) <= positionTolerence && Math.abs(errorDerivative) <= velocityTolerence;
	}

	public void setTolerence(double positionTolerence, double velocityTolerence) {
		if (positionTolerence <= 0.0 || velocityTolerence <= 0.0) {
			throw new IllegalArgumentException("All tolerences must be a unsigned number");
		}

		this.positionTolerence = positionTolerence;
		this.velocityTolerence = velocityTolerence;
	}

	public void setTolerence(double positionTolerence) {
		setTolerence(positionTolerence, Double.POSITIVE_INFINITY);
	}

	public void enableContinuousInput(double minimumInput, double maximumInput) {
		if (maximumInput <= minimumInput) {
			throw new IllegalArgumentException("The maximum input cannot be lower than the minimum input");
		}

		this.continuous = true;
		this.maximumInput = maximumInput;
		this.minimumInput = minimumInput;
	}

	public void setIZone(double iZone) {
		if (iZone < 0.0) {
			throw new IllegalArgumentException("The integral zone must be an unsigned number");
		}

		this.iZone = iZone;
	}

	public void setIntegralRange(double maximumIntegral, double minimumIntegral) {
		if (maximumIntegral <= minimumIntegral) {
			throw new IllegalArgumentException("The maximum integral bound cannot be lower than the minimum integral bound");
		}

		this.maximumIntegral = maximumIntegral;
		this.minimumIntegral = minimumIntegral;
	}

	public void setCoefficients(double kP, double kI, double kD) {
		if (kP < 0.0 || kI < 0.0 || kD < 0.0) {
			throw new IllegalArgumentException("All coefficients must be a unsigned number");
		}

		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
	}

	public double getKp() {
		return kP;
	}

	public double getKi() {
		return kI;
	}

	public double getKd() {
		return kD;
	}

	public double getSetpoint() {
		return setpoint;
	}

	public double getError() {
		return error;
	}

	public double getErrorDerivative() {
		return errorDerivative;
	}

	public double getPositionTolerence() {
		return positionTolerence;
	}

	public double getVelocityTolerence() {
		return velocityTolerence;
	}

	public double getPeriod() {
		return period;
	}

	public double getIZone() {
		return iZone;
	}
}
