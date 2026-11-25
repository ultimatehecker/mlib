package org.middletonrobotics.mlib.controller;

public class ArmFeedforward {
	private double kS;
	private double kG;
	private double kV;
	private double kA;
	private double period;

	public ArmFeedforward(double kS, double kG, double kV, double kA, double period) {
		if (kS < 0.0 || kG < 0.0 || kV < 0.0 || kA < 0.0) {
			throw new IllegalArgumentException("Feedforward gains must be an unsigned number");
		}

		if (period <= 0.0) {
			throw new IllegalArgumentException("Time period must be a positive number");
		}

		this.kS = kS;
		this.kG = kG;
		this.kV = kV;
		this.kA = kA;
		this.period = period;
	}

	public double calculate(double currentAngle, double velocity) {
		return calculate(currentAngle, velocity, velocity);
	}

	public double calculate(double currentAngle, double currentVelocity, double nextVelocity) {
		double acceleration = (nextVelocity - currentVelocity) / period;
		return kS * Math.signum(currentVelocity) + kV * currentVelocity + kA * acceleration + kG * Math.cos(currentAngle);
	}

	public double getKS() {
		return kS;
	}

	public double getKG() {
		return kG;
	}

	public double getKV() {
		return kV;
	}

	public double getKA() {
		return kA;
	}

	public void setKS(double kS) {
		this.kS = kS;
	}

	public void setKG(double kG) {
		this.kG = kG;
	}

	public void setKV(double kV) {
		this.kV = kV;
	}

	public void setKA(double kA) {
		this.kA = kA;
	}
}
