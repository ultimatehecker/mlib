package org.middletonrobotics.mlib.controller;

public class SimpleMotorFeedforward {
	private double kS;
	private double kV;
	private double kA;
	private double period;

	public SimpleMotorFeedforward(double kS, double kV, double kA, double period) {
		if (kS < 0.0 || kV < 0.0 || kA < 0.0) {
			throw new IllegalArgumentException("Feedforward gains must be an unsigned number");
		}

		if (period <= 0.0) {
			throw new IllegalArgumentException("Time period must be a postive number");
		}

		this.kS = kS;
		this.kV = kV;
		this.kA = kA;
		this.period = period;
	}

	public SimpleMotorFeedforward(double kS, double kV, double kA) {
		this(kS, kV, kA, 0.01);
	}

	public double calculate(double velocity) {
		return calculate(velocity, velocity);
	}

	public double calculate(double currentVelocity, double nextVelocity) {
		if (kA < 1e-9) {
			return kS * Math.signum(nextVelocity) + kV * nextVelocity;
		} else {
			double a = -kV/kA;
			double b = 1.0 / kA;
			double ad = Math.exp(a * period);
			double bd = a > -1e-9 ? b * period : 1.0 / a * (ad - 1.0) * b;

			return kS * Math.signum(currentVelocity) + 1.0 / bd * (nextVelcity - ad * currentVelocity);
		}
	}

	public double getKS() {
		return kS;
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

	public void setKV(double kV) {
		this.kV = kV;
	}

	public void setKA(double kA) {
		this.kA = kA;
	}
}
