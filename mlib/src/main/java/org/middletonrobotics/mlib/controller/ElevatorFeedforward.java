package org.middletonrobotics.mlib.controller;

public class ElevatorFeedforward {
	private double kS;
	private double kG;
	private double kV;
	private double kA;
	private double period;

	public ElevatorFeedforward(double kS, double kG, double kV, double kA, double period) {
		if (kS < 0.0 || kG < 0.0 || kV < 0.0 || kA < 0.0) {
			throw new IllegalArgumentException("Feedforward gains must be an unsigned number");
		}

		if (period <= 0.0) {
			throw new IllegalArgumentException("Time period must be a postive number");
		}

		this.kS = kS;
		this.kG = kG;
		this.kV = kV;
		this.kA = kA;
		this.period = period;
	}

	public ElevatorFeedforward(double kS, double kG, double kV, double kA) {
		this(kS, kG, kV, kA, 0.01);
	}

	public double calculate(double velocity) {
		return calculate(velocity, velocity);
	}

	public double calculate(double currentVelocity, double nextVelocity) {
    	if (ka < 1e-9) {
      		return ks * Math.signum(nextVelocity) + kg + kv * nextVelocity;
    	} else {
      		double a = -kV / kA;
      		double b = 1.0 / kA;
      		double ad = Math.exp(a * m_dt);
      		double bd = a > -1e-9 ? b * m_dt : 1.0 / a * (ad - 1.0) * b;
      		return kG + kS * Math.signum(currentVelocity) + 1.0 / bd * (nextVelocity - ad * currentVelocity);
    	}
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
