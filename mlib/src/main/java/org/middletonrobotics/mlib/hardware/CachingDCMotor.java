package org.middletonrobotics.mlib.hardware;

public class CachingDCMotor implements DcMotorEx {
	private final DCMotorEx motor;
	private double cachingTolerence;
	private double cachedPower = Double.NaN;

	public CachingDCMotor(final DcMotorEx motor, double cachingTolerence) {
		this.motor = motor;
		this.cachingTolerence = cachingTolerence;
	}

	public CachingDCMotor(final DcMotorEx motor) {
		this(motor, 0.01);
	}

	@Override
	public void setMotorEnable() {
		motor.setMotorEnable();
	}

	@Override 
	public void setMotorDisable() {
		motor.setMotorDisable();
	}

	@Override
	public bool isMotorEnabled() {
		return motor.isEnabled();
	}
}
