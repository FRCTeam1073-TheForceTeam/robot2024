// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

/** Add your docs here. */
public class MotorFault {
	private TalonFX motor;
	private int id;

	/** Creates a MotorFault object. Takes the motor and the can id (to allow the string result to include the id of the motor in question). */
	public MotorFault(TalonFX motor, int motorCanID){
		this.motor = motor;
		id = motorCanID;
	}

	/** Returns true if the motor has any faults. */
	public boolean hasFaults(){
		return motor.getFault_BootDuringEnable().getValue() ||
		motor.getFault_BridgeBrownout().getValue() ||
		motor.getFault_DeviceTemp().getValue() ||
		motor.getFault_ForwardHardLimit().getValue() ||
		motor.getFault_ForwardSoftLimit().getValue() ||
		motor.getFault_FusedSensorOutOfSync().getValue() ||
		motor.getFault_Hardware().getValue() ||
		motor.getFault_MissingDifferentialFX().getValue() ||
		motor.getFault_OverSupplyV().getValue() ||
		motor.getFault_ProcTemp().getValue() ||
		motor.getFault_RemoteSensorDataInvalid().getValue() ||
		motor.getFault_RemoteSensorPosOverflow().getValue() ||
		motor.getFault_RemoteSensorReset().getValue() ||
		motor.getFault_ReverseHardLimit().getValue() ||
		motor.getFault_ReverseSoftLimit().getValue() ||
		motor.getFault_StatorCurrLimit().getValue() ||
		motor.getFault_SupplyCurrLimit().getValue() ||
		motor.getFault_Undervoltage().getValue() ||
		motor.getFault_UnlicensedFeatureInUse().getValue() ||
		motor.getFault_UnstableSupplyV().getValue() ||
		motor.getFault_UsingFusedCANcoderWhileUnlicensed().getValue();
	}

	/** Returns a string with the number and type of faults. */
	public String getFaults(){
		String result = "";
		int counter = 0;

		if(motor.getFault_BootDuringEnable().getValue()){
			counter++;
			result += "Boot during enable! ";}
		if(motor.getFault_BridgeBrownout().getValue()){
			counter++;
			result += "Bridge brownout! ";}
		if(motor.getFault_DeviceTemp().getValue()){
			counter++;
			result += "Device temp! ";}
		if(motor.getFault_ForwardHardLimit().getValue()){
			counter++;
			result += "Forward hard limit! ";}
		if(motor.getFault_ForwardSoftLimit().getValue()){
			counter++;
			result += "Forward soft limit! ";}
		if(motor.getFault_FusedSensorOutOfSync().getValue()){
			counter++;
			result += "Fused sensor out of sync! ";}
		if(motor.getFault_Hardware().getValue()){
			counter++;
			result += "Hardware! ";}
		if(motor.getFault_MissingDifferentialFX().getValue()){
			counter++;
			result += "Missing differential FX! ";}
		if(motor.getFault_OverSupplyV().getValue()){
			counter++;
			result += "Over supply v! ";}
		if(motor.getFault_ProcTemp().getValue()){
			counter++;
			result += "Proc temp! ";}
		if(motor.getFault_RemoteSensorDataInvalid().getValue()){
			counter++;
			result += "Remote sensor data invalid! ";}
		if(motor.getFault_RemoteSensorPosOverflow().getValue()){
			counter++;
			result += "Remote sensor pos overflow! ";}
		if(motor.getFault_RemoteSensorReset().getValue()){
			counter++;
			result += "Remote sensor rest! ";}
		if(motor.getFault_ReverseHardLimit().getValue()){
			counter++;
			result += "Reverse hard limit! ";}
		if(motor.getFault_ReverseSoftLimit().getValue()){
			counter++;
			result += "Reverse soft limit! ";}
		if(motor.getFault_StatorCurrLimit().getValue()){
			counter++;
			result += "Stator curr limit! ";}
		if(motor.getFault_SupplyCurrLimit().getValue()){
			counter++;
			result += "Supply curr limit! ";}
		if(motor.getFault_Undervoltage().getValue()){
			counter++;
			result += "Undervoltage! ";}
		if(motor.getFault_UnlicensedFeatureInUse().getValue()){
			counter++;
			result += "Unliscensed feature in use! ";}
		if(motor.getFault_UnstableSupplyV().getValue()){
			counter++;
			result += "Unstable supply v! ";}
		if(motor.getFault_UsingFusedCANcoderWhileUnlicensed().getValue()){
			counter++;
			result += "Using fused CANcoder while unlicensed! ";}

		return "Motor " + id + "has " + counter + " errors. " + result;
	}
}
