#pragma once


#define od_controlWord            0x6040,0x00
#define od_statusWord             0x6041,0x00

#define od_operationMode          0x6060,0x00
#define od_operationModeDisplay   0x6061,0x00

#define od_targetPosition        0x607A,0x00
#define od_positionActualVal     0x6064,0x00
#define od_positionDemand        0x6062,0x00
#define od_positonFollowingError 0x60F4,0X00
#define od_positonCounts		 0x6063,0x00
#define od_maxFollowingError     0x6065,0x00

#define od_velocityActvalue       0x606c,0x00
#define od_velocityAddress        0x6069,0x00
#define od_targetVelocity         0x60FF,0x00
#define	od_velocityOffset		  0x60b1, 0x00

#define od_profileVelocity                    0x6081,0x00
#define od_maxProfileVelocity                 0x6082,0x00
#define od_profileAcceleration                0x6083,0x00
#define od_profileDeceleration                0x6084,0x00
#define od_quickStopDeceleration              0x6085,0x00
#define od_motionProfileType                  0x6086,0x00
#define od_linearRampTrapezoidal              0x00,0x00
#define od_velocityEncoderResolutionNum       0x6094,0x01
#define od_velocityEncoderResolutionDen       0x6094,0x02

#define od_digitalInputs			  0x60fd, 0x00
#define od_digitalOutputs 			  0x60fe,0x01

#define od_dcCircuitLinkVoltage	  	  0x6079,0x00
#define od_targetTorque               0x6071,0x00 
#define od_torqueMax                  0x6072,0x00
#define od_torqueActualValue		  0x6077,0x00
#define od_torqueOffset               0x60b2,0x00

#define od_maxCurrent				   0x6073, 0x00
#define od_currentActualValue		   0x6078, 0x00
#define od_errorCode				   0x603f, 0x00   // 2 bit

#define od_extraStatusRegister		   0x2085, 0x00

// State machine parameters; 

#define od_commReset              0x81
#define od_fullReset              0x82
#define od_start                  0x01
#define od_goreadyToSwitchOn      0x06
#define od_goSwitchOn             0x07
#define od_goEnable               0X0F
#define od_goSwitchonDisable      0x00
#define od_run                    0x1F
#define od_expedite               0x3F       //like run, but dont finish actual position profile
#define od_quickStop              0x02 


/* From CiA402, page 27
	Table 30 - State coding
	Statusword      |      PDS FSA state
xxxx xxxx x0xx 0000 | Not ready to switch on
xxxx xxxx x1xx 0000 | Switch on disabled
xxxx xxxx x01x 0001 | Ready to switch on
xxxx xxxx x01x 0011 | Switched on
xxxx xxxx x01x 0111 | Operation enabled
xxxx xxxx x00x 0111 | Quick stop active
xxxx xxxx x0xx 1111 | Fault reaction active
xxxx xxxx x0xx 1000 | Fault
*/
#define od_FSAFromStatusWord(SW) (SW & 0x006f)
#define od_NotReadyToSwitchOn   0b00000000
#define od_NotReadyToSwitchOn2  0b00100000
#define od_SwitchOnDisabled     0b01000000
#define od_SwitchOnDisabled2    0b01100000
#define od_ReadyToSwitchOn      0b00100001
#define od_SwitchedOn           0b00100011
#define od_OperationEnabled     0b00100111
#define od_QuickStopActive      0b00000111
#define od_FaultReactionActive  0b00001111
#define od_FaultReactionActive2 0b00101111
#define od_Fault                0b00001000
#define od_Fault2               0b00101000

// SatusWord bits :
#define od_SW_ReadyToSwitchOn     0x0001
#define od_SW_SwitchedOn          0x0002
#define od_SW_OperationEnabled    0x0004
#define od_SW_Fault               0x0008
#define od_SW_VoltageEnabled      0x0010
#define od_SW_QuickStop           0x0020
#define od_SW_SwitchOnDisabled    0x0040
#define od_SW_Warning             0x0080
#define od_SW_Remote              0x0200
#define od_SW_TargetReached       0x0400
#define od_SW_InternalLimitActive 0x0800

// ControlWord bits :
#define od_SwitchOn        0x0001
#define od_EnableVoltage   0x0002
#define od_QuickStop       0x0004
#define od_EnableOperation 0x0008
#define od_FaultReset      0x0080
#define od_Halt            0x0100
/* CiA402 statemachine definition end */

#define od_checkError              0x1002,0x00
#define od_quickStopMode           0x605A,0x00
#define od_stopOptionCode          0x605D,0x00