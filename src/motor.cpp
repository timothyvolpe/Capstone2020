#include "def.h"
#include "motor.h"

CMotorController::CMotorController( CUARTChannel *pUART ) {
	m_pMotorUARTReference = pUART;
}
CMotorController::~CMotorController() {
	m_pMotorUARTReference = 0;
}

int CMotorController::init() {
	return ERR_OK;
}
void CMotorController::shutdown() {
}