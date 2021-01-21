/*
* 
*
*/
#include <stdlib.h>
#include "xboxController.h"

int main(int argc, char **argv) {

	XboxController Controller;

	if (Controller.initXboxController(XBOX_DEVICE) >= 0) {
		xboxCtrl* xbox = Controller.getXboxDataStruct();
		Controller.readXboxControllerInformation(xbox);

		printf("xbox controller detected\n\naxis:\t\t%d\nbuttons:\t%d\nidentifier:\t%s\n",
				xbox->numOfAxis, xbox->numOfButtons, xbox->identifier);

		while (1) {
			Controller.readXboxData(xbox);
			Controller.printXboxCtrlValues(xbox);
		}

		Controller.deinitXboxController(xbox);
	}
	return 0;
}
