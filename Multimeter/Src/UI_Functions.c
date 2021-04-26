double scaleUpVdc (double oldV){
	// MATHS
	/*Vdc eq (x-1.5)/0.15*/
	
	//Take avg of 3 readings???//

	double newV = (oldV-1.5)/0.15;

	return newV;
}

double scaleUpIdc (double oldI){
	// MATHS
	/*Idc eq ((x-1.5)/0.15)/100 CURRENT*/

	double newI = ((oldI-1.5/0.15)/100);

	return newI;
}

double scaleUpRdc (double oldR){
	// MATHS

	double newR = 12345;//Stops prog breaking when sw4 is pressed

	return newR;
}

// Clear LCD display
void dispClear(void){
	PB_LCD_GoToXY(0,0);
	PB_LCD_WriteString("                ",16);
	PB_LCD_GoToXY(0,1);
	PB_LCD_WriteString("                ",16);
}

// Display voltage on LCD
void dispVotlage(double voltage){
	PB_LCD_GoToXY(0,0);
	PB_LCD_WriteString("Voltage", 7);
	PB_LCD_GoToXY(0,1);
	PB_LCD_WriteString(voltage, 5);
	PB_LCD_GoToXY(5,1);
	PB_LCD_WriteString("V", 1);
}

// Display current on LCD
void dispCurrent(double current){
	PB_LCD_GoToXY(0,0);
	PB_LCD_WriteString("Current", 7);
	PB_LCD_GoToXY(0,1);
	PB_LCD_WriteString(current, 5);
	PB_LCD_GoToXY(5,1);
	PB_LCD_WriteString("A", 1);
}

// Display resistance on LCD
void dispResistance(double resistance){
	PB_LCD_GoToXY(0,0);
	PB_LCD_WriteString("Resistance", 10);
	PB_LCD_GoToXY(0,1);
	PB_LCD_WriteString(resistance, 5);
	PB_LCD_GoToXY(5,1);
	PB_LCD_WriteString("Ohms", 4);
}