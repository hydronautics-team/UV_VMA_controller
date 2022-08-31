/*
  October 2012

  aq32Plus Rev -

  Copyright (c) 2012 John Ihlein.  All rights reserved.

  Open Source STM32 Based Multicopter Controller Software

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)S.O.H. Madgwick
  6)UAVX

  Designed to run on the AQ32 Flight Control Board

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

///////////////////////////////////////////////////////////////////////////////

#include "stm32f3xx_hal.h"
#include <stdbool.h>
#include <osdWidgets.h>


#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <errno.h>

///////////////////////////////////////////////////////////////////////////////
// Update OSD function
//  - handles timing and calling display functions of only enabled features
///////////////////////////////////////////////////////////////////////////////

uint32_t timerOSD = 0x00000000;
//
//void updateMax7456(uint32_t currentOSDTime, uint8_t updateOSD)
//{
//	if (eepromConfig.osdEnabled)
//	{
//		if ((timerOSD & 0x80008000) || updateOSD)  // 3.125Hz
//		{
//			if (eepromConfig.osdDisplayAlt || eepromConfig.osdDisplayAltHoldState)
//				displayAltitude(sensors.pressureAlt50Hz, 0.0f, verticalModeState, updateOSD);
//		}
//
//		if ((timerOSD & 0x55555555) || updateOSD)  // 25Hz
//        {
//			if (eepromConfig.osdDisplayAH)
//				displayArtificialHorizon(sensors.attitude500Hz[ROLL], sensors.attitude500Hz[PITCH], flightMode, updateOSD);
//
//			else if (eepromConfig.osdDisplayAtt)
//				displayAttitude(sensors.attitude500Hz[ROLL], sensors.attitude500Hz[PITCH], flightMode, updateOSD);
//		}
//
//
//		if ((timerOSD & 0x20202020) || updateOSD)  // 6.25Hz
//		{
//			if (eepromConfig.osdDisplayHdg || eepromConfig.osdDisplayHdgBar)
//				displayHeading(heading.mag, updateOSD);
//		}
//
//		if ((timerOSD & 0x00800080) || updateOSD)  // 3.125Hz
//		{
//			if (eepromConfig.osdDisplayTimer)
//				displayMotorArmedTime(currentOSDTime, updateOSD);
//		}
//
//		if ((timerOSD & 0x08000800) || updateOSD)  // 3.125Hz
//		{
//			if (eepromConfig.osdDisplayVoltage || eepromConfig.osdDisplayCurrent)
//				displayBattery(updateOSD);
//		}
//
//		if ((timerOSD & 0x4A4A4A4A) || updateOSD)  // 18.75Hz (3/8 cycles, 3/8 * 50)
//		{
//			if (eepromConfig.osdDisplayThrot)
//				displayThrottle(updateOSD);
//		}
//
//		if ((timerOSD & 0x00800080) || updateOSD)  // 3.125Hz
//		{
//			if (eepromConfig.osdDisplayRSSI)
//				displayRSSI(updateOSD);
//		}
//
//		timerOSD <<= 1;
//		if (!timerOSD)
//			timerOSD = 0x01;
//	}
//}

///////////////////////////////////////////////////////////////////////////////
// AltitudeHold Display
///////////////////////////////////////////////////////////////////////////////

uint8_t lastHoldState    = 99;

int16_t lastAltitude            = 12345;     // bogus value to force update
int16_t lastAltitudeRef         = 12345;     // bogus value to force update

int16_t lastVerticalVelocity    = 12345;     // bogus value to force update
int16_t lastVerticalVelocityRef = 12345;     // bogus value to force update

void displayAltitude(float pressureAltitude, float altitudeReference, uint8_t verticalModeState, uint8_t update)
{
    int16_t currentAltitude;
    int16_t currentAltitudeRef;

    if (1)
    {
		currentAltitude    = (int16_t)(pressureAltitude  * 10.0f);
		currentAltitudeRef = (int16_t)(altitudeReference * 10.0f);
	}
	else
	{
		currentAltitude    = (int16_t)(pressureAltitude  * 3.281f);
        currentAltitudeRef = (int16_t)(altitudeReference * 3.281f);
	}

    if ( (lastAltitude != currentAltitude)  || update)
    {
        char    buf[7];
        if (1)
        {
		    if (abs(currentAltitude) < 100)
		    {
                snprintf(buf,7,"\011%c%1d.%1dm",currentAltitude < 0 ? '-' : ' ', abs(currentAltitude/10),abs(currentAltitude%10));
            }
            else
            {
                snprintf(buf,7,"\011%4dm",currentAltitude / 10);
            }
        }
        else
        {
		    snprintf(buf,7,"\011%4df",currentAltitude);
	    }

        writeMax7456Chars(buf, 6, 0, 1, 1);

        lastAltitude = currentAltitude;
    }

    // Vertical mode handling:
    // - show altitude ref when active

    if (1)
    {
        bool    isWriteNeeded = false;
        char    buf[7];

		if (1)
		{
		    if (1)
			{
				lastHoldState = 1;
				memset(buf,0,6);
				isWriteNeeded = true;
			}
		}

		if (1)
		{
			if ((lastHoldState != verticalModeState) ||
				(lastAltitudeRef != currentAltitudeRef))
		    {
			    lastHoldState   = verticalModeState;
			    lastAltitudeRef = currentAltitudeRef;

			    if (1)
			    {
			        if (abs(currentAltitudeRef) < 100)
			        {
			            snprintf(buf,7,"\012%c%1d.%1dm", currentAltitudeRef < 0 ? '-' : ' ',abs(currentAltitudeRef / 10), abs(currentAltitudeRef % 10));
			        }
			        else
			        {
			            snprintf(buf,7,"\012%4dm",currentAltitudeRef / 10);
			        }
			    }
			    else
			    {
			        snprintf(buf,7,"\12%4df",currentAltitudeRef);
			    }

			    isWriteNeeded = true;
		    }
	    }

		if (isWriteNeeded)
	        writeMax7456Chars(buf, 6, 0, 1, 1+6);
    }

}

///////////////////////////////////////////////////////////////////////////////
// Artificial Horizon Display
///////////////////////////////////////////////////////////////////////////////

// 012345678901234567890123456789
//
//         - - - RR - - -

#define LINE_ROW_0 0x80            // character address of a character with a horizontal line in row 0. Other rows follow this one
#define AH_MAX_PITCH_ANGLE (3.14/8)  // bounds of scale used for displaying pitch.
                                   // when pitch is >= |this number|, the pitch lines will be at top or bottom of bounding box
#define RETICLE_COL 10             // reticle will be in this col, and col to the right

// columns where the roll line is printed
static const uint8_t ahColumns[6] = {8,10,12,17,19,21};

uint8_t  reticleRow;
uint8_t  ahTopPixel;
uint8_t  ahBottomPixel;
uint8_t  ahCenter = 110;

uint8_t ahOldLine[6]   = {10,10,10,10,10,10};
uint8_t lastAHflightMode = 25;

void displayArtificialHorizon(float roll, float pitch, uint8_t flightMode, uint8_t updateOSD)
{
	char    reticle[2];
	char    rollLine;
    uint8_t row;



//        rollLine = LINE_ROW_0 + (roll % 36);
//        writeMax7456Chars(&rollLine, 1, 0, roll%32, 2);
//        HAL_Delay(100);


    // Reticle on the center of the screen
    // 0 - rate mode (no letter)
    // 1 - Attitude 'S'
    // 2 - GPS position hold 'P'
    // 3 - GPS navigation 'N'

//    if ((lastAHflightMode != flightMode) || updateOSD)
//    {
//        reticle[0] = flightMode * 2 + 1;
//        reticle[1] = reticle[0] + 1;
//
//        //write 2 chars to row (middle), column 14
//        writeMax7456Chars(reticle, 2, 0, 10, 14);
//
//        lastAHflightMode = flightMode;
//    }
}

//////////////////////////////////////////////////////////////////////////////
// Attitude Display
//////////////////////////////////////////////////////////////////////////////

#define AI_MAX_PITCH_ANGLE (3.14/4)  // Bounds of scale used for displaying pitch.
                                   // when pitch is >= |this number|, the pitch lines will be at top or bottom of bounding box
#define PITCH_L_COL 7
#define PITCH_R_COL 22

// columns where the roll line is printed
static const uint8_t ROLL_COLUMNS[4] = {10,12,17,19};

uint8_t aiTopPixel;
uint8_t aiBottomPixel;
uint8_t aiCenter;

uint8_t aiOldline[5] = {0,0,0,0,0};
uint8_t lastATTflightMode = 9;

void displayAttitude(float roll, float pitch, uint8_t flightMode, uint8_t updateOSD)
{
	char     pitchLine;
	char     reticle[2];
	char     rollLine;
	float    gradient;
	uint8_t  aiRows[5] = {0,0,0,0,0};  //Holds the row, in pixels, of AI elements: pitch then roll from left to right.
    uint8_t  i;
    uint16_t distFar;
    uint16_t distNear;

    //Calculate row of new pitch lines
    aiRows[0] = (int)aiCenter +
    		              (int)((pitch / AI_MAX_PITCH_ANGLE) * (aiCenter - aiTopPixel));

    pitchLine = LINE_ROW_0 + (aiRows[0] % 18);

    if (aiOldline[0] != aiRows[0] / 18)
    {
        //Remove old pitch lines if not overwritten by new ones
        writeMax7456Chars(NULL, 1, 0, aiOldline[0], PITCH_L_COL);
        writeMax7456Chars(NULL, 1, 0, aiOldline[0], PITCH_R_COL);
        aiOldline[0] = aiRows[0] / 18;
    }

    //Write new pitch lines
    writeMax7456Chars(&pitchLine, 1, 0, aiOldline[0], PITCH_L_COL);
    writeMax7456Chars(&pitchLine, 1, 0, aiOldline[0], PITCH_R_COL);

    //Calculate row (in pixels) of new roll lines
    distFar  = (ROLL_COLUMNS[3] - (RETICLE_COL + 1))*12 + 6; //horizontal pixels between centre of reticle and centre of far angle line
    distNear = (ROLL_COLUMNS[2] - (RETICLE_COL + 1))*12 + 6;
    gradient = 1.4f * roll; // was "tan(roll)", yes rude but damn fast !!

    aiRows[3] = aiCenter - (int)(((float)distNear) * gradient);
    aiRows[4] = aiCenter - (int)(((float)distFar)  * gradient);
    aiRows[1] = 2 * aiCenter - aiRows[4];
    aiRows[2] = 2 * aiCenter - aiRows[3];

    //writing new roll lines to screen
    for (i = 1; i < 5; i++ )
    {
        // clear previous roll lines if not going to overwrite
        if (aiOldline[i] != aiRows[i] / 18)
        {
            writeMax7456Chars(NULL, 1, 0, aiOldline[i], ROLL_COLUMNS[i-1]);
            aiOldline[i] = aiRows[i]/18;
        }

        //converting rows (in pixels) to character addresses used for the 'lines'
        rollLine = LINE_ROW_0 + (aiRows[i] % 18);
        writeMax7456Chars(&rollLine, 1, 0, aiOldline[i], ROLL_COLUMNS[i-1]);
    }

    // Reticle on the center of the screen
    // 0 - rate mode (no letter)
    // 1 - Attitude 'S'
    // 2 - GPS position hold 'P'
    // 3 - GPS navigation 'N'

    if ((lastATTflightMode != flightMode) ||  updateOSD)
    {
        reticle[0] = flightMode * 2 + 1;
        reticle[1] = reticle[0] + 1;
        writeMax7456Chars(reticle, 2, 0, reticleRow, RETICLE_COL); //write 2 chars to row (middle), column 14
        lastATTflightMode = flightMode;
    }
}

///////////////////////////////////////////////////////////////////////////////
// Heading Display
///////////////////////////////////////////////////////////////////////////////

int16_t lastOSDheading = 361; // bogus value to force update
// N = 0x4e; E = 0x45; S = 0x53; W = 0x57; - = 0x2d; | = 0x7c;
static char headingBarShown[12];
const char headingBar[36] = {0x4e,0x2d,0x2d,0x7c,0x2d,0x2d,0x7c,0x2d,0x2d,
                           0x45,0x2d,0x2d,0x7c,0x2d,0x2d,0x7c,0x2d,0x2d,
                           0x53,0x2d,0x2d,0x7c,0x2d,0x2d,0x7c,0x2d,0x2d,
                           0x57,0x2d,0x2d,0x7c,0x2d,0x2d,0x7c,0x2d,0x2d};

void displayHeading(float currentHeading, uint8_t update)
{
    int16_t currentHeadingDeg;

    currentHeadingDeg = (int16_t)(currentHeading) % 360;
//    if (currentHeadingDeg < 0)
//		currentHeadingDeg += 360;

    if ((currentHeadingDeg != lastOSDheading) || update)
    {

    	if (1)
    	{
    		char buf[6];
			snprintf(buf ,6, "\026%3d\027", currentHeadingDeg); // \026 is compass \027 is degree symbol
			writeMax7456Chars(buf, 5, 0, 1,24);
    	}

        if (1) {
			int16_t lastPos;
			int16_t currentPos;

			currentPos = currentHeadingDeg / 10;
			lastPos = lastOSDheading / 10;

			if ((currentPos != lastPos) || update)
			{
			    uint16_t x = 0;
				currentPos -= 5;

				if (currentPos < 0)
					currentPos += 36;

				for (x = 0; x <= 10; ++x)
				{
					headingBarShown[x] = headingBar[currentPos];

					if (++currentPos > 35)
						currentPos = 0;
				}

				headingBarShown[11] = '\0';
				writeMax7456Chars(headingBarShown, 11, 0, 1, 8);
			}
		}

        lastOSDheading = currentHeadingDeg;
    }
}

void displayDepth(float depth)
{
	char buf[20]={0};
	depth*=0.53;
	snprintf(buf, sizeof(buf), "\12%3dcm", (int16_t)depth);//*0.53); //
	writeMax7456Chars(buf, 10, 0, 14,1);
}

/////////////////////////////////////////////////////// ////////////////////////
// Battery Display
///////////////////////////////////////////////////////////////////////////////

void displayBattery(uint8_t osdVoltage)
{
    char buf[20];
    osdVoltage = osdVoltage;
    if (osdVoltage!=osdVoltageLast)
    {
	    snprintf(buf, sizeof(buf), "\20%2d.%1dV", (uint8_t)(osdVoltage / 10), (uint8_t)(osdVoltage % 10));
	    writeMax7456Chars(buf, 7, 0, 14, 23);
	    osdVoltageLast = osdVoltage;
    }
    if(osdVoltage<200)
    {
    	char buf1[10] = "LOW BATTERY";
    	writeMax7456Chars(buf1, 10, 1, 6,5);
    }
    else
    {
    	char buf1[53] = "                                                      ";
    	writeMax7456Chars(buf1, 53, 0, 6,5);
    }
}



///////////////////////////////////////////////////////////////////////////////
// Motors Armed Timer Display
///////////////////////////////////////////////////////////////////////////////
uint32_t previousTime = 0;
uint16_t previousArmedTimeSeconds = 500;
uint32_t armedTime = 0;

void displayMotorArmedTime()
{
	uint32_t currentOSDTime = HAL_GetTick()/1000;

	if (previousTime != currentOSDTime)
	{
		previousArmedTimeSeconds = currentOSDTime;
		char buf[7];
		snprintf(buf, 7, "\025%02u:%02u", currentOSDTime / 60, currentOSDTime % 60);
		writeMax7456Chars(buf, 6, 0, 13, 23);
	}

}

///////////////////////////////////////////////////////////////////////////////
void displaycompas(int currentHeadingY, int currentHeadingR, int currentHeadingP,uint8_t speedVMA)
{
	int currentHeadingDegY;

	currentHeadingDegY = (int)(currentHeadingY) % 360;
//	if (currentHeadingDegY < 0)
//		currentHeadingDegY += 360;

	char bufY[6];
	snprintf(bufY ,6, "\026%3d\027", currentHeadingDegY); // \026 is compass \027 is degree symbol
	writeMax7456Chars(bufY, 5, 0, 3,24);

	int currentHeadingDegR;

	currentHeadingDegR = (int)(currentHeadingR) % 360;
//	if (currentHeadingDegR < 0)
//		currentHeadingDegR += 360;

	char bufR[6];
	snprintf(bufR ,6, "\026%3d\027", currentHeadingDegR); // \026 is compass \027 is degree symbol
	writeMax7456Chars(bufR, 5, 0, 5,24);

	int currentHeadingDegP;

	currentHeadingDegP = (int)(currentHeadingP) % 360;
	if (currentHeadingDegP < 0)
		currentHeadingDegP += 360;

	char bufP[6];
	snprintf(bufP ,6, "T%3d\027", -currentHeadingDegP+118); // \026 is compass \027 is degree symbol
	writeMax7456Chars(bufP, 5, 0, 7,24);

	char bufVMA[7];
	snprintf(bufVMA ,7, "S%2d", speedVMA); // \026 is compass \027 is degree symbol
	writeMax7456Chars(bufVMA, 5, 0, 1,1);

//	char buf1[6] = "HY";
//	writeMax7456Chars(buf1, 6, 0, 1,1);

}

void displayNoConnection(){
	char buf[15];
	snprintf(buf ,10, "Not connected");
	writeMax7456Chars(buf, 10, 0, 14,1);
}

