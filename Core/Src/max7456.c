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
#include "max7456font.h"
#include "max7456.h"
#include "string.h"
///////////////////////////////////////////////////////////////////////////////
// MAX7456 Variables
///////////////////////////////////////////////////////////////////////////////
extern SPI_HandleTypeDef hspi3;

uint8_t charbuf[54]; // for NVM_read

uint8_t osdDisabled        = 0;

uint16_t maxScreenSize     = 0;
uint16_t maxScreenRows     = 0;
uint8_t  enableDisplay     = 0;
uint8_t  enableDisplayVert = 0;
uint8_t  max7456Reset      = 0;
uint8_t  disableDisplay    = 0;

////////////////////////////////////////////////////////////////////////////////
// SPI Read MAX7456 Register
///////////////////////////////////////////////////////////////////////////////

uint8_t spiReadMax7456Register(uint8_t r)
{
	uint8_t datarec = 0;
	uint8_t state;
	uint8_t buffer_trans[2] = {r,0x00};
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)buffer_trans, datarec, 2, 10);
    return datarec;
}

///////////////////////////////////////////////////////////////////////////////
// SPI Write MAX7456 Register
///////////////////////////////////////////////////////////////////////////////

void spiWriteMax7456Register(uint8_t r, uint8_t d)
{
	uint8_t buffer_trans[2] = {r,d};
	HAL_SPI_Transmit(&hspi3, buffer_trans, 2, 10);
}

///////////////////////////////////////////////////////////////////////////////
// Hide OSD Display
///////////////////////////////////////////////////////////////////////////////

void hideOSD()
{
  if (!osdDisabled)
  {
    ENABLE_MAX7456;

    spiWriteMax7456Register(VM0_REG, disableDisplay);

    DISABLE_MAX7456;

    osdDisabled = 0;
  }
}

///////////////////////////////////////////////////////////////////////////////
// Unhide OSD Display
///////////////////////////////////////////////////////////////////////////////

void unhideOSD()
{
  if (osdDisabled)
  {
    ENABLE_MAX7456;

    spiWriteMax7456Register(VM0_REG, enableDisplay);

    DISABLE_MAX7456;

    osdDisabled = 0;
  }
}

///////////////////////////////////////////////////////////////////////////////
// Hide video from max and camera (VOUT is high impedance)
///////////////////////////////////////////////////////////////////////////////
void videoHideMax7456(uint8_t status)
{
	if(status == 0)
	{
	    ENABLE_MAX7456;

	    spiWriteMax7456Register(VM0_REG, VOUT_HIGH_IMP);

	    DISABLE_MAX7456;
	}
	else
	{
	    ENABLE_MAX7456;

	    spiWriteMax7456Register(VM0_REG, enableDisplay);

	    DISABLE_MAX7456;
	}
}
///////////////////////////////////////////////////////////////////////////////
// Write Characters
///////////////////////////////////////////////////////////////////////////////

// Writes 'len' character address bytes to the display memory corresponding to row y, column x
// - uses autoincrement mode when writing more than one character
// - will wrap around to next row if 'len' is greater than the remaining cols in row y
// - buf=NULL or len>strlen(buf) can be used to write zeroes (clear)
// - flags: 0x01 blink, 0x02 invert (can be combined)

void writeMax7456Chars( const char* buf, uint8_t len, uint8_t flags, uint8_t y, uint8_t x)
{
    uint8_t  i;
    uint16_t offset = y * 30 + x;

    if (flags)
        unhideOSD(); // make sure OSD is visible in case of alarms etc.

    ENABLE_MAX7456;

    // 16bit transfer, transparent BG, autoincrement mode (if len!=1)
    spiWriteMax7456Register(DMM_REG, ((flags & 1) ? 0x10 : 0x00) | ((flags & 2) ? 0x08 : 0x00) | ((len != 1) ? 0x01 : 0x00));

    // send starting display memory address (position of text)
    spiWriteMax7456Register(DMAH_REG, offset >> 8 );
    spiWriteMax7456Register(DMAL_REG, offset & 0xFF );

    // write out data
    for (i = 0; i < len; i++)
        spiWriteMax7456Register(DMDI_REG, (!buf || strlen(buf) < i) ? 0 : buf[i]);


    // Send escape 11111111 to exit autoincrement mode
    if (len != 1)
        spiWriteMax7456Register(DMDI_REG, END_STRING);

    // finished writing

    DISABLE_MAX7456;
}

///////////////////////////////////////////////////////////////////////////////
// Detect Video Standard
///////////////////////////////////////////////////////////////////////////////

void detectVideoStandard()
{
    // First set the default
    uint8_t pal = 0;
    uint8_t stat = 0xff;

    pal = AUTO;

    // if autodetect enabled modify the default if signal is present on either standard
    // otherwise default is preserved

	if (pal == AUTO) 
	{
		ENABLE_MAX7456;

		stat = spiReadMax7456Register(STAT_REG);

		DISABLE_MAX7456;

		if (stat & 0x01)
			pal = PAL;

		if (stat & 0x02)
			pal = NTSC;
			
		if ((pal != PAL) && (pal != NTSC))
			pal = PAL;
	}

    if (pal)
    {
        maxScreenSize     = 480;
        maxScreenRows     = 16;
        enableDisplay     = 0x48;
        enableDisplayVert = 0x4C;
        max7456Reset      = 0x42;
        disableDisplay    = 0x40;
    }
    else
    {
        maxScreenSize     = 390;
        maxScreenRows     = 13;
        enableDisplay     = 0x08;
        enableDisplayVert = 0x0C;
        max7456Reset      = 0x02;
        disableDisplay    = 0x00;
    }

    // Artificial Horizon Display Parameters

//    reticleRow    =  maxScreenRows / 2;
//    ahTopPixel    = (reticleRow - AH_DISPLAY_RECT_HEIGHT / 2) * 18;
//    ahBottomPixel = (reticleRow + AH_DISPLAY_RECT_HEIGHT / 2) * 18;
//    ahCenter      =  reticleRow * 18 + 10;
//
//    // Attitude Display Parameters
//
//    aiTopPixel    = (reticleRow - AI_DISPLAY_RECT_HEIGHT / 2) * 18;
//    aiBottomPixel = (reticleRow + AI_DISPLAY_RECT_HEIGHT / 2) * 18;
//    aiCenter      =  reticleRow * 18 + 10;
}

///////////////////////////////////////////////////////////////////////////////
// Initialize MAX7456
///////////////////////////////////////////////////////////////////////////////

void initMax7456()
{
    uint8_t i;

    //if (eepromConfig.osdEnabled == false) return;

    detectVideoStandard();

    //Soft reset the MAX7456 - clear display memory
    ENABLE_MAX7456;

    spiWriteMax7456Register(VM0_REG, max7456Reset);

    DISABLE_MAX7456;

    HAL_Delay(1);

    //Set white level to 90% for all rows
    ENABLE_MAX7456;

    for(i = 0; i < maxScreenRows; i++ )
        spiWriteMax7456Register(RB0_REG + i, WHITE_LEVEL_90 );

    //ensure device is enabled
    spiWriteMax7456Register(VM0_REG, enableDisplay);

    //finished writing
    DISABLE_MAX7456;
    unhideOSD();
}

//////////////////////////////////////////////////////////////////////////////
// Reset MAX7456
///////////////////////////////////////////////////////////////////////////////

void resetMax7456()
{
    uint8_t x;

    // force soft reset on Max7456
    ENABLE_MAX7456;

    spiWriteMax7456Register(VM0_REG, max7456Reset);

    DISABLE_MAX7456;

    HAL_Delay(500);

    // set all rows to same character white level, 90%
    ENABLE_MAX7456;

    for (x = 0; x < maxScreenRows; x++)
        spiWriteMax7456Register(RB0_REG + x, WHITE_LEVEL_90);

    // make sure the Max7456 is enabled
    spiWriteMax7456Register(VM0_REG, enableDisplay);
    
    HAL_Delay(100);

    DISABLE_MAX7456;
}

///////////////////////////////////////////////////////////////////////////////
// Show MAX7456 Font
///////////////////////////////////////////////////////////////////////////////

void showMax7456Font(void) //show all chars on 24 wide grid
{
    uint8_t i, x;

    // clear the screen
    ENABLE_MAX7456;

    spiWriteMax7456Register(DMM_REG, CLEAR_DISPLAY);

    DISABLE_MAX7456;

    HAL_Delay(1); // clearing display takes 20uS so wait some...

    ENABLE_MAX7456;

    // disable display
    //spiWriteRegister(VM0_REG, DISABLE_DISPLAY);

    spiWriteMax7456Register(DMM_REG,  0x01);  // 16 bit trans w/o background, autoincrement
    spiWriteMax7456Register(DMAH_REG,    0);  // set start address high
    spiWriteMax7456Register(DMAL_REG,   33);  // set start address low (line 1 col 3 (0 based)

    // show all characters on screen (actually 0-254)
    for (x = 0; x < 255; x++)
    {
        spiWriteMax7456Register(DMDI_REG, x);

        if ((x%24)==23)
        {
            for (i = 0; i < 6; i++)
                spiWriteMax7456Register(DMDI_REG, 0);
		}
    }

    spiWriteMax7456Register(DMDI_REG, END_STRING);

    //spiWriteMax7456Register(VM0_REG, ENABLE_DISPLAY_VERT);

    DISABLE_MAX7456;
}

///////////////////////////////////////////////////////////////////////////////
// Wait for NVM
///////////////////////////////////////////////////////////////////////////////

void waitNVM()
{
    while (spiReadMax7456Register(STAT_REG) & STATUS_REG_NVR_BUSY) ;
}

///////////////////////////////////////////////////////////////////////////////
// Write NVM Character
///////////////////////////////////////////////////////////////////////////////

void writeNVMcharacter(uint8_t ch, const uint16_t index)
{
    uint8_t x;

    // disable display
    ENABLE_MAX7456;

    spiWriteMax7456Register(VM0_REG, disableDisplay);

    spiWriteMax7456Register(CMAH_REG, ch);  // set start address high

    for(x = 0; x < NVM_RAM_SIZE; x++) // write out 54 (out of 64) bytes of character to shadow ram
    {
        spiWriteMax7456Register(CMAL_REG, x); // set start address low

        spiWriteMax7456Register(CMDI_REG, fontdata[index+x]);
    }

    // transfer a 54 bytes from shadow ram to NVM
    spiWriteMax7456Register(CMM_REG, WRITE_NVR);

    waitNVM(); // NVM should be busy around 12ms

    spiWriteMax7456Register(VM0_REG, enableDisplayVert);

    DISABLE_MAX7456;
}

///////////////////////////////////////////////////////////////////////////////
// Download MAX7456 Font Data
///////////////////////////////////////////////////////////////////////////////

void downloadMax7456Font(void)
{
    uint16_t ch;

    if (sizeof(fontdata)!=16384)
    {
        //cliPortPrint("\nERROR: fontdata with invalid size, aborting!!!\n\n");
        return;
    }

    //cliPortPrint("\nDownloading font to MAX7456 NVM, this may take a while...\n\n");

    for (ch = 0; ch < 256; ch++)
    {
        //cliPortPrintF("%d3", ch);
        writeNVMcharacter(ch, 64*ch);
        HAL_Delay(30);
        //cliPortPrint(" Done\n");
    }

    // force soft reset on Max7456
    resetMax7456();

    showMax7456Font();

    //cliPortPrint("\nDone with MAX7456 font download\n\n");
}

///////////////////////////////////////////////////////////////////////////////
