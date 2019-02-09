#!/usr/bin/python
# 
#   Toy code for simple control of the AD9833 DDS chip
# 
#   Cut down heavily from https://github.com/MajicDesigns/MD_AD9833
#     - The SPI code was lifted almost verbatim then manually
#       converted to python
# 
#   Copyright (C) 2018 M J Oldfield
#   
#   This library is free software; you can redistribute it and/or
#   modify it under the terms of the GNU Lesser General Public
#   License as published by the Free Software Foundation; either
#   version 2.1 of the License, or (at your option) any later version.
#   
#   This library is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#   Lesser General Public License for more details.
#   
#   You should have received a copy of the GNU Lesser General Public
#   License along with this library; if not, write to the Free Software
#   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
# 

'''
/***********************************************************************
						Control Register
------------------------------------------------------------------------
D15,D14(MSB)	10 = FREQ1 write, 01 = FREQ0 write,
 		11 = PHASE write, 00 = control write

D13	If D15,D14 = 00, 0 = individual LSB and MSB FREQ write,
	1 = both LSB and MSB FREQ writes consecutively
	If D15,D14 = 11, 0 = PHASE0 write, 1 = PHASE1 write
D12	0 = writing LSB independently
 	1 = writing MSB independently
D11	0 = output FREQ0,
	1 = output FREQ1
D10	0 = output PHASE0
	1 = output PHASE1
D9	Reserved. Must be 0.
D8	0 = RESET disabled
	1 = RESET enabled
D7	0 = internal clock is enabled
	1 = internal clock is disabled
D6	0 = onboard DAC is active for sine and triangle wave output,
 	1 = put DAC to sleep for square wave output
D5	0 = output depends on D1
	1 = output is a square wave
D4	Reserved. Must be 0.
D3	0 = square wave of half frequency output
	1 = square wave output
D2	Reserved. Must be 0.
D1	If D5 = 1, D1 = 0.
	Otherwise 0 = sine output, 1 = triangle output
D0	Reserved. Must be 0.
***********************************************************************/
'''

import gpiozero
import time, sys, math
import numpy as np

pow2_28 = 268435456 # 2^28 used in frequency word calculation
BITS_PER_DEG = 11.3777777777778	#  4096 / 360
RESET_CMD = 0x0100 #  Reset enabled (also CMD RESET)
'''
/* Sleep mode
 * D7	1 = internal clock is disabled
 * D6	1 = put DAC to sleep
 */
'''
SLEEP_MODE = 0x00C0      #  0000 0000 1100 0000 Both DAC and Internal Clock
DISABLE_DAC = 0x0040     #  0000 0000 0100 0000
DISABLE_INT_CLK	= 0x0080 #  0000 0000 1000 0000
PHASE_WRITE_CMD	= 0xC000 #  1100 0000 0000 0000 Setup for Phase write
PHASE0_WRITE_REG =0x9000 #  0000 0000 0000 0000 phase register 0
PHASE1_WRITE_REG =0x2000 #  0010 0000 0000 0000 phase register 1
FREQ0_WRITE_REG = 0x4000 #  0100 0000 0000 0000
FREQ1_WRITE_REG	= 0x8000 #  1000 0000 0000 0000
PHASE1_OUTPUT_REG=0x0400 #  0000 0100 0000 0000 Output is based off REG0/REG1
FREQ1_OUTPUT_REG =0x0800 #  0000 1000 0000 0000 ditto

phaseInDeg=0
REG0=0
REG1=1
SAME_AS_REG0=2

# SetWaveform
SINE_WAVE = 0x2000
TRIANGLE_WAVE = 0x2002
SQUARE_WAVE = 0x2028
HALF_SQUARE_WAVE = 0x2020

referenceFrequency = 25000000
refFrequency = referenceFrequency
	
MOSI=10
CLK=11
dataPin  = gpiozero.OutputDevice(pin = MOSI)
clkPin   = gpiozero.OutputDevice(pin = CLK)

class AD9833:
    def __init__(self, data, clk, fsync):
        # Setup some defaults
        self.DacDisabled = False
        self.IntClkDisabled = False
        self.outputEnabled = False
        self.waveForm = SQUARE_WAVE
        self.waveForm0 = self.waveForm1 = self.waveForm 
        self.frequency0 = frequency1 = 40000	# 40 KHz sine wave to start
        self.phase0 = phase1 = 0.0	# 0 phase
        self.activeFreq = REG0
        self.activePhase = REG0
        self.freqReg = REG0
        self.phaseVal=REG0
        self.phaseReg=REG0
        self.waveFormReg = REG0

        self.dataPin  = dataPin
        self.clkPin   = clkPin
        self.fsyncPin = gpiozero.OutputDevice(pin = fsync)

        self.fsyncPin.on()
        self.clkPin.on()
        self.dataPin.off()

        # FPIO CLOCK Frwq = 25.0e6
        self.clk_freq = 25.0e6

    def set_freq(self, f):
        if f > self.clk_freq/2:	# TODO: Fix this based on refFreq
            f = self.clk_freq/2
        if f < 0.0:
            f = 0.0

        flag_b28  = 1 << 13
        flag_freq = 1 << 14

        # refFrequency =
        # freqWord = (frequency * pow2_28) / refFrequency
        scale = 1 << 28
        n_reg = int(f * scale / self.clk_freq)

        # Fout=Fmclk/2^28 * FREQREG

        n_low = n_reg         & 0x3fff
        n_hi  = (n_reg >> 14) & 0x3fff

        flag_b28 |= self.waveForm 

        self.send16(flag_b28)
        self.send16(flag_freq | n_low)
        self.send16(flag_freq | n_hi)

    def set_phase(self, p):
        p=math.fmod(p,360)
        if p < 0:
            p = p + 360.0

        # PhaseOut= 2pi/4096 * phase
        self.phaseVal = (np.uint16)(BITS_PER_DEG * p) & 0x0FFF
        self.phaseVal |= PHASE_WRITE_CMD

        '''
        # Save phase for use by IncrementPhase function
        if  self.phaseReg == REG0:
            self.phase0 = p
        else: 
            self.phase1 = p
            self.phaseVal |= PHASE1_WRITE_REG
        '''
        self.send16(self.phaseVal)

    def send16(self, n):
        self.fsyncPin.off()
        mask = 1 << 15
        for i in range(0, 16):
            self.dataPin.value = bool(n & mask)
            self.clkPin.off()
            self.clkPin.on()
            mask = mask >> 1
        self.dataPin.off()
        self.fsyncPin.on()

    def WriteRegister(self, n):
        self.send16(n)

    def WriteControlRegister(self):
        if  self.activeFreq == REG0:
            self.waveForm = self.waveForm0
            self.waveForm &= ~FREQ1_OUTPUT_REG
        else: 
            self.waveForm = self.waveForm1
            self.waveForm |= FREQ1_OUTPUT_REG

        if self.activePhase == REG0:
            self.waveForm &= ~PHASE1_OUTPUT_REG
        else:
            self.waveForm |= PHASE1_OUTPUT_REG

        if self.outputEnabled:
            self.waveForm &= ~RESET_CMD
        else:
            self.waveForm |= RESET_CMD

        if self.DacDisabled:
            self.waveForm |= DISABLE_DAC
        else:
            self.waveForm &= ~DISABLE_DAC

        if self.IntClkDisabled:
            self.waveForm |= DISABLE_INT_CLK
        else:
            self.waveForm &= ~DISABLE_INT_CLK

        self.WriteRegister (self.waveForm)

    # SINE_WAVE, TRIANGLE_WAVE, SQUARE_WAVE, HALF_SQUARE_WAVE
    def SetWaveform( self, waveType ):
        if self.waveFormReg == REG0:
            self.waveForm0 = waveType
        else:
            self.waveForm1 = waveType
        self.waveForm = waveType
        self.send16(self.waveForm)

    def EnableOutput ( self,  enable ):
        self.outputEnabled = enable
        self.WriteControlRegister()

    def SetOutputSource ( self, phaseReg ):
        self.activeFreq = self.freqReg
        if phaseReg == SAME_AS_REG0 :
            self.activePhase = activeFreq
        else:
            self.activePhase = phaseReg
        self.WriteControlRegister()

    def DisableInternalClock ( self, enable ): 
        self.IntClkDisabled = enable
        self.WriteControlRegister()

    def SleepMode ( self, enable ):
        self.DacDisabled = enable
        self.IntClkDisabled = enable
        self.WriteControlRegister()

    def DisableDAC( self, enable ):
        self.DacDisabled = enable
        self.WriteControlRegister()


# GPIO CLOCK SET
#import subprocess
#subprocess.call( ["/usr/bin/sudo", "/home/pi/src/pilowlib/examples/gpclock.py", "32"] ) 


#   AD9833(data, clk, fsync):
ad0 = AD9833(10, 11, 8)
ad1 = AD9833(10, 11, 7)
ad2 = AD9833(10, 11, 23)
ad3 = AD9833(10, 11, 24)

# SetWaveform
'''
SINE_WAVE
TRIANGLE_WAVE
SQUARE_WAVE
HALF_SQUARE_WAVE
'''

wave=SINE_WAVE
wave=SQUARE_WAVE
wave=HALF_SQUARE_WAVE
wave=TRIANGLE_WAVE

ad0.SetWaveform(wave)
ad1.SetWaveform(wave)
ad2.SetWaveform(wave)
ad3.SetWaveform(wave)
time.sleep(1)

while 1:
    for f in range(1000,100000, 500):
        #print('Freq=',f)
        ad0.set_freq(f)
        ad1.set_freq(f)
        ad2.set_freq(f)
        ad3.set_freq(f)
        ad2.set_phase(f/100)
        #time.sleep(0.2)
'''
while 1:
    for f in range(1000,10000, 100):
        for p in range(0,360,30):
            print('Freq=',f,' Phase=',p)
            ad0.set_freq(f)
            ad1.set_freq(f)
            ad1.set_phase(p)
            ad2.set_freq(f)
            ad3.set_freq(f)
            ad3.set_phase(p)
            time.sleep(1)
'''
