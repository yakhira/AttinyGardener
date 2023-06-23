/*
The BSD 3-Clause License

Copyright (c) 2022, Dalimir Orfanus


Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
contributors may be used to endorse or promote products derived from
this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


//------------------------------------------------------------------------
// nrf24l01.h - API header for the nRF24L01 driver
//------------------------------------------------------------------------

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <Arduino.h>

int digitalRead_D(int pin);
void digitalWrite_B(int pin, int level);

#define SLEEP_016MS    (0<<WDP3 )|(0<<WDP2 )|(0<<WDP1)|(0<<WDP0)
#define SLEEP_032MS    (0<<WDP3 )|(0<<WDP2 )|(0<<WDP1)|(1<<WDP0)
#define SLEEP_64MS     (0<<WDP3 )|(0<<WDP2 )|(1<<WDP1)|(0<<WDP0)
#define SLEEP_125MS    (0<<WDP3 )|(0<<WDP2 )|(1<<WDP1)|(1<<WDP0)
#define SLEEP_250MS    (0<<WDP3 )|(1<<WDP2 )|(0<<WDP1)|(0<<WDP0)
#define SLEEP_500MS    (0<<WDP3 )|(1<<WDP2 )|(0<<WDP1)|(1<<WDP0)
#define SLEEP_1SEC     (0<<WDP3 )|(1<<WDP2 )|(1<<WDP1)|(0<<WDP0)
#define SLEEP_2SEC     (0<<WDP3 )|(1<<WDP2 )|(1<<WDP1)|(1<<WDP0)
#define SLEEP_4SEC     (1<<WDP3 )|(0<<WDP2 )|(0<<WDP1)|(0<<WDP0)
#define SLEEP_8SEC     (1<<WDP3 )|(0<<WDP2 )|(0<<WDP1)|(1<<WDP0)

void setSleep(int b);

#ifdef __cplusplus
}
#endif
