/**
 * MIT License
 * 
 * Copyright (c) 2019 Brian Amos
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */
#ifndef INTERFACES_ILED_H_
#define INTERFACES_ILED_H_
#ifdef __cplusplus
 extern "C" {
#endif


//Create a typedef defining a simple function pointer
//to be used for LED's
typedef void (*iLedFunc)(void);

/**
 * This struct definition holds function pointers to turn each LED
 * on and off
 */
typedef struct
{
	/**
	 * On turns on the LED - regardless of the driver logic
	 */
	const iLedFunc On;

	/**
	 * Off turns off the LED - regardless of the driver logic
	 */
	const iLedFunc Off;

	const iLedFunc Toggle;

	const iLedFunc Blink;
}iLed;


#ifdef __cplusplus
 }
#endif
#endif /* INTERFACES_ILED_H_ */
