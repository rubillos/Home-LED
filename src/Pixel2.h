/*********************************************************************************
 *  MIT License
 *  
 *  Copyright (c) 2020-2024 Gregg E. Berman
 *  
 *  https://github.com/HomeSpan/HomeSpan
 *  
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *  
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *  
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 *  
 ********************************************************************************/
 
////////////////////////////////////////////
//           Addressable LEDs             //
////////////////////////////////////////////

#pragma once

#include "HomeSpan.h"

#pragma GCC diagnostic ignored "-Wvolatile"

#include <driver/rmt_tx.h>      // IDF 5 RMT driver
#include <soc/rmt_struct.h>     // where RMT register structure is defined
#include <hal/rmt_ll.h>         // where low-level RMT calls are defined

#include <soc/gpio_struct.h>

////////////////////////////////////////////
//     Single-Wire RGB/RGBW NeoPixels     //
////////////////////////////////////////////

class Pixel2 : public Blinkable {

  private:
    typedef struct {
      rmt_symbol_word_t bit0; /*!< How to represent BIT0 in RMT symbol */
      rmt_symbol_word_t bit1; /*!< How to represent BIT1 in RMT symbol */
      Pixel2* pixel;
      bool multiColor;
    } rmt_pixel_encoder_config_t;
    
    rmt_pixel_encoder_config_t encoder_config;

    static IRAM_ATTR size_t pixelEncodeCallback(const void *data, size_t data_size,
                     size_t symbols_written, size_t symbols_free,
                     rmt_symbol_word_t *symbols, bool *done, void *arg);

    uint8_t pin;
    int channel=-1;
    char *pType=NULL;
    rmt_channel_handle_t tx_chan = NULL;
    rmt_encoder_handle_t encoder;
    rmt_transmit_config_t tx_config;
  
    uint32_t resetTime;            // minimum time (in usec) between pulse trains
    uint8_t bytesPerPixel;         // WC=2, RGB=3, RGBW=4, RGBWC=5
    float warmTemp=2000;           // default temperature (in Kelvin) of warm-white LED
    float coolTemp=7000;           // defult temperature (in Kelvin) of cool-white LED
    uint8_t map[5];                // color map representing order in which color bytes are transmitted
    Pixel::Color onColor;                 // color used for on() command
  
  public:
    Pixel2(int pin, const char *pixelType="GRB");                     // creates addressable single-wire LED of pixelType connected to pin (such as the SK68 or WS28)   
    void set(Pixel::Color *c, int nPixels, boolean multiColor=true);        // sets colors of nPixels based on array of Colors c; setting multiColor to false repeats Color in c[0] for all nPixels
    void set(Pixel::Color c, int nPixels=1){set(&c,nPixels,false);}         // sets color of nPixels to be equal to specific Color c

    int getPin(){return(channel>=0?pin:-1);}                                                        // returns pixel pin (=-1 if channel is not valid)
    Pixel2 *setTiming(float high0, float low0, float high1, float low1, uint32_t lowReset);          // changes default timings for bit pulse - note parameters are in MICROSECONDS
    Pixel2 *setTemperatures(float wTemp, float cTemp){warmTemp=wTemp;coolTemp=cTemp;return(this);}   // changes default warm-white and cool-white LED temperatures (in Kelvin)
        
    boolean hasColor(char c){return(strchr(pType,toupper(c))!=NULL || strchr(pType,tolower(c))!=NULL);}   // returns true if pixelType includes c (case-insensitive)

    operator bool(){         // override boolean operator to return true/false if creation succeeded/failed
      return(channel>=0);
    }

    void on() {set(onColor);}
    void off() {set(Pixel::RGB(0,0,0,0));}
    Pixel2 *setOnColor(Pixel::Color c){onColor=c;return(this);}
};
