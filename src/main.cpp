/*--------------------------------------------------------------------
Copyright 2020 fukuen
lvgl demo is free software: you can redistribute it
and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.
This software is distributed in the hope that it will be
useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with This software.  If not, see
<http://www.gnu.org/licenses/>.
--------------------------------------------------------------------*/

#include <Arduino.h>
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <Adafruit_FT6206.h>

#include "lv_ex_conf.h"
#include "lv_examples.h"

#define AXP173_ADDR 0x34
#ifdef MAIXAMIGO
#define PIN_SDA 27
#define PIN_SCL 24
#endif

TFT_eSPI tft = TFT_eSPI(); /* TFT instance */
static lv_disp_buf_t disp_buf;
static lv_color_t buf[LV_HOR_RES_MAX * 10];

// The FT6206 uses hardware I2C (SCL/SDA)
Adafruit_FT6206 ctp = Adafruit_FT6206();

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors(&color_p->full, w * h, true);
    tft.endWrite();
//    tft.pushRect(area->x1, area->y1, w, h, &color_p->full);

    lv_disp_flush_ready(disp);
}

/*Read the touchpad*/
bool my_touchpad_read(lv_indev_drv_t * indev_driver, lv_indev_data_t * data) {
    uint16_t touchX, touchY;

    if (! ctp.touched()) {
        data->state = LV_INDEV_STATE_REL;
        return false;
    } else {
        data->state = LV_INDEV_STATE_PR;
    }

    TS_Point p = ctp.getPoint();
    touchX = p.y;
    touchY = 320 - p.x;

    if(touchX > 480 || touchY > 320) {
        Serial.println("Y or y outside of expected parameters..");
        Serial.print("x:");
        Serial.print(touchX);
        Serial.print(" y:");
        Serial.print(touchY);
    } else {
        /*Set the coordinates*/
        data->point.x = touchX;
        data->point.y = touchY;
  
        Serial.print("Data x");
        Serial.println(touchX);
      
        Serial.print("Data y");
        Serial.println(touchY);
    }

    return false; /*Return `false` because we are not buffering and no more data to read*/
}

void axp173_init() {
    Wire.begin((uint8_t) PIN_SDA, (uint8_t) PIN_SCL, 400000);
    Wire.beginTransmission(AXP173_ADDR);
    int err = Wire.endTransmission();
    if (err) {
        Serial.printf("Power management ic not found.\n");
        return;
    }
    Serial.printf("AXP173 found.\n");
#ifdef MAIXAMIGO
    //LDO4 - 0.8V (default 0x48 1.8V)
    Wire.beginTransmission(AXP173_ADDR);
    Wire.write(0x27);
    Wire.write(0x20);
    Wire.endTransmission();
    //LDO2/3 - LDO2 1.8V / LDO3 3.0V
    Wire.beginTransmission(AXP173_ADDR);
    Wire.write(0x28);
    Wire.write(0x0C);
    Wire.endTransmission();
#else
    // Clear the interrupts
    Wire.beginTransmission(AXP173_ADDR);
    Wire.write(0x46);
    Wire.write(0xFF);
    Wire.endTransmission();
    // set target voltage and current of battery(axp173 datasheet PG.)
    // charge current (default)780mA -> 190mA
    Wire.beginTransmission(AXP173_ADDR);
    Wire.write(0x33);
    Wire.write(0xC1);
    Wire.endTransmission();
    // REG 10H: EXTEN & DC-DC2 control
    Wire.beginTransmission(AXP173_ADDR);
    Wire.write(0x10);
    Wire.endTransmission();
    Wire.requestFrom(AXP173_ADDR, 1, 1);
    int reg = Wire.read();
    Wire.beginTransmission(AXP173_ADDR);
    Wire.write(0x10);
    Wire.write(reg & 0xFC);
    Wire.endTransmission();
#endif
}

void setup() {
    Serial.begin(115200); /* prepare for possible serial debug */

    axp173_init();

    lv_init();

    tft.begin(); /* TFT init */
    tft.setRotation(1); /* Landscape orientation */

    if (! ctp.begin(40)) {  // pass in 'sensitivity' coefficient
        Serial.println("Couldn't start FT6206 touchscreen controller");
        while (1);
    }

    Serial.println("Capacitive touchscreen started");
  
    lv_disp_buf_init(&disp_buf, buf, NULL, LV_HOR_RES_MAX * 10);

    /*Initialize the display*/
    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = 480;
    disp_drv.ver_res = 320;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.buffer = &disp_buf;
    lv_disp_drv_register(&disp_drv);

    lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    /*Register the driver in LVGL and save the created input device object*/
    lv_indev_drv_register(&indev_drv);

//    lv_ex_get_started_1();
    lv_ex_get_started_2();
//    lv_ex_get_started_3();
}

void loop() {
    lv_task_handler(); /* let the GUI do its work */
    delay(5);
    lv_tick_inc(5);
}
