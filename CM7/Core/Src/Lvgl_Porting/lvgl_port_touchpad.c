/**
* @file indev.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "lvgl_port_touchpad.h"
#include "lvgl_port_lcd.h"
#include "lvgl/lvgl.h"

#include "stm32h745i_discovery.h"
#include "stm32h745i_discovery_ts.h"
#include "stm32h745i_discovery_lcd.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void touchpad_read(lv_indev_t *indev, lv_indev_data_t *data);

/**********************
 *  STATIC VARIABLES
 **********************/
static TS_State_t  TS_State;

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/**
 * Initialize your input devices here
 */
void touchpad_init(void)
{
  TS_Init_t tsInit;
  tsInit.Width = LCD_DEFAULT_WIDTH;
  tsInit.Height = LCD_DEFAULT_HEIGHT;
  tsInit.Orientation = LCD_ORIENTATION_LANDSCAPE;
  tsInit.Accuracy = 0;
  BSP_TS_Init(LCD_INSTANCE, &tsInit);

  lv_indev_t * indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, touchpad_read);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/**
 * Read an input device
 * @param indev_id id of the input device to read
 * @param x put the x coordinate here
 * @param y put the y coordinate here
 * @return true: the device is pressed, false: released
 */
static void touchpad_read(lv_indev_t *indev, lv_indev_data_t *data)
{
  /* Read your touchpad */
  static int32_t last_x = 0;
  static int32_t last_y = 0;
  BSP_LED_Toggle(LED1);

  BSP_TS_GetState(0, &TS_State);
  if(TS_State.TouchDetected) {
    data->point.x = (int32_t)TS_State.TouchY; //[0];
    data->point.y = (int32_t)TS_State.TouchX; //[0];
    last_x = data->point.x;
    last_y = data->point.y;
    data->state = LV_INDEV_STATE_PRESSED;
  } else {
    data->point.x = last_x;
    data->point.y = last_y;
    data->state = LV_INDEV_STATE_RELEASED;
  }
}