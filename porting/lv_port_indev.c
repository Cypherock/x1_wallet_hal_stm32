/**
 * @file lv_port_indev.c
 *
 */
#if 1

/*********************
 *      INCLUDES
 *********************/
#include "lv_port_indev.h"
#include "board.h"
#include "adafruit_pn532.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void keypad_init(void);
static bool keypad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data);
uint32_t keypad_get_key(void);

/**********************
 *  STATIC VARIABLES
 **********************/
lv_indev_t * indev_encoder;
lv_indev_t * indev_keypad;
static uint8_t invert = 0x00;
#ifdef DEV_BUILD
static ekp_process_queue_fptr process_key_presses_queue = NULL;
#endif

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void invert_key_pad()
{
    if(invert == 0) {
        invert = 2;
        return;
    }
    invert = 0;
}

void lv_port_indev_init(void)
{
    lv_indev_drv_t indev_drv;

    /*Initialize your keypad or keyboard if you have*/
    keypad_init();

    /*Register a keypad input device*/
    lv_indev_drv_init(&indev_drv);
    indev_drv.type    = LV_INDEV_TYPE_KEYPAD;
    indev_drv.read_cb = keypad_read;
    indev_keypad      = lv_indev_drv_register(&indev_drv);
}
#ifdef DEV_BUILD
void ekp_register_process_func(ekp_process_queue_fptr func) {
    process_key_presses_queue = func;
}
#endif

/**********************
 *   STATIC FUNCTIONS
 **********************/

/* Initialize your keypad */
static void keypad_init(void)
{
    // Only initializing one button here
}

/* Will be called by the library to read the mouse */
static bool keypad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data)
{

    static uint32_t last_key = 0;

    /*Get whether the a key is pressed and save the pressed key*/
    uint32_t act_key = keypad_get_key();//BSP_GetKeyPressed();
    if(act_key != 0) {
        data->state = LV_INDEV_STATE_PR;

        /*Translate the keys to LittlevGL control characters according to your key definitions*/
        if(invert == 0) {
            switch(act_key) {
                case 1: act_key = LV_KEY_UP; break;
                case 2: act_key = LV_KEY_DOWN; break;
                case 3: act_key = LV_KEY_LEFT; break;
                case 4: act_key = LV_KEY_RIGHT; break;
                case 5: act_key = LV_KEY_ENTER; break;
            }
        } else {
            switch(act_key) {
                case 1: act_key = LV_KEY_DOWN; break;
                case 2: act_key = LV_KEY_UP; break;
                case 3: act_key = LV_KEY_RIGHT; break;
                case 4: act_key = LV_KEY_LEFT; break;
                case 5: act_key = LV_KEY_ENTER; break;
            }
        }
        last_key = act_key;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }

    data->key = last_key;
#ifdef DEV_BUILD
    if (process_key_presses_queue != NULL)
        process_key_presses_queue(data);
#endif

    BSP_ClearKeyPressed();
    /*Return `false` because we are not buffering and no more data to read*/
    return false;
}

/*Get the currently being pressed key.  0 if no key is pressed*/
uint32_t keypad_get_key(void)
{
    uint32_t ret=0;
    if(READ_JOYSTICK(BSP_JOYSTICK_UP_PIN) == joystick_pressed) {
        ret = 1;
    } else if(READ_JOYSTICK(BSP_JOYSTICK_DOWN_PIN) == joystick_pressed) {
        ret = 2;
    } else if(READ_JOYSTICK(BSP_JOYSTICK_LEFT_PIN) == joystick_pressed) {
        ret = 3;
    } else if(READ_JOYSTICK(BSP_JOYSTICK_RIGHT_PIN) == joystick_pressed) {
        ret = 4;
    } else if(READ_JOYSTICK(BSP_JOYSTICK_ENTER_PIN) == joystick_pressed) {
        ret = 5;
    } else if(0 != BSP_GetKeyPressed()){
        ret = BSP_GetKeyPressed();
    }
    return ret;
}

#else /* Enable this file at the top */

/* This dummy typedef exists purely to silence -Wpedantic. */
typedef int keep_pedantic_happy;
#endif
