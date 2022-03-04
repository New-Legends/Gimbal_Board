#ifndef oled_H
#define oled_H

#include "main.h"
#include "struct_typedef.h"
#include "oledfont.h"

#include <string>

#include "detect_task.h"

using namespace std;

#define OLED_CONTROL_TIME 10
#define REFRESH_RATE 10


// the I2C address of oled
#define OLED_I2C_ADDRESS 0x78 //or be 0x7A
#define OLED_I2C I2C2

//the resolution of oled   128*64
#define MAX_COLUMN 128
#define MAX_ROW 64

#define X_WIDTH MAX_COLUMN
#define Y_WIDTH MAX_ROW

#define OLED_CMD 0x00
#define OLED_DATA 0x01

#define CHAR_SIZE_WIDTH 6
#define CHAR_SIZE_HIGHT 12

typedef enum
{
    PEN_CLEAR = 0x00,
    PEN_WRITE = 0x01,
    PEN_INVERSION = 0x02,
} pen_typedef;

typedef __packed struct
{
    uint8_t cmd_data;
    uint8_t oled_GRAM[8][128];
} oled_GRAM_strutct_t;




class Oled
{
public:
    oled_GRAM_strutct_t oled_gram;

    const error_t *error_list_local;

    string shoot_toe_name[4];
    string other_toe_name[4];

    uint8_t last_oled_error;
    uint8_t now_oled_errror;
    uint8_t refresh_tick;

    uint8_t show_col, show_row;

    void init();

    void run();

    //功能函数

    void oled_write_byte(uint8_t dat, uint8_t cmd);
    
    void oled_com_reset(void);

    void oled_init(void);

    bool_t oled_check_ack(void);

    void oled_display_on(void);

 
    void oled_display_off(void);


    void oled_operate_gram(pen_typedef pen);


    void oled_set_pos(uint8_t x, uint8_t y);


    void oled_draw_point(uint8_t x, uint8_t y, pen_typedef pen);

 
    void oled_draw_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, pen_typedef pen);


    void oled_show_char(uint8_t row, uint8_t col, uint8_t chr);


    void oled_show_string(uint8_t row, uint8_t col, string chr);


    void oled_printf(uint8_t row, uint8_t col, const char *fmt, ...);


    void oled_refresh_gram(void);

    void oled_show_graphic(uint8_t x, uint8_t y, const picture_t *graphic);

    void oled_LOGO(void);



};


extern Oled OLED;


#endif