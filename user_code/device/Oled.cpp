#include "oled.h"

#include "cmsis_os.h"

#include "oledfont.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "bsp_i2c.h"

#ifdef __cplusplus
}
#endif


#include <stdio.h>
#include <stdarg.h>
#include <string>

#include "detect_task.h"
using namespace std;

void Oled::init()
{

    shoot_toe_name[0] = "FR\0";
    shoot_toe_name[1] = "RR\0";
    shoot_toe_name[2] = "TRI\0";
    shoot_toe_name[3] = "COV\0";

    other_toe_name[0] = "IMU\0",
    other_toe_name[1] = "REF\0";
    other_toe_name[2] = "CAP\0";
    other_toe_name[3] = "   \0";

    last_oled_error = 0;
    now_oled_errror = 0;
    refresh_tick = 0;

    uint8_t i;
    error_list_local = get_error_list_point();

    osDelay(1000);

    oled_init();
    oled_LOGO();
    i = 100;
    while (i--)
    {
        if (oled_check_ack())
        {
            detect_hook(OLED_TOE);
        }
        osDelay(10);
    }
}

void Oled::run()
{
	int i=0;
    //use i2c ack to check the oled
    if (oled_check_ack())
    {
        detect_hook(OLED_TOE);
    }

    now_oled_errror = toe_is_error(OLED_TOE);
    //oled init
    if (last_oled_error == 1 && now_oled_errror == 0)
    {
        oled_init();
    }

    if (now_oled_errror == 0)
    {
        refresh_tick++;
        //10Hz refresh
        if (refresh_tick > configTICK_RATE_HZ / (OLED_CONTROL_TIME * REFRESH_RATE))
        {
            refresh_tick = 0;
            oled_operate_gram(PEN_CLEAR);
            oled_show_graphic(0, 1, &battery_box);
            
            //TODO 电池电量测量还未写
            // if (get_battery_percentage() < 10)
            // {
            //     oled_printf(9, 2, "%d", get_battery_percentage());
            // }
            // else if (get_battery_percentage() < 100)
            // {
            //     oled_printf(6, 2, "%d", get_battery_percentage());
            // }
            // else
            // {
            //     oled_printf(3, 2, "%d", get_battery_percentage());
            // }

            /*
                    oledÏÔÊ¾£º
                    µç³ØµçÁ¿ DBUS YAW PIT:  Ò£¿ØÆ÷£¬ yawÖáµç»ú£¬ pitchÖáµç»ú
                    FR RR TRI COV:          ·¢Éä»ú¹¹µç»ú£º ×óÄ¦²ÁÂÖ£¬ÓÒÄ¦²ÁÂÖ£¬²¦ÅÌ£¬µ¯²Ö¸Ç
                    M0 M1 M2 M3:            µ×ÅÌ¶¯Á¦µç»ú£º ÓÒÇ°£¬ ×óÇ°£¬ ×óºó£¬ ÓÒºó
                    R1 R2 R3 R4:            µ×ÅÌ¶æÏòµç»ú£º ÓÒÇ°£¬ ×óÇ°£¬ ×óºó£¬ ÓÒºó
                    IMU REF CAP:            ÍÓÂÝÒÇ£¬ ²ÃÅÐÍ¨ÐÅ£¬ ³¬¼¶µçÈÝ

                */
            oled_show_string(32, 2, "DBUS");
            oled_show_graphic(48, 2, &check_box[error_list_local[DBUS_TOE].error_exist]);

            oled_show_string(65, 2, "YAW");
            oled_show_graphic(83, 2, &check_box[error_list_local[GIMBAL_YAW_MOTOR_TOE].error_exist]);

            oled_show_string(97, 2, "PIT");
            oled_show_graphic(115, 2, &check_box[error_list_local[GIMBAL_PITCH_MOTOR_TOE].error_exist]);

            for (i = SHOOT_LEFT_FRIC_MOTOR_ID; i < SHOOT_COVER_MOTOR_TOE + 1; i++)
            {
                uint8_t j = i - SHOOT_LEFT_FRIC_MOTOR_ID;
                show_col = (j * 32) % 128;
                show_row = 13;
                oled_show_string(show_col, show_row, shoot_toe_name[j]);
                oled_show_graphic(show_col + 18, show_row, &check_box[error_list_local[i].error_exist]);
            }

            for (i = CHASSIS_RUDDER_FR_MOTOR_TOE; i < CHASSIS_RUDDER_BR_MOTOR_TOE + 1; i++)
            {
                uint8_t j = i - CHASSIS_RUDDER_FR_MOTOR_TOE;
                show_col = (j * 32) % 128;
                show_row = 26;
                oled_show_char(show_col, show_row, 'M');
                oled_show_char(show_col + 6, show_row, '0' + j);
                oled_show_graphic(show_col + 12, show_row, &check_box[error_list_local[i].error_exist]);
            }

            for (i = CHASSIS_RUDDER_FR_MOTOR_TOE; i < CHASSIS_RUDDER_BR_MOTOR_TOE + 1; i++)
            {
                uint8_t j = i - CHASSIS_RUDDER_FR_MOTOR_TOE;
                show_col = (j * 32) % 128;
                show_row = 39;
                oled_show_char(show_col, show_row, 'R');
                oled_show_char(show_col + 6, show_row, '0' + j);
                oled_show_graphic(show_col + 12, show_row, &check_box[error_list_local[i].error_exist]);
            }

            for (i = RM_IMU_TOE; i < SUPER_CAP_TOE + 1; i++)
            {
                uint8_t j = i - RM_IMU_TOE;
                show_col = (i * 32) % 128;
                show_row = 50;
                oled_show_string(show_col, show_row, other_toe_name[j]);
                oled_show_graphic(show_col + 18, show_row, &check_box[error_list_local[i].error_exist]);
            }

            oled_refresh_gram();
        }
    }

    last_oled_error = now_oled_errror;
    osDelay(OLED_CONTROL_TIME);
}

//TODO:暂时没用到
void Oled::oled_com_reset(void)
{
    static uint16_t time = 0;
    time++;
    if (time > 100)
    {
        bsp_I2C_reset(OLED_I2C);
        time = 0;
    }
}

/**
 * @brief   write data/command to oled, if you use spi, please rewrite the function
 * @param   dat: the data ready to write
 * @param   cmd: oled_CMD means command; oled_DATA means data
 * @retval  none
 */
void Oled::oled_write_byte(uint8_t dat, uint8_t cmd)
{
    static uint8_t cmd_data[2];
    if (cmd == OLED_CMD)
    {
        cmd_data[0] = 0x00;
    }
    else
    {
        cmd_data[0] = 0x40;
    }
    cmd_data[1] = dat;
    bsp_I2C_master_transmit(OLED_I2C, OLED_I2C_ADDRESS, cmd_data, 2);
}

/**
 * @brief   initialize the oled device
 * @param   none
 * @retval  none
 */
void Oled::oled_init(void)
{
    I2C2_tx_DMA_init();

#if defined(oled_ONE_COLOR)
    oled_write_byte(0xAE, oled_CMD); //display off
    oled_write_byte(0x40, oled_CMD); //--set start line address
    oled_write_byte(0x81, oled_CMD); //--set contrast control register
    oled_write_byte(0xFF, oled_CMD); //brightness 0x00~0xff
    oled_write_byte(0xa4, oled_CMD); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
    oled_write_byte(0xa6, oled_CMD); //--set normal display
    oled_write_byte(0x20, oled_CMD); //Set Memory Addressing Mode
    oled_write_byte(0x00, oled_CMD); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
    oled_write_byte(0x21, oled_CMD);
    oled_write_byte(0x00, oled_CMD);
    oled_write_byte(0x7F, oled_CMD);
    oled_write_byte(0x22, oled_CMD);
    oled_write_byte(0x00, oled_CMD);
    oled_write_byte(0x07, oled_CMD);
    oled_write_byte(0x3F, oled_CMD); //
    oled_write_byte(0xc8, oled_CMD); //Set COM Output Scan Direction
    oled_write_byte(0xa1, oled_CMD); //--set segment re-map 0 to 127
    oled_write_byte(0xa8, oled_CMD); //--set multiplex ratio(1 to 64)
    oled_write_byte(0x00, oled_CMD); //
    oled_write_byte(0xd3, oled_CMD); //-set display offset
    oled_write_byte(0x00, oled_CMD); //-not offset
    oled_write_byte(0xd5, oled_CMD); //--set display clock divide ratio/oscillator frequency
    oled_write_byte(0xF0, oled_CMD); //--set divide ratio
    oled_write_byte(0xd9, oled_CMD); //--set pre-charge period
    oled_write_byte(0x22, oled_CMD); //
    oled_write_byte(0xda, oled_CMD); //--set com pins hardware configuration
    oled_write_byte(0x12, oled_CMD);
    oled_write_byte(0xdb, oled_CMD); //--set vcomh
    oled_write_byte(0x20, oled_CMD); //0x20,0.77xVcc
    oled_write_byte(0x8d, oled_CMD); //--set DC-DC enable
    oled_write_byte(0x14, oled_CMD); //
    oled_write_byte(0xaf, oled_CMD); //--turn on oled panel
#elif defined(oled_TWO_COLOR)

    oled_write_byte(0xAE, oled_CMD);
    oled_write_byte(0x20, oled_CMD);
    oled_write_byte(0x00, oled_CMD);
    oled_write_byte(0x21, oled_CMD);
    oled_write_byte(0x00, oled_CMD);
    oled_write_byte(0x7F, oled_CMD);
    oled_write_byte(0x22, oled_CMD);
    oled_write_byte(0x00, oled_CMD);
    oled_write_byte(0x07, oled_CMD);
    oled_write_byte(0x3F, oled_CMD);
    oled_write_byte(0x81, oled_CMD);
    oled_write_byte(0xFF, oled_CMD);
    oled_write_byte(0xA1, oled_CMD);
    oled_write_byte(0xA6, oled_CMD);
    oled_write_byte(0xA8, oled_CMD);
    oled_write_byte(0x3F, oled_CMD);
    oled_write_byte(0xC8, oled_CMD);
    oled_write_byte(0xD3, oled_CMD);
    oled_write_byte(0x00, oled_CMD);
    oled_write_byte(0xD5, oled_CMD);
    oled_write_byte(0x80, oled_CMD);
    oled_write_byte(0xD9, oled_CMD);
    oled_write_byte(0x1F, oled_CMD);
    oled_write_byte(0xDA, oled_CMD);
    oled_write_byte(0x12, oled_CMD);
    oled_write_byte(0xDB, oled_CMD);
    oled_write_byte(0x30, oled_CMD);
    oled_write_byte(0x8d, oled_CMD);
    oled_write_byte(0x14, oled_CMD);
    oled_write_byte(0xAF, oled_CMD);

#endif
}

bool_t Oled::oled_check_ack(void)
{
    return bsp_I2C_check_ack(OLED_I2C, OLED_I2C_ADDRESS);
}

/**
 * @brief   turn on oled display
 * @param   None
 * @param   None
 * @retval  
 */
void Oled::oled_display_on(void)
{
    oled_write_byte(0x8d, OLED_CMD);
    oled_write_byte(0x14, OLED_CMD);
    oled_write_byte(0xaf, OLED_CMD);
}

/**
 * @brief   turn off oled display
 * @param   None
 * @param   None
 * @retval  
 */
void Oled::oled_display_off(void)
{
    oled_write_byte(0x8d, OLED_CMD);
    oled_write_byte(0x10, OLED_CMD);
    oled_write_byte(0xae, OLED_CMD);
}

/**
 * @brief   operate the graphic ram(size: 128*8 char)
 * @param   pen: the type of operate.
            PEN_CLEAR: set ram to 0x00
            PEN_WRITE: set ram to 0xff
            PEN_INVERSION: bit inversion 
 * @retval  none
 */
void Oled::oled_operate_gram(pen_typedef pen)
{
    uint8_t i, n;

    for (i = 0; i < 8; i++)
    {
        for (n = 0; n < 128; n++)
        {
            if (pen == PEN_WRITE)
            {
                oled_gram.oled_GRAM[i][n] = 0xff;
            }
            else if (pen == PEN_CLEAR)
            {
                oled_gram.oled_GRAM[i][n] = 0x00;
            }
            else
            {
                oled_gram.oled_GRAM[i][n] = 0xff - oled_gram.oled_GRAM[i][n];
            }
        }
    }
}

/**
 * @brief   cursor set to (x,y) point
 * @param   x:X-axis, from 0 to 127
 * @param   y:Y-axis, from 0 to 7
 * @retval  none
 */
void Oled::oled_set_pos(uint8_t x, uint8_t y)
{
    x &= 0x7F;
    y &= 0x07;

    oled_write_byte(0x21, OLED_CMD);
    oled_write_byte(0x00 + x, OLED_CMD);
    oled_write_byte(0x7F, OLED_CMD);

    oled_write_byte(0x22, OLED_CMD);
    oled_write_byte(0x00 + y, OLED_CMD);
    oled_write_byte(0x07, OLED_CMD);
}

/**
 * @brief   draw one bit of graphic raw, operate one point of screan(128*64)
 * @param   x: x-axis, [0, X_WIDTH-1]
 * @param   y: y-axis, [0, Y_WIDTH-1]
 * @param   pen: type of operation,
            PEN_CLEAR: set (x,y) to 0
            PEN_WRITE: set (x,y) to 1
            PEN_INVERSION: (x,y) value inversion 
 * @retval  none
 */
void Oled::oled_draw_point(uint8_t x, uint8_t y, pen_typedef pen)
{
    uint8_t page = 0, row = 0;

    /* check the corrdinate */
    if ((x > (X_WIDTH - 1)) || (y > (Y_WIDTH - 1)))
    {
        return;
    }
    page = y / 8;
    row = y % 8;

    if (pen == PEN_WRITE)
    {
        oled_gram.oled_GRAM[page][x] |= 1 << row;
    }
    else if (pen == PEN_INVERSION)
    {
        oled_gram.oled_GRAM[page][x] ^= 1 << row;
    }
    else
    {
        oled_gram.oled_GRAM[page][x] &= ~(1 << row);
    }
}

/**
 * @brief   draw a line from (x1, y1) to (x2, y2)
 * @param   x1, y1: the start point of line
 * @param   x2, y2: the end of line
 * @param   pen: Pen_Clear, Pen_Write, Pen_Inversion @Pen_Typedef
 * @retval  None
 */
void Oled::oled_draw_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, pen_typedef pen)
{
    uint8_t col = 0, row = 0;
    uint8_t x_st = 0, x_ed = 0, y_st = 0, y_ed = 0;
    float k = 0.0f, b = 0.0f;

    if (y1 == y2)
    {
        (x1 <= x2) ? (x_st = x1) : (x_st = x2);
        (x1 <= x2) ? (x_ed = x2) : (x_ed = x1);

        for (col = x_st; col <= x_ed; col++)
        {
            oled_draw_point(col, y1, pen);
        }
    }
    else if (x1 == x2)
    {
        (y1 <= y2) ? (y_st = y1) : (y_st = y2);
        (y1 <= y2) ? (y_ed = y2) : (y_ed = y1);

        for (row = y_st; row <= y_ed; row++)
        {
            oled_draw_point(x1, row, pen);
        }
    }
    else
    {
        k = ((float)(y2 - y1)) / (x2 - x1);
        b = (float)y1 - k * x1;

        (x1 <= x2) ? (x_st = x1) : (x_st = x2);
        (x1 <= x2) ? (x_ed = x2) : (x_ed = x2);

        for (col = x_st; col <= x_ed; col++)
        {
            oled_draw_point(col, (uint8_t)(col * k + b), pen);
        }
    }
}

/**
 * @brief   show a character
 * @param   row: row of character
 * @param   col: column of character
 * @param   chr: the character ready to show
 * @retval  None
 */
void Oled::oled_show_char(uint8_t col, uint8_t row, uint8_t chr)
{
    uint8_t x = col;
    uint8_t y = row;
    uint8_t temp, t, t1;
    uint8_t y0 = y;
    chr = chr - ' ';

    for (t = 0; t < 12; t++)
    {
        temp = asc2_1206[chr][t];

        for (t1 = 0; t1 < 8; t1++)
        {
            if (temp & 0x80)
                oled_draw_point(x, y, PEN_WRITE);
            else
                oled_draw_point(x, y, PEN_CLEAR);

            temp <<= 1;
            y++;
            if ((y - y0) == 12)
            {
                y = y0;
                x++;
                break;
            }
        }
    }
}

/**
 * @brief   show a character string
 * @param   row: row of character string begin
 * @param   col: column of character string begin
 * @param   chr: the pointer to character string
 * @retval  None
 */
void Oled::oled_show_string(uint8_t col, uint8_t row, string chr)
{
    uint8_t n = 0;

    while (chr[n] != '\0')
    {
        oled_show_char(col, row, chr[n]);
        col += 6;

        if (col > X_WIDTH - 6)
        {
            col = 0;
            row += 12;
        }
        n++;
    }
}

/**
 * @brief   formatted output in oled 128*64
 * @param   row: row of character string begin, 0 <= row <= 4;
 * @param   col: column of character string begin, 0 <= col <= 20;
 * @param   *fmt: the pointer to format character string
 * @retval  None
 * @note    if the character length is more than one row at a time, the extra characters will be truncated
 */
void Oled::oled_printf(uint8_t col, uint8_t row, const char *fmt, ...)
{
    static char LCD_BUF[22] = {0};
    static va_list ap;
    uint16_t remain_size = 0;

    va_start(ap, fmt);

    remain_size = vsprintf((char *)LCD_BUF, fmt, ap);

    va_end(ap);

    LCD_BUF[remain_size] = '\0';
    string temp_str = LCD_BUF;

    oled_show_string(col, row, temp_str);
}

/**
 * @brief   send the data of gram to oled sreen
 * @param   none
 * @retval  none
 */
void Oled::oled_refresh_gram(void)
{
    oled_set_pos(0, 0);
    oled_gram.cmd_data = 0x40;
    I2C2_DMA_transmit(OLED_I2C_ADDRESS, (uint8_t *)&oled_gram, 1025);
}

/**
 * @brief   show the logo of robomaster
 * @param   none
 * @retval  none
 */
void Oled::oled_LOGO(void)
{
    //TODO 暂时有问题 没写
    //memcpy(oled_gram.oled_GRAM, RM_LOGO_BMP_TRANS, 1024);
    //    oled_show_graphic(0, 0, &rm_logo);
    oled_refresh_gram();
}

void Oled::oled_show_graphic(uint8_t x, uint8_t y, const picture_t *graphic)
{
    uint8_t col, row;
    uint8_t temp_char, t;
    uint16_t i = 0;

    for (col = 0; col < graphic->length; col++)
    {
        for (row = 0; row < graphic->width;)
        {
            temp_char = graphic->data[i];
            i++;
            for (t = 0; t < 8; t++)
            {
                if (temp_char & 0x80)
                {
                    oled_draw_point(x + col, y + row, PEN_WRITE);
                }
                else
                {
                    oled_draw_point(x + col, y + row, PEN_CLEAR);
                }
                temp_char <<= 1;
                row++;
                if (row == graphic->width)
                {
                    break;
                }
            }
        }
    }
}
