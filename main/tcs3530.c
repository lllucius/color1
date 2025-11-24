/****************************************************************************
** Copyright (C) 2020 MikroElektronika d.o.o.
** Contact: https://www.mikroe.com/contact
**
** Permission is hereby granted, free of charge, to any person obtaining a copy
** of this software and associated documentation files (the "Software"), to deal
** in the Software without restriction, including without limitation the rights
** to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
** copies of the Software, and to permit persons to whom the Software is
** furnished to do so, subject to the following conditions:
** The above copyright notice and this permission notice shall be
** included in all copies or substantial portions of the Software.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
** EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
** OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
** IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
** DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
** OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
**  USE OR OTHER DEALINGS IN THE SOFTWARE.
****************************************************************************/

/*!
 * @file tcs3530.c
 * @brief TCS3530 Click Driver.
 */

#include "tcs3530.h"

err_t tcs3530_default_cfg ( tcs3530_t *ctx ) 
{
    err_t error_flag = TCS3530_OK;
    if ( TCS3530_ERROR == tcs3530_check_communication ( ctx ) )
    {
        return TCS3530_ERROR;
    }
    error_flag |= tcs3530_soft_reset ( ctx );
    Delay_100ms ( );
    error_flag |= tcs3530_write_reg ( ctx, TCS3530_REG_OSCEN, TCS3530_OSCEN_OSCEN );
    error_flag |= tcs3530_write_reg ( ctx, TCS3530_REG_ENABLE, TCS3530_ENABLE_PON );
    
    // Set measurement rate to approx. 200ms
    error_flag |= tcs3530_write_reg ( ctx, TCS3530_REG_WTIME, TCS3530_WTIME_DEFAULT );
    error_flag |= tcs3530_write_reg ( ctx, TCS3530_REG_TRIGGER_MODE, TCS3530_TRIGGER_MODE_NORMAL );

    // Enable FIFO interrupt at 16 bytes
    error_flag |= tcs3530_write_reg ( ctx, TCS3530_REG_INTENAB, TCS3530_INTENAB_FIEN );
    error_flag |= tcs3530_write_reg ( ctx, TCS3530_REG_FIFO_THR, ( uint8_t ) ( TCS3530_FIFO_THR_DEFAULT >> 3 ) );
    error_flag |= tcs3530_write_reg ( ctx, TCS3530_REG_CFG2, ( uint8_t ) ( TCS3530_FIFO_THR_DEFAULT & 0x07 ) );

    error_flag |= tcs3530_write_reg ( ctx, TCS3530_REG_ENABLE, TCS3530_ENABLE_PON | TCS3530_ENABLE_AEN );
    error_flag |= tcs3530_clear_fifo ( ctx );
    return error_flag;
}

err_t tcs3530_write_regs ( tcs3530_t *ctx, uint8_t reg, uint8_t *data_in, uint8_t len ) 
{
    uint8_t data_buf[ 256 ] = { 0 };
    data_buf[ 0 ] = reg;
    for ( uint8_t cnt = 0; cnt < len; cnt++ )
    {
        data_buf[ cnt + 1 ] = data_in[ cnt ];
    }
    return i2c_master_write( &ctx->i2c, data_buf, len + 1 );
}

err_t tcs3530_read_regs ( tcs3530_t *ctx, uint8_t reg, uint8_t *data_out, uint8_t len ) 
{
    return i2c_master_write_then_read( &ctx->i2c, &reg, 1, data_out, len );
}

err_t tcs3530_write_reg ( tcs3530_t *ctx, uint8_t reg, uint8_t data_in )
{
    return tcs3530_write_regs ( ctx, reg, &data_in, 1 );
}

err_t tcs3530_read_reg ( tcs3530_t *ctx, uint8_t reg, uint8_t *data_out )
{
    return tcs3530_read_regs ( ctx, reg, data_out, 1 );
}

err_t tcs3530_write_reg_word ( tcs3530_t *ctx, uint8_t reg, uint16_t data_in )
{
    uint8_t data_buf[ 2 ] = { 0 };
    data_buf[ 0 ] = ( uint8_t ) ( data_in & 0xFF );
    data_buf[ 1 ] = ( uint8_t ) ( ( data_in >> 8 ) & 0xFF );
    return tcs3530_write_regs ( ctx, reg, data_buf, 2 );
}

err_t tcs3530_read_reg_word ( tcs3530_t *ctx, uint8_t reg, uint16_t *data_out )
{
    uint8_t data_buf[ 2 ] = { 0 };
    err_t error_flag = tcs3530_read_regs ( ctx, reg, data_buf, 2 );
    if ( TCS3530_OK == error_flag )
    {
        *data_out = ( ( uint16_t ) data_buf[ 1 ] << 8 ) | data_buf[ 0 ];
    }
    return error_flag;
}

err_t tcs3530_check_communication ( tcs3530_t *ctx )
{
    uint8_t device_id = 0;
    if ( TCS3530_OK == tcs3530_read_reg ( ctx, TCS3530_REG_ID, &device_id ) )
    {
        if ( TCS3530_DEVICE_ID == device_id )
        {
            return TCS3530_OK;
        }
    }
    return TCS3530_ERROR;
}

uint8_t tcs3530_get_int_pin ( tcs3530_t *ctx )
{
    return digital_in_read( &ctx->int_pin );
}

err_t tcs3530_soft_reset ( tcs3530_t *ctx )
{
    return tcs3530_write_reg ( ctx, TCS3530_REG_CONTROL_SCL, TCS3530_CONTROL_SCL_SOFT_RESET );
}

err_t tcs3530_clear_fifo ( tcs3530_t *ctx )
{
    return tcs3530_write_reg ( ctx, TCS3530_REG_CONTROL, TCS3530_CONTROL_FIFO_CLR );
}

err_t tcs3530_read_fifo_size ( tcs3530_t *ctx, uint16_t *fifo_size )
{
    uint8_t fifo_status[ 2 ] = { 0 };
    err_t error_flag = tcs3530_read_regs ( ctx, TCS3530_REG_FIFO_STATUS0, fifo_status, 2 );
    if ( TCS3530_OK == error_flag )
    {
        *fifo_size = ( ( uint16_t ) fifo_status[ 0 ] << 3 ) | ( fifo_status[ 1 ] & 0x07 );
    }
    return error_flag;
}

err_t tcs3530_read_data ( tcs3530_t *ctx, tcs3530_data_t *data_out )
{
    uint16_t fifo_size = 0;
    uint8_t fifo_data[ 16 ] = { 0 };
    err_t error_flag = tcs3530_read_fifo_size ( ctx, &fifo_size );
    if ( ( TCS3530_OK == error_flag ) && ( fifo_size >= 16 ) )
    {
        error_flag |= tcs3530_read_regs ( ctx, TCS3530_REG_FIFO_DATA, fifo_data, 16 );
        if ( TCS3530_OK == error_flag )
        {
            data_out->x = ( ( uint16_t ) fifo_data[ 1 ] << 8 ) | fifo_data[ 0 ];
            data_out->y = ( ( uint16_t ) fifo_data[ 3 ] << 8 ) | fifo_data[ 2 ];
            data_out->z = ( ( uint16_t ) fifo_data[ 5 ] << 8 ) | fifo_data[ 4 ];
            data_out->ir = ( ( uint16_t ) fifo_data[ 7 ] << 8 ) | fifo_data[ 6 ];
            data_out->hgl = ( ( uint16_t ) fifo_data[ 9 ] << 8 ) | fifo_data[ 8 ];
            data_out->hgh = ( ( uint16_t ) fifo_data[ 11 ] << 8 ) | fifo_data[ 10 ];
            data_out->clear = ( ( uint16_t ) fifo_data[ 13 ] << 8 ) | fifo_data[ 12 ];
            data_out->flicker = ( ( uint16_t ) fifo_data[ 15 ] << 8 ) | fifo_data[ 14 ];
        }
    }
    return error_flag;
}

// ------------------------------------------------------------------------- END
