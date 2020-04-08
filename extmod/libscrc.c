/*****************************************************************************************************************
 *
 * Copyright (c) 2018-2020, Hexin Technology Co. Ltd All rights reserved.
 * Author  : Heyn (heyunhuan@gmail.com)
 * Version : V1.0
 * Web	   : Fork : https://github.com/hex-in/libscrc.git
 *
 * LICENSING TERMS:
 * ---------------
 *                          2020/04/07 V1.0 [Heyn] Release.
 * 
*****************************************************************************************************************/

#include <stdio.h>
#include <stdint.h>

#include "py/gc.h"
#include "py/obj.h"
#include "py/nlr.h"
#include "py/mphal.h"
#include "py/mperrno.h"
#include "py/runtime.h"

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"

#include "libscrc.h"

/******************************************************************************
 DECLARE PRIVATE DATA
 ******************************************************************************/
static unsigned short   crc16_table_1021[MAX_TABLE_ARRAY]   = { 0x0000 };
static unsigned int     crc16_table_1021_init               = FALSE;

unsigned char hexin_calc_crc8_lrc( const unsigned char *pSrc, unsigned int len, unsigned char crc8 ) 
{
    unsigned int i = 0;
    unsigned char crc = crc8;

	for ( i=0; i<len; i++ ) {
		crc += pSrc[i];
	}
    crc = (~crc) + 0x01;

	return crc;
}

unsigned int hexin_crc16_init_table_poly_is_low( unsigned short polynomial, unsigned short *table )
{
    unsigned int i = 0, j = 0;
    unsigned short crc = 0, c = 0;

    for ( i=0; i<MAX_TABLE_ARRAY; i++ ) {
        crc = 0;
        c   = ((unsigned short) i) << 8;
        for ( j=0; j<8; j++ ) {
            if ( (crc ^ c) & 0x8000 ) crc = ( crc << 1 ) ^ polynomial;
            else                      crc = crc << 1;
            c = c << 1;
        }
        table[i] = crc;
    }
    return TRUE;
}

unsigned short hexin_crc16_poly_is_low_calc( unsigned short crc16, unsigned char c, const unsigned short *table )
{
    unsigned short crc = crc16;
    unsigned short tmp, short_c;

    short_c  = 0x00FF & (unsigned short) c;
    tmp = (crc >> 8) ^ short_c;
    crc = (crc << 8) ^ table[tmp];

    return crc;
}

unsigned short hexin_calc_crc16_1021( const unsigned char *pSrc, unsigned int len, unsigned short crc16 )
{
    unsigned int i = 0;
    unsigned short crc = crc16;

    if ( crc16_table_1021_init == FALSE ) {
        crc16_table_1021_init = hexin_crc16_init_table_poly_is_low( CRC16_POLYNOMIAL_1021, crc16_table_1021 );
    }

	for ( i=0; i<len; i++ ) {
		crc = hexin_crc16_poly_is_low_calc( crc, pSrc[i], crc16_table_1021 );
	}
	return crc;
}

STATIC void __hexin_buf_get_for_crc( mp_obj_t o, mp_buffer_info_t *bufinfo, byte *tmp_data ) {
    if ( mp_obj_is_int( o ) ) {
        tmp_data[0] = mp_obj_get_int(o);
        bufinfo->buf = tmp_data;
        bufinfo->len = 1;
        bufinfo->typecode = 'B';
    } else {
        mp_get_buffer_raise( o, bufinfo, MP_BUFFER_READ );
    }
}

/****************************************************************************************************************/
STATIC mp_obj_t mod_libscrc_lrc ( mp_obj_t buf ) {

    mp_buffer_info_t bufinfo;
    uint8_t data[1];
    __hexin_buf_get_for_crc( buf, &bufinfo, data );

    unsigned char result = hexin_calc_crc8_lrc( (unsigned char *)bufinfo.buf, bufinfo.len, 0x00 );

    return mp_obj_new_int( result );
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1( mod_libscrc_lrc_obj, mod_libscrc_lrc );


STATIC mp_obj_t mod_libscrc_epc ( mp_obj_t buf ) {

    mp_buffer_info_t bufinfo;
    uint8_t data[1];
    __hexin_buf_get_for_crc( buf, &bufinfo, data );

    unsigned short result = hexin_calc_crc16_1021( (unsigned char *)bufinfo.buf, bufinfo.len, 0xFFFF );

    return mp_obj_new_int( result ^ 0xFFFF );
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1( mod_libscrc_epc_obj, mod_libscrc_epc );


STATIC const mp_rom_map_elem_t mp_module_libscrc_globals_table[] = {
    { MP_ROM_QSTR( MP_QSTR___name__ ),      MP_ROM_QSTR( MP_QSTR_libscrc )              },
    { MP_ROM_QSTR( MP_QSTR_lrc ),           MP_ROM_PTR( &mod_libscrc_lrc_obj )          },
    { MP_ROM_QSTR( MP_QSTR_epc16 ),         MP_ROM_PTR( &mod_libscrc_epc_obj )          },
};

STATIC MP_DEFINE_CONST_DICT( mp_module_libscrc_globals, mp_module_libscrc_globals_table );

const mp_obj_module_t mp_module_libscrc = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&mp_module_libscrc_globals,
};

/****************************************************************************************************************/
/*

>>> import libscrc
>>> libscrc.epc16(b'1234')
44214
>>> libscrc.epc16(b'123456789')
54862
>>> libscrc.epc16(b'123456')
53515
>>> libscrc.lrc(b'1234')
54

*/
