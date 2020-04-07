/*****************************************************************************************************************
 *
 * Copyright (c) 2018-2020, Hexin Technology Co. Ltd All rights reserved.
 * Author  : Heyn (heyunhuan@gmail.com)
 * Version : V1.4.0
 * Web	   : http://www.hex-in.com
 *
 * ---------------
 *                          2018/10/18 V1.4.0 [Heyn] Release.
 *                          2018/11/14 V1.4.1 [Heyn] Modify TASK_M100() function.
 * 
*****************************************************************************************************************
*/

#include <stdio.h>
#include <stdint.h>

#include "esp_task.h"

#include "py/gc.h"
#include "py/obj.h"
#include "py/nlr.h"
#include "py/mphal.h"
#include "py/mperrno.h"
#include "py/runtime.h"

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"

#include "drivers/magicrf/m100.h"
#include "modm100.h"

/******************************************************************************
 DECLARE PRIVATE DATA
 ******************************************************************************/
static hexin_ring_buffer_t      xRingBuffer;
TaskHandle_t                    xM100ThreadingHandle;
static uart_dev_t*              uart_driver_m100 = &UART1;

static m100_obj_t m100_obj = { .handler     = mp_const_none,
                               .handler_arg = mp_const_none,
                               .init        = false };

const mp_obj_type_t     m100_type;
static uart_config_t    m100_uart_config;
static mp_int_t         uart_port           = 1;

STATIC mp_obj_t __hexin_uart_write( uint8_t *data, uint32_t size )
{
    int bytes_written = uart_write_bytes( uart_port, (char*)(data), size );
    if ( bytes_written < 0 ) {
        return mp_const_false;
    }
    return mp_const_true;
}

static void __callback_event( unsigned char *data, unsigned int size )
{
    MP_THREAD_GIL_EXIT();
    m100_obj.value_len = size;
    MP_THREAD_GIL_ENTER();
}

static void hexin_threading_m100 ( void *pvParameters ) {
    size_t   rxbufsize = 0;
    size_t   rxsize    = 0;
    uint8_t  rxbuffer[HEXIN_M100_BUFFER_MAX_SIZE] = { 0x00 };

    while (1) {
        uart_get_buffered_data_len( uart_port, &rxbufsize );
        if ( 0 == rxbufsize ) {
            vTaskDelay( 10 / portTICK_PERIOD_MS );
            continue;
        }
        rxbufsize = (rxbufsize >= HEXIN_M100_BUFFER_MAX_SIZE ? HEXIN_M100_BUFFER_MAX_SIZE : rxbufsize);
        rxsize = uart_read_bytes( uart_port, rxbuffer, rxbufsize, 0 );
        packetHandler( &xRingBuffer,
                       m100_obj.trigger,
                       rxbuffer,
                       rxsize,
                       __callback_event );
    }
}

STATIC mp_obj_t m100_char_value(mp_obj_t self_in) {
    uint32_t    size = 0;
    uint8_t     data[HEXIN_RING_BUFFER_MAX_SIZE] = { 0x00 };

    hexinRingBufferRead( &xRingBuffer, data, HEXIN_RING_BUFFER_MAX_SIZE, &size );

    if ( 0 == size ) {
        return mp_const_none;
    }

    return mp_obj_new_str( (char *)data, size );
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_m100_value_obj, m100_char_value);

STATIC mp_obj_t mod_m100_rf_power( size_t n_args, const mp_obj_t *args )
{
    uint32_t ret    = 0;
    uint8_t  param[HEXIN_M100_BUFFER_MAX_SIZE] = { 0x00 };
    
    if ( n_args == 1 ) {
        ret = getPaPower( param );
    } else {
        ret = setPaPower( mp_obj_get_float( args[1] ), param );
    }

    return __hexin_uart_write( param, ret );
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN( mod_m100_rf_power_obj, 1, 2, mod_m100_rf_power );

STATIC mp_obj_t mod_m100_query(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_loop,     MP_ARG_INT, {.u_int = 1} },
    };

    uint32_t ret = 0;
    uint8_t  param[HEXIN_M100_BUFFER_MAX_SIZE] = { 0x00 };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(args), allowed_args, args);

    if ( args[0].u_int > 65535 ) {
        mp_raise_ValueError( "invalid argument(s) value" );
    }
    ret = query(args[0].u_int, param);

    return __hexin_uart_write( param, ret );;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW( mod_m100_query_obj, 1, mod_m100_query );

STATIC mp_obj_t mod_m100_mode(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_mode,     MP_ARG_INT, {.u_int = 0} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(args), allowed_args, args);

    uint32_t ret  = 0;
    uint8_t  mode = (unsigned char)args[0].u_int & 0xFF;
    uint8_t  param[HEXIN_M100_BUFFER_MAX_SIZE] = { 0x00 };

    ret = setMode(mode, param);
    return __hexin_uart_write( param, ret );
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mod_m100_mode_obj, 1, mod_m100_mode);

STATIC mp_obj_t mod_m100_trigger(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_trigger,     MP_ARG_INT,  {.u_int = 0} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(args), allowed_args, args);

    m100_obj.trigger = args[0].u_int;

    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mod_m100_trigger_obj, 1, mod_m100_trigger);

STATIC mp_obj_t mod_m100_stop( mp_obj_t self_in ) {
    uint32_t ret = 0;
    uint8_t  param[HEXIN_M100_BUFFER_MAX_SIZE] = { 0x00 };

    ret = stop(param);
    return __hexin_uart_write( param, ret );
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_m100_stop_obj, mod_m100_stop);

STATIC mp_obj_t mod_m100_param(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_select,       MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_session,      MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_target,       MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_q,            MP_ARG_INT, {.u_int = 4} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(args), allowed_args, args);

    uint32_t ret = 0;
    uint8_t  param[HEXIN_M100_BUFFER_MAX_SIZE] = { 0x00 };

    ret = setQueryParam( args[0].u_int, args[1].u_int, args[2].u_int, args[3].u_int, param );

    return __hexin_uart_write( param, ret );
}

STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mod_m100_param_obj, 1, mod_m100_param);

// STATIC const mp_rom_map_elem_t mp_module_magicrf_globals_table[] = {
//     { MP_ROM_QSTR(MP_QSTR___name__),    MP_ROM_QSTR(MP_QSTR_umagicrf) },
//     { MP_ROM_QSTR(MP_QSTR_power),       MP_ROM_PTR(&mod_m100_rf_power_obj) },
// };

STATIC mp_obj_t m100_init_uart_help( m100_obj_t *self, const mp_arg_val_t *args ) {
    enum { ARG_uart_num, ARG_tx, ARG_rx, ARG_baudrate, ARG_priority, ARG_affinity };

    mp_int_t uart_num = args[ARG_uart_num].u_int;

    if ( uart_num < 0 || uart_num >= UART_NUM_MAX ) {
        mp_raise_ValueError("UART does not exist\r\n");
    }

    if ( uart_num == UART_NUM_0 ) {
        mp_raise_ValueError("UART(0) is disabled (dedicated to REPL)\r\n");
    }
    uart_port = uart_num;
    uart_driver_delete( uart_num );

    m100_uart_config.baud_rate = args[ARG_baudrate].u_int;
    m100_uart_config.data_bits = UART_DATA_8_BITS;
    m100_uart_config.parity    = UART_PARITY_DISABLE;
    m100_uart_config.stop_bits = UART_STOP_BITS_1;
    m100_uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    m100_uart_config.rx_flow_ctrl_thresh = 64;

    uart_param_config( uart_num, &m100_uart_config );
    uart_set_pin( uart_num, args[ARG_tx].u_int, args[ARG_rx].u_int, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE );
    uart_driver_install( uart_num, 2048, 2048, 0, NULL, 0 );

    uart_driver_m100->idle_conf.tx_idle_num = 0;
    uart_driver_m100->conf1.rx_tout_thrhd = 10 & UART_RX_TOUT_THRHD_V;

    UBaseType_t priority = args[ARG_priority].u_int;
    BaseType_t  affinity = args[ARG_affinity].u_int == 0 ? 0 : 1;

    if ( priority >= 11 ) {
        mp_printf( &mp_plat_print, "< WARNNING> Task priority %d >= INTERRUPTS_TASK_PRIORITY, will be occur error.\n", priority );
    }

    BaseType_t result = xTaskCreatePinnedToCore( hexin_threading_m100,
                                                 "Hexin M100 Module",
                                                 (1024*5) / sizeof(StackType_t),
                                                 NULL,
                                                 priority,
                                                 &xM100ThreadingHandle,
                                                 affinity );

    if ( result != pdPASS ) {
        mp_raise_msg( &mp_type_OSError, "can't create thread" );
    }

    return mp_const_none;
}

STATIC mp_obj_t m100_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    enum { ARG_uart_num, ARG_tx, ARG_rx, ARG_baudrate, ARG_priority, ARG_affinity };
    static const mp_arg_t m100_uart_init_args[] = {
        { MP_QSTR_uart_num,  MP_ARG_INT,                    {.u_int = 1} },
        { MP_QSTR_tx,        MP_ARG_KW_ONLY | MP_ARG_INT,   {.u_int = UART_PIN_NO_CHANGE} },
        { MP_QSTR_rx,        MP_ARG_KW_ONLY | MP_ARG_INT,   {.u_int = UART_PIN_NO_CHANGE} },
        { MP_QSTR_baudrate,  MP_ARG_KW_ONLY | MP_ARG_INT,   {.u_int = 115200} },
        { MP_QSTR_priority,  MP_ARG_KW_ONLY | MP_ARG_INT,   {.u_int = (ESP_TASK_PRIO_MIN + 1)} },
        { MP_QSTR_affinity,  MP_ARG_KW_ONLY | MP_ARG_INT,   {.u_int = MP_TASK_COREID} },
    };

    
    mp_map_t kw_args;
    mp_map_init_fixed_table(&kw_args, n_kw, all_args + n_args);
    mp_arg_val_t args[MP_ARRAY_SIZE(m100_uart_init_args)];
    mp_arg_parse_all(n_args, all_args, &kw_args, MP_ARRAY_SIZE(args), m100_uart_init_args, args);

    m100_obj_t *self = (m100_obj_t *)&m100_obj;

    if ( false == self->init ) {
        self->base.type   = &m100_type;
        self->init        = true;
        self->trigger     = HEXIN_MAGICRF_QUERY;
        hexinRingBufferInit( &xRingBuffer, self->value, HEXIN_RING_BUFFER_MAX_SIZE );
        m100_init_uart_help( self, &args[0] );
    } else {
        mp_printf( &mp_plat_print, "M100 module already initialized.\r\n" );
    }
    return (mp_obj_t)self;
}

STATIC const mp_rom_map_elem_t m100_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_value),                       MP_ROM_PTR(&mod_m100_value_obj)         },

    { MP_ROM_QSTR(MP_QSTR_mode),                        MP_ROM_PTR(&mod_m100_mode_obj)          },
    { MP_ROM_QSTR(MP_QSTR_trigger),                     MP_ROM_PTR(&mod_m100_trigger_obj)       },
    { MP_ROM_QSTR(MP_QSTR_power),                       MP_ROM_PTR(&mod_m100_rf_power_obj)      },
    { MP_ROM_QSTR(MP_QSTR_query),                       MP_ROM_PTR(&mod_m100_query_obj)         },
    { MP_ROM_QSTR(MP_QSTR_stop),                        MP_ROM_PTR(&mod_m100_stop_obj)          },
    { MP_ROM_QSTR(MP_QSTR_param),                       MP_ROM_PTR(&mod_m100_param_obj)         },

    { MP_ROM_QSTR(MP_QSTR_PARAM_SELECT_ALL),            MP_OBJ_NEW_SMALL_INT(0)  },
    { MP_ROM_QSTR(MP_QSTR_PARAM_SELECT_NSL),            MP_OBJ_NEW_SMALL_INT(2)  },
    { MP_ROM_QSTR(MP_QSTR_PARAM_SELECT_SL),             MP_OBJ_NEW_SMALL_INT(3)  },

    { MP_ROM_QSTR(MP_QSTR_PARAM_SESSION_S0),            MP_OBJ_NEW_SMALL_INT(0)  },
    { MP_ROM_QSTR(MP_QSTR_PARAM_SESSION_S1),            MP_OBJ_NEW_SMALL_INT(1)  },
    { MP_ROM_QSTR(MP_QSTR_PARAM_SESSION_S2),            MP_OBJ_NEW_SMALL_INT(2)  },
    { MP_ROM_QSTR(MP_QSTR_PARAM_SESSION_S3),            MP_OBJ_NEW_SMALL_INT(3)  },

    { MP_ROM_QSTR(MP_QSTR_PARAM_TARGET_A),              MP_OBJ_NEW_SMALL_INT(0)  },
    { MP_ROM_QSTR(MP_QSTR_PARAM_TARGET_B),              MP_OBJ_NEW_SMALL_INT(1)  },

    { MP_ROM_QSTR(MP_QSTR_TRIGGER_QUERY),               MP_OBJ_NEW_SMALL_INT(HEXIN_MAGICRF_QUERY) },
    { MP_ROM_QSTR(MP_QSTR_TRIGGER_PA_POWER),            MP_OBJ_NEW_SMALL_INT(HEXIN_MAGICRF_GET_RF_POWER) },
    { MP_ROM_QSTR(MP_QSTR_TRIGGER_STOP),                MP_OBJ_NEW_SMALL_INT(HEXIN_MAGICRF_STOP) },
    { MP_ROM_QSTR(MP_QSTR_TRIGGER_RSSI),                MP_OBJ_NEW_SMALL_INT(HEXIN_MAGICRF_TEST_RSSI) },
    
    { MP_ROM_QSTR(MP_QSTR_BANK_RFU),                    MP_OBJ_NEW_SMALL_INT(BANK_RFU)  },
    { MP_ROM_QSTR(MP_QSTR_BANK_EPC),                    MP_OBJ_NEW_SMALL_INT(BANK_EPC)  },
    { MP_ROM_QSTR(MP_QSTR_BANK_TID),                    MP_OBJ_NEW_SMALL_INT(BANK_TID)  },
    { MP_ROM_QSTR(MP_QSTR_BANK_USER),                   MP_OBJ_NEW_SMALL_INT(BANK_USER) },

    { MP_ROM_QSTR(MP_QSTR_MODE_HIGH_SENSITIVITY),       MP_OBJ_NEW_SMALL_INT(0)         },
    { MP_ROM_QSTR(MP_QSTR_MODE_DENSE_READER),           MP_OBJ_NEW_SMALL_INT(1)         },

    { MP_ROM_QSTR(MP_QSTR_HFSS_AUTO),                   MP_OBJ_NEW_SMALL_INT(0xFF)      },
    { MP_ROM_QSTR(MP_QSTR_HFSS_STOP),                   MP_OBJ_NEW_SMALL_INT(0x00)      },


};

MP_DEFINE_CONST_DICT(m100_locals_dict, m100_locals_dict_table);

const mp_obj_type_t m100_type = {
    { &mp_type_type },
    .name = MP_QSTR_m100,
    .make_new = m100_make_new,
    .locals_dict = (mp_obj_dict_t *)&m100_locals_dict,
};

STATIC const mp_rom_map_elem_t mp_module_magicrf_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),    MP_ROM_QSTR(MP_QSTR_umagicrf) },
    { MP_ROM_QSTR(MP_QSTR_m100),        (mp_obj_t)&m100_type },
};

STATIC MP_DEFINE_CONST_DICT( mp_module_magicrf_globals, mp_module_magicrf_globals_table );

const mp_obj_module_t mp_module_magicrf = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&mp_module_magicrf_globals,
};

/****************************************************************************************************************/
/*

from magicrf import m100
reader = m100( 1, rx=36, tx=33, priority=7, affinity=0 )

reader.power( 22.0 )    # Set reader power value.

reader.trigger( m100.TRIGGER_PA_POWER )
reader.power( )         # Get reader power value.
reader.value( )

reader.trigger( m100.TRIGGER_QUERY )
reader.query(100)
reader.value( )

reader.mode( m100.MODE_HIGH_SENSITIVITY )
reader.mode( m100.MODE_DENSE_READER )

reader.param( q=5 )
reader.stop( )

*/
