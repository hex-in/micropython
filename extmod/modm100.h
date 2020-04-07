/*****************************************************************************************************************
 *
 * Copyright (c) 2018-2020, Hexin Technology Co. Ltd All rights reserved.
 * Author  : Heyn (heyunhuan@gmail.com)
 * Version : V1.5.0
 * Web	   : http://www.hex-in.com
 *
 * LICENSING TERMS:
 * ---------------
 *                          2018/10/18 V1.4.0 [Heyn] Release.
 * 
*****************************************************************************************************************/

#ifndef __MODM100_H_
#define	__MODM100_H_

#define HEXIN_RING_BUFFER_MAX_SIZE          (1024)

typedef struct {
    mp_obj_base_t       base;
    mp_obj_t            handler;
    mp_obj_t            handler_arg;
    bool                init;
    unsigned int        trigger;
    unsigned int        value_len;
    unsigned char       command;
    unsigned char       errorcode;
    unsigned char       value[HEXIN_RING_BUFFER_MAX_SIZE];
} m100_obj_t;


#endif /* __MODM100_H_ */
