# !/usr/bin/micropython
# -*- coding:utf-8 -*-
""" MicroPython queue."""
# Python:   3.6.5+
# Platform: MicroPython
# Author:   Heyn (heyunhuan@gmail.com)
# Program:  MicroPython queue.
# History:  2020-04-08 Ver:1.0 [Heyn] Initialization

from uasyncio import queues

class Queue( object ):
    def __init__( self, maxsize=1 ):
        self.__q = queues.Queue( maxsize=maxsize )
    
    def __val( self, gen ):
        while True:
            try:
                gen.send(None)
            except StopIteration as err:
                return err.value

    def put( self, vaule ):
        self.__val( self.__q.put( vaule ) )

    def put_nowait( self, value ):
        self.__val( self.__q.put_nowait( vaule ) )

    def get( self ):
        return self.__val( self.__q.get( ) )

    def get_nowait( self ):
        return self.__val( self.__q.get_nowait( ) )

    def qsize( self ):
        return self.__q.qsize( )

    def empty( self ):
        return self.__q.empty()

    def full( self ):
        return self.__q.full()