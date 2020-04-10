# !/usr/bin/micropython
# -*- coding:utf-8 -*-
""" Impinj R2000 driver for MicroPython on ESP32."""
# Python:   3.6.5+
# Platform: MicroPython
# Author:   Heyn (heyunhuan@gmail.com)
# Program:  Impinj R2000 module protocol.
# History:  2020-04-08 Ver:1.0 [Heyn] Initialization


import struct
import libscrc
import logging

from queue import Queue

TAG_MEMORY_BANK = { 'RESERVED' : 0,
                    'EPC'      : 1,
                    'TID'      : 2,
                    'USER'     : 3 }

READER_ANTENNA = {  'ANTENNA1' : 0,
                    'ANTENNA2' : 1,
                    'ANTENNA3' : 2,
                    'ANTENNA4' : 3,
                    'MAX'      : 4 }

FREQUENCY_TABLES = [ 865+(x*0.5) for x in range( 7 ) ] + [ 902+(x*0.5) for x in range( 53 ) ]

class ImpinjR2KFastSwitchInventory( object ):
    ANTENNA1 = 0
    ANTENNA2 = 1
    ANTENNA3 = 2
    ANTENNA4 = 3
    DISABLED = 0xFF

class ImpinjR2KCommands( object ):

    GET_GPIO_VALUE              = 0x60
    SET_GPIO_VALUE              = 0x61
    SET_ANT_CONNECTION_DETECTOR = 0x62
    GET_ANT_CONNECTION_DETECTOR = 0x63
    SET_TEMPORARY_OUTPUT_POWER  = 0x66
    SET_READER_IDENTIFIER       = 0x67
    GET_READER_IDENTIFIER       = 0x68
    SET_RF_LINK_PROFILE         = 0x69
    GET_RF_LINK_PROFILE         = 0x6A

    RESET                       = 0x70
    SET_UART_BAUDRATE           = 0x71
    GET_FIRMWARE_VERSION        = 0x72
    SET_READER_ADDRESS          = 0x73
    SET_WORK_ANTENNA            = 0x74
    GET_WORK_ANTENNA            = 0x75
    SET_RF_POWER                = 0x76
    GET_RF_POWER                = 0x77
    SET_FREQUENCY_REGION        = 0x78
    GET_FREQUENCY_REGION        = 0x79
    SET_BEEPER_MODE             = 0x7A
    GET_READER_TEMPERATURE      = 0x7B
    GET_RF_PORT_RETURN_LOSS     = 0x7E

    ### 18000-6C
    INVENTORY                   = 0x80
    READ                        = 0x81
    WRITE                       = 0x82
    LOCK                        = 0x83
    KILL                        = 0x84
    SET_ACCESS_EPC_MATCH        = 0x85
    GET_ACCESS_EPC_MATCH        = 0x86

    REAL_TIME_INVENTORY         = 0x89
    FAST_SWITCH_ANT_INVENTORY   = 0x8A
    CUSTOMIZED_SESSION_TARGET_INVENTORY = 0x8B
    SET_IMPINJ_FAST_TID         = 0x8C
    SET_AND_SAVE_IMPINJ_FAST_TID= 0x8D
    GET_IMPINJ_FAST_TID         = 0x8E

    ### ISO18000-6B
    ISO18000_6B_INVENTORY       = 0xB0
    ISO18000_6B_READ            = 0xB1
    ISO18000_6B_WRITE           = 0xB2
    ISO18000_6B_LOCK            = 0xB3
    ISO18000_6B_QUERY_LOCK      = 0xB4

    GET_INVENTORY_BUFFER            = 0x90
    GET_AND_RESET_INVENTORY_BUFFER  = 0x91
    GET_INVENTORY_BUFFER_TAG_COUNT  = 0x92
    RESET_INVENTORY_BUFFER          = 0x93

    WRITE_BLOCK                     = 0x94

class ImpinjR2KGlobalErrors( object ):
    SUCCESS                     = 0x10
    FAIL                        = 0x11

    MCU_RESET_ERROR             = 0x20
    CW_ON_ERROR                 = 0x21
    ANTENNA_MISSING_ERROR       = 0x22
    WRITE_FLASH_ERROR           = 0x23
    READ_FLASH_ERROR            = 0x24
    SET_OUTPUT_POWER_ERROR      = 0x25

    TAG_INVENTORY_ERROR         = 0x31
    TAG_READ_ERROR              = 0x32
    TAG_WRITE_ERROR             = 0x33
    TAG_LOCK_ERROR              = 0x34
    TAG_KILL_ERROR              = 0x35
    NO_TAG_ERROR                = 0x36
    INVENTORY_OK_BUT_ACCESS_FAIL= 0x37
    BUFFER_IS_EMPTY_ERROR       = 0x38
    NXP_CUSTOM_COMMAND_FAIL     = 0x3C

    ACCESS_OR_PASSWORD_ERROR                            = 0x40
    PARAMETER_INVALID                                   = 0x41
    PARAMETER_INVALID_WORDCNT_TOO_LONG                  = 0x42
    PARAMETER_INVALID_MEMBANK_OUT_OF_RANGE              = 0x43
    PARAMETER_INVALID_LOCK_REGION_OUT_OF_RANGE          = 0x44
    PARAMETER_INVALID_LOCK_ACTION_OUT_OF_RANGE          = 0x45
    PARAMETER_READER_ADDRESS_INVALID                    = 0x46
    PARAMETER_INVALID_ANTENNA_ID_OUT_OF_RANGE           = 0x47
    PARAMETER_INVALID_OUTPUT_POWER_OUT_OF_RANGE         = 0x48
    PARAMETER_INVALID_FREQUENCY_REGION_OUT_OF_RANGE     = 0x49
    PARAMETER_INVALID_BAUDRATE_OUT_OF_RANGE             = 0x4A
    PARAMETER_BEEPER_MODE_OUT_OF_RANGE                  = 0x4B
    PARAMETER_EPC_MATCH_LEN_TOO_LONG                    = 0x4C
    PARAMETER_EPC_MATCH_LEN_ERROR                       = 0x4D
    PARAMETER_INVALID_EPC_MATCH_MODE                    = 0x4E
    PARAMETER_INVALID_FREQUENCY_RANGE                   = 0x4F

    FAIL_TO_GET_RN16_FROM_TAG                           = 0x50
    PARAMETER_INVALID_DRM_MODE                          = 0x51
    PLL_LOCK_FAIL                                       = 0x52
    RF_CHIP_FAIL_TO_RESPONSE                            = 0x53
    FAIL_TO_ACHIEVE_DESIRED_OUTPUT_POWER                = 0x54
    COPYRIGHT_AUTHENTICATION_FAIL                       = 0x55
    SPECTRUM_REGULATION_ERROR                           = 0x56
    OUTPUT_POWER_TOO_LOW                                = 0x57
    FAIL_TO_GET_RF_PORT_RETURN_LOSS                     = 0xEE

    @classmethod
    def to_string( cls, error_code ):
        if error_code == cls.MCU_RESET_ERROR:
            return 'MCU reset error.'
        elif error_code == cls.WRITE_FLASH_ERROR:
            return 'Write flash error.'
        
        ### 0x2*
        elif error_code == cls.ANTENNA_MISSING_ERROR:
            return 'Antenna miss error.'
        elif error_code == cls.SET_OUTPUT_POWER_ERROR:
            return 'Set output power error.'

        ### 0x3*
        elif error_code == cls.TAG_INVENTORY_ERROR:
            return 'Tag inventory error'
        elif error_code == cls.TAG_READ_ERROR:
            return 'Tag read error'
        elif error_code == cls.TAG_WRITE_ERROR:
            return 'Tag write error'    
        elif error_code == cls.TAG_LOCK_ERROR:
            return 'Tag lock error'
        elif error_code == cls.TAG_KILL_ERROR:
            return 'Tag kill error'
        elif error_code == cls.NO_TAG_ERROR:
            return 'No tag error'
        elif error_code == cls.INVENTORY_OK_BUT_ACCESS_FAIL:
            return 'Inventory is ok, but access failed.'
        elif error_code == cls.BUFFER_IS_EMPTY_ERROR:
            return 'Buffer is empty.'
        elif error_code == cls.NXP_CUSTOM_COMMAND_FAIL:
            return 'NXP command failed.'

        return 'SUCCESS'

class ImpinjR2KMemoryBank( object ):
    RESERVED = 0
    EPC      = 1
    TID      = 2
    USER     = 3

class ImpinjR2KRegion( object ):
    FCC  = 1
    ETSI = 2
    CHN  = 3
    USER = 4

class ImpinjR2KRFLinkProfile( object ):
    PROFILE0 = 0xD0 # Tari 25uS,FM0 40KHz
    PROFILE1 = 0xD1 # Tari 25uS,Miller 4 250KHz ( Default )
    PROFILE2 = 0xD2 # Tari 25uS,Miller 4 300KHz
    PROFILE3 = 0xD3 # Tari 6.25uS,FM0 400KHz

class ImpinjR2KProtocols( object ):
    """
        R2000 = ImpinjR2KProtocols( )
        print( R2000.reset( ) )
        print( R2000.power( 20, 21, 20, 20 ) )
        print( R2000.work_antenna(0) )

    """
    def register( command ):
        def decorator( func ):
            def wrapper( self, *args, **kwargs ):
                data = func( self, *args, **kwargs )
                data = [] if data is None else data
                message = [ self.__head ]
                message.extend( [ 3 + len( data ), self.__address, command ] )
                message.extend(  data   )
                message.append( libscrc.lrc( bytes( message ) ) )
                self.__address = data[0] if command == ImpinjR2KCommands.SET_READER_ADDRESS else self.__address
                logging.debug( [ hex(x) for x in message ] )

                if self.serial is not None:
                    try:
                        return self.serial.write( bytes( message ) )
                    except BaseException as err:
                        logging.error( err )
                return bytes( message )
            return wrapper
        return decorator

    def __init__( self, address=0xFF, serial=None ):
        self.serial = serial
        self.__head, self.__address = 0xA0, address

    @register( ImpinjR2KCommands.RESET )
    def reset( self ):
        pass

    @register( ImpinjR2KCommands.SET_UART_BAUDRATE )
    def baudrate( self, value=115200 ):
        """
            38400bps or 115200bps.
        """
        return [ 4 if value == 115200 else 3 ]

    @register( ImpinjR2KCommands.SET_READER_ADDRESS )
    def address( self, addr=0 ):
        assert ( 0 <= addr <= 254 )
        return [ addr ]

    @register( ImpinjR2KCommands.GET_FIRMWARE_VERSION )
    def version( self ):
        pass

    @register( ImpinjR2KCommands.SET_WORK_ANTENNA )
    def set_work_antenna( self, antenna=READER_ANTENNA['ANTENNA1'] ):
        """ Set reader work antenna.
            @param      antenna(int) : 0 ~ 3
        """
        assert ( 0 <= antenna <= 3 )
        return [ antenna ]

    @register( ImpinjR2KCommands.GET_WORK_ANTENNA )
    def get_work_antenna( self ):
        pass

    @register( ImpinjR2KCommands.SET_RF_POWER )
    def set_rf_power( self, ant1=0x00, ant2=0x00, ant3=0x00, ant4=0x00 ):
        assert ( 0 <= ant1 <= 33 ) and ( 0 <= ant2 <= 33 ) and ( 0 <= ant3 <= 33 ) and ( 0 <= ant4 <= 33 )
        return [ ant1, ant2, ant3, ant4 ]

    @register( ImpinjR2KCommands.GET_RF_POWER )
    def get_rf_power( self ):
        pass

    @register( ImpinjR2KCommands.SET_TEMPORARY_OUTPUT_POWER )
    def fast_power( self, value=22 ):
        assert ( 22 <= value <= 33 )
        return [ value ]

    @register( ImpinjR2KCommands.SET_BEEPER_MODE )
    def beeper( self, mode=0 ):
        assert ( 0 <= mode <= 2 )
        return [ mode ]

    @register( ImpinjR2KCommands.GET_ANT_CONNECTION_DETECTOR )
    def get_ant_connection_detector( self ):
        pass

    @register( ImpinjR2KCommands.SET_ANT_CONNECTION_DETECTOR )
    def set_ant_connection_detector( self, loss=0 ):
        """
            @param  loss = 0 # Disabled detector.
                    loss (Unit:dB) 
                    (The higher the value, the higher the impedance matching requirements for the port)
        """
        return [ loss ]

    @register( ImpinjR2KCommands.SET_READER_IDENTIFIER )
    def set_reader_identifier( self, sn='0123456789AB' ):
        data = [ ord(x) for x in list( sn[0:12] ) ]
        data.extend( [ 0xFF ]*( 12 - len(data) ) )
        return data

    @register( ImpinjR2KCommands.GET_READER_IDENTIFIER )
    def get_reader_identifier( self ):
        pass

    @register( ImpinjR2KCommands.INVENTORY )
    def inventory( self, repeat=0xFF ):
        return [ repeat ]

    @register( ImpinjR2KCommands.GET_INVENTORY_BUFFER )
    def get_inventory_buffer( self ):
        pass

    @register( ImpinjR2KCommands.GET_INVENTORY_BUFFER_TAG_COUNT )
    def get_inventory_buffer_tag_count( self ):
        pass
    
    @register( ImpinjR2KCommands.GET_AND_RESET_INVENTORY_BUFFER )
    def get_and_reset_inventory_buffer( self ):
        pass

    @register( ImpinjR2KCommands.RESET_INVENTORY_BUFFER )  
    def reset_inventory_buffer( self ):
        pass

    @register( ImpinjR2KCommands.REAL_TIME_INVENTORY )
    def rt_inventory( self, repeat=0xFF ):
        return [ repeat ]

    @register( ImpinjR2KCommands.CUSTOMIZED_SESSION_TARGET_INVENTORY )
    def session_inventory( self, session='S1', target='A', repeat=1 ):
        sess = dict( S0=0, S1=1, S2=2, S3=3 )
        return [ sess.get( session, 1 ), 1 if target == 'B' else 0, repeat ]

    @register( ImpinjR2KCommands.FAST_SWITCH_ANT_INVENTORY )
    def fast_switch_ant_inventory( self, param = dict( A=ImpinjR2KFastSwitchInventory.ANTENNA1, Aloop=1,
                                                       B=ImpinjR2KFastSwitchInventory.DISABLED, Bloop=1,
                                                       C=ImpinjR2KFastSwitchInventory.DISABLED, Cloop=1,
                                                       D=ImpinjR2KFastSwitchInventory.DISABLED, Dloop=1,
                                                       Interval = 0,
                                                       Repeat   = 1 ) ):
        """
            Interval : ( Unit : ms  )
            Repeat   : ( Uint : int )

            param = dict( A=ImpinjR2KFastSwitchInventory.ANTENNA1, Aloop=1,
                          B=ImpinjR2KFastSwitchInventory.ANTENNA2, Bloop=1,
                          C=ImpinjR2KFastSwitchInventory.ANTENNA3, Cloop=1,
                          D=ImpinjR2KFastSwitchInventory.DISABLED, Dloop=1,
                          Interval = 5,
                          Repeat   = 1 )
        """
        return [ param.get( 'A', ImpinjR2KFastSwitchInventory.DISABLED ), param.get( 'Aloop', 1 ),
                 param.get( 'B', ImpinjR2KFastSwitchInventory.DISABLED ), param.get( 'Bloop', 1 ),
                 param.get( 'C', ImpinjR2KFastSwitchInventory.DISABLED ), param.get( 'Cloop', 1 ),
                 param.get( 'D', ImpinjR2KFastSwitchInventory.DISABLED ), param.get( 'Dloop', 1 ),
                 param.get( 'Interval', 5 ),
                 param.get( 'Repeat',   1 ) ]

    def gpio( self, port, level=False ):
        """
            ONLY [R] : GPIO1 and GPIO2
            ONLY [W] : GPIO3 and GPIO4

            e.g:
                R2000 = ImpinjR2KProtocols( )
                print( R2000.gpio( 1 ) )
        """
        assert ( 1 <= port <= 4 )
        if 3 <= port <= 4:
            ret = ImpinjR2KProtocols.register( ImpinjR2KCommands.SET_GPIO_VALUE )( lambda x, y : y )( self, [ port, 0x01 if level else 0x00 ] )
        else:
            ret = ImpinjR2KProtocols.register( ImpinjR2KCommands.GET_GPIO_VALUE )( lambda x, y : y )( self, [  ] )
        return ret

    @register( ImpinjR2KCommands.GET_READER_TEMPERATURE )
    def temperature( self ):
        pass

    @register( ImpinjR2KCommands.READ )
    def read( self, bank='EPC', addr=0, size=2, password=[ 0 ]*4 ):
        body = []
        body.extend( [ TAG_MEMORY_BANK.get( bank, 1 ), addr, size ] )
        body.extend( password )
        return body

    @register( ImpinjR2KCommands.WRITE )
    def write( self, data:list, bank='EPC', addr=0, password=[ 0 ]*4 ):
        body = []
        body.extend( password )
        body.append( TAG_MEMORY_BANK.get( bank, 1 ) )
        body.append( 2 if ( (bank == 'EPC') and (addr == 0) ) else addr )
        body.append( len(data)//2  )
        body.extend( data )
        return body

    @register( ImpinjR2KCommands.WRITE_BLOCK )
    def write_block( self, data:list, bank='EPC', addr=0, password=[ 0 ]*4 ):
        body = []
        body.extend( password )
        body.append( TAG_MEMORY_BANK.get( bank, 1 ) )
        body.append( 2 if ( (bank == 'EPC') and (addr == 0) ) else addr )
        body.append( len(data)//2 )
        body.extend( data )
        return body

    @register( ImpinjR2KCommands.LOCK )
    def lock( self, bank='EPC', lock_type='OPEN', password=[ 0 ]*4 ):
        """
            @param
                bank      = [ 'USER', 'TID', 'EPC', 'ACCESS_PASSWORD', 'KILL_PASSWORD' ]
                lock_type = [ 'OPEN', 'LOCK', 'OPEN_FOREVER', 'LOCK_FOREVER' ]
        """
        assert ( type( password ) is list )
        membank  = dict( USER=1, TID=2, EPC=3, ACCESS_PASSWORD=4, KILL_PASSWORD=5 )
        locktype = dict( OPEN=0, LOCK=1, OPEN_FOREVER=2, LOCK_FOREVER=3 )
        body = []
        body.extend( password )
        body.append( membank.get( bank, 1 ) )
        body.append( locktype.get( lock_type, 1 ) )
        return body

    @register( ImpinjR2KCommands.KILL )
    def kill( self, password=[ 0 ]*4 ):
        assert ( type( password ) is list )
        return password

    @register( ImpinjR2KCommands.SET_ACCESS_EPC_MATCH )
    def set_access_epc_match( self, mode, epc:list ):
        assert( mode in ( 0, 1 ) )
        body = [ mode, len(epc) ]
        body.extend( epc )
        return body

    @register( ImpinjR2KCommands.GET_ACCESS_EPC_MATCH )
    def get_access_epc_match( self ):
        pass

    @register( ImpinjR2KCommands.GET_RF_PORT_RETURN_LOSS )
    def get_rf_port_return_loss( self, param ):
        return [ param ]

    @register( ImpinjR2KCommands.SET_FREQUENCY_REGION )
    def set_frequency_region( self, region, start, stop ):
        return [ region, start, stop ]

    @register( ImpinjR2KCommands.GET_FREQUENCY_REGION )
    def get_frequency_region( self ):
        pass

    @register( ImpinjR2KCommands.SET_FREQUENCY_REGION )
    def set_frequency_region_user( self, start, space, quantity ):
        """
            start : e.g. 915000KHz --> 0D F6 38 (unit KHz)
            space : space*10 (unit KHz)
            quantity : must be above 0
        """
        assert( quantity > 0 )
        body = [ ImpinjR2KRegion.USER, space*10, quantity ]
        body.append( ( ( start & 0x00FF0000 ) >> 16 ) & 0x000000FF )
        body.append( ( ( start & 0x0000FF00 ) >>  8 ) & 0x000000FF )
        body.append( ( ( start & 0x000000FF ) >>  0 ) & 0x000000FF )
        return body

    @register( ImpinjR2KCommands.SET_RF_LINK_PROFILE )
    def set_rf_link_profile( self, profile_id ):
        return [ profile_id ]

    @register( ImpinjR2KCommands.GET_RF_LINK_PROFILE )
    def get_rf_link_profile( self ):
        pass

    @register( ImpinjR2KCommands.ISO18000_6B_INVENTORY )
    def iso1800_6b_inventory( self ):
        """ ISO 18000 - 6B """
        pass

import utime
import _thread
from machine import UART

def hexin_threading_uart_received( address, uart, command_queue, package_queue ):

    packet    = bytearray()
    in_packet = False

    def hexin_handle_packet( packet, command_queue, package_queue ):
        logging.debug( 'hexin_handle_packet = {}'.format( packet ) )
        try:
            length, command, message = packet[1], packet[3], packet[4:-1]
        except BaseException as err:
            logging.error( '[ERROR] ImpinjProtocolFactory.handle_packet : {}'.format( err ) )
            return
        ### Tags 
        if command in [ ImpinjR2KCommands.REAL_TIME_INVENTORY, ImpinjR2KCommands.ISO18000_6B_INVENTORY,
                        ImpinjR2KCommands.FAST_SWITCH_ANT_INVENTORY, ImpinjR2KCommands.CUSTOMIZED_SESSION_TARGET_INVENTORY ]:
            
            if len( message ) <= 1:
                package_queue.put( dict( type='ERROR', logs=ImpinjR2KGlobalErrors.to_string( message[0] ) ) )
                return
            
            ### Special process.
            if length == 0x0A:        # Operation successful.
                if command in [ ImpinjR2KCommands.REAL_TIME_INVENTORY, ImpinjR2KCommands.CUSTOMIZED_SESSION_TARGET_INVENTORY ]:
                     ### Head -- Length(fix=0x0A) -- Address -- Cmd -- AntID(1B) -- ReadRate(2B) -- TotalRead(4B) -- Check
                    duration   = struct.unpack( '>H', message[1:3] )[0]
                    total_read = struct.unpack( '>I', message[3:7] )[0]
                else:
                     ### Head -- Length(fix=0x0A) -- Address -- Cmd -- TotalRead(3B) -- CommandDuration(4B) -- Check
                    total_read = ((message[0]<<16) & 0x00FF0000) + ((message[1]<<8)& 0x0000FF00) + message[2]
                    duration   = struct.unpack( '>I', message[3:7] )[0]
                package_queue.put( dict( type='DONE', total_read=total_read, duration=duration ) )
                return

            elif length == 0x04:      # Operation failed.
                ### Head -- Length(fix=0x04) -- Address -- Cmd -- ErrorCode -- Check
                package_queue.put( dict( type='ERROR', logs='{}'.format( ImpinjR2KGlobalErrors.to_string( message[0] ) ) ) )
                return

            antenna   = ( message[0] & 0x03 ) + 1
            frequency = FREQUENCY_TABLES[ ( ( ( message[0] & 0xFC ) >> 2 ) & 0x3F ) ]

            try:
                pc = struct.unpack( '>H', message[1:3] )[0]
            except BaseException:
                if message[1] == ImpinjR2KGlobalErrors.ANTENNA_MISSING_ERROR:
                    package_queue.put( dict( type='ERROR', logs='Antenna-{} disconnect.'.format( antenna ) ) )
                return

            size = ( ( pc & 0xF800 ) >> 10 ) & 0x003E
            if size == 0:
                package_queue.put( dict( type='ERROR', logs='Nothing!' ) )
                return

            rssi = message[-1] - 129
            epc  = ''.join( [ '%02X' % x for x in message[3:size+3] ] )     # Bugfix:20200224
            package_queue.put( dict( type='TAG', antenna=antenna, frequency=frequency, rssi=rssi, epc=epc ) )
        else:
            command_queue.put( dict( command=command, data=message ) )

    while True:
        size = uart.any()
        if size <= 0:
            utime.sleep_ms( 10 )
            continue

        for byte in uart.read( size ):
            if ( byte == 0xA0 ) and ( in_packet is False ):
                in_packet = True
                packet.extend( bytes( [byte] ) )
            elif in_packet:
                packet.extend( bytes( [byte] ) )

                if ( ( packet[1] + 2 ) == len( packet ) ):
                    in_packet = False
                    if address == packet[2]:                           # Check if the address is correct.
                        if ( libscrc.lrc( bytes(packet) ) == 0 ):      # Check if the package's crc is correct.
                            hexin_handle_packet( bytes( packet ), command_queue, package_queue )
                    packet = bytearray()                               # Clear buffer.


class ImpinjR2KReader( object ):

    def analyze_data( method='RESULT' ):
        def decorator( func ):
            def wrapper( self, *args, **kwargs ):
                func( self, *args, **kwargs )
                try:
                    data = self.command_queue.get( )
                    if method == 'DATA':
                        return data['data']
                    else:
                        return ( True if data['data'][0] == ImpinjR2KGlobalErrors.SUCCESS else False, ImpinjR2KGlobalErrors.to_string( data['data'][0] ) )
                except BaseException as err:
                    logging.error( '[ERROR] ANALYZE_DATA error {} or COMMAND QUEUE is timeout.'.format( err ) )
                    return bytes( [ ImpinjR2KGlobalErrors.FAIL ] )
            return wrapper
        return decorator

    def __init__( self, id=1, rx=36, tx=33, address=0xFF, package_queue=None ):
        self.address = address

        self.package_queue = Queue( 1024 ) if package_queue is None else package_queue
        self.command_queue = Queue( 1024 )

        self.uart = UART( id, 115200, bits=8, parity=None, stop=1, rx=rx, tx=tx, rxbuf=1024 )
        self.protocol = ImpinjR2KProtocols( address=self.address, serial=self.uart )
        _thread.start_new_thread( hexin_threading_uart_received, ( self.address, self.uart, self.command_queue, self.package_queue ) )

    @analyze_data( 'DATA' )
    def identifier( self ):
        self.protocol.get_reader_identifier( )

    @analyze_data( )
    def fast_power( self, value=22 ):
        logging.info( '[FAST SET RF POWER] {}dBm'.format( value ) )
        self.protocol.fast_power( value=value )

    @analyze_data( )
    def set_rf_power( self, antenna1=20, antenna2=20, antenna3=20, antenna4=20 ):
        logging.info( '[SET RF POWER] Antenna1 = {}dBm'.format( antenna1 ) )
        logging.info( '[SET RF POWER] Antenna2 = {}dBm'.format( antenna2 ) )
        logging.info( '[SET RF POWER] Antenna3 = {}dBm'.format( antenna3 ) )
        logging.info( '[SET RF POWER] Antenna4 = {}dBm'.format( antenna4 ) )
        self.protocol.set_rf_power( ant1=antenna1, ant2=antenna2, ant3=antenna3, ant4=antenna4 )

    @analyze_data( )
    def set_work_antenna( self, antenna=READER_ANTENNA['ANTENNA1'] ):
        self.protocol.set_work_antenna( antenna=antenna )

    @analyze_data( 'DATA' )
    def get_work_antenna( self ):
        self.protocol.get_work_antenna( )

    def rt_inventory( self, repeat=1 ):
        self.protocol.rt_inventory( repeat=repeat )

    def fast_switch_ant_inventory( self, param = dict( A=ImpinjR2KFastSwitchInventory.ANTENNA1, Aloop=1,
                                                       B=ImpinjR2KFastSwitchInventory.DISABLED, Bloop=1,
                                                       C=ImpinjR2KFastSwitchInventory.DISABLED, Cloop=1,
                                                       D=ImpinjR2KFastSwitchInventory.DISABLED, Dloop=1,
                                                       Interval = 0,
                                                       Repeat   = 1 ) ):
        self.protocol.fast_switch_ant_inventory( param=param )
