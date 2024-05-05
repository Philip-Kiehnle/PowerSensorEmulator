#!/usr/bin/python3
import sys
import json
import time
import matplotlib.pyplot as plt
from pymodbus.client import ModbusSerialClient
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.constants import Endian

#from . import *
from chintPM.chintPM import *
from chintPM.const import *

UART_PORT = '/dev/ttyUSB0'
MODBUS_ID = DEFAULT_SERIAL_SLAVE_ID


data = {CONF_HOST       : None,
        CONF_PORT       : UART_PORT,
        CONF_SLAVE_IDS  : [MODBUS_ID],
        CONF_METER_TYPE : MeterTypes.METER_TYPE_H_3P}
print(data)

entry = ConfigEntry(data=data)
    

#modbus_register_list
# Input Registers (1x16bit R/W)
#0000H REV. version
#0001H UCode Programming password
#0002H ClrE Electric energy zero
#0003H net Selecting of the connection mode net(0:3P4W,13P3W)
#
# Input Registers (1x16bit R)
#000BH Meter type
#
# Input Registers (2x16bit R)
#0x2006 Ua Phase-phase voltage, the unit is V(It is invalid for 3 phase 3 wire)
#0x2008 Ub
#0x200A Uc
#0x2012 Pt Conjunction active power，the unit is W
#0x2014 Pa A phase active power，theunit is W
#0x2016 Pb B phase active power，the unit is W (invalid when three phase three wire)
#0x2018 Pc C phase active power，the unit is W

def _resolve_ph_mode(net: int) -> str:
    if net == 0:
        return PHMODE_3P4W
    else:
        return PHMODE_3P3W
    

def main():
    #######################################
    # Option A: Huawei inverter emualtion #
    #######################################
    if 1:
        if 1: ## Huawei inverter sends meter init bytes during boot
            import serial
            ser = serial.Serial(UART_PORT, 9600, timeout=0.1)
            for i in range(5):
                ser.write(b'\x01\x03\x00\x0B\x00\x01\xF5\xC8')
                #time.sleep(0.001)
                bs = ser.readline()
                if len(bs) > 0:
                    print('Power meter should not repond to init message, but responded:', bs)
                    sys.exit()
            ser.close()
            print('Finished Huawei init...')
        
        print('Create ModbusSerialClient...')
        client = ModbusSerialClient( port=UART_PORT, baudrate=9600, bytesize=8, stopbits=1, parity="N")
        print('connect...')
        client.connect()
        ts_start = time.time()
        pt_last = 0
        ts_rel_last_change = 0
        update_freq_max = 0
        runs = 0

        while 0:  ## fast loop test

            rr = client.read_holding_registers(
                address=2126, count=2, slave=MODBUS_ID
            )
            decoder = BinaryPayloadDecoder.fromRegisters(rr.registers, byteorder=Endian.BIG)
            pt = decoder.decode_32bit_float()

            ts_rel = time.time()-ts_start
            p_str = "ts:{:.3f}".format(ts_rel) + "Pt" + str(pt)
            if (pt != pt_last) :
                interval = ts_rel-ts_rel_last_change
                update_freq = 1/interval
                ts_rel_last_change = ts_rel
                if (runs > 0):
                #if (runs > 0 and runs > 50):  # does not make a difference
                    update_freq_max = max(update_freq, update_freq_max)
                    p_str += " change_interval={:.3f} freq={:.3f} freq_max={:.3f}".format(interval, update_freq, update_freq_max)
            pt_last = pt
            runs += 1
            #if (runs > 50):  # does not make a difference
            print(p_str)

        if 0:

            print("read_holding_registers...")
            rr = client.read_holding_registers(0X401E, 4, slave=MODBUS_ID)
            if rr.isError():
                print(f"Received Modbus library error({rr})")
            else:
                print('0X4026\n',rr.registers)
                decoder = BinaryPayloadDecoder.fromRegisters(
                        rr.registers, byteorder=Endian.BIG
                )
                impep = decoder.decode_32bit_float()
                print('energy import', impep)

            rr = client.read_holding_registers(
                address=0x0, count=4, slave=MODBUS_ID
            )

            if rr.isError():
                print(f"Received Modbus library error({rr})")
            else:
                print("done")

            decoder = BinaryPayloadDecoder.fromRegisters(rr.registers, byteorder=Endian.BIG)
            rev = decoder.decode_16bit_uint()
            ucode = decoder.decode_16bit_uint()
            clre = decoder.decode_16bit_uint()
            net = decoder.decode_16bit_uint()
            print('rev', rev)
            print('ucode', ucode)
            print('clre', clre)
            print('net', net)

            # read device_type
            rr = client.read_holding_registers(
                address=0xB, count=1, slave=MODBUS_ID
            )
            decoder = BinaryPayloadDecoder.fromRegisters(rr.registers, byteorder=Endian.BIG)
            device_type = decoder.decode_16bit_uint()

            print("Successfully connected to pm phase mode %s", net)

            #meter_type_name = "DTSU-666"
            meter_type_name = "DTSU-666-H"

            result = {
                "model_name": f"{meter_type_name} ({UART_PORT}@{MODBUS_ID})",
                "rev": rev,
                CONF_PHASE_MODE: _resolve_ph_mode(net),
            }

            print(result)

            rr = client.read_input_registers(0X2006, 3, slave=MODBUS_ID)

            if rr.isError():
                print(f"Received Modbus library error({rr})")
            else:
                print('Ua 0X2006\n',rr.registers)
                print(rr.registers[0], rr.registers[1])

                decoder = BinaryPayloadDecoder.fromRegisters(
                        rr.registers, byteorder=Endian.BIG
                )

                # Uab Line -line voltage, the unit is V
                uab = decoder.decode_32bit_float()
                print('uab', uab)

            rr = client.read_input_registers(0X2014, 3, slave=MODBUS_ID)

            if rr.isError():
                print(f"Received Modbus library error({rr})")
            else:
                print('Pa 0X2014\n',rr.registers)
                print(rr.registers[0], rr.registers[1])


        print('Read register of Huawei specific -H version\n')
        rr = client.read_holding_registers(
            address=2001, count=1, slave=MODBUS_ID
        )
        decoder = BinaryPayloadDecoder.fromRegisters(rr.registers, byteorder=Endian.BIG)
        magic_key = decoder.decode_16bit_uint()
        print('magic_key', magic_key)

        data = {}
        rr = client.read_holding_registers(
            address=2102, count=80, slave=MODBUS_ID
        )
        decoder = BinaryPayloadDecoder.fromRegisters(rr.registers, byteorder=Endian.BIG)
        data["Ia"] = decoder.decode_32bit_float()
        data["Ib"] = decoder.decode_32bit_float()
        data["Ic"] = decoder.decode_32bit_float()
        data["Umean"] = decoder.decode_32bit_float()
        data["Ua"] = decoder.decode_32bit_float()
        data["Ub"] = decoder.decode_32bit_float()
        data["Uc"] = decoder.decode_32bit_float()
        data["Ull_mean"] = decoder.decode_32bit_float()
        data["Uab"] = decoder.decode_32bit_float()
        data["Ubc"] = decoder.decode_32bit_float()
        data["Uca"] = decoder.decode_32bit_float()
        data["freq"] = decoder.decode_32bit_float()
        data["Pt"] = decoder.decode_32bit_float()
        data["Pa"] = decoder.decode_32bit_float()
        data["Pb"] = decoder.decode_32bit_float()
        data["Pc"] = decoder.decode_32bit_float()
        data["Qt"] = decoder.decode_32bit_float()
        data["Qa"] = decoder.decode_32bit_float()
        data["Qb"] = decoder.decode_32bit_float()
        data["Qc"] = decoder.decode_32bit_float()
        data["St"] = decoder.decode_32bit_float()
        data["Sa"] = decoder.decode_32bit_float()
        data["Sb"] = decoder.decode_32bit_float()
        data["Sc"] = decoder.decode_32bit_float()
        data["PFt"] = decoder.decode_32bit_float()
        data["PFa"] = decoder.decode_32bit_float()
        data["PFb"] = decoder.decode_32bit_float()
        data["PFc"] = decoder.decode_32bit_float()
        data["Et"] = decoder.decode_32bit_float()
        data["Ea"] = decoder.decode_32bit_float()
        data["Eb"] = decoder.decode_32bit_float()
        data["Ec"] = decoder.decode_32bit_float()
        data["Ept"] = decoder.decode_32bit_float()
        data["Epa"] = decoder.decode_32bit_float()
        data["Epb"] = decoder.decode_32bit_float()
        data["Epc"] = decoder.decode_32bit_float()
        data["Ent"] = decoder.decode_32bit_float()
        data["Ena"] = decoder.decode_32bit_float()
        data["Enb"] = decoder.decode_32bit_float()
        data["Enc"] = decoder.decode_32bit_float()
        
        print(json.dumps(data, sort_keys=True, indent=4))
        client.close()

    ############################
    # Option B: async read all #
    ############################
    if 1:
        newfeature = asyncio.run(setupPM())
        

async def setupPM():
    device, updateCoord = await async_setup_entry(entry)
    await updateCoord.async_update_data()

    print(json.dumps(device.data, sort_keys=True, indent=4))


if __name__=='__main__':
    main()