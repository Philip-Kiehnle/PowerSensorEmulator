/**

 * stModbus Libary: A tiny Modbus implementation for Modbus ASCII/RTU
 * Copyright (c) 2017+ [Iurii Bell] ds@inbox.ru (http://www.bel-tech.ru/)
 * All rights reserved.
 *
 * [MIT License](http://www.opensource.org/licenses/mit-license.php)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * @copyright   Copyright (C) 2017+
 * @author      Iurii Bell (ds@inbox.ru)
 * @license     MIT
 * @package     stModbus
 */

#include <chintPowerSensor_modbus.h>
#include <stdio.h>

#include <string.h>


#ifdef __cplusplus
extern          "C"
{
#endif




//Define device register
uint16_t device_reg_4xxxx[13];

//Define some 16-bit output
uint16_t output;

//Define some 16-bit input
uint16_t input;


extern float power_W;
extern float energy_export_kWh;
extern float energy_import_kWh;


//Function for initialization modbus on you device
mbus_t mbus_powersensor_open(Modbus_Conf_t *pconf)
{
	 mbus_t mb;

	 pconf->devaddr = 11;
	 pconf->device = (void*) 0;

	//Define read callback function
    pconf->read = &mbus_powersensor_read;
	
	//Define write callback function
    pconf->write = &mbus_powersensor_write;

    //Open modbus context
	mb = mbus_open(pconf);

    if (mb < 0 ) return (mbus_t)MBUS_ERROR;

	//We can map some function on modbus commands
    //if ( mbus_connect(mb, mbus_po_read_3xxxx, MBUS_FUNC_READ_INPUT_REGS ) ) return (mbus_t)MBUS_ERROR;

    //if ( mbus_connect(mb, mbus_somedevice_read_4xxxx, MBUS_FUNC_READ_REGS ) ) return (mbus_t)MBUS_ERROR;

	//Set default values for registers as you need

//    device_reg_4xxxx[0] = 0x322A;             //40001
//    device_reg_4xxxx[1] = pconf->devaddr;     //40002 Device addr
//    device_reg_4xxxx[2] = 0x7;                //40003
//
//    device_reg_4xxxx[3] = 0x0;                //40004
//    device_reg_4xxxx[4] = 0x0;                //
//
//    device_reg_4xxxx[5] = 0x0;                //40006
//    device_reg_4xxxx[6] = 0x0;                //
//
//    device_reg_4xxxx[7] = 0x0;                //40008
//    device_reg_4xxxx[8] = 0x0;                //
//
//    device_reg_4xxxx[11] = 0x0;                //Up time // 40012


    return mb;
}


//mbus_status_t mbus_powersensor_read_3xxxx(mbus_t mb_context)
//{
//    mbus_context_t ctx = mbus_context(mb_context);
//    if (ctx == 0) return MBUS_ERROR;
//   // printf("Isma read:%d\n", ctx->header.addr+30001);
//
//
//    return MBUS_OK;
//
//}

//mbus_status_t mbus_powersensor_read_4xxxx(mbus_t mb_context)
//{
//    mbus_context_t ctx = mbus_context(mb_context);
//    if (ctx == 0) return MBUS_ERROR;
//    //printf("Isma read:%d\n", ctx->header.addr+40001);
//
//    uint16_t size = 0;
//    uint16_t* buf = (uint16_t*)( ctx->conf.sendbuf + 3 );
//
//    /*Setting  register  40001  according to the table  below  will  enable  1  of  4 available
//     *   actions: module reset, reloading the settings, setting to default and entering the bootloader.     *  40001
//     */
//
//
//    if (ctx->header.addr + ctx->header.num < 13 ){
//        //memcpy(buf, isma_reg_4xxxx +  ctx->header.addr, ctx->header.num*2 );
//        size= ctx->header.num;
//    }else{
//        //Hack
//        //memset(buf,0,ctx->header.num*2);
//        buf[0]++;
//        size= ctx->header.num;
//    }
//
//
//    if ( size != 0 ){
//        ctx->conf.sendbuf[2] = size*2;
//        return mbus_send_data(mb_context, size*2 + 3 );
//    }
//
//    return mbus_response(mb_context, MBUS_RESPONSE_ILLEGAL_DATA_ADDRESS);
//}


uint16_t get_lowbytes(float value)
{
	uint32_t* ff = (uint32_t*) &value;
	return ((*ff)>>16)&0xFFFF;
}


uint16_t get_highbytes(float value)
{
	uint32_t* ff = (uint32_t*) &value;
	return (*ff)&0xFFFF;
}

void map(const uint16_t addr, const uint16_t A, const uint16_t B, const uint16_t target, uint32_t *la)
{
	 if (addr >= A && addr < B) *la=30001+target+(addr-A);  // 40001 is correct but 30001 is faster
}


/* It's modbus request on read register by logical address (la) */
uint16_t mbus_powersensor_read(uint32_t la)
{
	uint16_t addr = la-40001;

	// todo / possible issues:
	// 1. Power import with original sensor has negative power factor (-0.97) but we always report +1.0
	// 2. We always report 0 reactive power. Does the inverter compensate reactive power if reported?


	// **************************************************
	// * Chint Huawei(-H) version has special registers *
	// **************************************************

	// magic key
	if (addr == 2001) return 0x3111;

	// map currents
	map(addr, 2102, 2108, 0x200C, &la);

	// mean of phase voltages -> map to Ua
	map(addr, 2108, 2110, 0x2006, &la);

	// map phase voltages
	map(addr, 2110, 2116, 0x2006, &la);

	// mean of line-line voltages -> map to Uab
	map(addr, 2116, 2118, 0x2000, &la);

	// map line-line voltages
	map(addr, 2118, 2124, 0x2000, &la);

	// map frequency
	map(addr, 2124, 2126, 0x2044, &la);

	// map active power
	map(addr, 2126, 2134, 0x2012, &la);

	// map reactive power
	map(addr, 2134, 2142, 0x201A, &la);

	// apparent power -> map to active power
	map(addr, 2142, 2150, 0x2012, &la);

	// map power factor
	map(addr, 2150, 2158, 0x202A, &la);

	float energy_export_kWh_tmp = energy_export_kWh;
	float energy_export_phase_kWh_tmp = energy_export_kWh_tmp/3;

	float energy_import_kWh_tmp = energy_import_kWh;
	float energy_import_phase_kWh_tmp = energy_import_kWh_tmp/3;

	float energy_bidi_total_kWh_tmp = energy_import_kWh_tmp + energy_export_kWh_tmp;
	float energy_bidi_phase_kWh_tmp = energy_bidi_total_kWh_tmp/3;

	// total active electricity
	if (addr == 2158) return get_lowbytes(energy_bidi_total_kWh_tmp);
	if (addr == 2159) return get_highbytes(energy_bidi_total_kWh_tmp);

	if (addr == 2160 || addr == 2162 || addr == 2164) return get_lowbytes(energy_bidi_phase_kWh_tmp);
	if (addr == 2161 || addr == 2163 || addr == 2165) return get_highbytes(energy_bidi_phase_kWh_tmp);

	// total active pos. electricity
	// map current total active pos. electricity
	map(addr, 2166, 2168, 0x401E, &la);

	if (addr == 2168 || addr == 2170 || addr == 2172) return get_lowbytes(energy_import_phase_kWh_tmp);
	if (addr == 2169 || addr == 2171 || addr == 2173) return get_highbytes(energy_import_phase_kWh_tmp);

	// total active neg. electricity
	// map current total active neg. electricity
	map(addr, 2174, 2176, 0x4028, &la);

	if (addr == 2176 || addr == 2178 || addr == 2180) return get_lowbytes(energy_export_phase_kWh_tmp);
	if (addr == 2177 || addr == 2179 || addr == 2181) return get_highbytes(energy_export_phase_kWh_tmp);


	 // ****************************
	 // * Chint standard registers *
	 // ****************************

	addr = la-40001;

	// REV. version
	if (addr == 0) return 109;

	// UCode
	if (addr == 1) return 701;

	// ClrE
	if (addr == 2) return 0;

	// net
	if (addr == 3) return 0;

	// IrAt Current Transformer Ratio
	if (addr == 6) return 1;

	// UrAt Potential Transformer Ratio
	if (addr == 7) return 10;

	// meter type
	if (addr == 0xB) return 1;


	// Protocol
	if (addr == 0x2C) return 3;

	// Addr
	if (addr == 0x2D) return 3;

	// bAud
	if (addr == 0x2E) return 11;

	// Second
	if (addr == 0x2F) return 1;
	// Minute
	if (addr == 0x30) return 1;
	// Hour
	if (addr == 0x31) return 1;
	// Day
	if (addr == 0x32) return 1;
	// Month
	if (addr == 0x33) return 4;
	// Year
	if (addr == 0x34) return 2024;


	float v_grid = 230.0;
	float v_gridLL = 400.0;

	float power_W_tmp = power_W;
	float power_per_phase = power_W_tmp/3.0;

	float q_power = 0.0;
	float q_power_per_phase = q_power/3.0;

	float i_grid = power_per_phase/v_grid;

	float freq = 50.0;

	for (int i = 0; i<2; i++) {

		if (i==0) {
			addr = la-30001;
		} else {
			addr = la-40001;
		}

		switch (addr) {

			// ************
			// * Voltages *
			// ************
			// voltage phase AB
			case 0x2000:
				return get_lowbytes(v_gridLL);
			case 0x2001:
				return get_highbytes(v_gridLL);

			// voltage phase BC
			case 0x2002:
				return get_lowbytes(v_gridLL);
			case 0x2003:
				return get_highbytes(v_gridLL);

			// voltage phase CA
			case 0x2004:
				return get_lowbytes(v_gridLL);
			case 0x2005:
				return get_highbytes(v_gridLL);

			// voltage phase A
			case 0x2006:
				return get_lowbytes(v_grid);
			case 0x2007:
				return get_highbytes(v_grid);

			// voltage phase B
			case 0x2008:
				return get_lowbytes(v_grid);
			case 0x2009:
				return get_highbytes(v_grid);

			// voltage phase C
			case 0x200A:
				return get_lowbytes(v_grid);
			case 0x200B:
				return get_highbytes(v_grid);

			// ************
			// * Currents *
			// ************
			// current phase A
			case 0x200C:
				return get_lowbytes(i_grid);
			case 0x200D:
				return get_highbytes(i_grid);

			// current phase B
			case 0x200E:
				return get_lowbytes(i_grid);
			case 0x200F:
				return get_highbytes(i_grid);

			// current phase C
			case 0x2010:
				return get_lowbytes(i_grid);
			case 0x2011:
				return get_highbytes(i_grid);


			// ****************
			// * Active power *
			// ****************
			// total active power
			case 0x2012:
				return get_lowbytes(power_W_tmp);
			case 0x2013:
				return get_highbytes(power_W_tmp);

			// power phase A
			case 0x2014:
				return get_lowbytes(power_per_phase);
			case 0x2015:
				return get_highbytes(power_per_phase);

			// power phase B
			case 0x2016:
				return get_lowbytes(power_per_phase);
			case 0x2017:
				return get_highbytes(power_per_phase);

			// power phase C
			case 0x2018:
				return get_lowbytes(power_per_phase);
			case 0x2019:
				return get_highbytes(power_per_phase);


			// ******************
			// * Reactive power *
			// ******************
			// total reactive power
			case 0x201A:
				return get_lowbytes(q_power);
			case 0x201B:
				return get_highbytes(q_power);

			// reactive power phase A
			case 0x201C:
				return get_lowbytes(q_power_per_phase);
			case 0x201D:
				return get_highbytes(q_power_per_phase);

			// reactive power phase B
			case 0x201E:
				return get_lowbytes(q_power_per_phase);
			case 0x201F:
				return get_highbytes(q_power_per_phase);

			// reactive power phase C
			case 0x2020:
				return get_lowbytes(q_power_per_phase);
			case 0x2021:
				return get_highbytes(q_power_per_phase);


			// ****************
			// * Power factor *
			// ****************
			// total power factor
			case 0x202A:
				return get_lowbytes(1.0);
			case 0x202B:
				return get_highbytes(1.0);

			// power factor phase A
			case 0x202C:
				return get_lowbytes(1.0);
			case 0x202D:
				return get_highbytes(1.0);

			// power factor phase B
			case 0x202E:
				return get_lowbytes(1.0);
			case 0x202F:
				return get_highbytes(1.0);

			// power factor phase C
			case 0x2030:
				return get_lowbytes(1.0);
			case 0x2031:
				return get_highbytes(1.0);


			// *********
			// * Other *
			// *********
			// frequency
			case 0x2044:
				return get_lowbytes(freq);
			case 0x2045:
				return get_highbytes(freq);

			// DmPt  Total active power demand
			case 0x2050:
				return get_lowbytes(0.0);
			case 0x2051:
				return get_highbytes(0.0);


			// ImpEp  (current)positive active total energy
			case 0x401E:
				return get_lowbytes(energy_import_kWh_tmp);
			case 0x401F:
				return get_highbytes(energy_import_kWh_tmp);

			// ExpEp  (current)negative active total energy
			case 0x4028:
				return get_lowbytes(energy_export_kWh_tmp);
			case 0x4029:
				return get_highbytes(energy_export_kWh_tmp);

			// Q1Eq  (current) quadrant 1 reactive total energy
			case 0x4032:
				return get_lowbytes(0.0);
			case 0x4033:
				return get_highbytes(0.0);

			// Q2Eq  (current) quadrant 2 reactive total energy
			case 0x403C:
				return get_lowbytes(0.0);
			case 0x403D:
				return get_highbytes(0.0);

			// Q3Eq  (current) quadrant 3 reactive total energy
			case 0x4046:
				return get_lowbytes(0.0);
			case 0x4047:
				return get_highbytes(0.0);

			// Q4Eq  (current) quadrant 4 reactive total energy
			case 0x4050:
				return get_lowbytes(0.0);
			case 0x4051:
				return get_highbytes(0.0);

		}
	}

	// in case other (reserved) registers are read
	if (la > 40001 && la < 40001+0x36){
		return 0;
	}

	if (la > 40001+0x4000 && la < 40001+0x4060){
		return 0;
	}

#if 1
	if (la > 30001+0x2000 && la < 30001+0x2050){
		return 0;
	}

	if (la > 30001+0x4000 && la < 40001+0x4060){
		return 0;
	}

	if (la < 40000){
		return 1;
	}

	if (la > 40100){
		return 0;
	}
#endif

    return la;
}


//Function for write to some register by logic address
uint16_t mbus_powersensor_write(uint32_t la, uint16_t value)
{
    //printf("We write: %d %d\n",la, value);
		if ( la > 40000 && la <= 40016 ){
			//uint8_t ch  = la - 40001;
			//dac[ch] = value;
			//spi_dac_set(dac[ch]&0xFF,ch);
		}
		if (la == 40017 ){
			//ion_adc_writereg(0x1, 1, value&0xFF);
		}
		if (la == 40018 ){
			//ion_mux_select(value&0xF);
			//ion_mux_output(value&0x10);
		}

    return value;
}



#ifdef __cplusplus
}
#endif








