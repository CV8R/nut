/*  socomec_jbus.c - Driver for Socomec JBUS UPS
 *
 *  Copyright (C)
 *    2021 Thanos Chatziathanassiou <tchatzi@arx.net>
 *
 * Based on documentation found freely on 
 * https://www.socomec.com/files/live/sites/systemsite/files/GB-JBUS-MODBUS-for-Delphys-MP-and-Delphys-MX-operating-manual.pdf
 * but with dubious legal license. The document itself states:
 * ``CAUTION : “This is a product for restricted sales distribution to informed partners. 
 *   Installation restrictions or additional measures may be needed to prevent disturbances''
 * YMMV
 * 
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#include "main.h"
#include <modbus.h>

#define DRIVER_NAME	"Socomec jbus driver"
#define DRIVER_VERSION	"0.09.8"

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

#define BATTERY_CHARGE_LOW_PERCENT 20  // battery.charge.low See Below on Warning about using override.something in ups.conf
/* https://github.com/networkupstools/nut/wiki/Ensure-UPS-settings-with-volatile-device-memory */

/* Note for me the above workaround does not work because nut appears to restart before shutting down the UPS
	hence, we can use the vars in ups.conf */


#define SCHEDULE_DELAY_OFF 30 //Seconds to pass before UPS goes into Standby | Allowed seconds 20 to 600 secs
#define SCHEDULE_MIN_OFF 1 // Minutes of UPS Standby Operations | Allowed minutes 1 to 9999 mins
#define SCHEDULING_TYPE 4 // Scheduling Type | Allowed 0, 1 or 4

/* SCHEDULING_TYPES
0 = no scheduling / reset pendign schedule
1 = one_shot
2 = not used
3 = not used
4 = UPS shutdown management with restor time delay
*/

#define BAUD_RATE 9600
#define PARITY 'N'
#define DATA_BIT 8
#define STOP_BIT 1
#define MODBUS_SLAVE_ID 1

/* Variables */
static modbus_t *modbus_ctx = NULL;

static int mrir(modbus_t * arg_ctx, int addr, int nb, uint16_t * dest);
static int mwrs(modbus_t *ctx, int addr, int nb, uint16_t *src);

static int ser_baud_rate = BAUD_RATE;                      /* serial port baud rate */
static char ser_parity = PARITY;                           /* serial port parity */
static int ser_data_bit = DATA_BIT;                        /* serial port data bit */
static int ser_stop_bit = STOP_BIT;                        /* serial port stop bit */
static int rio_slave_id = MODBUS_SLAVE_ID;                 /* set device ID to default value */

void get_config_vars(void);

int DISCHARGING_FLAG = -1;

static int battery_charge_low = BATTERY_CHARGE_LOW_PERCENT;

static int ups_model = -1;

static uint16_t sch_delay_off = SCHEDULE_DELAY_OFF; 
static uint16_t sch_min_off = SCHEDULE_MIN_OFF; //sch_min_off - minutes becuase UPS is set in minutes not secs.
static uint16_t sch_scheduletype = SCHEDULING_TYPE; // Schedule Types




/* driver description structure */
upsdrv_info_t upsdrv_info = {
	DRIVER_NAME,
	DRIVER_VERSION,
	"Thanos Chatziathanassiou <tchatzi@arx.net>\n",
	DRV_BETA,
	{NULL}
};

static int instcmd(const char *cmdname, const char *extra)
{
	int r;
	uint16_t val[16];
	
	/* Comes from example safenet.c */

	upsdebugx(2, "instcmd");

	
	if (!strcasecmp(cmdname, "load.off")) {

		val[0] = 0x05; /* Stand_by Mode enable*/

		r = mwrs(modbus_ctx, 0x15B0, 1, val);
		
		upslogx(LOG_NOTICE, "instcmd: load.off: [%s] [%s]", cmdname, extra);	
		
		if (r != 1){
				return STAT_INSTCMD_FAILED;
			} else {
				return STAT_INSTCMD_HANDLED;
		}
	}

	if (!strcasecmp(cmdname, "load.on")) {

		val[0] = 0x06; /* Stand_by Mode (UPS ON) disable */

		r = mwrs(modbus_ctx, 0x15B0, 1, val);
		
		upslogx(LOG_NOTICE, "instcmd: load.on: [%s] [%s]", cmdname, extra);	
		
		if (r != 1){
				return STAT_INSTCMD_FAILED;
			} else {
				return STAT_INSTCMD_HANDLED;
		}
	}

	if (!strcasecmp(cmdname, "beeper.enable")) {

		val[0] = 0x07; /* Buzzer Enable */

		r = mwrs(modbus_ctx, 0x15B0, 1, val);
		
		upslogx(LOG_NOTICE, "instcmd: beeper.enable: [%s] [%s]", cmdname, extra);	
		
		if (r != 1){
				return STAT_INSTCMD_FAILED;
			} else {
				return STAT_INSTCMD_HANDLED;
		}
	}

	if (!strcasecmp(cmdname, "beeper.mute")) {

		val[0] = 0x08; /* Buzzer Off */

		r = mwrs(modbus_ctx, 0x15B0, 1, val);
		
		upslogx(LOG_NOTICE, "instcmd: beeper.mute: [%s] [%s]", cmdname, extra);	
		
		if (r != 1){
				return STAT_INSTCMD_FAILED;
			} else {
				return STAT_INSTCMD_HANDLED;
		}
	}
	
	if (!strcasecmp(cmdname, "test.panel.start")) {

		val[0] = 0x0D; /* Mimic panel LED test */

		r = mwrs(modbus_ctx, 0x15B0, 1, val);
		
		upslogx(LOG_NOTICE, "instcmd: test.panel.start: [%s] [%s]", cmdname, extra);	
		
		if (r != 1){
				return STAT_INSTCMD_FAILED;
			} else {
				return STAT_INSTCMD_HANDLED;
		}
	}

	if (!strcasecmp(cmdname, "beeper.disable")) {

		val[0] = 0x0E; /* Buzzer Disable */

		r = mwrs(modbus_ctx, 0x15B0, 1, val);
		
		upslogx(LOG_NOTICE, "instcmd: beeper.disable: [%s] [%s]", cmdname, extra);	
		
		if (r != 1){
				return STAT_INSTCMD_FAILED;
			} else {
				return STAT_INSTCMD_HANDLED;
		}
	}

	if (!strcasecmp(cmdname, "test.battery.start")) {

		val[0] = 0x10; /* Immediate Battery Test */

		r = mwrs(modbus_ctx, 0x15B0, 1, val);
		
		upslogx(LOG_NOTICE, "instcmd: test.battery.start: [%s] [%s]", cmdname, extra);	
		
		if (r != 1){
				return STAT_INSTCMD_FAILED;
			} else {
				return STAT_INSTCMD_HANDLED;
		}
	}

	if (!strcasecmp(cmdname, "load.off.delay")) {
	
			upslogx(LOG_NOTICE, "instcmd: load.off.delay [%s] [%s]", cmdname, extra);


		uint8_t sch_delay_off_MSB = (uint8_t)((sch_delay_off & 0xFF00) >> 8);
		uint8_t sch_delay_off_LSB = (uint8_t)(sch_delay_off & 0x00FF);
		uint8_t sch_min_off_MSB = (uint8_t)((sch_min_off & 0xFF00) >> 8);
		uint8_t sch_min_off_LSB = (uint8_t)(sch_min_off & 0x00FF);

		val[0] = sch_delay_off_MSB;
		val[1] = sch_delay_off_LSB;
		val[2] = sch_min_off_MSB;
		val[3] = sch_min_off_LSB;
		val[4] = sch_scheduletype;

		upsdebugx(4, "sch_delay_off_MSB - MSB: %02x", sch_delay_off_MSB);
		upsdebugx(4, "sch_delay_off_LSB - LSB: %02x", sch_delay_off_LSB);
		upsdebugx(4, "sch_min_off_MSB - MSB Dec: %02x", sch_min_off_MSB);
		upsdebugx(4, "sch_min_off_LSB - LSB Dec: %02x", sch_min_off_LSB);
		
		upsdebugx(2, "Schedule Delay OFF: %d seconds", (sch_delay_off_MSB << 8) | sch_delay_off_LSB);
		upsdebugx(2, "Schedule Min OFF: %d minutes", (sch_min_off_MSB << 8) | sch_min_off_LSB);

		upslogx(LOG_NOTICE, "Shutdown UPS after [%d]secs and return with OL after [%d]secs | [%s] [%s]", sch_delay_off, sch_min_off*60, cmdname, extra );	
		
		r = mwrs(modbus_ctx, 0x1580, 5, val);
				
		if (r != 1){
				return STAT_INSTCMD_FAILED;
			} else {
				return STAT_INSTCMD_HANDLED;
		}
	}

	if (!strcasecmp(cmdname, "shutdown.return")) {

		uint8_t sch_delay_off_MSB = (uint8_t)((sch_delay_off & 0xFF00) >> 8);
		uint8_t sch_delay_off_LSB = (uint8_t)(sch_delay_off & 0x00FF);
		uint8_t sch_min_off_MSB = (uint8_t)((sch_min_off & 0xFF00) >> 8);
		uint8_t sch_min_off_LSB = (uint8_t)(sch_min_off & 0x00FF);

		val[0] = sch_delay_off_MSB;
		val[1] = sch_delay_off_LSB;
		val[2] = sch_min_off_MSB;
		val[3] = sch_min_off_LSB;
		val[4] = sch_scheduletype;

		upsdebugx(4, "sch_delay_off_MSB - MSB: %02x", sch_delay_off_MSB);
		upsdebugx(4, "sch_delay_off_LSB - LSB: %02x", sch_delay_off_LSB);
		upsdebugx(4, "sch_min_off_MSB - MSB Dec: %02x", sch_min_off_MSB);
		upsdebugx(4, "sch_min_off_LSB - LSB Dec: %02x", sch_min_off_LSB);
		
		upsdebugx(2, "Schedule Delay OFF: %d seconds", (sch_delay_off_MSB << 8) | sch_delay_off_LSB);
		upsdebugx(2, "Schedule Min OFF: %d minutes", (sch_min_off_MSB << 8) | sch_min_off_LSB);

		upslogx(LOG_NOTICE, "Shutdown UPS after [%d]secs and return with OL after [%d]secs | [%s] [%s]", sch_delay_off, sch_min_off*60, cmdname, extra );	

		r = mwrs(modbus_ctx, 0x1580, 5, val);
		
		
		if (r != 1){
				return STAT_INSTCMD_FAILED;
			} else {
				return STAT_INSTCMD_HANDLED;
		}
	}

	if (!strcasecmp(cmdname, "shutdown.stayoff")) {

		val[0] = 0x05; /* Stand_by Mode enable*/

		r = mwrs(modbus_ctx, 0x15B0, 1, val);
		
		upslogx(LOG_NOTICE, "shutdown.stayoff: [%s] [%s]", cmdname, extra);	
		
		if (r != 1){
				return STAT_INSTCMD_FAILED;
			} else {
				return STAT_INSTCMD_HANDLED;
		}
	}

	upslogx(LOG_NOTICE, "instcmd: unknown command [%s] [%s]", cmdname, extra);
	return STAT_INSTCMD_UNKNOWN;
}



static int setvar(const char *varname, const char *val)
{
	int r;
	
	if (!strcasecmp(varname, "battery.charge.low")) {
		upsdebugx(2, "Setting Variable: [%s] to [%s]", varname, val);
		dstate_setinfo("battery.charge.low", "%s", val);
		battery_charge_low = atoi(val);
		return STAT_SET_HANDLED;
	}

	if (!strcasecmp(varname, "ups.timer.shutdown")) {
		upsdebugx(2, "Setting Variable: [%s] to [%s]", varname, val);
		dstate_setinfo("ups.timer.shutdown", "%s", val);
		sch_delay_off = atoi(val);
		return STAT_SET_HANDLED;
	}

	if (!strcasecmp(varname, "ups.delay.start")) {
		//check if divisible by 60
		if (atoi(val) % 60 == 0) {
		upsdebugx(2, "Setting Variable: [%s] to [%d] seconds", varname, atoi(val));
		r = dstate_setinfo("ups.delay.start", "%d", atoi(val));
		sch_min_off  = (atoi(val) / 60); //put back in minutes
		upsdebugx(4, "Setting Variable: [%s] to [%d] minutes", varname, sch_min_off);
			if (r !=1)
				return STAT_SET_FAILED;
			else
				return STAT_SET_HANDLED;
		}
		else
		{
			upsdebugx(2, "Setting Variable: [%s] to [%d] seconds FAILED needs to be divisible by 60", varname, atoi(val));
			return STAT_SET_FAILED;
		}
	}

	upslogx(LOG_NOTICE, "setvar: unknown variable [%s]", varname);
	return STAT_SET_UNKNOWN;
}


void upsdrv_initinfo(void)
{
	uint16_t tab_reg[12];
	int r;
	
	upsdebugx(2, "upsdrv_initinfo");

	dstate_setinfo("device.mfr", "socomec jbus");
	dstate_setinfo("device.model", "Socomec Generic");

	dstate_setinfo("battery.charge.low", "%d", battery_charge_low);
	dstate_setflags("battery.charge.low",  ST_FLAG_RW );
	dstate_addrange("battery.charge.low", 10, 100);

	dstate_setinfo("ups.timer.shutdown", "%d", sch_delay_off);
	dstate_setflags("ups.timer.shutdown", ST_FLAG_RW );
	dstate_addrange("ups.timer.shutdown", 20, 600);

	dstate_setinfo("ups.delay.start", "%d", sch_min_off*60); // upsrw var is entered in seconds but UPS responds in minutes
	dstate_setflags("ups.delay.start", ST_FLAG_RW );
	dstate_addrange("ups.delay.start", 60, 599940); // upsrw var is entered in seconds but UPS responds in minutes

	upsdebugx(2, "initial read");


	/* 
		this is a neat trick, but not really helpful right now
		https://stackoverflow.com/questions/25811662/spliting-an-hex-into-2-hex-values/41733170#41733170
		uint8_t *lowbyte;
		uint8_t *hibyte;
	*/

	r = mrir(modbus_ctx, 0x1000, 12, tab_reg);

	if (r == -1) {
		fatalx(EXIT_FAILURE, "failed to read UPS code from JBUS. r is %d error %s", r, modbus_strerror(errno));
	}
	
	upsdebugx(2, "read UPS Code %d", tab_reg[0]);

	if (tab_reg[1]) {
		upsdebugx(2, "read UPS Power %d (kVA * 10)", tab_reg[1]);
		dstate_setinfo("ups.power", "%u", tab_reg[1]*100 );
	}

	/* known Socomec Models */
	switch (tab_reg[0]) {
		case 30:
			dstate_setinfo("ups.model", "%s", "ITYS");   /* thanks to CV8R https://github.com/CV8R */
			ups_model = 30;
			break;

		case 130:
			dstate_setinfo("ups.model", "%s", "DIGYS");
			ups_model = 130;
			break;
		
		case 515:
			dstate_setinfo("ups.model", "%s", "DELPHYS MX");
			ups_model = 515;
			break;
		
		case 516:
			dstate_setinfo("ups.model", "%s", "DELPHYS MX elite");
			ups_model = 516;
			break;

		default:
			dstate_setinfo("ups.model", "Unknown Socomec JBUS. Send id %u and specify the model", tab_reg[0]);
			ups_model = 130; //default to a previous driver model version for STATES 0x1020 length 6
	}

	if (tab_reg[3] && tab_reg[4] && tab_reg[5] && tab_reg[6] && tab_reg[7]) {
		dstate_setinfo("ups.serial", "%c%c%c%c%c%c%c%c%c%c", 
													  (tab_reg[3]&0xFF), (tab_reg[3]>>8),
													  (tab_reg[4]&0xFF), (tab_reg[4]>>8),
													  (tab_reg[5]&0xFF), (tab_reg[5]>>8),
													  (tab_reg[6]&0xFF), (tab_reg[6]>>8),
													  (tab_reg[7]&0xFF), (tab_reg[7]>>8)
												);
	}
	
	dstate_addcmd("load.on");
	dstate_addcmd("load.off");
	dstate_addcmd("beeper.enable");
	dstate_addcmd("beeper.mute"); /* Temporary mute buzzer */
	dstate_addcmd("beeper.disable");
	dstate_addcmd("test.panel.start");
	dstate_addcmd("test.battery.start");
	dstate_addcmd("load.off.delay");
	dstate_addcmd("shutdown.return");
	dstate_addcmd("shutdown.stayoff");
	
	upsh.instcmd = instcmd;
	upsh.setvar = setvar;

}

void upsdrv_updateinfo(void)
{
	uint16_t tab_reg[64];
	int r;
	
	upsdebugx(2, "upsdrv_updateinfo");

	status_init();

	/* ups configuration */
	r = mrir(modbus_ctx, 0x10E0, 32, tab_reg);

	if (r == -1 || !tab_reg[0]) {
		upsdebugx(2, "Did not receive any data from the UPS at 0x10E0 ! Going stale r is %d error %s", r, modbus_strerror(errno));
		dstate_datastale();
		return;
	}

	dstate_setinfo("input.voltage", "%u", tab_reg[0]);
	dstate_setinfo("output.voltage", "%u", tab_reg[1]);
	dstate_setinfo("input.frequency", "%u", tab_reg[2]);
	dstate_setinfo("output.frequency", "%u", tab_reg[3]);

	upsdebugx(2, "battery capacity (Ah * 10) %u", tab_reg[8]);
	upsdebugx(2, "battery capacity (Ah) %.2f", tab_reg[8]/(float)10);
	
	dstate_setinfo("battery.capacity", "%.2f", tab_reg[8]/(float)10);
	
	/* Input Mode */
	switch (tab_reg[26]) {
		case 1:
			upsdebugx(2, "Input mode 1: NORMAL");
			break;

		case 2:
			upsdebugx(2, "Input mode 2: WIDE");
			break;
		
		default:
			upsdebugx(2, "Input mode: unknown");
	}

	upsdebugx(2, "Vout setting: %u", tab_reg[27]);
	
	if (tab_reg[28] != 0xFFFF){
		/* Battery Extensions */
		switch (tab_reg[28]) {
			case 0:
				upsdebugx(2, "Battery Extensions: 0");
				break;

			case 1:
				upsdebugx(2, "Battery Extensions: 1");
				dstate_setinfo("battery.packs.external", "%u", tab_reg[28]);
				break;
			
			case 2:
				upsdebugx(2, "Battery Extensions: 2");
				dstate_setinfo("battery.packs.external", "%u", tab_reg[28]);
				break;
			
			default:
				upsdebugx(2, "Battery Extensions: unknown");
		}
	}

	upsdebugx(2, "battery elements %u", tab_reg[9]);
	
	/* time and date */
	r = mrir(modbus_ctx, 0x1360, 4, tab_reg);
	if (r == -1) {
		upsdebugx(2, "Did not receive any data from the UPS at 0x1360 ! Ignoring ? r is %d error %s", r, modbus_strerror(errno));
	}

	if (tab_reg[0] != 0xFFFF && tab_reg[1] != 0xFFFF)
		dstate_setinfo("ups.time", "%02d:%02d:%02d", (tab_reg[1]&0xFF), (tab_reg[0]>>8), (tab_reg[0]&0xFF) );
	if (tab_reg[2] != 0xFFFF && tab_reg[03] != 0xFFFF)
		dstate_setinfo("ups.date", "%04d/%02d/%02d", (tab_reg[3]+2000), (tab_reg[2]>>8), (tab_reg[1]>>8) );

	/* ups status */

	if (ups_model == 30) {;
		upsdebugx(4, "Request STATES (0x1020) Length 4");
		r = mrir(modbus_ctx, 0x1020, 4, tab_reg);  //ITYS Gnereal Vector Index
	}
	else {
		upsdebugx(4, "Request STATES (0x1020) Length 6");
		r = mrir(modbus_ctx, 0x1020, 6, tab_reg);  //Per Genreal Map Data for MODBUS TCP DATA MAP IN SINGLE UNIT Length is 6, not 4.
	}
	
	if (r == -1) {
		upsdebugx(2, "Did not receive any data from the UPS at 0x1020 ! Ignoring ? r is %d error %s", r, modbus_strerror(errno));
		/* 
		dstate_datastale();
		return;
		*/
	}

	if (CHECK_BIT(tab_reg[0], 0))
		upsdebugx(2, "Rectifier Input supply present");
	if ((CHECK_BIT(tab_reg[0], 0) != 0) && (CHECK_BIT(tab_reg[0], 5) == 0)) {
		/*If On Input Supply and Not On Battery then set OL - UPS reports both OL and OB when changing back to OL*/
		upsdebugx(2, "Load On line");
		/* OL Receiving energy from public power supply*/
		status_set("OL");
		DISCHARGING_FLAG = 0; //Set we are not discharging
		}
	if (CHECK_BIT(tab_reg[0], 1)){
		upsdebugx(2, "Inverter ON ");
		}
		else
		{
		/* Inverter is OFF Set UPS in OFF State */
		status_set("OFF");
		}
	if (CHECK_BIT(tab_reg[0], 2))
		upsdebugx(2, "Rectifier ON");
	if (CHECK_BIT(tab_reg[0], 3))
		upsdebugx(2, "Load protected by inverter");
	if (CHECK_BIT(tab_reg[0], 4))
		upsdebugx(2, "Load on automatic bypass");
	
	/* Battery test runs at a regular interval and flags onbatt so lets not flag OB if running test */
	if ((CHECK_BIT(tab_reg[0], 10) != 0) && (CHECK_BIT(tab_reg[0], 5) != 0 )) { //If Battery test in progress and Load on Battery
		upsdebugx(3, "Active battery test");
		status_set("OL"); //Goes commbad if not setting status when running test.
	}
	
	if ((CHECK_BIT(tab_reg[0], 10) == 0) && (CHECK_BIT(tab_reg[0], 5) != 0 )) { //If Battery test is not in progress and Load on Battery
		upsdebugx(2, "Load on battery");
		// Set on Battery Condition
		status_set("OB");
		DISCHARGING_FLAG = 1; //Set we are now discharging
	}

	if (CHECK_BIT(tab_reg[0], 6))
		upsdebugx(2, "Remote controls disable");
	if (CHECK_BIT(tab_reg[0], 7))
		upsdebugx(2, "Eco-mode ON");

	if (CHECK_BIT(tab_reg[0], 10))
		upsdebugx(2, "Battery Test in progress");
	if (CHECK_BIT(tab_reg[0], 13))
		upsdebugx(2, "Battery Test supported");

	if (CHECK_BIT(tab_reg[0], 14))
		upsdebugx(2, "Battery Test failed");
	if (CHECK_BIT(tab_reg[0], 15)){
		upsdebugx(2, "UPS reporing - Battery near end of Back-up (Low Battery)");
		if (battery_charge_low == -1) {
			upsdebugx(2, "Low Battery Condition (LB)");
			/* Set on Battery Condition */
			status_set("LB");
			}
	}
	if (CHECK_BIT(tab_reg[0], 16))
		upsdebugx(2, "Battery disacharged");
	if (CHECK_BIT(tab_reg[1], 0))
		upsdebugx(2, "Battery OK");
	if (CHECK_BIT(tab_reg[1], 10))
		upsdebugx(2, "Bypass input supply present");
	if (CHECK_BIT(tab_reg[1], 11))
		upsdebugx(2, "Battery charging");
	if (CHECK_BIT(tab_reg[1], 12))
		upsdebugx(2, "Bypass input frequency out of tolerance");
	
	if (CHECK_BIT(tab_reg[2], 0))
		upsdebugx(2, "Unit operating");

	if (CHECK_BIT(tab_reg[3], 0))
		upsdebugx(2, "Maintenance mode active");

	if (CHECK_BIT(tab_reg[4], 0))
		upsdebugx(2, "Boost charge ON");
	if (CHECK_BIT(tab_reg[4], 2))
		upsdebugx(2, "Inverter switch closed");
	if (CHECK_BIT(tab_reg[4], 3))
		upsdebugx(2, "Bypass breaker closed");
	if (CHECK_BIT(tab_reg[4], 4))
		upsdebugx(2, "Maintenance bypass breaker closed");
	if (CHECK_BIT(tab_reg[4], 5))
		upsdebugx(2, "Remote maintenance bypass breaker closed");
	if (CHECK_BIT(tab_reg[4], 6))
		upsdebugx(2, "Output breaker closed (Q3)");
	if (CHECK_BIT(tab_reg[4], 9))
		upsdebugx(2, "Unit working");
	if (CHECK_BIT(tab_reg[4], 12))
		upsdebugx(2, "normal mode active");

	/* alarms */
	r = mrir(modbus_ctx, 0x1040, 4, tab_reg);
	
	alarm_init();

	if (r == -1) {
		upsdebugx(2, "Did not receive any data from the UPS at 0x1040 ! Ignoring ? r is %d error %s", r, modbus_strerror(errno));
		/*
		dstate_datastale();
		return;
		*/
	}

	if (CHECK_BIT(tab_reg[0], 0)) {
		upsdebugx(2, "General Alarm");
		alarm_set("General Alarm present.");
	}
	if (CHECK_BIT(tab_reg[0], 1)) {
		upsdebugx(2, "Battery failure");
		alarm_set("Battery failure.");
	}
	if (CHECK_BIT(tab_reg[0], 2)) {
		upsdebugx(2, "UPS overload");
		alarm_set("Overload fault.");
	}
	if (CHECK_BIT(tab_reg[0], 4)) {
		upsdebugx(2, "Control failure (com, internal supply...)");
		alarm_set("Control failure (com, internal supply...)");
	}
	if (CHECK_BIT(tab_reg[0], 5)) {
		upsdebugx(2, "Rectifier input supply out of tolerance ");
		alarm_set("Rectifier input supply out of tolerance.");
	}
	if (CHECK_BIT(tab_reg[0], 6)) {
		upsdebugx(2, "Bypass input supply out of tolerance ");
		alarm_set("Bypass input supply out of tolerance.");
	}
	if (CHECK_BIT(tab_reg[0], 7)) {
		upsdebugx(2, "Over temperature alarm ");
		alarm_set("Over temperature fault.");
	}
	if (CHECK_BIT(tab_reg[0], 8)) {
		upsdebugx(2, "Maintenance bypass closed");
		alarm_set("Maintenance bypass closed.");
	}
	if (CHECK_BIT(tab_reg[0], 10)) {
		upsdebugx(2, "Battery charger fault");
		alarm_set("Battery charger fault.");
	}
	
	if (CHECK_BIT(tab_reg[1], 1))
		upsdebugx(2, "Improper condition of use");
	if (CHECK_BIT(tab_reg[1], 2))
		upsdebugx(2, "Inverter stopped for overload (or bypass transfer)");
	if (CHECK_BIT(tab_reg[1], 3))
		upsdebugx(2, "Microprocessor control system");
	if (CHECK_BIT(tab_reg[1], 5))
		upsdebugx(2, "Synchronisation fault (PLL fault)");
	if (CHECK_BIT(tab_reg[1], 6))
		upsdebugx(2, "Rectifier input supply fault");
	if (CHECK_BIT(tab_reg[1], 7))
		upsdebugx(2, "Rectifier preventive alarm");
	if (CHECK_BIT(tab_reg[1], 9))
		upsdebugx(2, "Inverter preventive alarm");
	if (CHECK_BIT(tab_reg[1], 10))
		upsdebugx(2, "Charger general alarm");
	if (CHECK_BIT(tab_reg[1], 13))
		upsdebugx(2, "Bypass preventive alarm");
	if (CHECK_BIT(tab_reg[1], 15)) {
		upsdebugx(2, "Imminent STOP");
		alarm_set("Imminent STOP.");
	}

	if (CHECK_BIT(tab_reg[2], 12)) {
		upsdebugx(2, "Servicing alarm");
		alarm_set("Servicing alarm.");
	}
	if (CHECK_BIT(tab_reg[2], 15))
		upsdebugx(2, "Battery room alarm");

	if (CHECK_BIT(tab_reg[3], 0)) {
		upsdebugx(2, "Maintenance bypass alarm");
		alarm_set("Maintenance bypass.");
	}
	if (CHECK_BIT(tab_reg[3], 1)) {
		upsdebugx(2, "Battery discharged");
		alarm_set("Battery discharged.");
	}
	if (CHECK_BIT(tab_reg[3], 3))
		upsdebugx(2, "Synoptic alarm");
	if (CHECK_BIT(tab_reg[3], 4)) {
		upsdebugx(2, "Critical Rectifier fault"); 
		alarm_set("Critical Rectifier fault.");
	}
	if (CHECK_BIT(tab_reg[3], 6)) {
		upsdebugx(2, "Critical Inverter fault");
		alarm_set("Critical Inverter fault.");
	}
	if (CHECK_BIT(tab_reg[3], 10))
		upsdebugx(2, "ESD activated");
	if (CHECK_BIT(tab_reg[3], 11)) {
		upsdebugx(2, "Battery circuit open");
		alarm_set("Battery circuit open.");
	}
	if (CHECK_BIT(tab_reg[3], 14)) {
		upsdebugx(2, "Bypass critical alarm");
		alarm_set("Bypass critical alarm.");
	}

	/* measurements */
	r = mrir(modbus_ctx, 0x1060, 48, tab_reg);

	if (r == -1) {
		upsdebugx(2, "Did not receive any data from the UPS at 0x1060 ! Ignoring ? r is %d error %s", r, modbus_strerror(errno));
		/*
		dstate_datastale();
		return;
		*/
	}

	if (tab_reg[1] == 0xFFFF && tab_reg[2] == 0xFFFF) {
		/* this a 1-phase model */
		dstate_setinfo("input.phases", "1" );
		if (tab_reg[0] != 0xFFFF)
			dstate_setinfo("ups.load", "%u", tab_reg[0] );
		if (tab_reg[6] != 0xFFFF)
			dstate_setinfo("input.bypass.voltage", "%u", tab_reg[6] );

		if (tab_reg[9] != 0xFFFF)
			dstate_setinfo("output.voltage", "%u", tab_reg[9] );

		if (tab_reg[15] != 0xFFFF)
			dstate_setinfo("output.current", "%u", tab_reg[15] );
	}
	else {
		/* this a 3-phase model */
		dstate_setinfo("input.phases", "3" );

		dstate_setinfo("ups.load", "%u", tab_reg[3] );

		dstate_setinfo("ups.L1.load", "%u", tab_reg[0] );
		dstate_setinfo("ups.L2.load", "%u", tab_reg[1] );
		dstate_setinfo("ups.L3.load", "%u", tab_reg[2] );

		dstate_setinfo("input.bypass.L1-N.voltage", "%u", tab_reg[6] );
		dstate_setinfo("input.bypass.L2-N.voltage", "%u", tab_reg[7] );
		dstate_setinfo("input.bypass.L3-N.voltage", "%u", tab_reg[8] );

		dstate_setinfo("output.L1-N.voltage", "%u", tab_reg[9] );
		dstate_setinfo("output.L2-N.voltage", "%u", tab_reg[10] );
		dstate_setinfo("output.L3-N.voltage", "%u", tab_reg[11] );

		if (tab_reg[15] != 0xFFFF)
			dstate_setinfo("output.L1.current", "%u", tab_reg[15] );
		
		if (tab_reg[16] != 0xFFFF)
			dstate_setinfo("output.L2.current", "%u", tab_reg[16] );

		if (tab_reg[17] != 0xFFFF)
			dstate_setinfo("output.L3.current", "%u", tab_reg[17] );
	}

	if (tab_reg[4] != 0xFFFF)
		dstate_setinfo("battery.charge", "%u", tab_reg[4] );
	if (tab_reg[5] != 0xFFFF)
		dstate_setinfo("battery.capacity", "%u", (tab_reg[5]/10) );
	if (tab_reg[20] != 0xFFFF)
		dstate_setinfo("battery.voltage", "%.2f", (double) (tab_reg[20]) / 10);
	if (tab_reg[24] != 0xFFFF)
		dstate_setinfo("battery.current", "%.2f", (double) (tab_reg[24]) / 10 );
	if (tab_reg[23] != 0xFFFF)
		dstate_setinfo("battery.runtime", "%u", tab_reg[23] );
	
	if (tab_reg[18] != 0xFFFF)
		dstate_setinfo("input.bypass.frequency", "%u", (tab_reg[18]/10) );
	if (tab_reg[19] != 0xFFFF)
		dstate_setinfo("output.frequency", "%u", (tab_reg[19]/10) );

	if (tab_reg[22] != 0xFFFF) {
		dstate_setinfo("ups.temperature", "%u", tab_reg[22] );
		dstate_setinfo("ambient.1.present", "yes");
		dstate_setinfo("ambient.1.temperature", "%u", tab_reg[22] );
	}

	/* NOTE tab_reg[23] == 0xFFFF returns 0xFFFF all the time on the ITYS so need another way of detecting low batt */

	if ((DISCHARGING_FLAG == 1) && (tab_reg[4] < battery_charge_low)) {
		/* Discharging and Battery Level Below battery_charge_low so Set LB */
		upsdebugx(2, "Low Battery Condition (LB)");
		status_set("LB");
	}
 

	/*TODO:
	--essential
	ups.status TRIM/BOOST/OVER
	ups.alarm
	
	--dangerous
	ups.shutdown
	shutdown.return
	shutdown.stop
	shutdown.reboot
	shutdown.reboot.graceful
	bypass.start
	*/

	alarm_commit();
	status_commit();
	dstate_dataok();

	return;
}



void upsdrv_shutdown(void)
{
	int r;
	uint16_t val[16];
	
	uint8_t sch_delay_off_MSB = (uint8_t)((sch_delay_off & 0xFF00) >> 8);
	uint8_t sch_delay_off_LSB = (uint8_t)(sch_delay_off & 0x00FF);
	uint8_t sch_min_off_MSB = (uint8_t)((sch_min_off & 0xFF00) >> 8);
	uint8_t sch_min_off_LSB = (uint8_t)(sch_min_off & 0x00FF);

	val[0] = sch_delay_off_MSB;
	val[1] = sch_delay_off_LSB;
	val[2] = sch_min_off_MSB;
	val[3] = sch_min_off_LSB;
	val[4] = sch_scheduletype;

	//No logging per spec only upslogx(LOG_ERR, ...) or upslog_with_errno(LOG_ERR, ...)

	r = mwrs(modbus_ctx, 0x1580, 5, val);

	if (r == -1) {
		upslogx(LOG_ERR, "upsdrv_shutdown failed!");
		return;
	}
	else
		set_exit_flag(-2);	/* EXIT_SUCCESS */
}

void upsdrv_help(void)
{
}

/* list flags and values that you want to receive via -x */
void upsdrv_makevartable(void)
{
    addvar(VAR_VALUE, "ser_baud_rate", "serial port baud rate");
	addvar(VAR_VALUE, "ser_parity", "serial port parity");
	addvar(VAR_VALUE, "ser_data_bit", "serial port data bit");
	addvar(VAR_VALUE, "ser_stop_bit", "serial port stop bit");
	addvar(VAR_VALUE, "rio_slave_id", "Socomec modbus slave ID");

	addvar(VAR_VALUE, "battery_charge_low_percent", "Socomec Battery Charge Low [Percentage]");
	addvar(VAR_VALUE, "sch_delay_off_sec", "Socomec seconds that pass before UPS Off 20-600 [sec]");
	addvar(VAR_VALUE, "sch_min_off", "Socomec minutes of Stand-by 1-9999 [min]");
	addvar(VAR_VALUE, "scheduletype_1or4", "Socomec schedule type 1 Oneshot or 4 Schedule <default 4>");
}

void upsdrv_initups(void)
{
	int r;
	upsdebugx(2, "upsdrv_initups");

	get_config_vars();

	modbus_ctx = modbus_new_rtu(device_path, ser_baud_rate, ser_parity, ser_data_bit, ser_stop_bit);
	if (modbus_ctx == NULL)
		fatalx(EXIT_FAILURE, "Unable to create the libmodbus context");

	r = modbus_set_slave(modbus_ctx, rio_slave_id);	/* slave ID */
	if (r < 0) {
		modbus_free(modbus_ctx);
		fatalx(EXIT_FAILURE, "Invalid modbus slave ID %d",rio_slave_id);
	}

	if (modbus_connect(modbus_ctx) == -1) {
		modbus_free(modbus_ctx);
		fatalx(EXIT_FAILURE, "modbus_connect: unable to connect: %s", modbus_strerror(errno));
	}

}

void upsdrv_cleanup(void)
{
	if (modbus_ctx != NULL) {
		modbus_close(modbus_ctx);
		modbus_free(modbus_ctx);
	}
}

/* Modbus Read Input Registers */
static int mrir(modbus_t * arg_ctx, int addr, int nb, uint16_t * dest)
{
	int r, i;
	
	/* zero out the thing, because we might have reused it */
	for (i=0; i<nb; i++) {
		dest[i] = 0;
	}

	/*r = modbus_read_input_registers(arg_ctx, addr, nb, dest);*/
	r = modbus_read_registers(arg_ctx, addr, nb, dest);
	if (r == -1) {
		upslogx(LOG_ERR, "mrir: modbus_read_input_registers(addr:%d, count:%d): %s (%s)", addr, nb, modbus_strerror(errno), device_path);
	}
	return r;
}

static int mwrs(modbus_t *ctx, int addr, int nb, uint16_t *src)
{
	int r = -1;
	r = modbus_write_registers(ctx, addr, nb, src);
	
	if (r == -1) {
		upslogx(LOG_ERR, "mrir: modbus_write_registers(addr:%d, count:%d): %s (%s)", addr, nb, modbus_strerror(errno), device_path);
	}

	return r;

}

void get_config_vars(void)
{
    /* check if serial baud rate is set ang get the value */
	if (testvar("ser_baud_rate")) {
		ser_baud_rate = (int)strtol(getval("ser_baud_rate"), NULL, 10);
	}
	upsdebugx(2, "ser_baud_rate %d", ser_baud_rate);

	/* check if serial parity is set ang get the value */
	if (testvar("ser_parity")) {
		/* Dereference the char* we get */
		char *sp = getval("ser_parity");
		if (sp) {
			/* TODO? Sanity-check the char we get? */
			ser_parity = *sp;
		} else {
			upsdebugx(2, "Could not determine ser_parity, will keep default");
		}
	}
	upsdebugx(2, "ser_parity %c", ser_parity);

	/* check if serial data bit is set ang get the value */
	if (testvar("ser_data_bit")) {
		ser_data_bit = (int)strtol(getval("ser_data_bit"), NULL, 10);
	}
	upsdebugx(2, "ser_data_bit %d", ser_data_bit);

	/* check if serial stop bit is set ang get the value */
	if (testvar("ser_stop_bit")) {
		ser_stop_bit = (int)strtol(getval("ser_stop_bit"), NULL, 10);
	}
	upsdebugx(2, "ser_stop_bit %d", ser_stop_bit);

	/* check if device ID is set ang get the value */
	if (testvar("rio_slave_id")) {
		rio_slave_id = (int)strtol(getval("rio_slave_id"), NULL, 10);
	}
	upsdebugx(2, "rio_slave_id %d", rio_slave_id);


	/* Since Socomec does not allow us to store the timeouts in
	   Non-Volatile RAM we create some config vars in ups.conf
	   this allows us to override the defaults for low bat, delay
	   to turn off load and restore load. */

    /* check if battery charge low override is set ang get the value */
	if (testvar("battery_charge_low_percent")) {
		battery_charge_low = (int)strtol(getval("battery_charge_low_percent"), NULL, 10);
	}
	upsdebugx(2, "battery_charge_low %d", battery_charge_low);

    /* check if schedule delay off time override is set ang get the value */
	if (testvar("sch_delay_off_sec")) {
		sch_delay_off = (int)strtol(getval("sch_delay_off_sec"), NULL, 10);
	}
	upsdebugx(2, "sch_delay_off %d", sch_delay_off);

    /* check if schedule minutes off Stand-by Off time override is set ang get the value */
	if (testvar("sch_min_off")) {
		sch_min_off = (int)strtol(getval("sch_min_off"), NULL, 10);
	}
	upsdebugx(2, "sch_min_off %d", sch_min_off);

    /* check if schedule type override is set ang get the value */
	if (testvar("scheduletype_1or4")) {
		sch_scheduletype = (int)strtol(getval("scheduletype_1or4"), NULL, 10);
	}
	upsdebugx(2, "sch_scheduletype %d", sch_scheduletype);

}
