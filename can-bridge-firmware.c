#include "can-bridge-firmware.h"

//Because the MCP25625 transmit buffers seem to be able to corrupt messages (see errata), we're implementing
//our own buffering. This is an array of frames-to-be-sent, FIFO. Messages are appended to buffer_end++ as they
//come in and handled according to buffer_pos until buffer_pos == buffer_end, at which point both pointers reset
//the buffer size should be well in excess of what this device will ever see
can_frame_t tx0_buffer[TXBUFFER_SIZE];
uint8_t        tx0_buffer_pos        = 0;
uint8_t        tx0_buffer_end        = 0;

can_frame_t tx2_buffer[TXBUFFER_SIZE];
uint8_t        tx2_buffer_pos        = 0;
uint8_t        tx2_buffer_end        = 0;

can_frame_t tx3_buffer[5];
uint8_t        tx3_buffer_pos        = 0;
uint8_t        tx3_buffer_end        = 0;

volatile    uint8_t        can_busy                = 0;

#define vehicle_can_bus 1
#define primary_battery_can_bus 2
#define secondary_battery_can_bus 3

//timer variables
volatile    uint16_t    sec_timer            = 1;    //actually the same as ms_timer but counts down from 1000

static uint16_t primary_battery_current = 0;
static uint16_t secondary_battery_current = 0;
static uint8_t primary_battery_lb_soc = 255;
static uint8_t secondary_battery_lb_soc = 255;
static uint8_t primary_battery_lb_relay_cut_request = 0;
static uint8_t secondary_battery_lb_relay_cut_request = 0;
static uint8_t primary_battery_lb_failsafe_status = 0;
static uint8_t secondary_battery_lb_failsafe_status = 0;
static uint8_t primary_battery_lb_main_relay_on_flag = 0;
static uint8_t secondary_battery_lb_main_relay_on_flag = 1;
static uint8_t primary_battery_lb_full_charge_flag = 0;
static uint8_t secondary_battery_lb_full_charge_flag = 0;
static uint8_t primary_battery_lb_inter_lock = 0;
static uint8_t secondary_battery_lb_inter_lock = 1;
static uint8_t primary_battery_lb_discharge_power_status = 0;
static uint8_t secondary_battery_lb_discharge_power_status = 0;
static uint16_t primary_battery_lb_ir_sensor_wave_voltage = 0;
static uint16_t secondary_battery_lb_ir_sensor_wave_voltage = 0;

static uint16_t primary_battery_discharge_power_limit = 0;
static uint16_t secondary_battery_discharge_power_limit = 1023;
static uint16_t primary_battery_charge_power_limit = 0;
static uint16_t secondary_battery_charge_power_limit = 1023;
static uint16_t primary_battery_max_power_for_charger = 0;
static uint16_t secondary_battery_max_power_for_charger = 1023;
static uint8_t primary_battery_charge_power_status = 0;
static uint8_t secondary_battery_charge_power_status = 0;

static uint16_t primary_battery_gids = 0;
static uint16_t secondary_battery_gids = 502;
static uint16_t primary_battery_max_gids = 0;
static uint16_t secondary_battery_max_gids = 502;

#if 0
static uint16_t primary_battery_lb_full_capacity_for_qc = 0;
static uint16_t secondary_battery_lb_full_capacity_for_qc = 0;
static uint16_t primary_battery_lb_remain_capacity_for_qc = 0;
static uint16_t secondary_battery_lb_remain_capacity_for_qc = 0;
#endif

static uint16_t battery_lb_total_voltage = 0;
static uint16_t battery_max_power_for_charger = 0;

#define DISABLE_REGEN_IN_DRIVE

#ifdef DISABLE_REGEN_IN_DRIVE
static volatile uint8_t current_11A_shifter_state = 0;
static volatile uint8_t previous_11A_shifter_state = 0;
static volatile uint8_t disable_regen_toggle;
static volatile uint8_t eco_active = 0;
#endif

void hw_init(void){
    uint8_t caninit;

    /* Start the 32MHz internal RC oscillator and start the DFLL to increase it to 48MHz using the USB SOF as a reference */
    XMEGACLK_StartInternalOscillator(CLOCK_SRC_INT_RC32MHZ);
    XMEGACLK_StartDFLL(CLOCK_SRC_INT_RC32MHZ, DFLL_REF_INT_USBSOF, 48000000);        
    
    //turn off everything we don' t use
    PR.PRGEN        = PR_AES_bm | PR_RTC_bm | PR_DMA_bm;
    PR.PRPA            = PR_ADC_bm | PR_AC_bm;
    PR.PRPC            = PR_TWI_bm | PR_USART0_bm | PR_HIRES_bm;
    PR.PRPD            = PR_TWI_bm | PR_USART0_bm | PR_TC0_bm | PR_TC1_bm;
    PR.PRPE            = PR_TWI_bm | PR_USART0_bm;
    
    //blink output
    PORTB.DIRSET    = 3;
    
    //start 16MHz crystal and PLL it up to 48MHz
    OSC.XOSCCTRL    = OSC_FRQRANGE_12TO16_gc |        //16MHz crystal
    OSC_XOSCSEL_XTAL_16KCLK_gc;                        //16kclk startup
    OSC.CTRL       |= OSC_XOSCEN_bm;                //enable crystal
    while(!(OSC.STATUS & OSC_XOSCRDY_bm));            //wait until ready
    OSC.PLLCTRL        = OSC_PLLSRC_XOSC_gc | 2;        //XTAL->PLL, 2x multiplier (32MHz)
    OSC.CTRL       |= OSC_PLLEN_bm;                    //start PLL
    while (!(OSC.STATUS & OSC_PLLRDY_bm));            //wait until ready
    CCP                = CCP_IOREG_gc;                    //allow changing CLK.CTRL
    CLK.CTRL        = CLK_SCLKSEL_PLL_gc;            //use PLL output as system clock    
    
    //output 16MHz clock to MCP25625 chips (PE0)
    //next iteration: put this on some other port, pin  4 or 7, so we can use the event system
    TCE0.CTRLA        = TC0_CLKSEL_DIV1_gc;                        //clkper/1
    TCE0.CTRLB        = TC0_CCAEN_bm | TC0_WGMODE_SINGLESLOPE_bm;    //enable CCA, single-slope PWM
    TCE0.CCA        = 1;                                        //compare value
    TCE0.PER        = 1;                                        //period of 1, generates 24MHz output
    
    PORTE.DIRSET    = PIN0_bm;                                    //set CLKOUT pin to output
    
    //setup CAN pin interrupts
    PORTC.INTCTRL    = PORT_INT0LVL_HI_gc;
    PORTD.INTCTRL    = PORT_INT0LVL_HI_gc | PORT_INT1LVL_HI_gc;    
    
    PORTD.INT0MASK    = PIN0_bm;                        //PORTD0 has can1 interrupt
    PORTD.PIN0CTRL    = PORT_OPC_PULLUP_gc | PORT_ISC_LEVEL_gc;
    
    PORTD.INT1MASK    = PIN5_bm;                        //PORTD5 has can2 interrupt
    PORTD.PIN5CTRL    = PORT_OPC_PULLUP_gc | PORT_ISC_LEVEL_gc;
    
    PORTC.INT0MASK    = PIN2_bm;                        //PORTC2 has can3 interrupt
    PORTC.PIN0CTRL    = PORT_OPC_PULLUP_gc | PORT_ISC_LEVEL_gc;
    
    //buffer checking interrupt
    TCC1.CTRLA        = TC0_CLKSEL_DIV1_gc;            //32M/1/4800 ~ 100usec
    TCC1.PER        = 3200;
    TCC1.INTCTRLA    = TC0_OVFINTLVL_HI_gc;            //same priority as can interrupts
    
    //we want to optimize performance, so we're going to time stuff
    //48MHz/48=1us timer, which we just freerun and reset whenever we want to start timing something
    //frame time timer
    TCC0.CTRLA        = TC0_CLKSEL_DIV1_gc;
    TCC0.PER        = 32000;                        //32MHz/32000=1ms
    TCC0.INTCTRLA    = TC0_OVFINTLVL_HI_gc;            //interrupt on overflow
    
    PORTB.OUTCLR    = (1 << 0);
    
    can_system_init:
            
    //Init SPI and CAN interface:
    if(RST.STATUS & RST_WDRF_bm){ //if we come from a watchdog reset, we don't need to setup CAN
        caninit = can_init(MCP_OPMOD_NORMAL, 1); //on second thought, we do
    } else {
        caninit = can_init(MCP_OPMOD_NORMAL, 1);
    }
    
    if(caninit){        
        //PORTB.OUTSET |= (1 << 0);                    //green LED, uncommented to save power
    } else {        
        //PORTB.OUTSET |= (1 << 1);                    //red LED
        _delay_ms(10);
        goto can_system_init;
    }
    
    //Set and enable interrupts with round-robin
    XMEGACLK_CCP_Write((void * ) &PMIC.CTRL, PMIC_RREN_bm | PMIC_LOLVLEN_bm | PMIC_HILVLEN_bm);//PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm| PMIC_HILVLEN_bm;
    
    USB_Init(USB_OPT_RC32MCLKSRC | USB_OPT_BUSEVENT_PRILOW);

    wdt_enable(WDTO_15MS);
    
    sei();
}

void reset_state(){
#ifdef DISABLE_REGEN_IN_DRIVE
    current_11A_shifter_state = 0;
    previous_11A_shifter_state = 0;
    disable_regen_toggle = 0;
#endif  /* DISABLE_REGEN_IN_DRIVE */
}

int main(void){

    hw_init();

    while(1){
        //Setup complete, wait for can messages to trigger interrupts
    }
}

//fires every 1ms
ISR(TCC0_OVF_vect){    
    wdt_reset(); //Reset the watchdog
    sec_timer--; //Increment the 1000ms timer
    
    //fires every second (1000ms tasks go here)
    if(sec_timer == 0){
        PORTB.OUTCLR = (1 << 1);
    }
}

//fires approx. every 100us
ISR(TCC1_OVF_vect){
    check_can1();
    check_can2();
    check_can3();
}

//can1 interrupt
ISR(PORTD_INT0_vect){
    can_busy = 1;
    can_handler(1);
}

//can2 interrupt
ISR(PORTD_INT1_vect){
    can_busy = 1;
    can_handler(2);
}

//can3 receive interrupt
ISR(PORTC_INT0_vect){
    can_busy = 1;
    can_handler(3);
}

void can_handler(uint8_t can_bus){
    can_frame_t frame;
    uint8_t flag = can_read(MCP_REG_CANINTF, can_bus);
        
    if (flag & (MCP_RX0IF | MCP_RX1IF)){

        if(flag & MCP_RX1IF){ //prioritize the rollover buffer
            can_read_rx_buf(MCP_RX_1, &frame, can_bus);
            can_bit_modify(MCP_REG_CANINTF, MCP_RX1IF, 0x00, can_bus);
            } else {
            can_read_rx_buf(MCP_RX_0, &frame, can_bus);
            can_bit_modify(MCP_REG_CANINTF, MCP_RX0IF, 0x00, can_bus);
        }

#if 1
        // debug help: messages from batteries get added 100 respective 200 to the CAN id -> forward towards vehicle
        if (can_bus != vehicle_can_bus) {
            uint32_t can_id = frame.can_id;
            if (can_bus == primary_battery_can_bus)
                frame.can_id = 0x100 + can_id;
            else if (can_bus == secondary_battery_can_bus)
                frame.can_id = 0x200 + can_id;
            send_can(vehicle_can_bus, frame);
            frame.can_id = can_id;
        }
#endif
        
        switch(frame.can_id){
            
            case 0x1dc: {
                uint16_t LB_Discharge_Power_Limit =  (frame.data[0] << 2) + ((frame.data[1] & 0xc0) >> 6);
                uint16_t LB_Charge_Power_Limit = ((frame.data[1] & 0x3f) << 4) + ((frame.data[2] & 0xf0) >> 4);
                uint16_t LB_MAX_POWER_FOR_CHARGER = ((frame.data[2] & 0x0f) << 6) + ((frame.data[3] & 0xfc) >> 2);
                uint8_t LB_Charge_Power_Status = frame.data[3] & 0x03;

                if (can_bus == primary_battery_can_bus) {
                    primary_battery_discharge_power_limit = LB_Discharge_Power_Limit;
                    primary_battery_charge_power_limit = LB_Charge_Power_Limit;
                    primary_battery_max_power_for_charger = LB_MAX_POWER_FOR_CHARGER;
                    primary_battery_charge_power_status = LB_Charge_Power_Status;
                }
                else if (can_bus == secondary_battery_can_bus) {
                    secondary_battery_discharge_power_limit = LB_Discharge_Power_Limit;
                    secondary_battery_charge_power_limit = LB_Charge_Power_Limit;
                    secondary_battery_max_power_for_charger = LB_MAX_POWER_FOR_CHARGER;
                    secondary_battery_charge_power_status = LB_Charge_Power_Status;
                }

                // increase charging speed, we have two batteries after all...
                if (primary_battery_max_power_for_charger != 1023 && secondary_battery_max_power_for_charger != 1023 && battery_lb_total_voltage != 0 && primary_battery_current < 1024 && secondary_battery_current < 1024) {
                    uint16_t combined_max_power_for_charger = (primary_battery_max_power_for_charger + secondary_battery_max_power_for_charger) - 100;
                    int32_t max_power_bat0_watt = primary_battery_max_power_for_charger * 100 - 10000;
                    int32_t max_power_bat1_watt = secondary_battery_max_power_for_charger * 100 - 10000;
                    double max_power_bat0_amp = max_power_bat0_watt / (battery_lb_total_voltage * 0.5);
                    double max_power_bat1_amp = max_power_bat1_watt / (battery_lb_total_voltage * 0.5);
                    double bat0_amp = primary_battery_current / 2.0;
                    double bat1_amp = secondary_battery_current / 2.0;

                    // Begin with the fail-safe version: Minimum of both charger powers is it
                    if (battery_max_power_for_charger == 0) {
                        battery_max_power_for_charger = MIN(primary_battery_max_power_for_charger, secondary_battery_max_power_for_charger);
                    }

                    // only change the max charging speed every 250ms with 100W, almost as Leaf batteries do
                    static int counter = 0;
                    if ((counter++ % 25) == 0) {
                        // If both batteries have amperage below 80% of desired maximum, slowly raise the power by 100W
                        if (bat0_amp < max_power_bat0_amp * 0.8 && bat1_amp < max_power_bat1_amp * 0.8) {
                            battery_max_power_for_charger = MIN(MIN(1000, combined_max_power_for_charger), battery_max_power_for_charger + 1);
                        }
                        // If one of the batteries is above 90% of desired maximum, begin to lower by 100W
                        else if (bat0_amp > max_power_bat0_amp * 0.9 || bat1_amp > max_power_bat1_amp * 0.9) {
                            battery_max_power_for_charger = MAX(MIN(primary_battery_max_power_for_charger, secondary_battery_max_power_for_charger), battery_max_power_for_charger - 1);
                            // As long as both are below: Stick to exactly that value!
                            if (bat0_amp < max_power_bat0_amp && bat1_amp < max_power_bat1_amp) {
                                battery_max_power_for_charger = (((bat0_amp + bat1_amp) * battery_lb_total_voltage * 0.5) + 1) / 100;
                            }
                        }
                    }
                    LB_MAX_POWER_FOR_CHARGER = battery_max_power_for_charger;
                } else {
                    battery_max_power_for_charger = 0;
                    LB_MAX_POWER_FOR_CHARGER = MIN(primary_battery_max_power_for_charger, secondary_battery_max_power_for_charger);
                }

                // Now set the minimum for each max power rating
                LB_Discharge_Power_Limit = MIN(primary_battery_discharge_power_limit, secondary_battery_discharge_power_limit);
                LB_Charge_Power_Limit = MIN(primary_battery_charge_power_limit, secondary_battery_charge_power_limit);
                LB_Charge_Power_Status = primary_battery_charge_power_status | secondary_battery_charge_power_status;

#ifdef DISABLE_REGEN_IN_DRIVE
                if (disable_regen_toggle && (current_11A_shifter_state == SHIFT_D) && !eco_active) {
                    LB_Charge_Power_Limit = 0;
                }
#endif

                frame.data[0] = LB_Discharge_Power_Limit >> 2;
                frame.data[1] = ((LB_Discharge_Power_Limit & 0x03) << 6) | (LB_Charge_Power_Limit >> 4);
                frame.data[2] = ((LB_Charge_Power_Limit & 0x0f) << 4) | (LB_MAX_POWER_FOR_CHARGER >> 6);
                frame.data[3] = ((LB_MAX_POWER_FOR_CHARGER & 0x3f) << 2) | LB_Charge_Power_Status;
                calc_crc8(&frame);
            }
            break;

            case 0x1db: {
                uint16_t LB_Current = (frame.data[0] << 3) | ((frame.data[1] & 0xe0) >> 5);
                uint8_t LB_Usable_SOC = (frame.data[4] & 0x7f);

                uint8_t LB_Relay_Cut_Request = (frame.data[1] & 0x18) >> 3;
                uint8_t LB_Failsafe_status = (frame.data[1] & 0x07);
                uint8_t LB_MainRelayOn_flag = (frame.data[3] & 0x20) >> 5;
                uint8_t LB_Full_CHARGE_flag = (frame.data[3] & 0x10) >> 4;
                uint8_t LB_INTER_LOCK = (frame.data[3] & 0x08) >> 3;
                uint8_t LB_Discharge_Power_Status = (frame.data[3] & 0x06) >> 1;
                uint16_t LB_Total_Voltage = (frame.data[2] << 2) | ((frame.data[3] & 0xc0) >> 6);

                if (can_bus == primary_battery_can_bus) {
                    primary_battery_current = LB_Current;
                    primary_battery_lb_soc = LB_Usable_SOC;
                    primary_battery_lb_relay_cut_request = LB_Relay_Cut_Request;
                    primary_battery_lb_failsafe_status = LB_Failsafe_status;
                    primary_battery_lb_main_relay_on_flag = LB_MainRelayOn_flag;
                    primary_battery_lb_full_charge_flag = LB_Full_CHARGE_flag;
                    primary_battery_lb_inter_lock = LB_INTER_LOCK;
                    primary_battery_lb_discharge_power_status = LB_Discharge_Power_Status;
                    battery_lb_total_voltage = LB_Total_Voltage;
                }
                else if (can_bus == secondary_battery_can_bus) {
                    secondary_battery_current = LB_Current;
                    secondary_battery_lb_soc = LB_Usable_SOC;
                    secondary_battery_lb_relay_cut_request = LB_Relay_Cut_Request;
                    secondary_battery_lb_failsafe_status = LB_Failsafe_status;
                    // TODO: ignore that flag so far, don't know where it comes from :-/
                    if (secondary_battery_lb_failsafe_status == 4)
                        secondary_battery_lb_failsafe_status = 0;
                    secondary_battery_lb_main_relay_on_flag = LB_MainRelayOn_flag;
                    secondary_battery_lb_full_charge_flag = LB_Full_CHARGE_flag;
                    secondary_battery_lb_inter_lock = LB_INTER_LOCK;
                    secondary_battery_lb_discharge_power_status = LB_Discharge_Power_Status;
                }

                // Add the current
                LB_Current = primary_battery_current + secondary_battery_current;

                // SOC is calculated as weighted average
                if (primary_battery_lb_soc != 255 && secondary_battery_lb_soc != 255) {
		     // average as [0.0-1.0]
                    float average_soc = (primary_battery_lb_soc + secondary_battery_lb_soc) / 200.0;
                    LB_Usable_SOC = MAX(primary_battery_lb_soc, secondary_battery_lb_soc) * average_soc + MIN(primary_battery_lb_soc, secondary_battery_lb_soc) * (1.0 - average_soc) + 0.5;
                }

                // and merge the flags
                LB_Relay_Cut_Request = primary_battery_lb_relay_cut_request | secondary_battery_lb_relay_cut_request;
                LB_Failsafe_status = primary_battery_lb_failsafe_status | secondary_battery_lb_failsafe_status;
                LB_MainRelayOn_flag = primary_battery_lb_main_relay_on_flag & secondary_battery_lb_main_relay_on_flag;
                LB_Full_CHARGE_flag = primary_battery_lb_full_charge_flag | secondary_battery_lb_full_charge_flag;
                LB_INTER_LOCK = primary_battery_lb_inter_lock & secondary_battery_lb_inter_lock;
                LB_Discharge_Power_Status = primary_battery_lb_discharge_power_status | secondary_battery_lb_discharge_power_status;


                frame.data[0] = (LB_Current & 0x7f8) >> 3;
                frame.data[1] = ((LB_Current & 0x07) << 5) | (LB_Relay_Cut_Request << 3) | LB_Failsafe_status;
                frame.data[3] = (frame.data[3] & 0xc1) | (LB_MainRelayOn_flag << 5) | (LB_Full_CHARGE_flag << 4) | (LB_INTER_LOCK << 3) | (LB_Discharge_Power_Status << 1);
                frame.data[4] = (frame.data[4] & 0x80) | LB_Usable_SOC;
                calc_crc8(&frame);
            }
            break;

            case 0x5bc: {
                const bool max_gids = (frame.data[5] & 0x10) != 0;
                uint16_t gids = (frame.data[0] << 2) + ((frame.data[1] & 0xc0) >> 6);
                if (max_gids) {
                    if (can_bus == primary_battery_can_bus)
                        primary_battery_max_gids = gids;
                    else if (can_bus == secondary_battery_can_bus)
                        secondary_battery_max_gids = gids;
                }
                else {
                    if (can_bus == primary_battery_can_bus)
                        primary_battery_gids = gids;
                    else if (can_bus == secondary_battery_can_bus)
                        secondary_battery_gids = gids;
                }

                // Now add the remaining GIDS
                gids = max_gids ? (primary_battery_max_gids + secondary_battery_max_gids) : (primary_battery_gids + secondary_battery_gids);

                frame.data[0] = gids >> 2;
                frame.data[1] = (gids << 6) & 0xc0;
            }
            break;

#if 0
            case 0x59e: {
                uint16_t LB_Full_Capacity_for_QC = ((frame.data[2] & 0x1f) << 4) + ((frame.data[3] & 0xf0) >> 4);
                uint16_t LB_Remain_Capacity_for_QC = ((frame.data[3] & 0x0f) << 5) + ((frame.data[4] & 0xf8) >> 3);

                if (can_bus == primary_battery_can_bus) {
                    primary_battery_lb_full_capacity_for_qc = LB_Full_Capacity_for_QC;
                    primary_battery_lb_remain_capacity_for_qc = LB_Remain_Capacity_for_QC;
                }
                else if (can_bus == secondary_battery_can_bus) {
                    secondary_battery_lb_full_capacity_for_qc = LB_Full_Capacity_for_QC;
                    secondary_battery_lb_remain_capacity_for_qc = LB_Remain_Capacity_for_QC;
                }

                // Now add the capacity values
                LB_Full_Capacity_for_QC = primary_battery_lb_full_capacity_for_qc + secondary_battery_lb_full_capacity_for_qc;
                LB_Remain_Capacity_for_QC = primary_battery_lb_remain_capacity_for_qc + secondary_battery_lb_remain_capacity_for_qc;

                frame.data[2] = (frame.data[2] & 0xe0) | ((LB_Full_Capacity_for_QC & 0x1f0) >> 4);
                frame.data[3] = ((LB_Full_Capacity_for_QC & 0x00f) << 4) | ((LB_Remain_Capacity_for_QC & 0x1e0) >> 5);
                frame.data[4] = ((LB_Remain_Capacity_for_QC & 0x01f) << 3) | (frame.data[4] & 0x07);
                calc_crc8(&frame);
            }
            break;
#endif

            case 0x55b: {
                uint16_t LB_IR_Sensor_Wave_Voltage = (frame.data[4] << 2) + ((frame.data[5] & 0xc0) >> 6);

                if (can_bus == primary_battery_can_bus) {
                    primary_battery_lb_ir_sensor_wave_voltage = LB_IR_Sensor_Wave_Voltage;
                }
                else if (can_bus == secondary_battery_can_bus) {
                    secondary_battery_lb_ir_sensor_wave_voltage = LB_IR_Sensor_Wave_Voltage;
                }

                if (primary_battery_lb_ir_sensor_wave_voltage != 0 && secondary_battery_lb_ir_sensor_wave_voltage != 0) {
                    LB_IR_Sensor_Wave_Voltage = MAX(primary_battery_lb_ir_sensor_wave_voltage, secondary_battery_lb_ir_sensor_wave_voltage);
                }

                // Double the insulation resistance if too low:
                if (LB_IR_Sensor_Wave_Voltage < 512) {
                    LB_IR_Sensor_Wave_Voltage *= 2;
                }
                else if (LB_IR_Sensor_Wave_Voltage < 640) {
                    LB_IR_Sensor_Wave_Voltage *= 1.5;
                }

                frame.data[4] = (LB_IR_Sensor_Wave_Voltage & 0x3fc) >> 2;
                frame.data[5] = ((LB_IR_Sensor_Wave_Voltage & 0x03) << 6) | (frame.data[5] & 0x3f);

                calc_crc8(&frame);
            }
            break;

#ifdef DISABLE_REGEN_IN_DRIVE
            case 0x11a: {
                current_11A_shifter_state = frame.data[0] & 0xf0;
                eco_active = (frame.data[1] & 0x10) >> 4;

                if (previous_11A_shifter_state == SHIFT_D) {
                    // If we go from D to N, toggle the regen disable feature
                    if (current_11A_shifter_state == SHIFT_N) {
                        if (disable_regen_toggle == 0) {
                            disable_regen_toggle = 1;
                            //timeToSetCapacityDisplay = FADE_OUT_CAP_AFTER_SETTING_REGEN;
                            //SetCapacityDisplay = 2;
                        }
                       else {
                            disable_regen_toggle = 0;
                            //timeToSetCapacityDisplay = FADE_OUT_CAP_AFTER_SETTING_REGEN;
                            //SetCapacityDisplay = 4;
                        }
                    }
                }

                previous_11A_shifter_state =current_11A_shifter_state;
            }
            break;
#endif

#if 0
            case 0x390: {
                uint8_t OBC_Flag_QC_IR_Sensor = (frame.data[5] & 0x80) >> 7;
                OBC_Flag_QC_IR_Sensor = 0;
                frame.data[5] = (OBC_Flag_QC_IR_Sensor << 7) | (frame.data[5] & 0x7f);
            }
            break;
#endif

            default:
            break;
            }
        
        
        //block unwanted messages
            uint8_t blocked = 0;
            switch(frame.can_id){
#if 0
                case 0x633:    //new 40kWh message, block to save CPU
                    blocked = 1;
#endif
                default:
                    blocked = 0;
                    break;
            }
            if(!blocked){

                static const int debug_battery = secondary_battery_can_bus;

                switch (can_bus) {
                    // signals from vehicle are sent to both batteries
                    case vehicle_can_bus:
                        send_can(primary_battery_can_bus, frame);
                        send_can(secondary_battery_can_bus, frame);
                        break;
                    // signals from primary battery are sent to vehicle, but not poll answers!
                    case primary_battery_can_bus:
                        if (frame.can_id != 0x7bb || debug_battery == primary_battery_can_bus)
                            send_can(vehicle_can_bus, frame);
                        break;
                    // only poll answers are forwarded from secondary battery to vehicle
                    case secondary_battery_can_bus:
                        if (frame.can_id == 0x7bb && debug_battery == secondary_battery_can_bus)
                            send_can(vehicle_can_bus, frame);
                        break;
                    default:
                        break;
                }
            }
        }        
    
    if(flag & 0xA0){
        uint8_t flag2 = can_read(MCP_REG_EFLG, can_bus);
        if(flag2 & 0xC0){
            can_write(MCP_REG_EFLG, 0, can_bus); //reset all errors, we hit an CANX RX OVF
        }
        if(flag2 > 0){ PORTB.OUTSET = (1 << 1); }
        if(flag & 0xE0){ can_bit_modify(MCP_REG_CANINTF, (flag & 0xE0), 0x00, can_bus);    }
    }
    can_busy = 0;
}


void send_can(uint8_t can_bus, can_frame_t frame){
    if(can_bus == 1) send_can1(frame);
    if(can_bus == 2) send_can2(frame);
    if(can_bus == 3) send_can3(frame);
}

void send_can1(can_frame_t frame){    
    //put in the buffer
    memcpy(&tx0_buffer[tx0_buffer_end++], &frame, sizeof(frame));
    
    if(tx0_buffer_end >= TXBUFFER_SIZE){ //silently handle buffer overflows
        tx0_buffer_end = TXBUFFER_SIZE - 1;
    }
    
    check_can1();
}



void check_can1(void){
    uint8_t reg;
    
    if(tx0_buffer_end != tx0_buffer_pos){
        //check if TXB0 is free use
        reg = can1_read(MCP_REG_TXB0CTRL);
    
        if(!(reg & MCP_TXREQ_bm)){ //we're free to send
            can1_load_txbuff(0, (can_frame_t *) &tx0_buffer[tx0_buffer_pos++]);
            can1_rts(0);
            if(tx0_buffer_pos == tx0_buffer_end){ //end of buffer, reset
                tx0_buffer_end = 0;
                tx0_buffer_pos = 0;
            }
        }
    }
}

void send_can2(can_frame_t frame){
    //put in the buffer
    memcpy(&tx2_buffer[tx2_buffer_end++], &frame, sizeof(frame));
    
    if(tx2_buffer_end >= TXBUFFER_SIZE){ //silently handle buffer overflows
        tx2_buffer_end = TXBUFFER_SIZE - 1;
    }
    
    check_can2();
}

void check_can2(void){
    uint8_t reg;
    
    if(tx2_buffer_end != tx2_buffer_pos){
        //check if TXB0 is free use
        reg = can2_read(MCP_REG_TXB0CTRL);
        
        if(!(reg & MCP_TXREQ_bm)){ //we're free to send
            can2_load_txbuff(0, (can_frame_t *) &tx2_buffer[tx2_buffer_pos++]);
            can2_rts(0);
            if(tx2_buffer_pos == tx2_buffer_end){ //end of buffer, reset
                tx2_buffer_end = 0;
                tx2_buffer_pos = 0;
            }
        }
    }
}

void send_can3(can_frame_t frame){
    //put in the buffer
    memcpy(&tx3_buffer[tx3_buffer_end++], &frame, sizeof(frame));
    
    if(tx3_buffer_end >= TXBUFFER_SIZE){ //silently handle buffer overflows
        tx3_buffer_end = TXBUFFER_SIZE - 1;
    }
    
    check_can3();
}

void check_can3(void){
    uint8_t reg;
    
    if(tx3_buffer_end != tx3_buffer_pos){
        //check if TXB0 is free use
        reg = can3_read(MCP_REG_TXB0CTRL);
        
        if(!(reg & MCP_TXREQ_bm)){ //we're free to send
            can3_load_txbuff(0, (can_frame_t *) &tx3_buffer[tx3_buffer_pos++]);
            can3_rts(0);
            if(tx3_buffer_pos == tx3_buffer_end){ //end of buffer, reset
                tx3_buffer_end = 0;
                tx3_buffer_pos = 0;
            }
        }
    }
}
