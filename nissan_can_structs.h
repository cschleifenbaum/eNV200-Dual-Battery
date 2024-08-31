
typedef struct {
	int			LB_CAPR:10;		//LB_Remain_Capacity or more commonly known as GIDS
	int			LB_FULLCAP:10;	//LB_New_Full_Capacity, Wh 20000 - 24000 [offset 250, factor 80]
	int			LB_CAPSEG:4;	//LB_Remaining_Capacity_Segment, 0-12 bars [SWITCHES BETWEEN REMAINING & FULL]
	int			LB_AVET:8;		//LB_Average_Battery_Temperature, -40 - 105*C [offset -40]
	int			LB_SOH:7;		//LB_Capacity_Deterioration_Rate, 0 - 100% SOH
	int			LB_CAPSW:1;		//LB_Remaining_Capaci_Segment_Switch, controls LB_CAPSEG
	int			LB_RLIMIT:3;	//LB_Output_Power_Limit_Reason
	int			SPACER:2;		//Empty
	int			LB_CAPBALCOMP:1;//LB_Capacity_Bal_Complete_Flag, signals that capacity balancing is complete
	int			LB_RCHGTCON:5;	//LB_Remain_charge_time_condition, Mux for LB_Remain_charge_time
	int			LB_RCHGTIM:13;	//LB_Remain_charge_time, 0 - 8190 minutes
} Leaf_2011_5BC_message;

typedef struct {
	int			LB_HIS_DATA_SW:2;
	int			SPACER_1:2;
	int			LB_HIS_HLVOL_TIMS:4;
	int			LB_HIS_TEMP_WUP:7;
	int			SPACER_2:1;
	int			LB_HIS_TEMP:7;
	int			SPACER_3:1;
	int			LB_HIS_INTG_CUR:8;
	int			LB_HIS_DEG_REGI:7;
	int			SPACER_4:1;
	int			LB_HIS_CELL_VOL:6;
	int			SPACER_5:10;
	int			LB_DTC:8;
} Leaf_2011_5C0_message;

typedef struct {
	uint32_t	can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
	uint8_t		can_dlc; /* frame payload length in byte (0 .. CAN_MAX_DLEN) */
    int SPACER_0:3;
    int SPACER_1:16;
    uint32_t LB_Full_Capacity_for_QC:9;
    uint32_t LB_Remain_Capacity_for_QC:9;
} Battery_59e_message;

typedef struct {
	uint32_t	can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
	uint8_t		can_dlc; /* frame payload length in byte (0 .. CAN_MAX_DLEN) */
    uint32_t LB_Remain_Capacity_GIDS:10;
    int SPACER_0:6;
    uint32_t LB_Remaining_Capacity_Segments:8;
    uint32_t LB_Temperature_Segment_For_Dash:8;
    uint32_t LB_Capacity_Deterioration_Rate:7;
    uint32_t LB_Remain_Cap_Segment_Swit_Flag:1;
    uint32_t LB_Output_Power_Limit_Reason:3;
    uint32_t LB_MaxGIDS:1;
    int SPACER_1:2;
    uint32_t LB_Remain_Charge_Time_Condition:5;
    uint32_t LB_Remain_Charge_Time:13;
} Battery_5bc_message;

typedef struct {
	uint32_t	can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
	uint8_t		can_dlc; /* frame payload length in byte (0 .. CAN_MAX_DLEN) */
    uint32_t LB_Current:11;
    uint32_t LB_Relay_Cut_Request:2;
    uint32_t LB_Failsafe_Status:3;
    uint32_t LB_Total_Voltage:10;
    uint32_t LB_MainRelayOn_flag:1;
    uint32_t LB_Full_CHARGE_flag:1;
    uint32_t LB_INTER_LOCK:1;
    uint32_t LB_Discharge_Power_Status:2;
    uint32_t LB_Voltage_Latch_Flag:1;
    int SPACER_0:1;
    uint32_t LB_Usable_SOC:7;
    int SPACER_1:14;
} Battery_1db_message;

typedef struct {
	uint32_t	can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
	uint8_t		can_dlc; /* frame payload length in byte (0 .. CAN_MAX_DLEN) */
    uint32_t LB_Discharge_Power_Limit:10;
    uint32_t LB_Charge_Power_Limit:10;
    uint32_t LB_MAX_POWER_FOR_CHARGER:10;
} Battery_1dc_message;