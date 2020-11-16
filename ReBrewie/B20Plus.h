#ifndef B20Plus
// Step Positions
#define STEP_WATER_INLET  1
#define STEP_MASH_INLET   2
#define STEP_BOIL_INLET   3
#define STEP_MASH_TEMP    4
#define STEP_BOIL_TEMP    5
#define STEP_HOP_1        6
#define STEP_HOP_2        7
#define STEP_HOP_3        8
#define STEP_HOP_4        9
#define STEP_COOL_VALVE   10
#define STEP_VALVE_11     11
#define STEP_OUTLET       12
#define STEP_MASH_PUMP    13
#define STEP_BOIL_PUMP    14
#define STEP_WATER        15
#define STEP_TIME         16
#define STEP_PRIMARY      17
#define STEP_SECONDARY    18
#define STEP_MASH_RETURN  19
#define STEP_BOIL_RETURN  20

// Power
#define PWR_EN_SERVO        13
#define PWR_EN_ARM          12
#define PWR_EN_5V           11
#define PWR_EN_12V          PORTH |= 0x80;

// Heaters
#define MASH_HEATER         4
#define BOIL_TEMP           40
#define BOIL_HEATER         PORTE |= 0x04
#define MASH_TEMP           41
#define BOIL_PORT           PORTE
#define BOIL_DIR            DDRE
#define BOIL_PIN            2

// Diagnostic
#define LED1                A2
#define LED2                A1
#define AC_MEAS             A8
#define BOARD_TEMP          A9
#define I_INLET1            A10
#define I_INLET2            A11
#define I_VALVES            A12
#define I_BOIL_PUMP         A13
#define I_MASH_PUMP         A14

// Pumps
#define BOIL_PUMP           32 
#define BOIL_PUMP_TACH      2       
#define MASH_PUMP           34
#define MASH_PUMP_TACH      3

#define SPEED_CTRL_MOSI     51
#define SPEED_CTRL_CS       10
#define SPEED_CTRL_SCLK     52
#define SPEED_CTRL_LDAC     9

// Solenoids
#define INLET_1             23
#define INLET_2             24

// Valves
#define VALVE_MASH_IN       0
#define VALVE_BOIL_RET      1
#define VALVE_OUTLET        2
#define VALVE_COOL          3
#define VALVE_HOP_4         4
#define VALVE_HOP_3         5
#define VALVE_HOP_2         6
#define VALVE_HOP_1         7
#define VALVE_MASH_RET      8
#define VALVE_BOIL_IN       9
#define VALVE_OPEN_ANGLE    25
#define VALVE_CLOSE_ANGLE   130
#define VALVE_OPEN          1
#define VALVE_CLOSE         0

// Fans
#define VENT_FAN            22
#define POWER_FAN           A0

// Buttons
#define POWER_BUTTON        38
#define DRAIN_BUTTON        19
#define POWER_LIGHT         44
#define DRAIN_LIGHT         30

#endif
