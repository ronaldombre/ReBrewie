#ifndef B20
#define B20

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

// Heaters
#define MASH_HEATER         13
#define BOIL_TEMP           10
#define BOIL_HEATER         4
#define MASH_TEMP           11

// Diagnostic
#define LED1                41
#define LED2                40

#define AC_MEAS             A8
#define BOARD_TEMP          A9
#define I_INLET1            A10
#define I_INLET2            A11
#define I_VALVES            A12
#define I_BOIL_PUMP         A13
#define I_MASH_PUMP         A14

// Pumps
#define BOIL_PUMP           2      
#define MASH_PUMP           5

// Solenoids
#define INLET_1             7
#define INLET_2             6

// Fans
#define VENT_FAN            46
#define POWER_FAN           8

// Buttons
#define POWER_BUTTON        38
#define DRAIN_BUTTON        19
#define POWER_LIGHT         44
#define DRAIN_LIGHT         30

#endif
