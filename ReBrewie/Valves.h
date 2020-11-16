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

// Valve structures
#ifdef  B20
uint8_t* Valve_Port[10] = { &PORTA, &PORTA, &PORTJ, &PORTJ, &PORTA, &PORTA, &PORTA, &PORTA, &PORTA, &PORTA };
uint8_t Valve_Bitmask[10] = { 0x01,   0x08,   0x02,   0x01,   0x10,   0x20,   0x40,   0x80,   0x02,   0x04 };
uint8_t valveState[10]    = {    1,      1,      1,      1,      1,      1,      1,      1,      1,      1 };
#else
uint8_t* Valve_Port[10] = { &PORTJ, &PORTJ, &PORTJ, &PORTJ, &PORTA, &PORTA, &PORTA, &PORTA, &PORTA, &PORTC };
uint8_t Valve_Bitmask[10] = { 0x04,   0x08,   0x20,   0x40,   0x20,   0x08,   0x80,   0x10,   0x40,   0x02 };
uint8_t valveState[10]    = {    1,      1,      1,      1,      1,      1,      1,      1,      1,      1 };
#endif

uint8_t valvePWM = 0;
uint8_t valveCount = 0;
