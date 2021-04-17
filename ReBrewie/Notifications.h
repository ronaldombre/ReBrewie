#ifndef Notifications_h
#define Notifications_h

#include <Arduino.h>

extern char brewieMessage[4][5];              // Standard message buffer to report commands to Olimex
extern uint8_t notificationCount;             // Count variables if multiple commands or errors arise
extern bool notificationsEnabled;
//uint8_t ErrorList[10][2];
//uint8_t errorCount;  

bool AddNotification(char message[5]);        // Add notification to status messages
void ClearNotifications();                    // Clear variables between each status update
void EnableNotifications(bool);               // Enable or disable notifications 

#endif
