#include "Notifications.h"

char brewieMessage[4][5];      // Standard message buffer to report commands to Olimex
uint8_t notificationCount = 0;         // Count variable if multiple messages or errors arise
bool notificationsEnabled = true;

bool AddNotification(char message[5]) {
  //String test = &message[1];
  //test.toInt();
  bool added = false;
  if (message[0] == 'P' && notificationCount < 4) {
    sprintf(brewieMessage[notificationCount], message);
    notificationCount++;
    added = true;
  } else if (notificationCount < 3 && notificationsEnabled) {
    sprintf(brewieMessage[notificationCount], message);
    notificationCount++;
    added = true;
  }
  return added;
}

void ClearNotifications() {
  notificationCount = 0;
}

void EnableNotifications(bool enable) {
  notificationsEnabled = enable;
}

