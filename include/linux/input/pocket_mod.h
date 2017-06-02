#ifndef _LINUX_POCKET_MOD_H
#define _LINUX_POCKET_MOD_H

int epl259x_pocket_detection_check(void); // If proximity sensor return value 0-65536

int device_is_pocketed(void); // Return value 1 for close or 0 for open

#endif // _LINUX_POCKET_MOD_H
