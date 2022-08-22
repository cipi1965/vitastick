#ifndef VITASTICK_UAPI_H
#define VITASTICK_UAPI_H

#ifdef __cplusplus
extern "C" {
#endif

#define VITASTICK_ERROR_DRIVER_NOT_REGISTERED		0x91337000
#define VITASTICK_ERROR_DRIVER_NOT_ACTIVATED		0x91337001
#define VITASTICK_ERROR_DRIVER_ALREADY_ACTIVATED	0x91337002

#define CTRL_L2     0x00
#define CTRL_R2     0x01
#define CTRL_L3     0x02
#define CTRL_R3     0x03

int vitastick_start(void);
int vitastick_stop(void);
int upload_trigger_state(uint8_t triggers);
int upload_battery_strength(uint8_t strength);

#ifdef __cplusplus
}
#endif


#endif
