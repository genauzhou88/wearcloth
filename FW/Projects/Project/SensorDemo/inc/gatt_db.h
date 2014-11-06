
#ifndef _GATT_DB_H_
#define _GATT_DB_H_

#include <lis3dh_driver.h>

tBleStatus Add_Acc_Service(void);
void Read_Request_CB(tHalUint16 handle);
tBleStatus Free_Fall_Notify(void);
tBleStatus Acc_Update(AxesRaw_t *data);

tBleStatus Add_Environmental_Sensor_Service(void);

extern volatile tHalUint8 request_free_fall_notify;

#endif /* _GATT_DB_H_ */