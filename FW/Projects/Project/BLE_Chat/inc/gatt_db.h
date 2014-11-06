

#ifndef _GATT_DB_H_
#define _GATT_DB_H_

#include <lis3dh_driver.h>

tBleStatus Add_Chat_Service(void);
void Attribute_Modified_CB(tHalUint16 handle, tHalUint8 data_length, tHalUint8 *att_data);

#endif /* _GATT_DB_H_ */