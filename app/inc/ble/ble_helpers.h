#ifndef BLE_HELPERS_h
#define BLE_HELPERS_H

#include <bluetooth/uuid.h>


/**
 * @brief Create a bt_uuid from a human readable UUID string.
 * 
 * NOTE: Human readable UUID string is one given in the form given by 
 *       bt_uuid_to_str().
 *       e.g.: "7aaf1b67-f3c0-4a54-b314-58fff1960a40"
 * 
 * NOTE: A UUID of the form "7aaf1b67f3c04a54b31458fff1960a40" should be valid.
 *       However, this is **UNTESTED**.
 * 
 * @param uuid Storage for created UUID
 * @param uuid_str Human readable UUID string
 * @return 0 on success, else negative error code
 */
int bt_uuid_from_str(struct bt_uuid *uuid, char *uuid_str);

#endif /* BLE_HELPERS_H */