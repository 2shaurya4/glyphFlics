#include "bno055_priv.h"

/**
 * @brief Internal function to convert a bno055_operation_mode_t to a string.
 *
 * This function is private and should not be used directly.
 *
 * @param[in] operation_mode The operation mode to be converted to a string.
 *
 * @return A pointer to a string containing the operation mode name.
 */
char *operation_mode_to_string(uint8_t operation_mode)
{
    switch (operation_mode)
    {
    case 0:
        return "CONFIG_MODE";

    case 1:
        return "ACC_ONLY_MODE";

    case 2:
        return "MAG_ONLY_MODE";

    case 3:
        return "GYRO_ONLY_MODE";

    case 4:
        return "ACC_MAG_MODE";

    case 5:
        return "ACC_GYRO_MODE";

    case 6:
        return "MAG_GYRO_MODE";

    case 7:
        return "AMG_MODE";

    case 8:
        return "IMU_MODE";

    case 9:
        return "COMPASS_MODE";

    case 10:
        return "M4G_MODE";

    case 11:
        return "NDOF_FMC_OFF_MODE";

    case 12:
        return "NDOF_MODE";

    default:
        return "UNKNOWN_MODE";
    }
}