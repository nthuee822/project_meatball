/*************************************************************************
  Raspberry Pi 5 / Linux I2C port for the ICM-20948 DMP wrapper.

  The original file was an Arduino HAL using Wire/SPI/Serial. This version
  keeps the public ArduinoICM20948 class and InvenSense DMP integration, but
  replaces the transport and timing hooks with Linux /dev/i2c-* calls.
*************************************************************************/

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <string>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "Arduino-ICM20948.h"

extern "C" {
#include "Icm20948.h"
#include "Icm20948MPUFifoControl.h"
#include "SensorTypes.h"
}

/*************************************************************************
  Linux I2C configuration
*************************************************************************/

namespace {

constexpr uint8_t EXPECTED_WHOAMI = 0xEA;
constexpr uint8_t WHO_AM_I_REG = 0x00;
constexpr uint8_t DEFAULT_I2C_ADDR_REVA = 0x68;
constexpr uint8_t DEFAULT_I2C_ADDR_REVB = 0x69;
constexpr const char *DEFAULT_I2C_DEV = "/dev/i2c-1";
constexpr int ICM20948_I2C_RETRY_COUNT = 3;
constexpr int I2C_RETRY_DELAY_US = 1000;

int i2c_fd = -1;
uint8_t i2c_address = DEFAULT_I2C_ADDR_REVB;
bool is_interface_spi = false;
int requested_i2c_speed = 0;

void log_error(const char *message)
{
    std::fprintf(stderr, "ICM20948 ERROR: %s\n", message);
}

void log_errno(const char *message)
{
    std::fprintf(stderr, "ICM20948 ERROR: %s: %s\n", message, std::strerror(errno));
}

std::string get_i2c_device_path()
{
    const char *env_path = std::getenv("ICM20948_I2C_DEV");
    if (env_path && env_path[0] != '\0') {
        return std::string(env_path);
    }

    const char *env_bus = std::getenv("ICM20948_I2C_BUS");
    if (env_bus && env_bus[0] != '\0') {
        return std::string("/dev/i2c-") + env_bus;
    }

    return DEFAULT_I2C_DEV;
}

bool parse_i2c_address_from_env(uint8_t *address)
{
    const char *env_addr = std::getenv("ICM20948_I2C_ADDR");
    if (!env_addr || env_addr[0] == '\0') {
        return false;
    }

    char *end = nullptr;
    const long parsed = std::strtol(env_addr, &end, 0);
    if (end == env_addr || parsed < 0x03 || parsed > 0x77) {
        std::fprintf(stderr, "ICM20948 ERROR: invalid ICM20948_I2C_ADDR='%s'\n", env_addr);
        return false;
    }

    *address = static_cast<uint8_t>(parsed);
    return true;
}

int i2c_read_register_at(uint8_t address, uint8_t reg, uint8_t *buffer, uint32_t length)
{
    if (i2c_fd < 0 || !buffer) {
        return -1;
    }

    struct i2c_msg messages[2];
    std::memset(messages, 0, sizeof(messages));

    messages[0].addr = address;
    messages[0].flags = 0;
    messages[0].len = 1;
    messages[0].buf = &reg;

    messages[1].addr = address;
    messages[1].flags = I2C_M_RD;
    messages[1].len = static_cast<__u16>(length);
    messages[1].buf = buffer;

    struct i2c_rdwr_ioctl_data transaction;
    transaction.msgs = messages;
    transaction.nmsgs = 2;

    for (int attempt = 0; attempt < ICM20948_I2C_RETRY_COUNT; ++attempt) {
        if (ioctl(i2c_fd, I2C_RDWR, &transaction) == 2) {
            return 0;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(I2C_RETRY_DELAY_US));
    }

    return -1;
}

int i2c_write_register_at(uint8_t address, uint8_t reg, const uint8_t *data, uint32_t length)
{
    if (i2c_fd < 0) {
        return -1;
    }

    std::vector<uint8_t> buffer(length + 1);
    buffer[0] = reg;
    if (length > 0 && data) {
        std::memcpy(buffer.data() + 1, data, length);
    }

    struct i2c_msg message;
    std::memset(&message, 0, sizeof(message));
    message.addr = address;
    message.flags = 0;
    message.len = static_cast<__u16>(buffer.size());
    message.buf = buffer.data();

    struct i2c_rdwr_ioctl_data transaction;
    transaction.msgs = &message;
    transaction.nmsgs = 1;

    for (int attempt = 0; attempt < ICM20948_I2C_RETRY_COUNT; ++attempt) {
        if (ioctl(i2c_fd, I2C_RDWR, &transaction) == 1) {
            return 0;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(I2C_RETRY_DELAY_US));
    }

    return -1;
}

bool detect_i2c_address(uint8_t *address)
{
    uint8_t env_address = 0;
    if (parse_i2c_address_from_env(&env_address)) {
        *address = env_address;
        return true;
    }

    const uint8_t candidates[] = {DEFAULT_I2C_ADDR_REVA, DEFAULT_I2C_ADDR_REVB};
    for (uint8_t candidate : candidates) {
        uint8_t whoami = 0;
        if (i2c_read_register_at(candidate, WHO_AM_I_REG, &whoami, 1) == 0 && whoami == EXPECTED_WHOAMI) {
            *address = candidate;
            return true;
        }
    }

    return false;
}

bool initialize_linux_i2c(int i2c_speed)
{
    requested_i2c_speed = i2c_speed;

    if (i2c_fd >= 0) {
        return true;
    }

    const std::string path = get_i2c_device_path();
    i2c_fd = open(path.c_str(), O_RDWR);
    if (i2c_fd < 0) {
        log_errno(path.c_str());
        return false;
    }

    if (!detect_i2c_address(&i2c_address)) {
        std::fprintf(stderr,
                     "ICM20948 ERROR: no ICM-20948 found at 0x68 or 0x69 on %s. "
                     "Set ICM20948_I2C_ADDR to override.\n",
                     path.c_str());
        close(i2c_fd);
        i2c_fd = -1;
        return false;
    }

    std::fprintf(stderr, "ICM20948: using %s address 0x%02X", path.c_str(), i2c_address);
    if (requested_i2c_speed > 0) {
        std::fprintf(stderr, " (requested %d Hz; set Pi baudrate in config.txt)", requested_i2c_speed);
    }
    std::fprintf(stderr, "\n");
    return true;
}

uint16_t sensor_period_ms(int frequency_hz)
{
    if (frequency_hz <= 0) {
        return 1000;
    }

    const int period = 1000 / frequency_hz;
    return static_cast<uint16_t>(period > 0 ? period : 1);
}

} // namespace

/*************************************************************************
  Sensor data buffers
*************************************************************************/

float gyro[3];
bool gyro_data_ready = false;

float accel[3];
bool accel_data_ready = false;

float mag[3];
bool mag_data_ready = false;

float grav[3];
bool grav_data_ready = false;

float lAccel[3];
bool linearAccel_data_ready = false;

float quat6[4];
bool quat6_data_ready = false;

float euler6[3];
bool euler6_data_ready = false;

float quat9[4];
bool quat9_data_ready = false;

float euler9[3];
bool euler9_data_ready = false;

int har;
bool har_data_ready = false;

unsigned long steps;
bool steps_data_ready = false;

/*************************************************************************
  HAL hooks for InvenSense driver
*************************************************************************/

int i2c_master_write_register(uint8_t address, uint8_t reg, uint32_t len, const uint8_t *data)
{
    return i2c_write_register_at(address, reg, data, len);
}

int i2c_master_read_register(uint8_t address, uint8_t reg, uint32_t len, uint8_t *buff)
{
    return i2c_read_register_at(address, reg, buff, len);
}

int spi_master_read_register(uint8_t, uint8_t *, uint32_t)
{
    log_error("SPI transport is not implemented in the Raspberry Pi 5 port; use Linux I2C");
    return -1;
}

int spi_master_write_register(uint8_t, const uint8_t *, uint32_t)
{
    log_error("SPI transport is not implemented in the Raspberry Pi 5 port; use Linux I2C");
    return -1;
}

inv_icm20948_t icm_device;
int rc = 0;

#define AK0991x_DEFAULT_I2C_ADDR 0x0C

#define THREE_AXES 3
static int unscaled_bias[THREE_AXES * 2];

static const float cfg_mounting_matrix[9] = {
    1.f, 0, 0,
    0, 1.f, 0,
    0, 0, 1.f
};

int32_t cfg_acc_fsr = 4;
int32_t cfg_gyr_fsr = 2000;

static const uint8_t dmp3_image[] = {
#include "icm20948_img.dmp3a.h"
};

void check_rc(int result, const char *msg_context)
{
    if (result < 0) {
        std::fprintf(stderr, "ICM20948 ERROR: %s (rc=%d)\n", msg_context, result);
        std::abort();
    }
}

int load_dmp3(void)
{
    return inv_icm20948_load(&icm_device, dmp3_image, sizeof(dmp3_image));
}

void inv_icm20948_sleep_us(int us)
{
    if (us > 0) {
        std::this_thread::sleep_for(std::chrono::microseconds(us));
    }
}

void inv_icm20948_sleep(int ms)
{
    if (ms > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }
}

uint64_t inv_icm20948_get_time_us(void)
{
    using clock = std::chrono::steady_clock;
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(clock::now().time_since_epoch()).count());
}

void initiliaze_SPI(void)
{
    log_error("SPI transport is not implemented in this Raspberry Pi 5 port");
}

void initiliaze_I2C(void)
{
    initialize_linux_i2c(requested_i2c_speed);
}

void set_comm_interface(ArduinoICM20948Settings settings)
{
    is_interface_spi = settings.is_SPI;
    if (is_interface_spi) {
        initiliaze_SPI();
        return;
    }

    initialize_linux_i2c(settings.i2c_speed);
}

inv_bool_t interface_is_SPI(void)
{
    return is_interface_spi;
}

int idd_io_hal_read_reg(void *, uint8_t reg, uint8_t *rbuffer, uint32_t rlen)
{
    if (interface_is_SPI()) {
        return spi_master_read_register(reg, rbuffer, rlen);
    }
    return i2c_master_read_register(i2c_address, reg, rlen, rbuffer);
}

int idd_io_hal_write_reg(void *, uint8_t reg, const uint8_t *wbuffer, uint32_t wlen)
{
    if (interface_is_SPI()) {
        return spi_master_write_register(reg, wbuffer, wlen);
    }
    return i2c_master_write_register(i2c_address, reg, wlen, wbuffer);
}

static void icm20948_apply_mounting_matrix(void)
{
    for (int ii = 0; ii < INV_ICM20948_SENSOR_MAX; ii++) {
        inv_icm20948_set_matrix(&icm_device, cfg_mounting_matrix, static_cast<inv_icm20948_sensor>(ii));
    }
}

static void icm20948_set_fsr(void)
{
    inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_RAW_ACCELEROMETER, (const void *)&cfg_acc_fsr);
    inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_ACCELEROMETER, (const void *)&cfg_acc_fsr);
    inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_RAW_GYROSCOPE, (const void *)&cfg_gyr_fsr);
    inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_GYROSCOPE, (const void *)&cfg_gyr_fsr);
    inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED, (const void *)&cfg_gyr_fsr);
}

int icm20948_sensor_setup(void)
{
    int result;
    uint8_t whoami = 0xff;

    inv_icm20948_soft_reset(&icm_device);

    result = inv_icm20948_get_whoami(&icm_device, &whoami);
    if (result != 0 || whoami != EXPECTED_WHOAMI) {
        std::fprintf(stderr, "ICM20948 ERROR: bad WHOAMI value 0x%02X (rc=%d)\n", whoami, result);
        return result != 0 ? result : -1;
    }

    inv_icm20948_init_matrix(&icm_device);

    result = inv_icm20948_initialize(&icm_device, dmp3_image, sizeof(dmp3_image));
    if (result != 0) {
        std::fprintf(stderr, "ICM20948 ERROR: initialization failed (rc=%d)\n", result);
        return result;
    }

    inv_icm20948_register_aux_compass(&icm_device, INV_ICM20948_COMPASS_ID_AK09916, AK0991x_DEFAULT_I2C_ADDR);
    result = inv_icm20948_initialize_auxiliary(&icm_device);
    if (result == -1) {
        std::fprintf(stderr, "ICM20948 WARNING: compass not detected\n");
    }

    icm20948_apply_mounting_matrix();
    icm20948_set_fsr();
    inv_icm20948_init_structure(&icm_device);

    return 0;
}

static uint8_t icm20948_get_grv_accuracy(void)
{
    const uint8_t accel_accuracy = (uint8_t)inv_icm20948_get_accel_accuracy();
    const uint8_t gyro_accuracy = (uint8_t)inv_icm20948_get_gyro_accuracy();
    return accel_accuracy < gyro_accuracy ? accel_accuracy : gyro_accuracy;
}

static uint8_t convert_to_generic_ids[INV_ICM20948_SENSOR_MAX] = {
    INV_SENSOR_TYPE_ACCELEROMETER,
    INV_SENSOR_TYPE_GYROSCOPE,
    INV_SENSOR_TYPE_RAW_ACCELEROMETER,
    INV_SENSOR_TYPE_RAW_GYROSCOPE,
    INV_SENSOR_TYPE_UNCAL_MAGNETOMETER,
    INV_SENSOR_TYPE_UNCAL_GYROSCOPE,
    INV_SENSOR_TYPE_BAC,
    INV_SENSOR_TYPE_STEP_DETECTOR,
    INV_SENSOR_TYPE_STEP_COUNTER,
    INV_SENSOR_TYPE_GAME_ROTATION_VECTOR,
    INV_SENSOR_TYPE_ROTATION_VECTOR,
    INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR,
    INV_SENSOR_TYPE_MAGNETOMETER,
    INV_SENSOR_TYPE_SMD,
    INV_SENSOR_TYPE_PICK_UP_GESTURE,
    INV_SENSOR_TYPE_TILT_DETECTOR,
    INV_SENSOR_TYPE_GRAVITY,
    INV_SENSOR_TYPE_LINEAR_ACCELERATION,
    INV_SENSOR_TYPE_ORIENTATION,
    INV_SENSOR_TYPE_B2S
};

void build_sensor_event_data(void *, enum inv_icm20948_sensor sensortype, uint64_t timestamp, const void *data, const void *arg)
{
    float raw_bias_data[6];
    inv_sensor_event_t event;
    (void)timestamp;

    if (sensortype >= INV_ICM20948_SENSOR_MAX) {
        return;
    }

    const uint8_t sensor_id = convert_to_generic_ids[sensortype];
    std::memset((void *)&event, 0, sizeof(event));
    event.sensor = sensor_id;

    switch (sensor_id) {
    case INV_SENSOR_TYPE_UNCAL_GYROSCOPE:
        std::memcpy(raw_bias_data, data, sizeof(raw_bias_data));
        std::memcpy(event.data.gyr.vect, &raw_bias_data[0], sizeof(event.data.gyr.vect));
        std::memcpy(event.data.gyr.bias, &raw_bias_data[3], sizeof(event.data.gyr.bias));
        std::memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
        break;
    case INV_SENSOR_TYPE_UNCAL_MAGNETOMETER:
        std::memcpy(raw_bias_data, data, sizeof(raw_bias_data));
        std::memcpy(event.data.mag.vect, &raw_bias_data[0], sizeof(event.data.mag.vect));
        std::memcpy(event.data.mag.bias, &raw_bias_data[3], sizeof(event.data.mag.bias));
        std::memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
        break;
    case INV_SENSOR_TYPE_GYROSCOPE:
        std::memcpy(event.data.gyr.vect, data, sizeof(event.data.gyr.vect));
        std::memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
        gyro[0] = event.data.gyr.vect[0];
        gyro[1] = event.data.gyr.vect[1];
        gyro[2] = event.data.gyr.vect[2];
        gyro_data_ready = true;
        break;
    case INV_SENSOR_TYPE_GRAVITY:
        std::memcpy(event.data.grav.vect, data, sizeof(event.data.grav.vect));
        event.data.grav.accuracy_flag = inv_icm20948_get_accel_accuracy();
        grav[0] = event.data.grav.vect[0];
        grav[1] = event.data.grav.vect[1];
        grav[2] = event.data.grav.vect[2];
        grav_data_ready = true;
        break;
    case INV_SENSOR_TYPE_LINEAR_ACCELERATION:
        std::memcpy(event.data.linAcc.vect, data, sizeof(event.data.linAcc.vect));
        std::memcpy(&(event.data.linAcc.accuracy_flag), arg, sizeof(event.data.linAcc.accuracy_flag));
        lAccel[0] = event.data.linAcc.vect[0];
        lAccel[1] = event.data.linAcc.vect[1];
        lAccel[2] = event.data.linAcc.vect[2];
        linearAccel_data_ready = true;
        break;
    case INV_SENSOR_TYPE_ACCELEROMETER:
        std::memcpy(event.data.acc.vect, data, sizeof(event.data.acc.vect));
        std::memcpy(&(event.data.acc.accuracy_flag), arg, sizeof(event.data.acc.accuracy_flag));
        accel[0] = event.data.acc.vect[0];
        accel[1] = event.data.acc.vect[1];
        accel[2] = event.data.acc.vect[2];
        accel_data_ready = true;
        break;
    case INV_SENSOR_TYPE_MAGNETOMETER:
        std::memcpy(event.data.mag.vect, data, sizeof(event.data.mag.vect));
        std::memcpy(&(event.data.mag.accuracy_flag), arg, sizeof(event.data.mag.accuracy_flag));
        mag[0] = event.data.mag.vect[0];
        mag[1] = event.data.mag.vect[1];
        mag[2] = event.data.mag.vect[2];
        mag_data_ready = true;
        break;
    case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:
    case INV_SENSOR_TYPE_ROTATION_VECTOR:
        std::memcpy(&(event.data.quaternion9DOF.accuracy), arg, sizeof(event.data.quaternion9DOF.accuracy));
        std::memcpy(event.data.quaternion9DOF.quat, data, sizeof(event.data.quaternion9DOF.quat));
        quat9[0] = event.data.quaternion9DOF.quat[0];
        quat9[1] = event.data.quaternion9DOF.quat[1];
        quat9[2] = event.data.quaternion9DOF.quat[2];
        quat9[3] = event.data.quaternion9DOF.quat[3];
        quat9_data_ready = true;
        euler9_data_ready = true;
        break;
    case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
        std::memcpy(event.data.quaternion6DOF.quat, data, sizeof(event.data.quaternion6DOF.quat));
        event.data.quaternion6DOF.accuracy_flag = icm20948_get_grv_accuracy();
        quat6[0] = event.data.quaternion6DOF.quat[0];
        quat6[1] = event.data.quaternion6DOF.quat[1];
        quat6[2] = event.data.quaternion6DOF.quat[2];
        quat6[3] = event.data.quaternion6DOF.quat[3];
        quat6_data_ready = true;
        euler6_data_ready = true;
        break;
    case INV_SENSOR_TYPE_BAC:
        std::memcpy(&(event.data.bac.event), data, sizeof(event.data.bac.event));
        har = event.data.bac.event;
        har_data_ready = true;
        break;
    case INV_SENSOR_TYPE_PICK_UP_GESTURE:
    case INV_SENSOR_TYPE_TILT_DETECTOR:
    case INV_SENSOR_TYPE_STEP_DETECTOR:
    case INV_SENSOR_TYPE_SMD:
        event.data.event = true;
        break;
    case INV_SENSOR_TYPE_B2S:
        event.data.event = true;
        std::memcpy(&(event.data.b2s.direction), data, sizeof(event.data.b2s.direction));
        break;
    case INV_SENSOR_TYPE_STEP_COUNTER:
        std::memcpy(&(event.data.step.count), data, sizeof(event.data.step.count));
        steps = event.data.step.count;
        steps_data_ready = true;
        break;
    case INV_SENSOR_TYPE_ORIENTATION:
        std::memcpy(&(event.data.orientation), data, 3 * sizeof(float));
        break;
    case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
    case INV_SENSOR_TYPE_RAW_GYROSCOPE:
        std::memcpy(event.data.raw3d.vect, data, sizeof(event.data.raw3d.vect));
        break;
    default:
        return;
    }
}

static enum inv_icm20948_sensor idd_sensortype_conversion(int sensor)
{
    switch (sensor) {
    case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
        return INV_ICM20948_SENSOR_RAW_ACCELEROMETER;
    case INV_SENSOR_TYPE_RAW_GYROSCOPE:
        return INV_ICM20948_SENSOR_RAW_GYROSCOPE;
    case INV_SENSOR_TYPE_ACCELEROMETER:
        return INV_ICM20948_SENSOR_ACCELEROMETER;
    case INV_SENSOR_TYPE_GYROSCOPE:
        return INV_ICM20948_SENSOR_GYROSCOPE;
    case INV_SENSOR_TYPE_UNCAL_MAGNETOMETER:
        return INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED;
    case INV_SENSOR_TYPE_UNCAL_GYROSCOPE:
        return INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED;
    case INV_SENSOR_TYPE_BAC:
        return INV_ICM20948_SENSOR_ACTIVITY_CLASSIFICATON;
    case INV_SENSOR_TYPE_STEP_DETECTOR:
        return INV_ICM20948_SENSOR_STEP_DETECTOR;
    case INV_SENSOR_TYPE_STEP_COUNTER:
        return INV_ICM20948_SENSOR_STEP_COUNTER;
    case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
        return INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR;
    case INV_SENSOR_TYPE_ROTATION_VECTOR:
        return INV_ICM20948_SENSOR_ROTATION_VECTOR;
    case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:
        return INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR;
    case INV_SENSOR_TYPE_MAGNETOMETER:
        return INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD;
    case INV_SENSOR_TYPE_SMD:
        return INV_ICM20948_SENSOR_WAKEUP_SIGNIFICANT_MOTION;
    case INV_SENSOR_TYPE_PICK_UP_GESTURE:
        return INV_ICM20948_SENSOR_FLIP_PICKUP;
    case INV_SENSOR_TYPE_TILT_DETECTOR:
        return INV_ICM20948_SENSOR_WAKEUP_TILT_DETECTOR;
    case INV_SENSOR_TYPE_GRAVITY:
        return INV_ICM20948_SENSOR_GRAVITY;
    case INV_SENSOR_TYPE_LINEAR_ACCELERATION:
        return INV_ICM20948_SENSOR_LINEAR_ACCELERATION;
    case INV_SENSOR_TYPE_ORIENTATION:
        return INV_ICM20948_SENSOR_ORIENTATION;
    case INV_SENSOR_TYPE_B2S:
        return INV_ICM20948_SENSOR_B2S;
    default:
        return INV_ICM20948_SENSOR_MAX;
    }
}

/*************************************************************************
  Class Functions
*************************************************************************/

ArduinoICM20948::ArduinoICM20948()
{
}

void ArduinoICM20948::init(ArduinoICM20948Settings settings)
{
    set_comm_interface(settings);
    if (!interface_is_SPI() && i2c_fd < 0) {
        check_rc(-1, "Linux I2C initialization failed");
    }

    std::fprintf(stderr, "Initializing ICM-20948 DMP...\n");

    struct inv_icm20948_serif icm20948_serif;
    icm20948_serif.context = 0;
    icm20948_serif.read_reg = idd_io_hal_read_reg;
    icm20948_serif.write_reg = idd_io_hal_write_reg;
    icm20948_serif.max_read = 1024 * 16;
    icm20948_serif.max_write = 1024 * 16;
    icm20948_serif.is_spi = interface_is_SPI();

    inv_icm20948_reset_states(&icm_device, &icm20948_serif);
    inv_icm20948_register_aux_compass(&icm_device, INV_ICM20948_COMPASS_ID_AK09916, AK0991x_DEFAULT_I2C_ADDR);

    rc = icm20948_sensor_setup();

    if (icm_device.selftest_done && !icm_device.offset_done) {
        inv_icm20948_set_offset(&icm_device, unscaled_bias);
        icm_device.offset_done = 1;
    }

    rc += load_dmp3();
    check_rc(rc, "Error sensor_setup/DMP loading.");

    inv_icm20948_set_lowpower_or_highperformance(&icm_device, settings.mode);

    rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GYROSCOPE), sensor_period_ms(settings.gyroscope_frequency));
    rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_ACCELEROMETER), sensor_period_ms(settings.accelerometer_frequency));
    rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_MAGNETOMETER), sensor_period_ms(settings.magnetometer_frequency));
    rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GAME_ROTATION_VECTOR), sensor_period_ms(settings.quaternion6_frequency));
    rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_ROTATION_VECTOR), sensor_period_ms(settings.quaternion9_frequency));
    rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GRAVITY), sensor_period_ms(settings.gravity_frequency));
    rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_LINEAR_ACCELERATION), sensor_period_ms(settings.linearAcceleration_frequency));
    rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_BAC), sensor_period_ms(settings.har_frequency));
    rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_STEP_COUNTER), sensor_period_ms(settings.steps_frequency));

    rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GYROSCOPE), settings.enable_gyroscope);
    rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_ACCELEROMETER), settings.enable_accelerometer);
    rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_MAGNETOMETER), settings.enable_magnetometer);
    rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GAME_ROTATION_VECTOR), settings.enable_quaternion6);
    rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_ROTATION_VECTOR), settings.enable_quaternion9);
    rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GRAVITY), settings.enable_gravity);
    rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_LINEAR_ACCELERATION), settings.enable_linearAcceleration);
    rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_BAC), settings.enable_har);
    rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_STEP_COUNTER), settings.enable_steps);
    check_rc(rc, "Error enabling configured sensors.");
}

void ArduinoICM20948::task()
{
    inv_icm20948_poll_sensor(&icm_device, (void *)0, build_sensor_event_data);
}

bool ArduinoICM20948::gyroDataIsReady() { return gyro_data_ready; }
bool ArduinoICM20948::accelDataIsReady() { return accel_data_ready; }
bool ArduinoICM20948::magDataIsReady() { return mag_data_ready; }
bool ArduinoICM20948::linearAccelDataIsReady() { return linearAccel_data_ready; }
bool ArduinoICM20948::gravDataIsReady() { return grav_data_ready; }
bool ArduinoICM20948::quat6DataIsReady() { return quat6_data_ready; }
bool ArduinoICM20948::euler6DataIsReady() { return euler6_data_ready; }
bool ArduinoICM20948::quat9DataIsReady() { return quat9_data_ready; }
bool ArduinoICM20948::euler9DataIsReady() { return euler9_data_ready; }
bool ArduinoICM20948::harDataIsReady() { return har_data_ready; }
bool ArduinoICM20948::stepsDataIsReady() { return steps_data_ready; }

void ArduinoICM20948::readGyroData(float *x, float *y, float *z)
{
    *x = gyro[0];
    *y = gyro[1];
    *z = gyro[2];
    gyro_data_ready = false;
}

void ArduinoICM20948::readAccelData(float *x, float *y, float *z)
{
    *x = accel[0];
    *y = accel[1];
    *z = accel[2];
    accel_data_ready = false;
}

void ArduinoICM20948::readMagData(float *x, float *y, float *z)
{
    *x = mag[0];
    *y = mag[1];
    *z = mag[2];
    mag_data_ready = false;
}

void ArduinoICM20948::readLinearAccelData(float *x, float *y, float *z)
{
    *x = lAccel[0];
    *y = lAccel[1];
    *z = lAccel[2];
    linearAccel_data_ready = false;
}

void ArduinoICM20948::readGravData(float *x, float *y, float *z)
{
    *x = grav[0];
    *y = grav[1];
    *z = grav[2];
    grav_data_ready = false;
}

void ArduinoICM20948::readQuat6Data(float *w, float *x, float *y, float *z)
{
    *w = quat6[0];
    *x = quat6[1];
    *y = quat6[2];
    *z = quat6[3];
    quat6_data_ready = false;
}

void ArduinoICM20948::readEuler6Data(float *roll, float *pitch, float *yaw)
{
    *roll = (std::atan2f(quat6[0] * quat6[1] + quat6[2] * quat6[3], 0.5f - quat6[1] * quat6[1] - quat6[2] * quat6[2])) * 57.29578f;
    *pitch = (std::asinf(-2.0f * (quat6[1] * quat6[3] - quat6[0] * quat6[2]))) * 57.29578f;
    *yaw = (std::atan2f(quat6[1] * quat6[2] + quat6[0] * quat6[3], 0.5f - quat6[2] * quat6[2] - quat6[3] * quat6[3])) * 57.29578f + 180.0f;
    euler6_data_ready = false;
}

void ArduinoICM20948::readQuat9Data(float *w, float *x, float *y, float *z)
{
    *w = quat9[0];
    *x = quat9[1];
    *y = quat9[2];
    *z = quat9[3];
    quat9_data_ready = false;
}

void ArduinoICM20948::readEuler9Data(float *roll, float *pitch, float *yaw)
{
    *roll = (std::atan2f(quat9[0] * quat9[1] + quat9[2] * quat9[3], 0.5f - quat9[1] * quat9[1] - quat9[2] * quat9[2])) * 57.29578f;
    *pitch = (std::asinf(-2.0f * (quat9[1] * quat9[3] - quat9[0] * quat9[2]))) * 57.29578f;
    *yaw = (std::atan2f(quat9[1] * quat9[2] + quat9[0] * quat9[3], 0.5f - quat9[2] * quat9[2] - quat9[3] * quat9[3])) * 57.29578f + 180.0f;
    euler9_data_ready = false;
}

void ArduinoICM20948::readHarData(char *activity)
{
    char temp = 'n';
    switch (har) {
    case 1:
        temp = 'd';
        break;
    case 2:
        temp = 'w';
        break;
    case 3:
        temp = 'r';
        break;
    case 4:
        temp = 'b';
        break;
    case 5:
        temp = 't';
        break;
    case 6:
        temp = 's';
        break;
    }
    *activity = temp;
    har_data_ready = false;
}

void ArduinoICM20948::readStepsData(unsigned long *step_count)
{
    *step_count = steps;
    steps_data_ready = false;
}
