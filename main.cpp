#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

#include <unistd.h>


using namespace sl;

// create the driver instance
ILidarDriver *drv;


static inline void delay(sl_word_size_t ms) {
  while (ms >= 1000) {
    usleep(1000 * 1000);
    ms -= 1000;
  };
  if (ms != 0)
    usleep(ms * 1000);
}


bool checkSLAMTECLIDARHealth(ILidarDriver *drv) {
  sl_lidar_response_device_health_t healthinfo;

  sl_result statusCode = drv->getHealth(healthinfo);
  if (SL_IS_OK(statusCode)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
    printf("SLAMTEC Lidar health status : %d\n", healthinfo.status);
    if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
      fprintf(stderr, "Error, slamtec lidar internal error detected. Please reboot the device to retry.\n");
      // enable the following code if you want slamtec lidar to be reboot by software
      // drv->reset();
      return false;
    } else {
      return true;
    }

  } else {
    fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", statusCode);
    return false;
  }
}


void stopLidarOnSignalInterrupt(int) {
  drv->stop();
  delay(200);
  drv->setMotorSpeed(0);
  // done!
  if (drv) {
    delete drv;
    drv = NULL;
  }
}


int main() {
  const char *portAddress = "/dev/ttyUSB0";
  sl_u32 baudrate = 460800;
  IChannel *_channel;

  // create the driver instance
  drv = *createLidarDriver();

  if (!drv) {
    fprintf(stderr, "insufficent memory, exit\n");
    exit(-2);
  }

  sl_lidar_response_device_info_t devinfo;
  bool connectSuccess = false;

  _channel = (*createSerialPortChannel(portAddress, baudrate));

  if (SL_IS_OK((drv)->connect(_channel))) {
    sl_result statusCode = drv->getDeviceInfo(devinfo);

    if (SL_IS_OK(statusCode)) {
      connectSuccess = true;
    } else {
      delete drv;
      drv = NULL;
    }
  }

  if (!connectSuccess) {
    fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n", portAddress);
    delete drv;
    drv = NULL;
    return -1;
  }

  // print out the device serial number, firmware and hardware version number..
  printf("SLAMTEC LIDAR S/N: ");
  for (int pos = 0; pos < 16; ++pos) {
    printf("%02X", devinfo.serialnum[pos]);
  }

  printf("\n"
         "Firmware Ver: %d.%02d\n"
         "Hardware Rev: %d\n", devinfo.firmware_version >> 8, devinfo.firmware_version & 0xFF,
         (int) devinfo.hardware_version);

  // check health...
  if (!checkSLAMTECLIDARHealth(drv)) {
    delete drv;
    drv = NULL;
    return -1;
  }

  // Binding a function to SIGNAL INTERRUPT
  // Whenever the SIGNAL INTERRUPT happens, the function is called
  signal(SIGINT, stopLidarOnSignalInterrupt);

  // Start Lidar spinning
  drv->setMotorSpeed();

  // start scan...
  drv->startScan(0, 1);

  // fetech result and print it out...
  while (1) {
    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = sizeof(nodes) / sizeof(nodes[0]);

    sl_result statusCode = drv->grabScanDataHq(nodes, count);

    if (SL_IS_OK(statusCode)) {
      drv->ascendScanData(nodes, count);
      for (int pos = 0; pos < (int) count; ++pos) {
        printf("%s theta: %03.2f Dist: %08.2f Q: %d \n",
               (nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) ? "S " : "  ",
               (nodes[pos].angle_z_q14 * 90.f) / 16384.f,
               nodes[pos].dist_mm_q2 / 4.0f,
               nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
      }
    }
  }

  return 0;
}

