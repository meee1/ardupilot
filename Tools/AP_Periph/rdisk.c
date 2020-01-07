// Disk Image

#include <string.h>
#include "ch.h"
#include "hal.h"

typedef struct {
  const struct BaseBlockDeviceVMT *vmt;
  _base_block_device_data
} ramBlockDevice;


static bool is_inserted(void *instance) {
  (void) instance;
  return true;
}

static bool is_protected(void *instance) {
  (void)instance;
  return false;
}

static bool connect(void *instance) {
  (void) instance;
  printf("connect\r\n");
  return HAL_SUCCESS;
}

static bool disconnect(void *instance) {
  (void) instance;
  return HAL_SUCCESS;
}

static bool read(void *instance, uint32_t startblk, uint8_t *buffer, uint32_t n) {
  printf("read %d - %d\r\n",startblk,n);
  while (blkGetDriverState(&SDCD1) != BLK_READY) { chThdSleepMilliseconds(1); }
  bool ans = sdcRead(&SDCD1, startblk, buffer, n);
  printf("read %d - %d done\r\n",startblk,n);
  return ans;
}

static bool write(void *instance, uint32_t startblk, const uint8_t *buffer, uint32_t n) {
  printf("write %d - %d\r\n",startblk,n);
  return false;
}

static bool sync(void *instance){
  (void) instance;
  return HAL_SUCCESS;
}

static bool get_info(void *instance, BlockDeviceInfo *bdip) {
  //printf("get_info\r\n");
  //ramBlockDevice *pDisk = (ramBlockDevice *) instance;
  bdip->blk_num = SDCD1.capacity;//AP::FS().disk_space('/')// *pDisk->pDiskImageLen/512;
  bdip->blk_size = 512;
  return HAL_SUCCESS;
}

static struct BaseBlockDeviceVMT vmt = {
  (size_t)0,
  is_inserted, 
  is_protected, 
  connect, 
  disconnect, 
  read, 
  write, 
  sync, 
  get_info
};

static ramBlockDevice rDisk = {&vmt, BLK_READY};

BaseBlockDevice *pDisk_old = (BaseBlockDevice *) &rDisk;



