#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <glib.h>
#include <math.h>
#include "fpi-device.h"
#include "fpi-ssm.h"
#include "pwd.h"
#include <sys/utsname.h>

G_DECLARE_FINAL_TYPE (FpiDeviceMafpmoc, fpi_device_mafpmoc, FPI, DEVICE_MAFPMOC, FpDevice)

#define PRINT_CMD 0
#define PRINT_SSM_DEBUG 0

#define LOGD(format, ...) g_debug ("[%s][%d]" format, __FUNCTION__, __LINE__, ## __VA_ARGS__)
#define LOGE(format, ...) g_error ("[%s][%d]" format, __FUNCTION__, __LINE__, ## __VA_ARGS__)

#define MAFP_INTERFACE_CLASS G_USB_DEVICE_CLASS_VENDOR_SPECIFIC
#define MAFP_INTERFACE_SUB_CLASS 0x01
#define MAFP_INTERFACE_PROTOCOL 0x01

/* Usb port setting */
#define MAFP_EP_BULK_OUT 0x03
#define MAFP_EP_BULK_IN 0x83
#define MAFP_EP_INT_IN 0x82

/* Command transfer timeout: ms */
#define CMD_TIMEOUT 5000
#define DATA_TIMEOUT 5000
#define CTRL_TIMEOUT 200

#define MAFP_USB_BUFFER_SIZE 512
#define PACKAGE_CRC_SIZE 2
#define PACKAGE_HEADER_SIZE 9
#define PACKAGE_DATA_SIZE_MAX (MAFP_USB_BUFFER_SIZE - PACKAGE_HEADER_SIZE - PACKAGE_CRC_SIZE)

#define TEMPLATE_ID_SIZE 2
#define TEMPLATE_UID_SIZE 128
#define DEVICE_SN_SIZE 32
#define MAX_FINGER_NUM 10
#define MAX_USER_NUM 3
#define MAX_NOTEPAD_PAGE 16

#define MOC_CMD_GET_IMAGE 0x01
#define MOC_CMD_GEN_FEATURE 0x02
#define MOC_CMD_SEARCH 0x04
#define MOC_CMD_GEN_TEMPLATE 0x05
#define MOC_CMD_SAVE_TEMPLATE 0x06
#define MOC_CMD_READ_TEMPLATE 0x07
#define MOC_CMD_DELETE_TEMPLATE 0x0C
#define MOC_CMD_EMPTY 0x0D
#define MOC_CMD_WRITE_NOTEPAD 0x18
#define MOC_CMD_READ_NOTEPAD 0x19
#define MOC_CMD_GET_TEMPLATE_NUM 0x1D
#define MOC_CMD_GET_TEMPLATE_TABLE 0x1F
#define MOC_CMD_CANCEL 0x30
#define MOC_CMD_SLEEP 0x33
#define MOC_CMD_HANDSHAKE 0x35
#define MOC_CMD_CALIBRATE 0x36
#define MOC_CMD_FACTORY_RESET 0x3B
#define MOC_CMD_FACTORY_TEST 0x56
#define MOC_CMD_MATCH_WITHFID 0x66
#define MOC_CMD_GET_MAX_ID 0x6d
#define MOC_CMD_DUPAREA_TEST 0x6f
#define MOC_CMD_SAVE_TEMPLATE_INFO 0x86
#define MOC_CMD_GET_TEMPLATE_INFO 0x87
#define MOC_CMD_GET_INIT_STATUS 0x88

#define MAFP_SUCCESS 0
#define MAFP_RE_TPL_NUM_OVERSIZE 0x0B
#define MAFP_RE_GET_IMAGE_SUCCESS 0x00
#define MAFP_RE_GET_IMAGE_NONE 0x02
/* calibrate error (un-calibrate or calibrate failed) */
#define MAFP_RE_CALIBRATE_ERROR 0x02

#define MAFP_HANDSHAKE_CODE1 'M'
#define MAFP_HANDSHAKE_CODE2 'A'

/* Default enroll stages number */
#define DEFAULT_ENROLL_SAMPLES 12
#define MAFP_ENV_ENROLL_SAMPLES "MAFP_ENROLL_SAMPLES"

#define MAFP_ENROLL_IDENTIFY_DISABLED 0
#define MAFP_ENROLL_IDENTIFY_ENABLED 1
#define MAFP_ENROLL_IDENTIFY_ONCE 2
#define MAFP_ENROLL_DUPLICATE_DELETE_DISABLED 0
#define MAFP_ENROLL_DUPLICATE_DELETE_ENABLED 1
#define MAFP_ENROLL_DUPLICATE_AREA_DENY 0
#define MAFP_ENROLL_DUPLICATE_AREA_ALLOW 1

#define MAFP_SLEEP_INT_WAIT 0
#define MAFP_SLEEP_INT_CHECK 1
#define MAFP_SLEEP_INT_REFRESH 2

#define MAFP_PRESS_WAIT_UP 0
#define MAFP_PRESS_WAIT_DOWN 1

#define MAFP_IMAGE_ERR_TRRIGER 30

#define FPRINT_DATA_PATH "/var/lib/fprint/"

typedef enum {
  FP_CMD_SEND = 0,
  FP_CMD_RECEIVE,
  FP_DATA_RECEIVE,
  FP_TRANSFER_STATES,
} FpCmdState;

typedef enum {
  FP_INIT_CLEAN_EPIN = 0,
  FP_INIT_CLEAN_EPOUT,
  FP_INIT_CLEAN_EPIN2,
  FP_INIT_HANDSHAKE,
  FP_INIT_MODULE_STATUS,
  FP_INIT_STATES,
} FpInitState;

typedef enum {
  FP_ENROLL_PWR_BTN_SHIELD_ON = 0,
  FP_ENROLL_CHECK_EMPTY,
  FP_ENROLL_TEMPLATE_TABLE,
  FP_ENROLL_READ_TEMPLATE,
  FP_ENROLL_VERIFY_GET_IMAGE,
  FP_ENROLL_CHECK_INT_PARA,
  FP_ENROLL_DETECT_MODE,
  FP_ENROLL_ENABLE_INT,
  FP_ENROLL_WAIT_INT,
  FP_ENROLL_DISBALE_INT,
  FP_ENROLL_REFRESH_INT_PARA,
  FP_ENROLL_VERIFY_GENERATE_FEATURE,
  FP_ENROLL_VERIFY_DUPLICATE_AREA,
  FP_ENROLL_VERIFY_SEARCH,
  FP_ENROLL_VERIFY_SEARCH_STEP, //match assigned id
  FP_ENROLL_GET_TEMPLATE_INFO,
  FP_ENROLL_SAVE_TEMPLATE_INFO,
  FP_ENROLL_SAVE_TEMPLATE,
  FP_ENROLL_DELETE_TEMPLATE_INFO_IF_FAILED,
  FP_ENROLL_EXIT,
  FP_ENROLL_STATES,
} FpEnrollState;

typedef enum {
  FP_VERIFY_PWR_BTN_SHIELD_ON = 0,
  FP_VERIFY_TEMPLATE_TABLE,
  FP_VERIFY_GET_STARTUP_RESULT,
  FP_VERIFY_GET_IMAGE,
  FP_VERIFY_CHECK_INT_PARA,
  FP_VERIFY_DETECT_MODE,
  FP_VERIFY_ENABLE_INT,
  FP_VERIFY_WAIT_INT,
  FP_VERIFY_DISBALE_INT,
  FP_VERIFY_REFRESH_INT_PARA,
  FP_VERIFY_GENERATE_FEATURE,
  FP_VERIFY_SEARCH_STEP, //match assigned id
  FP_VERIFY_GET_TEMPLATE_INFO,
  FP_VERIFY_EXIT,
  FP_VERIFY_STATES,
} FpVerifyState;

typedef enum {
  FP_LIST_TEMPLATE_TABLE = 0,
  FP_LIST_GET_TEMPLATE_INFO,
  FP_LIST_STATES,
} FpListState;

typedef enum {
  FP_DELETE_TEMPLATE_TABLE = 0,
  FP_DELETE_GET_TEMPLATE_INFO,
  FP_DELETE_CLEAR_TEMPLATE_INFO,
  FP_DELETE_TEMPLATE,
  FP_DELETE_STATES,
} FpDeleteState;

typedef enum {
  FP_EMPTY_TEMPLATE = 0,
  FP_EMPTY_STATES,
} FpDeleteAllState;

typedef enum {
  PACK_CMD          = 0x01,   //cmd packet
  PACK_DATA         = 0x02,   //data packet, with more data packets later, must follow cmd or answer packet
  PACK_ANSWER       = 0x07,   //answer packet for cmd packet
  PACK_END          = 0x08,   //last data packet
  PACK_DATA_ANDSWER = 0x09,   //answer packet for data packet
} pack_mark;

#pragma pack(push, 1)
typedef struct _mafp_handshake
{
  uint8_t code[2];
  // uint8_t state;
  // uint8_t desc[16];
  // uint8_t ver[4];
} mafp_handshake_t, *pmafp_handshake_t;

typedef struct _mafp_search
{
  uint8_t id[2];
  uint8_t score[2];
} mafp_search_t, *pmafp_search_t;

typedef struct _mafp_tpl_table
{
  uint8_t used_num;
  uint8_t list[256];
} mafp_tpl_table_t, *pmafp_tpl_table_t;

typedef struct _mafp_tpl_info
{
  char uid[128];
} mafp_tpl_info_t, *pmafp_tpl_info_t;

typedef struct _mafp_boot_handshake
{
  char     code[2];
  uint8_t  state;
  char     descrip[16];
  uint32_t version;
} mafp_boot_handshake_t, *pmafp_boot_handshake_t;

typedef struct _mafp_template
{
  char     sn[32];
  uint16_t id;
  char     uid[128];
} mafp_template_t, *pmafp_template_t;

typedef struct _mafp_templates
{
  uint16_t        index;
  uint16_t        total_num;
  uint16_t        priv_num;
  mafp_template_t total_list[256];
  mafp_template_t priv_list[MAX_FINGER_NUM];
  GPtrArray      *list;
} mafp_templates_t, *pmafp_templates_t;
#pragma pack(pop)

typedef struct _fp_cmd_response
{
  uint8_t result;
  union
  {
    mafp_handshake_t      handshake;
    mafp_search_t         search;
    mafp_tpl_table_t      tpl_table;
    mafp_tpl_info_t       tpl_info;
    mafp_boot_handshake_t boot_handshake;
  };
} mafp_cmd_response_t, *pmafp_cmd_response_t;

typedef struct _pack_header
{
  uint8_t head0;
  uint8_t head1;
  uint8_t addr0;
  uint8_t addr1;
  uint8_t addr2;
  uint8_t addr3;
  uint8_t flag;
  uint8_t frame_len0;
  uint8_t frame_len1;
} pack_header, *ppack_header;
