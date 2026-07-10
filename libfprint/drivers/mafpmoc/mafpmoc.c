#define FP_COMPONENT "mafpmoc"

#include "glib.h"
#include "drivers_api.h"
#include "mafpmoc.h"
#include "fpi-byte-writer.h"

struct _FpiDeviceMafpmoc
{
  FpDevice          parent;
  FpiSsm           *task_ssm;
  FpiSsm           *cmd_ssm;
  FpiUsbTransfer   *cmd_transfer;
  gboolean          cmd_cancelable;
  gboolean          cmd_force_pass;
  int               enroll_stage;
  int               max_stored_prints;
  uint8_t           interface_num;
  uint8_t           press_state;
  uint32_t          finger_status;
  char             *serial_number;
  uint16_t          enroll_id;
  char             *enroll_user_id;
  unsigned          enroll_identify_index;
  uint16_t          enroll_identify_id;
  uint8_t           enroll_identify_state;
  uint8_t           enroll_dupl_del_state;
  uint8_t           enroll_dupl_area_state;
  pmafp_templates_t templates;
  uint16_t          search_id;
  unsigned          capture_cnt;
  FpPrint          *identify_new_print;
  FpPrint          *identify_match_print;
};

G_DEFINE_TYPE (FpiDeviceMafpmoc, fpi_device_mafpmoc, FP_TYPE_DEVICE)

typedef void (*SynCmdMsgCallback) (FpiDeviceMafpmoc    *self,
                                   mafp_cmd_response_t *resp,
                                   GError              *error);

typedef struct
{
  uint16_t          cmd;
  SynCmdMsgCallback callback;
  FpiUsbTransfer   *cmd_transfer;
  gboolean          cmd_cancelable;
  uint16_t          cmd_request_len;
  uint16_t          cmd_actual_len;
  uint8_t           recv_buffer[MAFP_USB_BUFFER_SIZE];
  gboolean          cmd_force_pass;
  uint16_t          crc;
} CommandData;

static void mafp_sensor_cmd (FpiDeviceMafpmoc *self,
                             uint16_t          cmd,
                             const uint8_t    *data,
                             uint8_t           data_len,
                             SynCmdMsgCallback callback);

static uint16_t
ma_protocol_crc16_calc ( uint8_t *data, uint32_t data_len, uint32_t start)
{
  const uint8_t *temp = data;
  uint32_t sum = 0;
  uint32_t i;

  for (i = start; i < data_len; temp++, i++)
    sum += *(temp + start) & 0xff;
  uint16_t sum_s = (sum & 0xffff);

  return sum_s;
}

/* data tansfer:
 *      while cmd_len = 0, put flag(end or not) in data[0]
 */
static uint8_t *
ma_protocol_build_package (uint32_t       package_len,
                           int16_t        cmd,
                           uint8_t        cmd_len,
                           const uint8_t *data,
                           uint32_t       data_len)
{
  g_autoptr(FpiByteWriter) writer = NULL;
  uint8_t flag;
  uint16_t frame_len;
  uint16_t crc;
  gboolean written;

  writer = fpi_byte_writer_new_with_size (package_len, TRUE);

  written = fpi_byte_writer_put_uint8 (writer, 0xEF);
  written &= fpi_byte_writer_put_uint8 (writer, 0x01);
  written &= fpi_byte_writer_put_uint8 (writer, 0xFF);
  written &= fpi_byte_writer_put_uint8 (writer, 0xFF);
  written &= fpi_byte_writer_put_uint8 (writer, 0xFF);
  written &= fpi_byte_writer_put_uint8 (writer, 0xFF);

  flag = (uint8_t) MAPF_PACK_CMD;

  if (!cmd_len && data_len)
    flag = data[0];

  fpi_byte_writer_put_uint8 (writer, flag);

  frame_len = package_len - PACKAGE_HEADER_SIZE;

  written &= fpi_byte_writer_put_uint16_be (writer, frame_len);

  if (cmd_len)
    written &= fpi_byte_writer_put_uint8 (writer, cmd);

  if (data_len)
    written &= fpi_byte_writer_put_data (writer, data + !cmd_len, data_len);

  crc = ma_protocol_crc16_calc ((uint8_t *) writer->parent.data,
                                PACKAGE_HEADER_SIZE + cmd_len + data_len,
                                6);

  written &= fpi_byte_writer_put_uint16_be (writer, crc);

  if (!written)
    g_return_val_if_reached (NULL);

  return fpi_byte_writer_free_and_get_data (g_steal_pointer (&writer));
}

static int
ma_protocol_parse_header (  uint8_t     *buffer,
                            uint32_t     buffer_len,
                            pack_header *pheader)
{
  if (!buffer || !pheader || buffer_len < PACKAGE_HEADER_SIZE)
    return -1;

  memcpy (pheader, buffer, sizeof (pack_header));
  return 0;
}

static uint8_t
get_one_bit_value (  uint8_t src,
                     uint8_t bit_num)
{
  return (uint8_t) ((src >> (bit_num - 1)) & 1);
}

static int
ma_protocol_parse_body (  int16_t              cmd,
                          uint8_t             *buffer,
                          uint16_t             buffer_len,
                          pmafp_cmd_response_t presp)
{
  const int data_len = buffer_len - 1 - PACKAGE_CRC_SIZE;

  if (!buffer || !presp || buffer_len < 1)
    return -1;

  presp->result = buffer[0];

  switch (cmd)
    {
    case MOC_CMD_HANDSHAKE:
      if (data_len >= sizeof (mafp_handshake_t))
        memcpy (&presp->handshake, buffer + 1, sizeof (mafp_handshake_t));
      break;

    case MOC_CMD_SEARCH:
      if (data_len >= sizeof (mafp_search_t))
        memcpy (&presp->search, buffer + 1, sizeof (mafp_search_t));
      break;

    case MOC_CMD_GET_TEMPLATE_NUM:
      if (data_len >= 2)
        presp->tpl_table.used_num = ((buffer[1] & 0xff) << 8) | (buffer[2] & 0xff);
      break;

    case MOC_CMD_GET_TEMPLATE_TABLE:
      if (data_len >= 32)
        {
          uint16_t num = 0;
          for (uint8_t i = 1; i < 33; i++)
            {
              uint8_t data = buffer[i];
              for (uint8_t bit = 1; bit <= 8 && num < sizeof (presp->tpl_table.list); bit++, num++)
                presp->tpl_table.list[num] = get_one_bit_value (data, bit);
            }
        }
      break;

    case MOC_CMD_GET_TEMPLATE_INFO:
      if (data_len >= 128)
        memcpy (&presp->tpl_info, buffer + 1, sizeof (mafp_tpl_info_t));
      break;

    case MOC_CMD_DUPAREA_TEST:
      if (data_len >= 1)
        presp->result = buffer[1];
      break;

    default:
      memcpy (presp, buffer, MIN ((gsize) buffer_len, sizeof (*presp)));
      break;
    }
  return 0;
}

static void
mafp_clean_usb_bulk_in (FpDevice *device)
{
  g_autoptr(FpiUsbTransfer) transfer = fpi_usb_transfer_new (device);
  fpi_usb_transfer_fill_bulk (transfer, MAFP_EP_BULK_IN, MAFP_USB_BUFFER_SIZE);
  g_autoptr(GError) error = NULL;

  fp_dbg ("bulk clean");
  if (!fpi_usb_transfer_submit_sync (transfer, 200, &error))
    fp_dbg ("bulk transfer out fail, %s", error->message);
}

static G_GNUC_PRINTF (4, 5) void
mafp_mark_failed (FpDevice   *dev,
                  FpiSsm     *ssm,
                  uint8_t     err_code,
                  const char *msg,
                  ...)
{
  if (err_code == FP_DEVICE_ERROR_PROTO)
    mafp_clean_usb_bulk_in (dev);

  if (msg == NULL)
    {
      fpi_ssm_mark_failed (ssm, fpi_device_error_new (err_code));
    }
  else
    {
      va_list args;
      va_start (args, msg);
      fpi_ssm_mark_failed (ssm, g_error_new_valist (FP_DEVICE_ERROR, err_code, msg, args));
      va_end (args);
    }
}

static void
fp_cmd_receive_cb (FpiUsbTransfer *transfer,
                   FpDevice       *device,
                   gpointer        user_data,
                   GError         *error)
{
  FpiDeviceMafpmoc *self = FPI_DEVICE_MAFPMOC (device);
  CommandData *data = user_data;
  int ret = -1, ssm_state = 0;
  mafp_cmd_response_t cmd_reponse = {0, };
  pack_header header;
  uint32_t data_index = 0;

  if (error)
    {
      fp_dbg ("error: %d, %s", error->code, error->message);
      if (data->cmd_force_pass) /* ex: G_USB_DEVICE_ERROR_TIMED_OUT */
        {
          if (data->callback)
            data->callback (self, &cmd_reponse, NULL);
          fpi_ssm_mark_completed (transfer->ssm);
          g_clear_error (&error);
          return;
        }
      fpi_ssm_mark_failed (transfer->ssm, g_steal_pointer (&error));
      return;
    }
  if (data == NULL)
    {
      fp_dbg ("data null");
      mafp_mark_failed (device, transfer->ssm, FP_DEVICE_ERROR_PROTO, "resp data null");
      return;
    }
  ssm_state = fpi_ssm_get_cur_state (transfer->ssm);

  /* skip zero length package */
  if (transfer->actual_length == 0)
    {
      fpi_ssm_jump_to_state (transfer->ssm, ssm_state);
      return;
    }

  if (ssm_state == MAPF_CMD_RECEIVE)
    {
      ret = ma_protocol_parse_header (transfer->buffer, transfer->actual_length, &header);
      if (ret != 0 || header.flag != MAPF_PACK_ANSWER)
        {
          mafp_mark_failed (device, transfer->ssm, FP_DEVICE_ERROR_PROTO, "Corrupted resp header received");
          return;
        }
      data->cmd_request_len = ((header.frame_len0 & 0xff) << 8) | (header.frame_len1 & 0xff);
      if (!data->cmd_request_len)
        {
          mafp_mark_failed (device, transfer->ssm, FP_DEVICE_ERROR_PROTO, "Corrupted resp length received");
          return;
        }
      if (data->cmd_request_len + PACKAGE_HEADER_SIZE > sizeof (data->recv_buffer))
        {
          mafp_mark_failed (device, transfer->ssm, FP_DEVICE_ERROR_PROTO, "Resp length too large");
          return;
        }
      data_index = PACKAGE_HEADER_SIZE;
    }
  if (transfer->actual_length < data_index)
    {
      mafp_mark_failed (device, transfer->ssm, FP_DEVICE_ERROR_PROTO, "Corrupted resp payload received");
      return;
    }
  if (data->cmd_actual_len + transfer->actual_length > sizeof (data->recv_buffer))
    {
      mafp_mark_failed (device, transfer->ssm, FP_DEVICE_ERROR_PROTO, "Resp data overflow");
      return;
    }
  memcpy (data->recv_buffer + data->cmd_actual_len, transfer->buffer, transfer->actual_length);
  data->cmd_actual_len += transfer->actual_length - data_index;

  if (data->cmd_request_len <= data->cmd_actual_len)
    {
      ret = ma_protocol_parse_body (data->cmd, &data->recv_buffer[PACKAGE_HEADER_SIZE],
                                    data->cmd_request_len, &cmd_reponse);
      if (ret != 0)
        {
          mafp_mark_failed (device, transfer->ssm, FP_DEVICE_ERROR_PROTO, "Corrupted resp body received");
          return;
        }
      uint32_t no_crc_len = PACKAGE_HEADER_SIZE + data->cmd_request_len - PACKAGE_CRC_SIZE;
      data->crc = ma_protocol_crc16_calc (&data->recv_buffer[0], no_crc_len, 6);
      uint16_t frame_crc = ((data->recv_buffer[no_crc_len] & 0xff) << 8)
                           | (data->recv_buffer[no_crc_len + 1] & 0xff);
      if (data->crc != frame_crc)
        {
          mafp_mark_failed (device, transfer->ssm, FP_DEVICE_ERROR_PROTO, "Package crc check failed");
          return;
        }
      if (data->callback)
        data->callback (self, &cmd_reponse, NULL);
      fpi_ssm_mark_completed (transfer->ssm);

    }
  else if (data->cmd_request_len > data->cmd_actual_len)
    {
      fpi_ssm_next_state (transfer->ssm);
      return;
    }
}

static void
fp_cmd_run_state (FpiSsm   *ssm,
                  FpDevice *dev)
{
  FpiUsbTransfer *transfer;
  CommandData *data = fpi_ssm_get_data (ssm);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case MAPF_CMD_SEND:
      if (data->cmd_transfer)
        {
          data->cmd_transfer->ssm = ssm;
          fpi_usb_transfer_submit (g_steal_pointer (&data->cmd_transfer),
                                   CMD_TIMEOUT, NULL, fpi_ssm_usb_transfer_cb, NULL);
        }
      else
        {
          fpi_ssm_next_state (ssm);
        }
      break;

    case MAPF_CMD_RECEIVE:
      transfer = fpi_usb_transfer_new (dev);
      transfer->ssm = ssm;
      fpi_usb_transfer_fill_bulk (transfer, MAFP_EP_BULK_IN, MAFP_USB_BUFFER_SIZE);
      fpi_usb_transfer_submit (transfer,
                               data->cmd_cancelable ? 0 : data->cmd_force_pass ? CTRL_TIMEOUT : CMD_TIMEOUT,
                               data->cmd_cancelable ? fpi_device_get_cancellable (dev) : NULL,
                               fp_cmd_receive_cb,
                               fpi_ssm_get_data (ssm));
      break;

    case MAPF_CMD_DATA_RECEIVE:
      fp_dbg ("req: %d, act: %d", data->cmd_request_len, data->cmd_actual_len);
      int req_len = MAFP_USB_BUFFER_SIZE;
      if (data->cmd_request_len > 0 && data->cmd_actual_len > 0 && (data->cmd_request_len > data->cmd_actual_len))
        req_len = data->cmd_request_len - data->cmd_actual_len;
      transfer = fpi_usb_transfer_new (dev);
      transfer->ssm = ssm;
      fpi_usb_transfer_fill_bulk (transfer, MAFP_EP_BULK_IN, req_len);
      fpi_usb_transfer_submit (transfer,
                               data->cmd_cancelable ? 0 : DATA_TIMEOUT,
                               data->cmd_cancelable ? fpi_device_get_cancellable (dev) : NULL,
                               fp_cmd_receive_cb,
                               fpi_ssm_get_data (ssm));
      break;
    }
}

static void
fp_cmd_ssm_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDeviceMafpmoc *self = FPI_DEVICE_MAFPMOC (dev);
  CommandData *data = fpi_ssm_get_data (ssm);

  self->cmd_ssm = NULL;
  if (error)
    {
      if (data->callback)
        data->callback (self, NULL, error);
      else
        g_error_free (error);
    }
}

static void
fp_cmd_ssm_done_data_free (CommandData *data)
{
  g_free (data);
}

static FpiUsbTransfer *
alloc_cmd_transfer (FpiDeviceMafpmoc *self,
                    uint16_t          cmd,
                    uint8_t           cmd_len,
                    const uint8_t    *data,
                    uint32_t          data_len)
{
  g_autoptr(FpiUsbTransfer) transfer = fpi_usb_transfer_new (FP_DEVICE (self));
  uint32_t total_len = PACKAGE_HEADER_SIZE + cmd_len + data_len + PACKAGE_CRC_SIZE;
  uint8_t *buffer;

  g_return_val_if_fail (data || data_len == 0, NULL);

  buffer = ma_protocol_build_package (total_len, cmd, cmd_len, data, data_len);
  g_return_val_if_fail (buffer, NULL);

  fpi_usb_transfer_fill_bulk_full (transfer, MAFP_EP_BULK_OUT, buffer, total_len, g_free);
  return g_steal_pointer (&transfer);
}

static void
mafp_sensor_cmd (FpiDeviceMafpmoc *self,
                 uint16_t          cmd,
                 const uint8_t    *data,
                 uint8_t           data_len,
                 SynCmdMsgCallback callback)
{
  g_autoptr(FpiUsbTransfer) transfer = NULL;
  CommandData *cmd_data = NULL;

  if (!(transfer = alloc_cmd_transfer (self, cmd, 1, data, data_len)))
    {
      g_critical ("Failed to allocate command transfer");

      if (callback)
        callback (self, NULL, g_error_new (FP_DEVICE_ERROR, FP_DEVICE_ERROR_PROTO,
                                           "Failed to allocate command transfer"));

      return;
    }

  cmd_data = g_new0 (CommandData, 1);
  cmd_data->cmd = cmd;
  cmd_data->callback = callback;
  cmd_data->cmd_transfer = g_steal_pointer (&transfer);
  cmd_data->cmd_cancelable = FALSE;
  cmd_data->cmd_force_pass = self->cmd_force_pass;
  cmd_data->cmd_request_len = 0;
  cmd_data->cmd_actual_len = 0;
  self->cmd_force_pass = FALSE;

  self->cmd_ssm = fpi_ssm_new (FP_DEVICE (self), fp_cmd_run_state, MAPF_CMD_TRANSFER_STATES);
  if (!PRINT_SSM_DEBUG)
    fpi_ssm_silence_debug (self->cmd_ssm);
  fpi_ssm_set_data (self->cmd_ssm, cmd_data, (GDestroyNotify) fp_cmd_ssm_done_data_free);
  fpi_ssm_start (self->cmd_ssm, fp_cmd_ssm_done);
}

static void
mafp_sensor_control (FpiDeviceMafpmoc      *self,
                     uint8_t                request,
                     uint16_t               value,
                     FpiUsbTransferCallback callback,
                     gpointer               user_data,
                     uint16_t               timeout)
{
  FpiUsbTransfer *transfer = fpi_usb_transfer_new (FP_DEVICE (self));

  transfer->ssm = self->task_ssm;
  fpi_usb_transfer_fill_control (transfer,
                                 G_USB_DEVICE_DIRECTION_DEVICE_TO_HOST,
                                 G_USB_DEVICE_REQUEST_TYPE_VENDOR,
                                 G_USB_DEVICE_RECIPIENT_DEVICE, request, value, 0, 1);
  fpi_usb_transfer_submit (transfer, timeout ? timeout : CTRL_TIMEOUT, NULL, callback, user_data);
  return;
}

static mafp_template_t
mafp_template_from_print (FpPrint *print)
{
  g_autoptr(GVariant) data = NULL;
  g_autoptr(GVariant) tpl_uid = NULL;
  g_autoptr(GVariant) dev_sn = NULL;
  uint16_t tpl_id = 0;
  const char *user_id;
  const char *serial_num;
  gsize user_id_len = 0;
  gsize serial_num_len = 0;
  mafp_template_t template = { 0, };

  g_object_get (print, "fpi-data", &data, NULL);
  if (!data)
    return template;

  g_variant_get (data, "(q@ay@ay)", &tpl_id, &tpl_uid, &dev_sn);
  user_id = g_variant_get_fixed_array (tpl_uid, &user_id_len, 1);
  serial_num = g_variant_get_fixed_array (dev_sn, &serial_num_len, 1);

  template.id = tpl_id;
  memset (template.uid, 0, TEMPLATE_UID_SIZE);
  if (user_id && user_id_len > 0)
    memcpy (template.uid, user_id, MIN (user_id_len, sizeof (template.uid) - 1));
  memset (template.sn, 0, DEVICE_SN_SIZE);
  if (serial_num && serial_num_len > 0)
    memcpy (template.sn, serial_num, MIN (serial_num_len, sizeof (template.sn) - 1));

  return template;
}

static FpPrint *
mafp_print_from_template (FpiDeviceMafpmoc *self, mafp_template_t *template)
{
  FpPrint *print;
  GVariant *data;
  GVariant *uid;
  GVariant *dev_sn;
  g_autofree char *uid_str = g_strndup (template->uid, TEMPLATE_UID_SIZE);
  const char *serial = self->serial_number ? self->serial_number : "";
  unsigned user_id_len = strlen (uid_str);
  unsigned serial_num_len = strnlen (serial, DEVICE_SN_SIZE);

  print = fp_print_new (FP_DEVICE (self));

  uid = g_variant_new_fixed_array (G_VARIANT_TYPE_BYTE, uid_str, user_id_len, 1);

  dev_sn = g_variant_new_fixed_array (G_VARIANT_TYPE_BYTE, serial, serial_num_len, 1);
  fp_dbg ("print: %d/%s/%s", template->id, uid_str, serial);

  data = g_variant_new ("(q@ay@ay)", template->id, uid, dev_sn);

  fpi_print_set_type (print, FPI_PRINT_RAW);
  fpi_print_set_device_stored (print, true);
  g_object_set (print, "description", uid_str, NULL);
  g_object_set (print, "fpi-data", data, NULL);

  fpi_print_fill_from_user_id (print, uid_str);

  return print;
}

static void
mafp_load_enrolled_ids (FpiDeviceMafpmoc *self, mafp_cmd_response_t *resp)
{
  uint16_t num = 0;
  char msg[1024] = {0};
  size_t used = 0;

  for (uint16_t i = 0; i < sizeof (resp->tpl_table.list); i++)
    {
      if (resp->tpl_table.list[i])
        {
          self->templates->total_list[num++].id = i;
          if (used < sizeof (msg))
            {
              int written = g_snprintf (msg + used, sizeof (msg) - used, "%u ", i);
              if (written > 0)
                used += MIN ((size_t) written, sizeof (msg) - used);
            }
        }
    }
  self->templates->index = 0;
  self->templates->total_num = num;
  fp_dbg ("enrolled ids: %s", msg);
  fp_dbg ("enrolled num: %d", self->templates->total_num);
}

static void
fp_init_handeshake_cb (FpiDeviceMafpmoc    *self,
                       mafp_cmd_response_t *resp,
                       GError              *error)
{
  FpDevice *dev = FP_DEVICE (self);

  if (error)
    {
      fp_dbg ("handshake fail");
      fpi_ssm_mark_failed (self->task_ssm, g_steal_pointer (&error));
      return;
    }

  fp_dbg ("result: %d, handshake code 0x%02x 0x%02x",
          resp->result, resp->handshake.code[0], resp->handshake.code[1]);

  if (resp->result == MAFP_SUCCESS &&
      resp->handshake.code[0] == MAFP_HANDSHAKE_CODE1 &&
      resp->handshake.code[1] == MAFP_HANDSHAKE_CODE2)
    {
      fpi_ssm_next_state (self->task_ssm);
      return;
    }

  mafp_mark_failed (dev, self->task_ssm, FP_DEVICE_ERROR_GENERAL,
                    "Failed to handshake, result: 0x%x", resp->result);
}

static void
fp_init_module_status_cb (FpiDeviceMafpmoc    *self,
                          mafp_cmd_response_t *resp,
                          GError              *error)
{
  if (error)
    {
      resp->result = 0xff;
      g_clear_error (&error);
    }

  fp_dbg ("result: %d", resp->result);

  if ((resp->result & MAFP_RE_CALIBRATE_ERROR) == MAFP_RE_CALIBRATE_ERROR)
    fp_dbg ("no calibrate data");

  fpi_ssm_next_state (self->task_ssm);
}

static void
fp_init_clean_epin_cb (FpiUsbTransfer *transfer,
                       FpDevice       *device,
                       gpointer        user_data,
                       GError         *error)
{
  FpiDeviceMafpmoc *self = FPI_DEVICE_MAFPMOC (device);

  fpi_ssm_next_state (self->task_ssm);
}

static void
fp_init_clean_epout_cb (FpiUsbTransfer *transfer,
                        FpDevice       *device,
                        gpointer        user_data,
                        GError         *error)
{
  FpiDeviceMafpmoc *self = FPI_DEVICE_MAFPMOC (device);

  fpi_ssm_next_state (self->task_ssm);
}

static void
fp_init_run_state (FpiSsm *ssm, FpDevice *device)
{
  FpiDeviceMafpmoc *self = FPI_DEVICE_MAFPMOC (device);
  FpiUsbTransfer *transfer;

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case MAPF_INIT_CLEAN_EPIN:
      transfer = fpi_usb_transfer_new (device);
      fpi_usb_transfer_fill_bulk (transfer, MAFP_EP_BULK_IN, MAFP_USB_BUFFER_SIZE);
      fpi_usb_transfer_submit (transfer, 100, NULL, fp_init_clean_epin_cb, NULL);
      break;

    case MAPF_INIT_CLEAN_EPOUT:
      transfer = fpi_usb_transfer_new (device);
      fpi_usb_transfer_fill_bulk (transfer, MAFP_EP_BULK_OUT, MAFP_USB_BUFFER_SIZE);
      fpi_usb_transfer_submit (transfer, 100, NULL, fp_init_clean_epout_cb, NULL);
      break;

    case MAPF_INIT_CLEAN_EPIN2:
      transfer = fpi_usb_transfer_new (device);
      fpi_usb_transfer_fill_bulk (transfer, MAFP_EP_BULK_IN, MAFP_USB_BUFFER_SIZE);
      fpi_usb_transfer_submit (transfer, 100, NULL, fp_init_clean_epin_cb, NULL);
      break;

    case MAPF_INIT_HANDSHAKE:
      mafp_sensor_cmd (self, MOC_CMD_HANDSHAKE, NULL, 0, fp_init_handeshake_cb);
      break;

    case MAPF_INIT_MODULE_STATUS:
      self->cmd_force_pass = TRUE;
      mafp_sensor_cmd (self, MOC_CMD_GET_INIT_STATUS, NULL, 0, fp_init_module_status_cb);
      break;
    }
}

static void
fp_init_ssm_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDeviceMafpmoc *self = FPI_DEVICE_MAFPMOC (dev);

  self->task_ssm = NULL;

  if (error)
    {
      fp_dbg ("%d %s", error->code, error->message);
      fpi_device_open_complete (dev, g_steal_pointer (&error));
      return;
    }

  fpi_device_open_complete (dev, NULL);
}

static void
fp_enroll_tpl_table_cb (FpiDeviceMafpmoc    *self,
                        mafp_cmd_response_t *resp,
                        GError              *error)
{
  FpDevice *dev = FP_DEVICE (self);

  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, g_steal_pointer (&error));
      return;
    }

  fp_dbg ("result: %d", resp->result);

  if (resp->result == MAFP_SUCCESS)
    {
      mafp_load_enrolled_ids (self, resp);
      self->enroll_id = G_MAXUINT16;
      for (uint16_t i = 0; i < sizeof (resp->tpl_table.list); i++)
        {
          if (!resp->tpl_table.list[i])
            {
              self->enroll_id = i;
              break;
            }
        }
      if (self->enroll_id == G_MAXUINT16)
        {
          mafp_mark_failed (dev, self->task_ssm, FP_DEVICE_ERROR_DATA_FULL,
                            "fingerprints total num reached max");
          return;
        }
      fpi_ssm_next_state (self->task_ssm);
    }
  else
    {
      mafp_mark_failed (dev, self->task_ssm, FP_DEVICE_ERROR_GENERAL,
                        "Failed to get fingerprints index, result: 0x%x", resp->result);
    }
}

static void
fp_enroll_read_tpl_cb (FpiDeviceMafpmoc    *self,
                       mafp_cmd_response_t *resp,
                       GError              *error)
{
  FpDevice *dev = FP_DEVICE (self);

  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, g_steal_pointer (&error));
      return;
    }

  fp_dbg ("result: %d", resp->result);

  uint8_t *resp_buff = (uint8_t *) resp;
  uint16_t max_id = 0;

  if (resp->result == MAFP_SUCCESS)
    {
      max_id = resp_buff[1] * 256 + resp_buff[2];
      fp_dbg ("max_id: %d, %x %x %x %x", max_id, resp_buff[0], resp_buff[1], resp_buff[2], resp_buff[3]);
      if (self->enroll_id >= max_id)
        {
          mafp_mark_failed (dev, self->task_ssm, FP_DEVICE_ERROR_DATA_FULL,
                            "fingerprints total num reached max");
          return;
        }
    }
  else
    {
      mafp_mark_failed (dev, self->task_ssm, FP_DEVICE_ERROR_DATA_FULL,
                        "fingerprints query total num fail");
      return;
    }
  fpi_device_report_finger_status (dev, FP_FINGER_STATUS_NONE | FP_FINGER_STATUS_NEEDED);
  fpi_ssm_next_state (self->task_ssm);
}

static void
fp_enroll_get_image_cb (FpiDeviceMafpmoc    *self,
                        mafp_cmd_response_t *resp,
                        GError              *error)
{
  g_autoptr(GError) local_error = NULL;
  FpDevice *dev = FP_DEVICE (self);
  MapfEnrollState nextState = MAFP_ENROLL_VERIFY_GET_IMAGE;

  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, g_steal_pointer (&error));
      return;
    }

  if (g_cancellable_set_error_if_cancelled (fpi_device_get_cancellable (dev),
                                            &local_error))
    {
      fpi_ssm_mark_failed (self->task_ssm, g_steal_pointer (&local_error));
      return;
    }

  if (self->press_state == MAFP_PRESS_WAIT_DOWN)
    {
      fp_dbg ("wait finger down state %d", resp->result);
      if (resp->result == MAFP_RE_GET_IMAGE_SUCCESS)
        {
          nextState = MAFP_ENROLL_VERIFY_GENERATE_FEATURE;
        }
      else if (resp->result == MAFP_RE_GET_IMAGE_NONE)
        {
          self->capture_cnt++;
          fp_dbg ("capture_cnt %d", self->capture_cnt);
          if (self->capture_cnt > MAFP_IMAGE_ERR_TRRIGER)
            nextState = MAFP_ENROLL_REFRESH_INT_PARA;
          else
            nextState = MAFP_ENROLL_DETECT_MODE;
        }
    }
  else if (self->press_state == MAFP_PRESS_WAIT_UP)
    {
      fp_dbg ("wait finger up state %d", resp->result);
      if (resp->result == MAFP_RE_GET_IMAGE_SUCCESS)
        {
          nextState = MAFP_ENROLL_VERIFY_GET_IMAGE;
        }
      else if (resp->result == MAFP_RE_GET_IMAGE_NONE)
        {
          self->press_state = MAFP_PRESS_WAIT_DOWN;
          fpi_device_report_finger_status (dev, FP_FINGER_STATUS_NONE | FP_FINGER_STATUS_NEEDED);
          nextState = MAFP_ENROLL_CHECK_INT_PARA;
        }
    }
  fpi_ssm_jump_to_state (self->task_ssm, nextState);
}

static void
fp_enroll_verify_search_cb (FpiDeviceMafpmoc    *self,
                            mafp_cmd_response_t *resp,
                            GError              *error)
{
  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, g_steal_pointer (&error));
      return;
    }

  fp_dbg ("result: %d", resp->result);

  if (resp->result == MAFP_SUCCESS)
    {
      self->search_id = ((resp->search.id[0] & 0xff) << 8) | (resp->search.id[1] & 0xff);
      fp_dbg ("search_id: %d", self->search_id);
      fpi_ssm_jump_to_state (self->task_ssm, MAFP_ENROLL_GET_TEMPLATE_INFO);
    }
  else
    {
      self->search_id = G_MAXUINT16;
      if (self->enroll_stage >= fp_device_get_nr_enroll_stages (FP_DEVICE (self)))
        fpi_ssm_jump_to_state (self->task_ssm, MAFP_ENROLL_SAVE_TEMPLATE_INFO);
      else
        fpi_ssm_jump_to_state (self->task_ssm, MAFP_ENROLL_VERIFY_GET_IMAGE);
    }
}

static void
fp_enroll_get_tpl_info_cb (FpiDeviceMafpmoc    *self,
                           mafp_cmd_response_t *resp,
                           GError              *error)
{
  FpDevice *dev = FP_DEVICE (self);
  g_autofree char *uid = NULL;

  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, g_steal_pointer (&error));
      return;
    }

  uid = g_strndup (resp->tpl_info.uid, TEMPLATE_UID_SIZE);
  fp_dbg ("result: %d, %s", resp->result, uid);

  if (resp->result == MAFP_SUCCESS)
    {
      if (resp->tpl_info.uid[0] == 'F' && resp->tpl_info.uid[1] == 'P')
        {
          g_autoptr(FpPrint) print = NULL;
          mafp_template_t tpl;

          tpl.id = self->search_id;
          memcpy (tpl.uid, resp->tpl_info.uid, sizeof (resp->tpl_info.uid));
          print = mafp_print_from_template (self, &tpl);

          mafp_mark_failed (dev, self->task_ssm, FP_DEVICE_ERROR_DATA_DUPLICATE,
                            "Finger was already enrolled as '%s'",
                            fp_print_get_description (print));
          return;
        }
    }
  if (self->enroll_stage >= fp_device_get_nr_enroll_stages (FP_DEVICE (self)))
    fpi_ssm_jump_to_state (self->task_ssm, MAFP_ENROLL_SAVE_TEMPLATE_INFO);
  else
    fpi_ssm_jump_to_state (self->task_ssm, MAFP_ENROLL_VERIFY_GET_IMAGE);
}

static void
fp_enroll_once_complete_cb (FpiDeviceMafpmoc    *self,
                            mafp_cmd_response_t *resp)
{
  FpDevice *dev = FP_DEVICE (self);

  if (resp->result == MAFP_SUCCESS)
    {
      self->enroll_stage++;
      self->press_state = MAFP_PRESS_WAIT_UP;
      fpi_device_enroll_progress (dev, self->enroll_stage, NULL, NULL);

      if (self->enroll_identify_state == MAFP_ENROLL_IDENTIFY_DISABLED)
        {
          if (self->enroll_stage >= fp_device_get_nr_enroll_stages (FP_DEVICE (self)))
            fpi_ssm_jump_to_state (self->task_ssm, MAFP_ENROLL_SAVE_TEMPLATE_INFO);
          else
            fpi_ssm_jump_to_state (self->task_ssm, MAFP_ENROLL_VERIFY_GET_IMAGE);
          return;
        }
      if (self->enroll_identify_state == MAFP_ENROLL_IDENTIFY_ONCE)
        self->enroll_identify_state = MAFP_ENROLL_IDENTIFY_DISABLED;
      fpi_ssm_jump_to_state (self->task_ssm, MAFP_ENROLL_VERIFY_SEARCH);
    }
  else
    {
      self->press_state = MAFP_PRESS_WAIT_UP;
      fpi_device_enroll_progress (dev, self->enroll_stage, NULL,
                                  fpi_device_retry_new (FP_DEVICE_RETRY_GENERAL));
      fpi_ssm_jump_to_state (self->task_ssm, MAFP_ENROLL_VERIFY_GET_IMAGE);
    }
}

static void
fp_enroll_gen_feature_cb (FpiDeviceMafpmoc    *self,
                          mafp_cmd_response_t *resp,
                          GError              *error)
{
  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, g_steal_pointer (&error));
      return;
    }

  fp_dbg ("result: %d", resp->result);

  if (self->enroll_dupl_area_state == MAFP_ENROLL_DUPLICATE_AREA_DENY)
    {
      int device_stages = fp_device_get_nr_enroll_stages (FP_DEVICE (self));
      int remain_stage = device_stages - self->enroll_stage;

      /* check duplicate area in last 3 times */
      if (remain_stage > 0 && remain_stage <= 3)
        {
          fpi_ssm_next_state (self->task_ssm);
          return;
        }
    }
  fp_enroll_once_complete_cb (self, resp);
}

static void
fp_enroll_verify_duparea_cb (FpiDeviceMafpmoc    *self,
                             mafp_cmd_response_t *resp,
                             GError              *error)
{
  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, g_steal_pointer (&error));
      return;
    }

  fp_dbg ("result: %d", resp->result);

  if (resp->result != MAFP_SUCCESS)
    resp->result = 1;
  fp_enroll_once_complete_cb (self, resp);
}

static void
fp_enroll_save_tpl_info_cb (FpiDeviceMafpmoc    *self,
                            mafp_cmd_response_t *resp,
                            GError              *error)
{
  FpDevice *dev = FP_DEVICE (self);

  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, g_steal_pointer (&error));
      return;
    }

  fp_dbg ("result: %d", resp->result);

  if (resp->result == MAFP_RE_TPL_NUM_OVERSIZE)
    {
      mafp_mark_failed (dev, self->task_ssm, FP_DEVICE_ERROR_DATA_FULL,
                        "fingerprints total num reached max");
      return;
    }
  if (resp->result != MAFP_SUCCESS)
    {
      mafp_mark_failed (dev, self->task_ssm, FP_DEVICE_ERROR_GENERAL,
                        "Failed to save template info, result: 0x%x", resp->result);
      return;
    }
  fpi_ssm_next_state (self->task_ssm);
}

static void
fp_enroll_save_tpl_cb (FpiDeviceMafpmoc    *self,
                       mafp_cmd_response_t *resp,
                       GError              *error)
{
  FpDevice *dev = FP_DEVICE (self);
  FpPrint *print = NULL;
  GVariant *uid = NULL;
  GVariant *data = NULL;
  GVariant *dev_sn;
  unsigned user_id_len;
  unsigned serial_num_len;
  const char *user_id = NULL;
  const char *serial_num = NULL;

  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, g_steal_pointer (&error));
      return;
    }

  fp_dbg ("result: %d", resp->result);

  if (resp->result == MAFP_SUCCESS)
    {
      fpi_device_get_enroll_data (dev, &print);

      user_id = self->enroll_user_id ? self->enroll_user_id : "";
      user_id_len = strnlen (user_id, TEMPLATE_UID_SIZE);
      fp_dbg ("user_id(%d): %s", user_id_len, user_id);
      uid = g_variant_new_fixed_array (G_VARIANT_TYPE_BYTE, user_id, user_id_len, 1);

      serial_num = self->serial_number ? self->serial_number : "";
      serial_num_len = strnlen (serial_num, DEVICE_SN_SIZE);
      fp_dbg ("dev_sn(%d): %s", serial_num_len, serial_num);
      dev_sn = g_variant_new_fixed_array (G_VARIANT_TYPE_BYTE, serial_num, serial_num_len, 1);

      data = g_variant_new ("(q@ay@ay)", self->enroll_id, uid, dev_sn);

      fpi_print_set_type (print, FPI_PRINT_RAW);
      fpi_print_set_device_stored (print, TRUE);
      g_object_set (print, "description", user_id, NULL);
      g_object_set (print, "fpi-data", data, NULL);

      fpi_ssm_jump_to_state (self->task_ssm, MAFP_ENROLL_EXIT);
      return;
    }
  fpi_ssm_next_state (self->task_ssm);
}

static void
fp_enroll_del_tpl_info_cb (FpiDeviceMafpmoc    *self,
                           mafp_cmd_response_t *resp,
                           GError              *error)
{
  FpDevice *dev = FP_DEVICE (self);

  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, g_steal_pointer (&error));
      return;
    }

  fp_dbg ("result: %d", resp->result);
  mafp_mark_failed (dev, self->task_ssm, FP_DEVICE_ERROR_GENERAL,
                    "Failed to save template, result: 0x%x", resp->result);
}

static void
mafp_sleep_cb (FpiDeviceMafpmoc    *self,
               mafp_cmd_response_t *resp,
               GError              *error)
{
  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, g_steal_pointer (&error));
      return;
    }
  fpi_ssm_next_state (self->task_ssm);
}

static void
mafp_pwr_btn_shield_off_cb (FpiUsbTransfer *transfer,
                            FpDevice       *device,
                            gpointer        user_data,
                            GError         *error)
{
  FpiDeviceMafpmoc *self = FPI_DEVICE_MAFPMOC (device);

  if (error)
    {
      fpi_ssm_mark_failed (transfer->ssm, g_steal_pointer (&error));
      return;
    }
  uint8_t para = 0;

  mafp_sensor_cmd (self, MOC_CMD_SLEEP, &para, 1, mafp_sleep_cb);
}

static void
mafp_pwr_btn_shield_on_cb (FpiUsbTransfer *transfer,
                           FpDevice       *device,
                           gpointer        user_data,
                           GError         *error)
{
  if (error)
    {
      fpi_ssm_mark_failed (transfer->ssm, g_steal_pointer (&error));
      return;
    }
  fpi_ssm_next_state (transfer->ssm);
}

static void
mafp_pwr_btn_shield_on (FpiDeviceMafpmoc *self, int on)
{
  GError *pre_error = fpi_ssm_get_error (self->task_ssm);

  if (g_error_matches (pre_error, G_USB_DEVICE_ERROR, G_USB_DEVICE_ERROR_FAILED))
    {
      fpi_ssm_next_state (self->task_ssm);
      return;
    }

  if (on)
    mafp_sensor_control (self, 0x8B, 0x01, mafp_pwr_btn_shield_on_cb, NULL, 1000);
  else
    mafp_sensor_control (self, 0x8B, 0x00, mafp_pwr_btn_shield_off_cb, NULL, 0);
}

static void
fp_enroll_int_check_cb (FpiDeviceMafpmoc    *self,
                        mafp_cmd_response_t *resp,
                        GError              *error)
{
  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, g_steal_pointer (&error));
      return;
    }
  fpi_ssm_next_state (self->task_ssm);
}

static void
fp_enroll_int_detect_cb (FpiDeviceMafpmoc    *self,
                         mafp_cmd_response_t *resp,
                         GError              *error)
{
  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, g_steal_pointer (&error));
      return;
    }
  fpi_ssm_next_state (self->task_ssm);
}

static void
fp_enroll_int_refresh_cb (FpiDeviceMafpmoc    *self,
                          mafp_cmd_response_t *resp,
                          GError              *error)
{
  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, g_steal_pointer (&error));
      return;
    }
  self->capture_cnt = 0;
  fpi_ssm_jump_to_state (self->task_ssm, MAFP_ENROLL_VERIFY_GET_IMAGE);
}

static void
fp_enroll_enable_int_cb (FpiUsbTransfer *transfer,
                         FpDevice       *device,
                         gpointer        user_data,
                         GError         *error)
{
  if (error)
    {
      fpi_ssm_mark_failed (transfer->ssm, g_steal_pointer (&error));
      return;
    }
  fpi_ssm_next_state (transfer->ssm);
}

static void
fp_enroll_disable_int_cb (FpiUsbTransfer *transfer,
                          FpDevice       *device,
                          gpointer        user_data,
                          GError         *error)
{
  if (error)
    {
      fpi_ssm_mark_failed (transfer->ssm, g_steal_pointer (&error));
      return;
    }
  fpi_ssm_jump_to_state (transfer->ssm, MAFP_ENROLL_VERIFY_GET_IMAGE);
}

static void
fp_enroll_wait_int_cb (FpiUsbTransfer *transfer,
                       FpDevice       *device,
                       gpointer        user_data,
                       GError         *error)
{
  FpiDeviceMafpmoc *self = FPI_DEVICE_MAFPMOC (device);

  if (error)
    {
      fp_dbg ("code %d", error->code);
      if (error->code == G_USB_DEVICE_ERROR_TIMED_OUT)
        {
          fpi_ssm_jump_to_state (self->task_ssm, MAFP_ENROLL_VERIFY_GET_IMAGE);
          g_clear_error (&error);
          return;
        }

      fpi_ssm_mark_failed (self->task_ssm, g_steal_pointer (&error));
      return;
    }
  fp_dbg ("actual_length %zd", transfer->actual_length);
  if (transfer->actual_length == 2)
    {
      if (transfer->buffer[0] == 0x04 && transfer->buffer[1] == 0xe5)
        {
          fp_dbg ("int trigger");
          fpi_ssm_next_state (self->task_ssm);
          return;
        }
    }
  fpi_ssm_mark_failed (self->task_ssm, fpi_device_error_new (FP_DEVICE_ERROR_GENERAL));
}

static void
fp_enroll_wait_int (FpiDeviceMafpmoc *self)
{
  fp_dbg ("wait interrupt");
  FpiUsbTransfer *transfer = fpi_usb_transfer_new (FP_DEVICE (self));

  fpi_usb_transfer_fill_interrupt (transfer, MAFP_EP_INT_IN, 2);
  fpi_usb_transfer_submit (transfer,
                           30 * 1000,
                           fpi_device_get_cancellable (FP_DEVICE (self)),
                           fp_enroll_wait_int_cb,
                           NULL);
}

static void
fp_empty_cb (FpiDeviceMafpmoc    *self,
             mafp_cmd_response_t *resp,
             GError              *error)
{
  if (error)
    {
      fp_dbg ("error: %s", error->message);
      g_clear_error (&error);
    }
  else
    {
      fp_dbg ("result: %d", resp ? resp->result : -1);
    }

  fpi_ssm_next_state (self->task_ssm);
}

static void
mafp_check_empty (FpiDeviceMafpmoc *self)
{
  mafp_sensor_cmd (self, MOC_CMD_EMPTY, NULL, 0, fp_empty_cb);
}

static void
fp_enroll_sm_run_state (FpiSsm *ssm, FpDevice *device)
{
  FpiDeviceMafpmoc *self = FPI_DEVICE_MAFPMOC (device);
  uint8_t para[PACKAGE_DATA_SIZE_MAX] = { 0 };
  FpPrint *print = NULL;
  uint16_t range = 1000;

  switch(fpi_ssm_get_cur_state (ssm))
    {
    case MAFP_ENROLL_PWR_BTN_SHIELD_ON:
      mafp_pwr_btn_shield_on (self, 1);
      break;

    case MAFP_ENROLL_CHECK_EMPTY:
      mafp_check_empty (self);
      break;

    case MAFP_ENROLL_TEMPLATE_TABLE:
      para[0] = 0; /* page no. */
      mafp_sensor_cmd (self, MOC_CMD_GET_TEMPLATE_TABLE, (const uint8_t *) &para, 1, fp_enroll_tpl_table_cb);
      break;

    case MAFP_ENROLL_READ_TEMPLATE:
      mafp_sensor_cmd (self, MOC_CMD_GET_MAX_ID, NULL, 0, fp_enroll_read_tpl_cb);
      break;

    case MAFP_ENROLL_VERIFY_GET_IMAGE:
      mafp_sensor_cmd (self, MOC_CMD_GET_IMAGE, NULL, 0, fp_enroll_get_image_cb);
      break;

    case MAFP_ENROLL_CHECK_INT_PARA:
      para[0] = MAFP_SLEEP_INT_CHECK;
      mafp_sensor_cmd (self, MOC_CMD_SLEEP, para, 1, fp_enroll_int_check_cb);
      break;

    case MAFP_ENROLL_DETECT_MODE:
      para[0] = MAFP_SLEEP_INT_WAIT;
      mafp_sensor_cmd (self, MOC_CMD_SLEEP, para, 1, fp_enroll_int_detect_cb);
      break;

    case MAFP_ENROLL_ENABLE_INT:
      mafp_sensor_control (self, 0x89, 1, fp_enroll_enable_int_cb, NULL, 0);
      break;

    case MAFP_ENROLL_WAIT_INT:
      fp_enroll_wait_int (self);
      break;

    case MAFP_ENROLL_DISBALE_INT:
      mafp_sensor_control (self, 0x89, 0, fp_enroll_disable_int_cb, NULL, 0);
      break;

    case MAFP_ENROLL_REFRESH_INT_PARA:
      fp_dbg ("refresh param");
      para[0] = MAFP_SLEEP_INT_REFRESH;
      mafp_sensor_cmd (self, MOC_CMD_SLEEP, para, 1, fp_enroll_int_refresh_cb);
      break;

    case MAFP_ENROLL_VERIFY_GENERATE_FEATURE:
      para[0] = self->enroll_stage + 1;   /* verify buffer id start from 1 */
      mafp_sensor_cmd (self, MOC_CMD_GEN_FEATURE, (const uint8_t *) &para, 1, fp_enroll_gen_feature_cb);
      break;

    case MAFP_ENROLL_VERIFY_DUPLICATE_AREA:
      mafp_sensor_cmd (self, MOC_CMD_DUPAREA_TEST, NULL, 0, fp_enroll_verify_duparea_cb);
      break;

    case MAFP_ENROLL_VERIFY_SEARCH:
      para[0] = 1;                     /* buffer id */
      para[1] = 0;                     /* start id high */
      para[2] = 0;                     /* start id low */
      para[3] = (range >> 8) & 0xff;   /* range high */
      para[4] = range & 0xff;          /* range low */
      mafp_sensor_cmd (self, MOC_CMD_SEARCH, (const uint8_t *) &para, 5, fp_enroll_verify_search_cb);
      break;

    case MAFP_ENROLL_GET_TEMPLATE_INFO:
      para[0] = (self->search_id >> 8) & 0xff;   /* fp id high */
      para[1] = self->search_id & 0xff;          /* fp id low */
      mafp_sensor_cmd (self, MOC_CMD_GET_TEMPLATE_INFO, (const uint8_t *) &para, 2, fp_enroll_get_tpl_info_cb);
      break;

    case MAFP_ENROLL_SAVE_TEMPLATE_INFO:
      fpi_device_get_enroll_data (device, &print);
      self->enroll_user_id = fpi_print_generate_user_id (print);
      para[0] = (self->enroll_id >> 8) & 0xff;   /* fp id high */
      para[1] = self->enroll_id & 0xff;          /* fp id low */
      memcpy (para + 2,
              self->enroll_user_id,
              MIN (strnlen (self->enroll_user_id, TEMPLATE_UID_SIZE), (gsize) TEMPLATE_UID_SIZE));
      fp_dbg ("user_id: %s", self->enroll_user_id);
      mafp_sensor_cmd (self, MOC_CMD_SAVE_TEMPLATE_INFO, (const uint8_t *) &para, 2 + TEMPLATE_UID_SIZE, fp_enroll_save_tpl_info_cb);
      break;

    case MAFP_ENROLL_SAVE_TEMPLATE:
      para[0] = 1;                               /* buffer id */
      para[1] = (self->enroll_id >> 8) & 0xff;   /* fp id high */
      para[2] = self->enroll_id & 0xff;          /* fp id low */
      mafp_sensor_cmd (self, MOC_CMD_SAVE_TEMPLATE, (const uint8_t *) &para, 3, fp_enroll_save_tpl_cb);
      break;

    case MAFP_ENROLL_DELETE_TEMPLATE_INFO_IF_FAILED:
      para[0] = (self->enroll_id >> 8) & 0xff;   /* fp id high */
      para[1] = self->enroll_id & 0xff;          /* fp id low */
      mafp_sensor_cmd (self, MOC_CMD_SAVE_TEMPLATE_INFO, (const uint8_t *) &para, 130, fp_enroll_del_tpl_info_cb);
      break;

    case MAFP_ENROLL_EXIT:
      mafp_pwr_btn_shield_on (self, 0);
      break;
    }
}

static void
fp_enroll_ssm_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDeviceMafpmoc *self = FPI_DEVICE_MAFPMOC (dev);
  FpPrint *print = NULL;

  self->task_ssm = NULL;

  if (error)
    {
      fp_dbg ("enroll done fail");
      fpi_device_enroll_complete (dev, NULL, g_steal_pointer (&error));
      return;
    }

  fp_dbg ("enroll completed");
  fpi_device_get_enroll_data (dev, &print);
  fpi_device_enroll_complete (dev, g_object_ref (print), NULL);
}


static void
fp_identify_tpl_table_cb (FpiDeviceMafpmoc    *self,
                          mafp_cmd_response_t *resp,
                          GError              *error)
{
  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, g_steal_pointer (&error));
      return;
    }

  fp_dbg ("result: %d", resp->result);

  if (resp->result == MAFP_SUCCESS)
    mafp_load_enrolled_ids (self, resp);
  fpi_device_report_finger_status (FP_DEVICE (self), FP_FINGER_STATUS_NONE | FP_FINGER_STATUS_NEEDED);
  fpi_ssm_next_state (self->task_ssm);
}

static void
fp_identify_get_image_cb (FpiDeviceMafpmoc    *self,
                          mafp_cmd_response_t *resp,
                          GError              *error)
{
  g_autoptr(GError) local_error = NULL;
  FpDevice *dev = FP_DEVICE (self);
  MapfIdentifyState nextState = MAPF_IDENTIFY_GET_IMAGE;

  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, g_steal_pointer (&error));
      return;
    }

  if (g_cancellable_set_error_if_cancelled (fpi_device_get_cancellable (dev),
                                            &local_error))
    {
      fpi_ssm_mark_failed (self->task_ssm, g_steal_pointer (&local_error));
      return;
    }

  if (self->press_state == MAFP_PRESS_WAIT_DOWN)
    {
      fp_dbg ("wait finger down state %d", resp->result);
      if (resp->result == MAFP_RE_GET_IMAGE_SUCCESS)
        {
          nextState = MAPF_IDENTIFY_GENERATE_FEATURE;
        }
      else if (resp->result == MAFP_RE_GET_IMAGE_NONE)
        {
          self->capture_cnt++;
          fp_dbg ("self->capture_cnt %d", self->capture_cnt);
          if (self->capture_cnt > MAFP_IMAGE_ERR_TRRIGER)
            nextState = MAPF_IDENTIFY_REFRESH_INT_PARA;
          else
            nextState = MAPF_IDENTIFY_DETECT_MODE;
        }
    }
  else if (self->press_state == MAFP_PRESS_WAIT_UP)
    {
      fp_dbg ("wait finger up state %d", resp->result);
      if (resp->result == MAFP_RE_GET_IMAGE_SUCCESS)
        {
          nextState = MAPF_IDENTIFY_GET_IMAGE;
        }
      else if (resp->result == MAFP_RE_GET_IMAGE_NONE)
        {
          self->press_state = MAFP_PRESS_WAIT_DOWN;
          fpi_device_report_finger_status (dev, FP_FINGER_STATUS_NONE | FP_FINGER_STATUS_NEEDED);
          nextState = MAPF_IDENTIFY_CHECK_INT_PARA;
        }
    }

  fpi_ssm_jump_to_state (self->task_ssm, nextState);
}

static void
fp_identify_gen_feature_cb (FpiDeviceMafpmoc    *self,
                            mafp_cmd_response_t *resp,
                            GError              *error)
{
  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, g_steal_pointer (&error));
      return;
    }

  fp_dbg ("result: %d", resp->result);

  if (resp->result == MAFP_SUCCESS)
    {
      self->enroll_identify_index = 0;
      self->press_state = MAFP_PRESS_WAIT_UP;
      fpi_ssm_jump_to_state (self->task_ssm, MAPF_IDENTIFY_SEARCH_STEP);
    }
  else
    {
      self->press_state = MAFP_PRESS_WAIT_UP;
      fpi_ssm_jump_to_state (self->task_ssm, MAPF_IDENTIFY_GET_IMAGE);
    }
}

static void
mafp_scl_ctl_cb (FpiUsbTransfer *transfer,
                 FpDevice       *device,
                 gpointer        user_data,
                 GError         *error)
{
  if (error)
    fp_dbg ("control transfer out fail, %s", error->message);

  fpi_ssm_jump_to_state (transfer->ssm, MAPF_IDENTIFY_EXIT);
}

static void
fp_identify_get_tpl_info_cb (FpiDeviceMafpmoc    *self,
                             mafp_cmd_response_t *resp,
                             GError              *error)
{
  g_autoptr(GError) local_error = NULL;
  FpDevice *dev = FP_DEVICE (self);
  FpPrint *new_scan = NULL;
  FpPrint *matching = NULL;

  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, g_steal_pointer (&error));
      return;
    }

  if (g_cancellable_set_error_if_cancelled (fpi_device_get_cancellable (dev),
                                            &local_error))
    {
      fpi_ssm_mark_failed (self->task_ssm, g_steal_pointer (&local_error));
      return;
    }

  fp_dbg ("result: %d", resp->result);

  if (resp->result == MAFP_SUCCESS)
    {
      if (resp->tpl_info.uid[0] == 'F' && resp->tpl_info.uid[1] == 'P')
        {
          mafp_template_t tpl;

          tpl.id = self->search_id;
          memcpy (tpl.uid, resp->tpl_info.uid, sizeof (resp->tpl_info.uid));
          new_scan = mafp_print_from_template (self, &tpl);
        }

      if (new_scan != NULL)
        {
          GPtrArray *templates = NULL;
          guint matching_index;

          fpi_device_get_identify_data (dev, &templates);

          if (g_ptr_array_find_with_equal_func (templates, new_scan,
                                                (GEqualFunc) fp_print_equal,
                                                &matching_index))
            matching = g_ptr_array_index (templates, matching_index);
        }
    }

  self->identify_match_print = matching;
  self->identify_new_print = new_scan;

  if (!matching)
    {
      mafp_sensor_control (self, 0x8C, 0x00, mafp_scl_ctl_cb, NULL, 0);
      return;
    }
  fpi_ssm_jump_to_state (self->task_ssm, MAPF_IDENTIFY_EXIT);
}

static void
fp_identify_search_step_cb (FpiDeviceMafpmoc    *self,
                            mafp_cmd_response_t *resp,
                            GError              *error)
{
  GPtrArray *prints = NULL;
  FpDevice *dev = FP_DEVICE (self);

  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, g_steal_pointer (&error));
      return;
    }

  fp_dbg ("result: %d", resp->result);
  if (resp->result == MAFP_SUCCESS)
    {
      fp_dbg ("identify ok, search_id: %d", self->search_id);
      fpi_ssm_jump_to_state (self->task_ssm, MAPF_IDENTIFY_GET_TEMPLATE_INFO);
    }
  else
    {
      fp_dbg ("identify fail");
      fpi_device_get_identify_data (dev, &prints);
      self->enroll_identify_index++;
      if (self->enroll_identify_index < prints->len)
        {
          fpi_ssm_jump_to_state (self->task_ssm, MAPF_IDENTIFY_SEARCH_STEP);
          return;
        }
      self->search_id = G_MAXUINT16;
      fpi_ssm_jump_to_state (self->task_ssm, MAPF_IDENTIFY_GET_TEMPLATE_INFO);
    }
}

static void
mafp_get_startup_result_cb (FpiUsbTransfer *transfer,
                            FpDevice       *device,
                            gpointer        user_data,
                            GError         *error)
{
  FpiDeviceMafpmoc *self = FPI_DEVICE_MAFPMOC (device);

  if (error)
    {
      fp_dbg ("error: %s", error->message);
      fpi_ssm_next_state (transfer->ssm);
      g_clear_error (&error);
      return;
    }
  if (transfer->actual_length >= 5)
    {
      fp_dbg ("0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x", transfer->buffer[0], transfer->buffer[1],
              transfer->buffer[2], transfer->buffer[3], transfer->buffer[4]);
      if (transfer->buffer[0])
        {
          self->search_id = transfer->buffer[2] * 256 + transfer->buffer[1];
          usleep (1000 * 1000);
          fpi_ssm_jump_to_state (transfer->ssm, MAPF_IDENTIFY_GET_TEMPLATE_INFO);
          return;
        }
    }
  fpi_ssm_next_state (transfer->ssm);
}

static void
fp_identify_int_check_cb (FpiDeviceMafpmoc    *self,
                          mafp_cmd_response_t *resp,
                          GError              *error)
{
  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, g_steal_pointer (&error));
      return;
    }
  fpi_ssm_next_state (self->task_ssm);
}

static void
fp_identify_int_detect_cb (FpiDeviceMafpmoc    *self,
                           mafp_cmd_response_t *resp,
                           GError              *error)
{
  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, g_steal_pointer (&error));
      return;
    }
  fpi_ssm_next_state (self->task_ssm);
}

static void
fp_identify_int_refresh_cb (FpiDeviceMafpmoc    *self,
                            mafp_cmd_response_t *resp,
                            GError              *error)
{
  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, g_steal_pointer (&error));
      return;
    }
  self->capture_cnt = 0;
  fpi_ssm_jump_to_state (self->task_ssm, MAPF_IDENTIFY_GET_IMAGE);
}

static void
fp_identify_enable_int_cb (FpiUsbTransfer *transfer,
                           FpDevice       *device,
                           gpointer        user_data,
                           GError         *error)
{
  if (error)
    {
      fpi_ssm_mark_failed (transfer->ssm, g_steal_pointer (&error));
      return;
    }
  fpi_ssm_next_state (transfer->ssm);
}

static void
fp_identify_disable_int_cb (FpiUsbTransfer *transfer,
                            FpDevice       *device,
                            gpointer        user_data,
                            GError         *error)
{
  if (error)
    {
      fpi_ssm_mark_failed (transfer->ssm, g_steal_pointer (&error));
      return;
    }
  fpi_ssm_jump_to_state (transfer->ssm, MAPF_IDENTIFY_GET_IMAGE);
}

static void
fp_identify_wait_int_cb (FpiUsbTransfer *transfer,
                         FpDevice       *device,
                         gpointer        user_data,
                         GError         *error)
{
  FpiDeviceMafpmoc *self = FPI_DEVICE_MAFPMOC (device);

  if (error)
    {
      fp_dbg ("code %d", error->code);
      if (error->code == G_USB_DEVICE_ERROR_TIMED_OUT)
        {
          fpi_ssm_jump_to_state (self->task_ssm, MAPF_IDENTIFY_GET_IMAGE);
          g_clear_error (&error);
          return;
        }

      fpi_ssm_mark_failed (self->task_ssm, g_steal_pointer (&error));
      return;
    }
  fp_dbg ("actual_length %zd", transfer->actual_length);
  if (transfer->actual_length == 2)
    {
      if (transfer->buffer[0] == 0x04 && transfer->buffer[1] == 0xe5)
        {
          fp_dbg ("int trigger");
          fpi_ssm_next_state (self->task_ssm);
          return;
        }
    }
  fpi_ssm_mark_failed (self->task_ssm, fpi_device_error_new (FP_DEVICE_ERROR_GENERAL));
}

static void
fp_identify_wait_int (FpiDeviceMafpmoc *self)
{
  fp_dbg ("wait interrupt");
  FpiUsbTransfer *transfer = fpi_usb_transfer_new (FP_DEVICE (self));

  fpi_usb_transfer_fill_interrupt (transfer, MAFP_EP_INT_IN, 2);
  fpi_usb_transfer_submit (transfer,
                           30 * 60 * 1000,
                           fpi_device_get_cancellable (FP_DEVICE (self)),
                           fp_identify_wait_int_cb,
                           NULL);
}

static void
fp_identify_sm_run_state (FpiSsm *ssm, FpDevice *device)
{
  FpiDeviceMafpmoc *self = FPI_DEVICE_MAFPMOC (device);
  uint8_t para[PACKAGE_DATA_SIZE_MAX] = { 0 };
  GPtrArray *prints = NULL;
  FpPrint *print = NULL;

  switch(fpi_ssm_get_cur_state (ssm))
    {
    case MAPF_IDENTIFY_PWR_BTN_SHIELD_ON:
      mafp_pwr_btn_shield_on (self, 1);
      break;

    case MAPF_IDENTIFY_TEMPLATE_TABLE:
      para[0] = 0; /* page no. */
      mafp_sensor_cmd (self, MOC_CMD_GET_TEMPLATE_TABLE, (const uint8_t *) &para, 1, fp_identify_tpl_table_cb);
      break;

    case MAPF_IDENTIFY_GET_STARTUP_RESULT:
      mafp_sensor_control (self, 0x8D, 0x00, mafp_get_startup_result_cb, NULL, 0);
      break;

    case MAPF_IDENTIFY_GET_IMAGE:
      mafp_sensor_cmd (self, MOC_CMD_GET_IMAGE, NULL, 0, fp_identify_get_image_cb);
      break;

    case MAPF_IDENTIFY_CHECK_INT_PARA:
      para[0] = MAFP_SLEEP_INT_CHECK;
      mafp_sensor_cmd (self, MOC_CMD_SLEEP, para, 1, fp_identify_int_check_cb);
      break;

    case MAPF_IDENTIFY_DETECT_MODE:
      para[0] = MAFP_SLEEP_INT_WAIT;
      mafp_sensor_cmd (self, MOC_CMD_SLEEP, para, 1, fp_identify_int_detect_cb);
      break;

    case MAPF_IDENTIFY_ENABLE_INT:
      mafp_sensor_control (self, 0x89, 1, fp_identify_enable_int_cb, NULL, 0);
      break;

    case MAPF_IDENTIFY_WAIT_INT:
      fp_identify_wait_int (self);
      break;

    case MAPF_IDENTIFY_DISBALE_INT:
      mafp_sensor_control (self, 0x89, 0, fp_identify_disable_int_cb, NULL, 0);
      break;

    case MAPF_IDENTIFY_REFRESH_INT_PARA:
      fp_dbg ("refresh param");
      para[0] = MAFP_SLEEP_INT_REFRESH;
      mafp_sensor_cmd (self, MOC_CMD_SLEEP, para, 1, fp_identify_int_refresh_cb);
      break;

    case MAPF_IDENTIFY_GENERATE_FEATURE:
      para[0] = 1;  /* buffer id */
      mafp_sensor_cmd (self, MOC_CMD_GEN_FEATURE, (const uint8_t *) &para, 1, fp_identify_gen_feature_cb);
      break;

    case MAPF_IDENTIFY_SEARCH_STEP:
      fpi_device_get_identify_data (device, &prints);
      if (!prints || prints->len == 0)
        {
          self->search_id = G_MAXUINT16;
          fpi_ssm_jump_to_state (self->task_ssm, MAPF_IDENTIFY_GET_TEMPLATE_INFO);
          break;
        }
      print = g_ptr_array_index (prints, self->enroll_identify_index);
      mafp_template_t tpl = mafp_template_from_print (print);
      self->search_id = tpl.id;
      para[0] = (tpl.id >> 8) & 0xff;
      para[1] = tpl.id & 0xff;
      mafp_sensor_cmd (self, MOC_CMD_MATCH_WITHFID, (const uint8_t *) &para, 2, fp_identify_search_step_cb);
      break;

    case MAPF_IDENTIFY_GET_TEMPLATE_INFO:
      if (self->search_id == G_MAXUINT16)
        {
          mafp_cmd_response_t resp;
          resp.result = 1;
          fp_identify_get_tpl_info_cb (self, &resp, NULL);
        }
      else
        {
          para[0] = (self->search_id >> 8) & 0xff;   /* fp id high */
          para[1] = self->search_id & 0xff;          /* fp id low */
          mafp_sensor_cmd (self, MOC_CMD_GET_TEMPLATE_INFO, (const uint8_t *) &para, 2, fp_identify_get_tpl_info_cb);
        }
      break;

    case MAPF_IDENTIFY_EXIT:
      mafp_pwr_btn_shield_on (self, 0);
      break;
    }
}

static void
fp_identify_ssm_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  fp_dbg ("identify completed");
  FpiDeviceMafpmoc *self = FPI_DEVICE_MAFPMOC (dev);
  g_autoptr(FpPrint) new_print = g_steal_pointer (&self->identify_new_print);
  FpPrint *match_print = g_steal_pointer (&self->identify_match_print);

  self->task_ssm = NULL;

  if (error && error->domain == FP_DEVICE_RETRY)
    {
      fpi_device_identify_report (dev, NULL, NULL, g_steal_pointer (&error));

      return;
    }

  if (error)
    {
      fpi_device_action_error (dev, g_steal_pointer (&error));
      return;
    }

  fpi_device_identify_report (dev, match_print,
                              self->enroll_dupl_del_state ?
                              g_steal_pointer (&new_print) : NULL,
                              NULL);
  fpi_device_identify_complete (dev, NULL);
}

static void
fp_list_tpl_table_cb (FpiDeviceMafpmoc    *self,
                      mafp_cmd_response_t *resp,
                      GError              *error)
{
  FpDevice *dev = FP_DEVICE (self);

  if (error)
    {
      fpi_device_list_complete (dev, NULL, g_steal_pointer (&error));
      return;
    }

  fp_dbg ("result: %d", resp->result);

  if (resp->result == MAFP_SUCCESS)
    {
      mafp_load_enrolled_ids (self, resp);

      if (self->templates->total_num == 0)
        {
          fpi_ssm_jump_to_state (self->task_ssm, MAPF_LIST_STATES);
          return;
        }
      fpi_ssm_next_state (self->task_ssm);
    }
  else
    {
      mafp_mark_failed (dev, self->task_ssm, FP_DEVICE_ERROR_GENERAL,
                        "Failed to get fingerprints index, result: 0x%x", resp->result);
    }
}

static void
fp_list_get_tpl_info_cb (FpiDeviceMafpmoc    *self,
                         mafp_cmd_response_t *resp,
                         GError              *error)
{
  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, g_steal_pointer (&error));
      return;
    }

  fp_dbg ("result: %d", resp->result);

  if (resp->result == MAFP_SUCCESS)
    {
      FpPrint *print;
      g_autofree char *uid = g_strndup (resp->tpl_info.uid, TEMPLATE_UID_SIZE);
      mafp_template_t *template = &self->templates->total_list[self->templates->index];

      fp_dbg ("tpl_info: %s", uid);

      if (resp->tpl_info.uid[0] == 'F' && resp->tpl_info.uid[1] == 'P')
        memcpy (template->uid, resp->tpl_info.uid, sizeof (resp->tpl_info.uid));
      else
        strncpy (template->uid, "NOT-A-FPRINT-PRINT", sizeof (resp->tpl_info.uid));

      print = mafp_print_from_template (self, template);
      g_ptr_array_add (self->templates->list, g_object_ref_sink (print));
    }
  if (++self->templates->index < self->templates->total_num)
    {
      fpi_ssm_jump_to_state (self->task_ssm, MAPF_LIST_GET_TEMPLATE_INFO);
      return;
    }
  fpi_ssm_next_state (self->task_ssm);
}

static void
fp_list_run_state (FpiSsm *ssm, FpDevice *device)
{
  FpiDeviceMafpmoc *self = FPI_DEVICE_MAFPMOC (device);
  uint8_t para[PACKAGE_DATA_SIZE_MAX] = { 0 };

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case MAPF_LIST_TEMPLATE_TABLE:
      para[0] = 0;  /* page no. */
      mafp_sensor_cmd (self, MOC_CMD_GET_TEMPLATE_TABLE, (const uint8_t *) &para, 1, fp_list_tpl_table_cb);
      break;

    case MAPF_LIST_GET_TEMPLATE_INFO:
      para[0] = (self->templates->total_list[self->templates->index].id >> 8) & 0xff; /* fp id high */
      para[1] = self->templates->total_list[self->templates->index].id & 0xff;        /* fp id low */
      mafp_sensor_cmd (self, MOC_CMD_GET_TEMPLATE_INFO, (const uint8_t *) &para, 2, fp_list_get_tpl_info_cb);
      break;
    }
}

static void
fp_list_ssm_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDeviceMafpmoc *self = FPI_DEVICE_MAFPMOC (dev);

  self->task_ssm = NULL;

  if (error)
    {
      fp_dbg ("list tpl fail");
      g_clear_pointer (&self->templates->list, g_ptr_array_unref);
      fpi_device_list_complete (dev, NULL, g_steal_pointer (&error));
      return;
    }

  fpi_device_list_complete (FP_DEVICE (self), g_steal_pointer (&self->templates->list), NULL);
}

static void
fp_delete_tpl_table_cb (FpiDeviceMafpmoc    *self,
                        mafp_cmd_response_t *resp,
                        GError              *error)
{
  FpDevice *dev = FP_DEVICE (self);
  FpPrint *print = NULL;
  gboolean id_exist = FALSE;

  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, g_steal_pointer (&error));
      return;
    }

  fp_dbg ("result: %d", resp->result);

  if (resp->result == MAFP_SUCCESS)
    {
      mafp_load_enrolled_ids (self, resp);
      fpi_device_get_delete_data (dev, &print);
      mafp_template_t tpl = mafp_template_from_print (print);

      for (int i = 0; i < self->templates->total_num; i++)
        {
          if (self->templates->total_list[i].id == tpl.id)
            {
              id_exist = true;
              break;
            }
        }
    }
  if (!id_exist)
    {
      fpi_ssm_jump_to_state (self->task_ssm, MAPF_DELETE_CLEAR_TEMPLATE_INFO);
      return;
    }
  fpi_ssm_next_state (self->task_ssm);
}

static void
fp_delete_get_tpl_info_cb (FpiDeviceMafpmoc    *self,
                           mafp_cmd_response_t *resp,
                           GError              *error)
{
  FpDevice *dev = FP_DEVICE (self);
  FpPrint *print = NULL;

  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, g_steal_pointer (&error));
      return;
    }

  fp_dbg ("result: %d", resp->result);

  if (resp->result == MAFP_SUCCESS)
    {
      g_autofree char *uid = g_strndup (resp->tpl_info.uid, TEMPLATE_UID_SIZE);

      fpi_device_get_delete_data (dev, &print);
      mafp_template_t tpl = mafp_template_from_print (print);
      fp_dbg ("target: %s/%s", tpl.uid, tpl.sn);
      fp_dbg ("find: %s/%s", uid, self->serial_number);
      if (g_strcmp0 (self->serial_number, tpl.sn) != 0)
        {
          mafp_mark_failed (dev, self->task_ssm, FP_DEVICE_ERROR_GENERAL,
                            "Failed to match device serial number");
          return;
        }
      if (!g_str_equal (uid, tpl.uid))
        {
          mafp_mark_failed (dev, self->task_ssm, FP_DEVICE_ERROR_GENERAL,
                            "Failed to match template uid");
          return;
        }
      fpi_ssm_next_state (self->task_ssm);
    }
  else
    {
      mafp_mark_failed (dev, self->task_ssm, FP_DEVICE_ERROR_GENERAL,
                        "Failed to get template info, result: 0x%x", resp->result);
    }
}

static void
fp_delete_clear_tpl_info_cb (FpiDeviceMafpmoc    *self,
                             mafp_cmd_response_t *resp,
                             GError              *error)
{
  FpDevice *dev = FP_DEVICE (self);

  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, g_steal_pointer (&error));
      return;
    }

  fp_dbg ("result: %d", resp->result);

  if (resp->result != MAFP_SUCCESS)
    {
      mafp_mark_failed (dev, self->task_ssm, FP_DEVICE_ERROR_GENERAL,
                        "Failed to delete template info, result: 0x%x", resp->result);
      return;
    }
  fpi_ssm_next_state (self->task_ssm);
}

static void
fp_delete_tpl_cb (FpiDeviceMafpmoc    *self,
                  mafp_cmd_response_t *resp,
                  GError              *error)
{
  FpDevice *dev = FP_DEVICE (self);

  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, g_steal_pointer (&error));
      return;
    }

  fp_dbg ("result: %d", resp->result);

  if (resp->result != MAFP_SUCCESS)
    {
      mafp_mark_failed (dev, self->task_ssm, FP_DEVICE_ERROR_GENERAL,
                        "Failed to delete template, result: 0x%x", resp->result);
      return;
    }
  fpi_ssm_next_state (self->task_ssm);
}

static void
fp_delete_run_state (FpiSsm *ssm, FpDevice *device)
{
  FpiDeviceMafpmoc *self = FPI_DEVICE_MAFPMOC (device);
  uint8_t para[PACKAGE_DATA_SIZE_MAX] = { 0 };
  FpPrint *print = NULL;

  fpi_device_get_delete_data (device, &print);
  mafp_template_t delete_tpl = mafp_template_from_print (print);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case MAPF_DELETE_TEMPLATE_TABLE:
      para[0] = 0;                             /* page no. */
      mafp_sensor_cmd (self, MOC_CMD_GET_TEMPLATE_TABLE, (const uint8_t *) &para, 1, fp_delete_tpl_table_cb);
      break;

    case MAPF_DELETE_GET_TEMPLATE_INFO:
      para[0] = (delete_tpl.id >> 8) & 0xff;   /* fp id high */
      para[1] = delete_tpl.id & 0xff;          /* fp id low */
      mafp_sensor_cmd (self, MOC_CMD_GET_TEMPLATE_INFO, (const uint8_t *) &para, 2, fp_delete_get_tpl_info_cb);
      break;

    case MAPF_DELETE_CLEAR_TEMPLATE_INFO:
      para[0] = (delete_tpl.id >> 8) & 0xff;   /* fp id high */
      para[1] = delete_tpl.id & 0xff;          /* fp id low */
      mafp_sensor_cmd (self, MOC_CMD_SAVE_TEMPLATE_INFO, (const uint8_t *) &para, 130, fp_delete_clear_tpl_info_cb);
      break;

    case MAPF_DELETE_TEMPLATE:
      para[0] = (delete_tpl.id >> 8) & 0xff;   /* tpl id high */
      para[1] = delete_tpl.id & 0xff;          /* tpl id low */
      para[2] = 0;                             /* range high */
      para[3] = 1;                             /* range low */
      mafp_sensor_cmd (self, MOC_CMD_DELETE_TEMPLATE, (const uint8_t *) &para, 4, fp_delete_tpl_cb);
      break;
    }
}

static void
fp_delete_ssm_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDeviceMafpmoc *self = FPI_DEVICE_MAFPMOC (dev);

  self->task_ssm = NULL;

  if (error)
    fp_dbg ("delete tpl fail: %s", error->message);
  else
    fp_dbg ("delete tpl success");

  fpi_device_delete_complete (dev, g_steal_pointer (&error));
}

static void
fp_delete_all_cb (FpiDeviceMafpmoc    *self,
                  mafp_cmd_response_t *resp,
                  GError              *error)
{
  FpDevice *dev = FP_DEVICE (self);

  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, g_steal_pointer (&error));
      return;
    }

  fp_dbg ("result: %d", resp->result);

  if (resp->result != MAFP_SUCCESS)
    {
      mafp_mark_failed (dev, self->task_ssm, FP_DEVICE_ERROR_GENERAL,
                        "Failed to empty templates, result: 0x%x", resp->result);
      return;
    }
  fpi_ssm_next_state (self->task_ssm);
}

static void
fp_delete_all_run_state (FpiSsm *ssm, FpDevice *device)
{
  FpiDeviceMafpmoc *self = FPI_DEVICE_MAFPMOC (device);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case MAPF_EMPTY_TEMPLATE:
      mafp_sensor_cmd (self, MOC_CMD_EMPTY, NULL, 0, fp_delete_all_cb);
      break;
    }
}

static void
fp_delete_all_ssm_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDeviceMafpmoc *self = FPI_DEVICE_MAFPMOC (dev);

  self->task_ssm = NULL;

  if (error)
    fp_dbg ("delete all fail: %s", error->message);
  else
    fp_dbg ("delete all success");

  fpi_device_clear_storage_complete (dev, g_steal_pointer (&error));
}

static void
mafp_probe (FpDevice *device)
{
  g_autoptr(GUsbInterface) interface = NULL;
  GUsbDevice *usb_dev;
  g_autoptr(GError) error = NULL;
  FpiDeviceMafpmoc *self = FPI_DEVICE_MAFPMOC (device);
  g_autofree char *serial = NULL;
  uint64_t driver_data;

  fp_dbg ("mafp_probe");

  usb_dev = fpi_device_get_usb_device (device);
  if (!g_usb_device_open (usb_dev, &error))
    {
      fpi_device_probe_complete (device, NULL, NULL, g_steal_pointer (&error));
      return;
    }

  driver_data = fpi_device_get_driver_data (device);
  fp_dbg ("driver_data 0x%zx", driver_data);
  fp_dbg ("g_usb_device_reset");
  if (!g_usb_device_reset (usb_dev, &error))
    goto err_close;

  fp_dbg ("g_usb_device_get_interface");
  interface = g_usb_device_get_interface (usb_dev, MAFP_INTERFACE_CLASS,
                                          MAFP_INTERFACE_SUB_CLASS, MAFP_INTERFACE_PROTOCOL, &error);
  if (!interface)
    {
      fp_dbg ("interface null");
      goto err_close;
    }
  self->interface_num = g_usb_interface_get_number (interface);
  fp_dbg ("interface number %d", self->interface_num);

  /* Claim usb interface */
  if (!g_usb_device_claim_interface (usb_dev, self->interface_num, 0, &error))
    goto err_close;

  if (fpi_device_emulation_mode_enabled (device))
    {
      serial = g_strdup ("emulated-device");
    }
  else
    {
      serial = g_usb_device_get_string_descriptor (usb_dev,
                                                   g_usb_device_get_serial_number_index (usb_dev),
                                                   &error);
      if (error || !serial)
        {
          g_usb_device_release_interface (fpi_device_get_usb_device (device), 0, 0, NULL);
          goto err_close;
        }
    }

  g_clear_pointer (&self->serial_number, g_free);
  self->serial_number = g_strndup (serial, DEVICE_SN_SIZE - 1);
  fp_dbg ("serial: %s", serial);

  g_usb_device_close (usb_dev, NULL);
  fpi_device_probe_complete (device, serial, NULL, NULL);
  return;

err_close:
  g_usb_device_close (usb_dev, NULL);
  fpi_device_probe_complete (device, NULL, NULL, g_steal_pointer (&error));
}

static void
mafp_init (FpDevice *device)
{
  fp_dbg ("mafp_init");
  FpiDeviceMafpmoc *self = FPI_DEVICE_MAFPMOC (device);
  g_autoptr(GError) error = NULL;
  g_autofree char *serial = NULL;
  uint64_t driver_data;

  driver_data = fpi_device_get_driver_data (device);
  fp_dbg ("driver_data 0x%zx", driver_data);
  fp_dbg ("g_usb_device_reset");
  if (!g_usb_device_reset (fpi_device_get_usb_device (device), &error))
    {
      fp_dbg ("g_usb_device_reset err: %s", error->message);
      fpi_device_open_complete (FP_DEVICE (self), g_steal_pointer (&error));
      return;
    }

  /* Claim usb interface */
  fp_dbg ("g_usb_device_claim_interface");
  if (!g_usb_device_claim_interface (fpi_device_get_usb_device (device), 0, 0, &error))
    {
      fpi_device_open_complete (FP_DEVICE (self), g_steal_pointer (&error));
      return;
    }

  if (fp_device_has_feature (device, FP_DEVICE_FEATURE_STORAGE))
    fp_dbg ("device has storage");
  else
    fp_dbg ("device no storage");

  if (fpi_device_emulation_mode_enabled (device))
    {
      serial = g_strdup ("emulated-device");
    }
  else
    {
      g_autoptr(GError) serial_error = NULL;

      serial = g_usb_device_get_string_descriptor (fpi_device_get_usb_device (device),
                                                   g_usb_device_get_serial_number_index (fpi_device_get_usb_device (device)),
                                                   &serial_error);
      if (serial_error)
        g_propagate_prefixed_error (&error,
                                    g_steal_pointer (&serial_error),
                                    "Failed to read device serial number: ");
    }

  if (!serial)
    {
      if (!error)
        error = fpi_device_error_new_msg (FP_DEVICE_ERROR_GENERAL,
                                          "Failed to read device serial number");
      g_usb_device_release_interface (fpi_device_get_usb_device (device), 0, 0, NULL);
      fpi_device_open_complete (FP_DEVICE (self), g_steal_pointer (&error));
      return;
    }

  g_clear_pointer (&self->serial_number, g_free);
  self->serial_number = g_strndup (serial, DEVICE_SN_SIZE - 1);

  self->templates = g_new0 (mafp_templates_t, 1);
  self->task_ssm = fpi_ssm_new (device, fp_init_run_state, MAPF_INIT_STATES);

  if (!PRINT_SSM_DEBUG)
    fpi_ssm_silence_debug (self->task_ssm);
  fpi_ssm_start (self->task_ssm, fp_init_ssm_done);
}

static void
mafp_enroll (FpDevice *device)
{
  FpiDeviceMafpmoc *self = FPI_DEVICE_MAFPMOC (device);

  self->enroll_stage = 0;
  self->finger_status = 0;
  self->press_state = MAFP_PRESS_WAIT_UP;
  self->capture_cnt = 0;
  self->enroll_identify_state = MAFP_ENROLL_IDENTIFY_ENABLED;
  self->enroll_dupl_del_state = MAFP_ENROLL_DUPLICATE_DELETE_ENABLED;
  self->enroll_dupl_area_state = MAFP_ENROLL_DUPLICATE_AREA_DENY;
  memset (self->templates, 0, sizeof (mafp_templates_t));

  self->task_ssm = fpi_ssm_new_full (device, fp_enroll_sm_run_state,
                                     MAFP_ENROLL_STATES,
                                     MAFP_ENROLL_EXIT,
                                     "enroll");

  if (!PRINT_SSM_DEBUG)
    fpi_ssm_silence_debug (self->task_ssm);
  fpi_ssm_start (self->task_ssm, fp_enroll_ssm_done);
}

static void
mafp_identify (FpDevice *device)
{
  FpiDeviceMafpmoc *self = FPI_DEVICE_MAFPMOC (device);

  memset (self->templates, 0, sizeof (mafp_templates_t));

  self->press_state = MAFP_PRESS_WAIT_UP;
  self->capture_cnt = 0;
  self->identify_match_print = NULL;
  g_clear_object (&self->identify_new_print);

  self->task_ssm = fpi_ssm_new_full (device, fp_identify_sm_run_state,
                                     MAPF_IDENTIFY_STATES,
                                     MAPF_IDENTIFY_EXIT,
                                     "identify");

  if (!PRINT_SSM_DEBUG)
    fpi_ssm_silence_debug (self->task_ssm);
  fpi_ssm_start (self->task_ssm, fp_identify_ssm_done);
}

static void
mafp_template_list (FpDevice *device)
{
  FpiDeviceMafpmoc *self = FPI_DEVICE_MAFPMOC (device);

  memset (self->templates, 0, sizeof (mafp_templates_t));
  self->templates->list = g_ptr_array_new_with_free_func (g_object_unref);

  self->task_ssm = fpi_ssm_new (device, fp_list_run_state, MAPF_LIST_STATES);
  if (!PRINT_SSM_DEBUG)
    fpi_ssm_silence_debug (self->task_ssm);
  fpi_ssm_start (self->task_ssm, fp_list_ssm_done);
}

static void
mafp_template_delete (FpDevice *device)
{
  FpiDeviceMafpmoc *self = FPI_DEVICE_MAFPMOC (device);

  self->task_ssm = fpi_ssm_new (device, fp_delete_run_state, MAPF_DELETE_STATES);
  if (!PRINT_SSM_DEBUG)
    fpi_ssm_silence_debug (self->task_ssm);
  fpi_ssm_start (self->task_ssm, fp_delete_ssm_done);
}

static void
mafp_template_delete_all (FpDevice *device)
{
  FpiDeviceMafpmoc *self = FPI_DEVICE_MAFPMOC (device);

  self->task_ssm = fpi_ssm_new (device, fp_delete_all_run_state, MAPF_EMPTY_STATES);
  if (!PRINT_SSM_DEBUG)
    fpi_ssm_silence_debug (self->task_ssm);
  fpi_ssm_start (self->task_ssm, fp_delete_all_ssm_done);
}

static void
mafp_cancel (FpDevice *device)
{
  fp_dbg ("mafp_cancel");
}

static gboolean
mafp_release_interface (FpiDeviceMafpmoc *self,
                        GError          **error)
{
  /* Release usb interface */
  return g_usb_device_release_interface (fpi_device_get_usb_device (FP_DEVICE (self)),
                                         0, 0, error);
}

static void
mafp_exit (FpDevice *device)
{
  g_autoptr(GError) error = NULL;

  fp_dbg ("mafp_exit");
  FpiDeviceMafpmoc *self = FPI_DEVICE_MAFPMOC (device);

  mafp_release_interface (self, &error);
  fpi_device_close_complete (FP_DEVICE (self), g_steal_pointer (&error));
  g_clear_pointer (&self->serial_number, g_free);
  g_clear_pointer (&self->enroll_user_id, g_free);
  if (self->templates)
    g_clear_pointer (&self->templates->list, g_ptr_array_unref);
  g_clear_pointer (&self->templates, g_free);
}

static void
fpi_device_mafpmoc_init (FpiDeviceMafpmoc *self)
{
  fp_dbg ("fpi_device_mafpmoc_init");
}

static const FpIdEntry id_table[] = {
  { .vid = 0x3274,  .pid = 0x8012,  },
  { .vid = 0,  .pid = 0,  .driver_data = 0 },   /* terminating entry */
};

static void
fpi_device_mafpmoc_class_init (FpiDeviceMafpmocClass *klass)
{
  FpDeviceClass *dev_class = FP_DEVICE_CLASS (klass);
  const char *env_enroll_samples;

  dev_class->id = "mafpmoc";
  dev_class->full_name = "MAFP MOC Fingerprint Sensor";
  dev_class->type = FP_DEVICE_TYPE_USB;
  dev_class->scan_type = FP_SCAN_TYPE_PRESS;
  dev_class->id_table = id_table;
  dev_class->nr_enroll_stages = DEFAULT_ENROLL_SAMPLES;
  dev_class->temp_hot_seconds = -1;

  dev_class->open   = mafp_init;
  dev_class->close  = mafp_exit;
  dev_class->probe  = mafp_probe;
  dev_class->enroll = mafp_enroll;
  dev_class->cancel = mafp_cancel;
  dev_class->identify = mafp_identify;
  dev_class->delete = mafp_template_delete;
  dev_class->clear_storage = mafp_template_delete_all;
  dev_class->list = mafp_template_list;

  env_enroll_samples = getenv (MAFP_ENV_ENROLL_SAMPLES);
  if (env_enroll_samples)
    {
      guint64 max_enroll_stage = g_ascii_strtoll (env_enroll_samples, NULL, 10);

      if (max_enroll_stage > 0 && max_enroll_stage <= 30)
        dev_class->nr_enroll_stages = max_enroll_stage;
    }

  fpi_device_class_auto_initialize_features (dev_class);
}
