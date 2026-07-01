/*
 * Driver for Egis Technology (LighTuning) Match-On-Chip sensors
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#define FP_COMPONENT "egis_etu905"

#include <stdio.h>
#include <glib.h>
#include <sys/param.h>

#include "drivers_api.h"
#include "fpi-byte-writer.h"

#include "egis_etu905.h"

struct _FpiDeviceEgisEtu905
{
  FpDevice        parent;
  FpiSsm         *task_ssm;
  FpiSsm         *cmd_ssm;
  FpiUsbTransfer *cmd_transfer;
  GPtrArray      *enrolled_ids;
  gint            max_enroll_stages;
};

G_DEFINE_TYPE (FpiDeviceEgisEtu905, fpi_device_egis_etu905, FP_TYPE_DEVICE);

static const FpIdEntry egis_etu905_id_table[] = {
  { .vid = 0x1c7a, .pid = 0x05ae, .driver_data = EGIS_ETU905_DRIVER_CHECK_PREFIX_TYPE1 },
  { .vid = 0x1c7a, .pid = 0x9201, .driver_data = EGIS_ETU905_DRIVER_CHECK_PREFIX_TYPE1 },
  { .vid = 0,      .pid = 0,      .driver_data = 0 }
};

typedef void (*SynCmdMsgCallback) (FpDevice *device,
                                   guchar   *buffer_in,
                                   gsize     length_in,
                                   GError   *error);

typedef struct egis_etu905_command_data
{
  SynCmdMsgCallback callback;
  guchar           *buffer_in;
  gsize             length_in;
} CommandData;

static void
egis_etu905_command_data_free (CommandData *data)
{
  g_free (data->buffer_in);
  g_free (data);
}

typedef struct egis_etu905_enroll_print
{
  FpPrint *print;
  int      stage;
} EnrollPrint;

static void
egis_etu905_finger_on_sensor_cb (FpiUsbTransfer *transfer,
                                 FpDevice       *device,
                                 gpointer        userdata,
                                 GError         *error)
{
  fp_dbg ("Finger on sensor callback");
  fpi_device_report_finger_status (device, FP_FINGER_STATUS_PRESENT);

  fpi_ssm_usb_transfer_cb (transfer, device, userdata, error);
}

static void
egis_etu905_wait_finger_on_sensor (FpiSsm   *ssm,
                                   FpDevice *device)
{
  fp_dbg ("Wait for finger on sensor");
  g_autoptr(FpiUsbTransfer) transfer = fpi_usb_transfer_new (device);

  fpi_usb_transfer_fill_interrupt (transfer, EGIS_ETU905_EP_CMD_INTERRUPT_IN,
                                   EGIS_ETU905_USB_INTERRUPT_IN_RECV_LENGTH);
  transfer->ssm = ssm;
  /* Interrupt on this device always returns 1 byte short; this is expected */
  fpi_usb_transfer_set_short_error (transfer, FALSE);

  fpi_device_report_finger_status (device, FP_FINGER_STATUS_NEEDED);

  fpi_usb_transfer_submit (g_steal_pointer (&transfer),
                           EGIS_ETU905_USB_INTERRUPT_TIMEOUT,
                           fpi_device_get_cancellable (device),
                           egis_etu905_finger_on_sensor_cb,
                           NULL);
}

static gboolean
egis_etu905_validate_response_prefix (const guchar *buffer_in,
                                      const gsize   buffer_in_len,
                                      const guchar *valid_prefix,
                                      const gsize   valid_prefix_len)
{
  FpiByteReader reader;
  const guint8 *data = NULL;
  gboolean result;

  fpi_byte_reader_init (&reader, buffer_in, buffer_in_len);

  if (!fpi_byte_reader_set_pos (&reader, egis_etu905_read_prefix_len +
                                EGIS_ETU905_CHECK_BYTES_LENGTH) ||
      !fpi_byte_reader_get_data (&reader, valid_prefix_len, &data))
    {
      fp_dbg ("Response too short for prefix validation");
      return FALSE;
    }

  result = memcmp (data, valid_prefix, valid_prefix_len) == 0;

  fp_dbg ("Response prefix valid: %s", result ? "yes" : "NO");
  return result;
}

static gboolean
egis_etu905_validate_response_suffix (const guchar *buffer_in,
                                      const gsize   buffer_in_len,
                                      const guchar *valid_suffix,
                                      const gsize   valid_suffix_len)
{
  FpiByteReader reader;
  const guint8 *data = NULL;
  gboolean result;

  fpi_byte_reader_init (&reader, buffer_in, buffer_in_len);

  /* Guard against unsigned underflow before computing the suffix position. */
  if (valid_suffix_len > buffer_in_len ||
      !fpi_byte_reader_set_pos (&reader, buffer_in_len - valid_suffix_len) ||
      !fpi_byte_reader_get_data (&reader, valid_suffix_len, &data))
    {
      fp_dbg ("Response too short for suffix validation");
      return FALSE;
    }

  result = memcmp (data, valid_suffix, valid_suffix_len) == 0;

  fp_dbg ("Response suffix valid: %s", result ? "yes" : "NO");
  return result;
}

static void
egis_etu905_task_ssm_done (FpiSsm   *ssm,
                           FpDevice *device,
                           GError   *error)
{
  FpiDeviceEgisEtu905 *self = FPI_DEVICE_EGIS_ETU905 (device);

  fp_dbg ("Task SSM done");

  /* task_ssm is going to be freed by completion of SSM */
  g_assert (!self->task_ssm || self->task_ssm == ssm);
  self->task_ssm = NULL;

  g_clear_pointer (&self->enrolled_ids, g_ptr_array_unref);

  if (error)
    fpi_device_action_error (device, error);
}

static void
egis_etu905_commit_start_cb (FpDevice *device,
                             guchar   *buffer_in,
                             gsize     length_in,
                             GError   *error)
{
  fp_dbg ("Task SSM commit start callback");
  FpiDeviceEgisEtu905 *self = FPI_DEVICE_EGIS_ETU905 (device);

  if (error)
    fpi_ssm_mark_failed (self->task_ssm, error);
  else
    fpi_ssm_jump_to_state (self->task_ssm, ENROLL_COMMIT);
}

/*
 * Generic callback for commands that don't need special handling on success.
 * Advances the task SSM to the next state on success, or marks it failed on error.
 */
static void
egis_etu905_task_ssm_next_state_cb (FpDevice *device,
                                    guchar   *buffer_in,
                                    gsize     length_in,
                                    GError   *error)
{
  FpiDeviceEgisEtu905 *self = FPI_DEVICE_EGIS_ETU905 (device);

  fp_dbg ("Task SSM next state callback");

  if (error)
    fpi_ssm_mark_failed (self->task_ssm, error);
  else
    fpi_ssm_next_state (self->task_ssm);
}

static void
egis_etu905_cmd_receive_cb (FpiUsbTransfer *transfer,
                            FpDevice       *device,
                            gpointer        userdata,
                            GError         *error)
{
  CommandData *data = userdata;

  fp_dbg ("Command receive callback");

  if (error)
    {
      fpi_ssm_mark_failed (transfer->ssm, error);
      return;
    }

  if (transfer->actual_length < egis_etu905_read_prefix_len)
    {
      fpi_ssm_mark_failed (transfer->ssm,
                           fpi_device_error_new (FP_DEVICE_ERROR_GENERAL));
      return;
    }

  /* Store the response data and let the cmd_ssm_done callback invoke
   * the actual callback with the stored data */
  g_assert (data != NULL);
  data->buffer_in = g_steal_pointer (&transfer->buffer);
  data->length_in = transfer->actual_length;

  fpi_ssm_mark_completed (transfer->ssm);
}

static void
egis_etu905_cmd_run_state (FpiSsm   *ssm,
                           FpDevice *device)
{
  g_autoptr(FpiUsbTransfer) transfer = NULL;
  FpiDeviceEgisEtu905 *self = FPI_DEVICE_EGIS_ETU905 (device);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case CMD_SEND:
      if (self->cmd_transfer)
        {
          self->cmd_transfer->ssm = ssm;
          fpi_usb_transfer_submit (g_steal_pointer (&self->cmd_transfer),
                                   EGIS_ETU905_USB_SEND_TIMEOUT,
                                   fpi_device_get_cancellable (device),
                                   fpi_ssm_usb_transfer_cb,
                                   NULL);
          break;
        }

      fpi_ssm_next_state (ssm);
      break;

    case CMD_GET:
      transfer = fpi_usb_transfer_new (device);
      transfer->ssm = ssm;
      fpi_usb_transfer_fill_bulk (transfer, EGIS_ETU905_EP_CMD_IN,
                                  EGIS_ETU905_USB_IN_RECV_LENGTH);
      fpi_usb_transfer_submit (g_steal_pointer (&transfer),
                               EGIS_ETU905_USB_RECV_TIMEOUT,
                               fpi_device_get_cancellable (device),
                               egis_etu905_cmd_receive_cb,
                               fpi_ssm_get_data (ssm));
      break;
    }
}

/*
 * Command SSM done callback.
 * Always invokes the callback with stored data on success or error.
 */
static void
egis_etu905_cmd_ssm_done (FpiSsm   *ssm,
                          FpDevice *device,
                          GError   *error)
{
  g_autoptr(GError) local_error = error;
  FpiDeviceEgisEtu905 *self = FPI_DEVICE_EGIS_ETU905 (device);
  CommandData *data = fpi_ssm_get_data (ssm);

  g_assert (self->cmd_ssm == ssm);
  g_assert (!self->cmd_transfer || self->cmd_transfer->ssm == ssm);

  self->cmd_ssm = NULL;
  self->cmd_transfer = NULL;

  if (data && data->callback)
    {
      data->callback (device,
                      data->buffer_in,
                      data->length_in,
                      g_steal_pointer (&local_error));
    }
}

/*
 * Derive the 2 "check bytes" for write payloads
 * 32-bit big-endian sum of all 16-bit words (including check bytes) MOD 0xFFFF
 * should be 0, otherwise the device will reject the payload
 */
static guint16
egis_etu905_get_check_bytes (FpiByteReader *reader)
{
  fp_dbg ("Get check bytes");
  guint64 sum_values = 0;
  guint16 val;

  fpi_byte_reader_set_pos (reader, 0);

  while (fpi_byte_reader_get_uint16_be (reader, &val))
    sum_values += val;

  return G_MAXUINT16 - (sum_values % G_MAXUINT16);
}

static void
egis_etu905_exec_cmd (FpDevice         *device,
                      guchar           *cmd,
                      const gsize       cmd_length,
                      GDestroyNotify    cmd_destroy,
                      SynCmdMsgCallback callback)
{
  g_auto(FpiByteWriter) writer = {0};
  g_autoptr(FpiUsbTransfer) transfer = NULL;
  FpiDeviceEgisEtu905 *self = FPI_DEVICE_EGIS_ETU905 (device);
  g_autofree CommandData *data = NULL;
  gsize buffer_out_length = 0;
  gboolean written = TRUE;
  guint16 check_value;

  fp_dbg ("Execute command and get response");

  /*
   * buffer_out should be a fully composed command (with prefix, check bytes, etc)
   * which looks like this:
   *   E G I S 00 00 00 01 {cb1} {cb2} {payload}
   * where cb1 and cb2 are some check bytes generated by the
   * egis_etu905_get_check_bytes() method and payload is what is passed via the cmd
   * parameter
   */
  buffer_out_length = egis_etu905_write_prefix_len
                      + EGIS_ETU905_CHECK_BYTES_LENGTH
                      + cmd_length;

  fpi_byte_writer_init_with_size (&writer, buffer_out_length +
                                  (buffer_out_length % 2 ? 1 : 0), TRUE);

  /* Prefix */
  written &= fpi_byte_writer_put_data (&writer, egis_etu905_write_prefix,
                                       egis_etu905_write_prefix_len);

  /* Check Bytes - leave them as 00 for now then later generate and copy over
   * the real ones */
  written &= fpi_byte_writer_change_pos (&writer, EGIS_ETU905_CHECK_BYTES_LENGTH);

  /* Command Payload */
  written &= fpi_byte_writer_put_data (&writer, cmd, cmd_length);

  /* Now fetch and set the "real" check bytes based on the currently
   * assembled payload */
  check_value = egis_etu905_get_check_bytes (FPI_BYTE_READER (&writer));
  fpi_byte_writer_set_pos (&writer, egis_etu905_write_prefix_len);
  written &= fpi_byte_writer_put_uint16_be (&writer, check_value);

  /* destroy cmd if requested */
  if (cmd_destroy)
    g_clear_pointer (&cmd, cmd_destroy);

  g_assert (self->cmd_ssm == NULL);
  self->cmd_ssm = fpi_ssm_new (device,
                               egis_etu905_cmd_run_state,
                               CMD_STATES);

  data = g_new0 (CommandData, 1);
  data->callback = callback;
  fpi_ssm_set_data (self->cmd_ssm, g_steal_pointer (&data),
                    (GDestroyNotify) egis_etu905_command_data_free);

  if (!written)
    {
      fpi_ssm_start (self->cmd_ssm, egis_etu905_cmd_ssm_done);
      fpi_ssm_mark_failed (self->cmd_ssm,
                           fpi_device_error_new (FP_DEVICE_ERROR_PROTO));
      return;
    }

  transfer = fpi_usb_transfer_new (device);
  fpi_usb_transfer_set_short_error (transfer, TRUE);
  transfer->ssm = self->cmd_ssm;

  fpi_usb_transfer_fill_bulk_full (transfer,
                                   EGIS_ETU905_EP_CMD_OUT,
                                   fpi_byte_writer_reset_and_get_data (&writer),
                                   buffer_out_length,
                                   g_free);

  g_assert (self->cmd_transfer == NULL);
  self->cmd_transfer = g_steal_pointer (&transfer);
  fpi_ssm_start (self->cmd_ssm, egis_etu905_cmd_ssm_done);
}

static void
egis_etu905_set_print_data (FpPrint     *print,
                            const gchar *device_print_id,
                            const gchar *user_id)
{
  GVariant *print_id_var = NULL;
  GVariant *fpi_data = NULL;
  g_autofree gchar *fill_user_id = NULL;

  if (user_id)
    fill_user_id = g_strdup (user_id);
  else
    fill_user_id = g_strndup (device_print_id, EGIS_ETU905_FINGERPRINT_DATA_SIZE);

  fpi_print_fill_from_user_id (print, fill_user_id);

  fpi_print_set_type (print, FPI_PRINT_RAW);
  fpi_print_set_device_stored (print, TRUE);

  g_object_set (print, "description", fill_user_id, NULL);

  print_id_var = g_variant_new_fixed_array (G_VARIANT_TYPE_BYTE,
                                            device_print_id,
                                            EGIS_ETU905_FINGERPRINT_DATA_SIZE,
                                            sizeof (guchar));
  fpi_data = g_variant_new ("(@ay)", print_id_var);
  g_object_set (print, "fpi-data", fpi_data, NULL);
}

static GPtrArray *
egis_etu905_get_enrolled_prints (FpDevice *device)
{
  g_autoptr(GPtrArray) result = g_ptr_array_new_with_free_func (g_object_unref);
  FpiDeviceEgisEtu905 *self = FPI_DEVICE_EGIS_ETU905 (device);

  if (!self->enrolled_ids)
    return g_steal_pointer (&result);

  for (guint i = 0; i < self->enrolled_ids->len; i++)
    {
      FpPrint *print = fp_print_new (device);
      egis_etu905_set_print_data (print, g_ptr_array_index (self->enrolled_ids, i), NULL);
      g_ptr_array_add (result, g_object_ref_sink (print));
    }

  return g_steal_pointer (&result);
}

static void
egis_etu905_list_fill_enrolled_ids_cb (FpDevice *device,
                                       guchar   *buffer_in,
                                       gsize     length_in,
                                       GError   *error)
{
  FpiDeviceEgisEtu905 *self = FPI_DEVICE_EGIS_ETU905 (device);

  fp_dbg ("List callback");

  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, error);
      return;
    }

  g_clear_pointer (&self->enrolled_ids, g_ptr_array_unref);
  self->enrolled_ids = g_ptr_array_new_with_free_func (g_free);

  FpiByteReader reader;
  gboolean read = TRUE;

  fpi_byte_reader_init (&reader, buffer_in, length_in);

  read &= fpi_byte_reader_set_pos (&reader, EGIS_ETU905_LIST_RESPONSE_PREFIX_SIZE);

  /*
   * Each fingerprint ID will be returned in this response as a 32 byte array
   * The other stuff in the payload is 16 bytes long, so if there is at least 1
   * print then the length should be at least 16+32=48 bytes long
   */
  while (read)
    {
      const guint8 data[EGIS_ETU905_FINGERPRINT_DATA_SIZE];
      g_autofree gchar *print_id = NULL;

      read &= fpi_byte_reader_get_data_static (&reader, data);
      if (!read)
        break;

      print_id = g_memdup2 (data, EGIS_ETU905_FINGERPRINT_DATA_SIZE);
      g_ptr_array_add (self->enrolled_ids, g_steal_pointer (&print_id));
    }

  fp_info ("Number of currently enrolled fingerprints on the device is %d",
           self->enrolled_ids->len);

  if (self->task_ssm)
    fpi_ssm_next_state (self->task_ssm);
}

static void
egis_etu905_list_run_state (FpiSsm   *ssm,
                            FpDevice *device)
{
  g_autoptr(GPtrArray) enrolled_prints = NULL;

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case LIST_GET_ENROLLED_IDS:
      egis_etu905_exec_cmd (device, cmd_list, cmd_list_len, NULL,
                            egis_etu905_list_fill_enrolled_ids_cb);
      break;

    case LIST_RETURN_ENROLLED_PRINTS:
      enrolled_prints = egis_etu905_get_enrolled_prints (device);
      fpi_device_list_complete (device, g_steal_pointer (&enrolled_prints), NULL);
      fpi_ssm_next_state (ssm);
      break;
    }
}

static void
egis_etu905_list (FpDevice *device)
{
  fp_dbg ("List");
  FpiDeviceEgisEtu905 *self = FPI_DEVICE_EGIS_ETU905 (device);

  g_assert (self->task_ssm == NULL);
  self->task_ssm = fpi_ssm_new (device,
                                egis_etu905_list_run_state,
                                LIST_STATES);
  fpi_ssm_start (self->task_ssm, egis_etu905_task_ssm_done);
}

static guchar *
egis_etu905_get_delete_cmd (FpDevice *device,
                            FpPrint  *delete_print,
                            gsize    *length_out)
{
  fp_dbg ("Get delete command");
  FpiDeviceEgisEtu905 *self = FPI_DEVICE_EGIS_ETU905 (device);
  g_auto(FpiByteWriter) writer = {0};
  g_autoptr(GVariant) print_data = NULL;
  g_autoptr(GVariant) print_data_id_var = NULL;
  const guchar *print_data_id = NULL;
  gsize print_data_id_len = 0;
  g_autofree gchar *print_description = NULL;
  gboolean written = TRUE;

  /*
   * The final command body should contain:
   * 1) hard-coded 00 00
   * 2) 2-byte size indiciator, 20*Number deleted identifiers plus 7 in form of:
   *    num_to_delete * 0x20 + 0x07
   *    Since max prints can be higher than 7 then this goes up to 2 bytes
   *    (e9 + 9 = 109)
   * 3) Hard-coded prefix (cmd_delete_prefix)
   * 4) 2-byte size indiciator, 20*Number of enrolled identifiers without plus 7
   *    (num_to_delete * 0x20)
   * 5) All of the currently registered prints to delete in their 32-byte device
   *    identifiers (enrolled_list)
   */

  int num_to_delete = 0;
  if (delete_print)
    num_to_delete = 1;
  else if (self->enrolled_ids)
    num_to_delete = self->enrolled_ids->len;

  const gsize body_length = sizeof (guchar) * EGIS_ETU905_FINGERPRINT_DATA_SIZE *
                            num_to_delete;
  /* total_length is the 6 various bytes plus prefix and body payload */
  const gsize total_length = (sizeof (guchar) * 6) + cmd_delete_prefix_len +
                             body_length;

  /* pre-fill entire payload with 00s */
  fpi_byte_writer_init_with_size (&writer, total_length, TRUE);

  /* start with 00 00 (just move starting offset up by 2) */
  written &= fpi_byte_writer_set_pos (&writer, 2);

  /* Size Counter bytes */
  /* "easiest" way to handle 2-bytes size for counter is to hard-code logic for
   * when we go to the 2nd byte
   * note this will not work in case any model ever supports more than 14 prints
   * (assumed max is 10) */
  if (num_to_delete > 7)
    {
      written &= fpi_byte_writer_put_uint8 (&writer, 0x01);
      written &= fpi_byte_writer_put_uint8 (&writer, ((num_to_delete - 8) * 0x20) + 0x07);
    }
  else
    {
      /* first byte is 0x00, just skip it */
      written &= fpi_byte_writer_change_pos (&writer, 1);
      written &= fpi_byte_writer_put_uint8 (&writer, (num_to_delete * 0x20) + 0x07);
    }

  /* command prefix */
  written &= fpi_byte_writer_put_data (&writer, cmd_delete_prefix,
                                       cmd_delete_prefix_len);

  /* 2-bytes size logic for counter again */
  if (num_to_delete > 7)
    {
      written &= fpi_byte_writer_put_uint8 (&writer, 0x01);
      written &= fpi_byte_writer_put_uint8 (&writer, (num_to_delete - 8) * 0x20);
    }
  else
    {
      /* first byte is 0x00, just skip it */
      written &= fpi_byte_writer_change_pos (&writer, 1);
      written &= fpi_byte_writer_put_uint8 (&writer, num_to_delete * 0x20);
    }

  /* append desired 32-byte fingerprint IDs */
  /* if passed a delete_print then fetch its data from the FpPrint */
  if (delete_print)
    {
      g_object_get (delete_print, "description", &print_description, NULL);
      g_object_get (delete_print, "fpi-data", &print_data, NULL);

      if (!print_data ||
          !g_variant_check_format_string (print_data, "(@ay)", FALSE))
        {
          fpi_ssm_mark_failed (self->task_ssm,
                               fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID));
          return NULL;
        }

      g_variant_get (print_data, "(@ay)", &print_data_id_var);
      print_data_id = g_variant_get_fixed_array (print_data_id_var,
                                                 &print_data_id_len, sizeof (guchar));

      if (print_data_id_len != EGIS_ETU905_FINGERPRINT_DATA_SIZE)
        {
          fpi_ssm_mark_failed (self->task_ssm,
                               fpi_device_error_new_msg (FP_DEVICE_ERROR_DATA_INVALID,
                                                         "Stored print id has "
                                                         "unexpected length %zu",
                                                         print_data_id_len));
          return NULL;
        }

      if (!print_description || !g_str_has_prefix (print_description, "FP"))
        fp_dbg ("Fingerprint '%s' was not created by libfprint; deleting anyway.",
                print_description ? print_description : "(null)");

      fp_info ("Delete fingerprint %s", print_description ? print_description : "(null)");

      written &= fpi_byte_writer_put_data (&writer, print_data_id,
                                           EGIS_ETU905_FINGERPRINT_DATA_SIZE);
    }
  /* Otherwise assume this is a "clear" - just loop through and append all enrolled IDs */
  else if (self->enrolled_ids)
    {
      for (guint i = 0; i < self->enrolled_ids->len && written; i++)
        {
          written &= fpi_byte_writer_put_data (&writer,
                                               g_ptr_array_index (self->enrolled_ids, i),
                                               EGIS_ETU905_FINGERPRINT_DATA_SIZE);
        }
    }

  g_assert (written);

  if (length_out)
    *length_out = total_length;

  return fpi_byte_writer_reset_and_get_data (&writer);
}

static void
egis_etu905_delete_cb (FpDevice *device,
                       guchar   *buffer_in,
                       gsize     length_in,
                       GError   *error)
{
  fp_dbg ("Delete callback");
  FpiDeviceEgisEtu905 *self = FPI_DEVICE_EGIS_ETU905 (device);

  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, error);
      return;
    }

  /* Check that the read payload indicates "success" with the delete */
  if (egis_etu905_validate_response_prefix (buffer_in,
                                            length_in,
                                            rsp_delete_success_prefix,
                                            rsp_delete_success_prefix_len))
    {
      if (fpi_device_get_current_action (device) == FPI_DEVICE_ACTION_CLEAR_STORAGE)
        {
          fpi_device_clear_storage_complete (device, NULL);
          fpi_ssm_next_state (self->task_ssm);
        }
      else if (fpi_device_get_current_action (device) == FPI_DEVICE_ACTION_DELETE)
        {
          fpi_device_delete_complete (device, NULL);
          fpi_ssm_next_state (self->task_ssm);
        }
      else
        {
          fpi_ssm_mark_failed (self->task_ssm,
                               fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                         "Unsupported delete action."));
        }
    }
  else
    {
      fpi_ssm_mark_failed (self->task_ssm,
                           fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                     "Delete print was not successful"));
    }
}

static void
egis_etu905_delete_run_state (FpiSsm   *ssm,
                              FpDevice *device)
{
  g_autofree guchar *payload = NULL;
  gsize payload_length = 0;

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case DELETE_GET_ENROLLED_IDS:
      /* get enrolled_ids from device for use building delete payload below */
      egis_etu905_exec_cmd (device, cmd_list, cmd_list_len, NULL,
                            egis_etu905_list_fill_enrolled_ids_cb);
      break;

    case DELETE_DELETE:
      if (fpi_device_get_current_action (device) == FPI_DEVICE_ACTION_DELETE)
        payload = egis_etu905_get_delete_cmd (device, fpi_ssm_get_data (ssm),
                                              &payload_length);
      else
        payload = egis_etu905_get_delete_cmd (device, NULL, &payload_length);

      /* get_delete_cmd already marked task_ssm as failed */
      if (!payload)
        return;

      egis_etu905_exec_cmd (device, g_steal_pointer (&payload), payload_length,
                            g_free, egis_etu905_delete_cb);
      break;
    }
}

static void
egis_etu905_clear_storage (FpDevice *device)
{
  fp_dbg ("Clear storage");
  FpiDeviceEgisEtu905 *self = FPI_DEVICE_EGIS_ETU905 (device);

  g_assert (self->task_ssm == NULL);
  self->task_ssm = fpi_ssm_new (device,
                                egis_etu905_delete_run_state,
                                DELETE_STATES);
  fpi_ssm_start (self->task_ssm, egis_etu905_task_ssm_done);
}

static void
egis_etu905_delete (FpDevice *device)
{
  fp_dbg ("Delete");
  FpiDeviceEgisEtu905 *self = FPI_DEVICE_EGIS_ETU905 (device);
  FpPrint *delete_print = NULL;

  fpi_device_get_delete_data (device, &delete_print);

  g_assert (self->task_ssm == NULL);
  self->task_ssm = fpi_ssm_new (device,
                                egis_etu905_delete_run_state,
                                DELETE_STATES);
  /* the print is owned by libfprint during deletion task */
  fpi_ssm_set_data (self->task_ssm, delete_print, NULL);
  fpi_ssm_start (self->task_ssm, egis_etu905_task_ssm_done);
}

static void
egis_etu905_enroll_status_report (FpDevice    *device,
                                  EnrollPrint *enroll_print,
                                  EnrollStatus status,
                                  GError      *error)
{
  FpiDeviceEgisEtu905 *self = FPI_DEVICE_EGIS_ETU905 (device);

  switch (status)
    {
    case ENROLL_STATUS_DEVICE_FULL:
    case ENROLL_STATUS_DUPLICATE:
      fpi_ssm_mark_failed (self->task_ssm, error);
      break;

    case ENROLL_STATUS_RETRY:
      fpi_device_enroll_progress (device, enroll_print->stage, NULL, error);
      break;

    case ENROLL_STATUS_PARTIAL_OK:
      enroll_print->stage++;
      fp_info ("Partial capture successful. Please touch the sensor again (%d/%d)",
               enroll_print->stage,
               self->max_enroll_stages);
      fpi_device_enroll_progress (device, enroll_print->stage, enroll_print->print, NULL);
      break;

    case ENROLL_STATUS_COMPLETE:
      fp_info ("Enrollment was successful!");
      fpi_device_enroll_complete (device, g_object_ref (enroll_print->print), NULL);
      break;

    default:
      if (error)
        fpi_ssm_mark_failed (self->task_ssm, error);
      else
        fpi_ssm_mark_failed (self->task_ssm,
                             fpi_device_error_new_msg (FP_DEVICE_ERROR_GENERAL,
                                                       "Unknown error"));
    }
}

static void
egis_etu905_read_capture_cb (FpDevice *device,
                             guchar   *buffer_in,
                             gsize     length_in,
                             GError   *error)
{
  fp_dbg ("Read capture callback");
  FpiDeviceEgisEtu905 *self = FPI_DEVICE_EGIS_ETU905 (device);
  EnrollPrint *enroll_print = fpi_ssm_get_data (self->task_ssm);

  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, error);
      return;
    }

  /* Check that the read payload indicates "success" */
  if (egis_etu905_validate_response_prefix (buffer_in,
                                            length_in,
                                            rsp_read_success_prefix,
                                            rsp_read_success_prefix_len) &&
      egis_etu905_validate_response_suffix (buffer_in,
                                            length_in,
                                            rsp_read_success_suffix,
                                            rsp_read_success_suffix_len))
    {
      egis_etu905_enroll_status_report (device, enroll_print,
                                        ENROLL_STATUS_PARTIAL_OK, NULL);
    }
  else
    {
      /* If not success then the sensor can either report "off center" or "sensor is dirty" */

      /* "Off center" */
      if (egis_etu905_validate_response_prefix (buffer_in,
                                                length_in,
                                                rsp_read_offcenter_prefix,
                                                rsp_read_offcenter_prefix_len) &&
          egis_etu905_validate_response_suffix (buffer_in,
                                                length_in,
                                                rsp_read_offcenter_suffix,
                                                rsp_read_offcenter_suffix_len))
        error = fpi_device_retry_new (FP_DEVICE_RETRY_CENTER_FINGER);

      /* "Sensor is dirty" */
      else if (egis_etu905_validate_response_prefix (buffer_in,
                                                     length_in,
                                                     rsp_read_dirty_prefix,
                                                     rsp_read_dirty_prefix_len))
        error = fpi_device_retry_new_msg (FP_DEVICE_RETRY_REMOVE_FINGER,
                                          "Your device is having trouble recognizing you. "
                                          "Make sure your sensor is clean.");

      else
        error = fpi_device_retry_new_msg (FP_DEVICE_RETRY_REMOVE_FINGER,
                                          "Unknown failure trying to read your finger. "
                                          "Please try again.");

      egis_etu905_enroll_status_report (device, enroll_print, ENROLL_STATUS_RETRY, error);
    }

  if (enroll_print->stage == self->max_enroll_stages)
    fpi_ssm_next_state (self->task_ssm);
  else
    fpi_ssm_jump_to_state (self->task_ssm, ENROLL_CAPTURE_SENSOR_RESET);
}

static void
egis_etu905_enroll_duplicate_check_cb (FpDevice *device,
                                       guchar   *buffer_in,
                                       gsize     length_in,
                                       GError   *error)
{
  fp_dbg ("Task SSM enroll duplicate check callback");
  FpiDeviceEgisEtu905 *self = FPI_DEVICE_EGIS_ETU905 (device);

  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, error);
      return;
    }

  /* Check that the read payload reports "not yet enrolled" */
  if (egis_etu905_validate_response_suffix (buffer_in,
                                            length_in,
                                            rsp_check_not_yet_enrolled_suffix,
                                            rsp_check_not_yet_enrolled_suffix_len))
    fpi_ssm_jump_to_state (self->task_ssm, ENROLL_COMMIT_START);
  else
    egis_etu905_enroll_status_report (device, NULL, ENROLL_STATUS_DUPLICATE,
                                      fpi_device_error_new (FP_DEVICE_ERROR_DATA_DUPLICATE));
}

static void
egis_etu905_enroll_begin_cb (FpDevice *device,
                             guchar   *buffer_in,
                             gsize     length_in,
                             GError   *error)
{
  fp_dbg ("Enroll begin callback");
  FpiDeviceEgisEtu905 *self = FPI_DEVICE_EGIS_ETU905 (device);

  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, error);
      return;
    }

  fpi_ssm_next_state (self->task_ssm);
}

/*
 * Builds the full "check" payload which includes identifiers for all
 * fingerprints which currently should exist on the storage. This payload is
 * used during both enrollment and verify actions.
 */
static guchar *
egis_etu905_get_check_cmd (FpDevice *device,
                           gsize    *length_out)
{
  fp_dbg ("Get check command");
  FpiDeviceEgisEtu905 *self = FPI_DEVICE_EGIS_ETU905 (device);
  g_auto(FpiByteWriter) writer = {0};
  gboolean written = TRUE;

  /*
   * The final command body should contain:
   * 1) hard-coded 00 00
   * 2) 2-byte size indiciator, 20*Number enrolled identifiers plus 9 in form of:
   *    (enrolled_ids->len + 1) * 0x20 + 0x09
   *    Since max prints can be higher than 7 then this goes up to 2 bytes
   *    (e9 + 9 = 109)
   * 3) Hard-coded prefix (cmd_check_prefix)
   * 4) 2-byte size indiciator, 20*Number of enrolled identifiers without plus 9
   *    ((enrolled_ids->len + 1) * 0x20)
   * 5) Hard-coded 32 * 0x00 bytes
   * 6) All of the currently registered prints in their 32-byte device identifiers
   *    (enrolled_list)
   * 7) Hard-coded suffix (cmd_check_suffix)
   */

  g_assert (self->enrolled_ids);
  const gsize body_length = sizeof (guchar) * self->enrolled_ids->len *
                            EGIS_ETU905_FINGERPRINT_DATA_SIZE;

  /* prefix length can depend on the type */
  const gsize check_prefix_length = (fpi_device_get_driver_data (device) &
                                     EGIS_ETU905_DRIVER_CHECK_PREFIX_TYPE2) ?
                                    cmd_check_prefix_type2_len :
                                    cmd_check_prefix_type1_len;

  /* total_length is the 6 various bytes plus all other prefixes/suffixes and
   * the body payload */
  const gsize total_length = (sizeof (guchar) * 6)
                             + check_prefix_length
                             + EGIS_ETU905_CMD_CHECK_SEPARATOR_LENGTH
                             + body_length
                             + cmd_check_suffix_len;

  /* pre-fill entire payload with 00s */
  fpi_byte_writer_init_with_size (&writer, total_length, TRUE);

  /* start with 00 00 (just move starting offset up by 2) */
  written &= fpi_byte_writer_set_pos (&writer, 2);

  /* Size Counter bytes */
  /* "easiest" way to handle 2-bytes size for counter is to hard-code logic for
   * when we go to the 2nd byte
   * note this will not work in case any model ever supports more than 14 prints
   * (assumed max is 10) */
  if (self->enrolled_ids->len > 6)
    {
      written &= fpi_byte_writer_put_uint8 (&writer, 0x01);
      written &= fpi_byte_writer_put_uint8 (&writer,
                                            ((self->enrolled_ids->len - 7) * 0x20)
                                            + 0x09);
    }
  else
    {
      /* first byte is 0x00, just skip it */
      written &= fpi_byte_writer_change_pos (&writer, 1);
      written &= fpi_byte_writer_put_uint8 (&writer,
                                            ((self->enrolled_ids->len + 1) * 0x20) +
                                            0x09);
    }

  /* command prefix */
  if (fpi_device_get_driver_data (device) & EGIS_ETU905_DRIVER_CHECK_PREFIX_TYPE2)
    written &= fpi_byte_writer_put_data (&writer, cmd_check_prefix_type2,
                                         cmd_check_prefix_type2_len);
  else
    written &= fpi_byte_writer_put_data (&writer, cmd_check_prefix_type1,
                                         cmd_check_prefix_type1_len);

  /* 2-bytes size logic for counter again */
  if (self->enrolled_ids->len > 6)
    {
      written &= fpi_byte_writer_put_uint8 (&writer, 0x01);
      written &= fpi_byte_writer_put_uint8 (&writer,
                                            (self->enrolled_ids->len - 7) * 0x20);
    }
  else
    {
      /* first byte is 0x00, just skip it */
      written &= fpi_byte_writer_change_pos (&writer, 1);
      written &= fpi_byte_writer_put_uint8 (&writer,
                                            (self->enrolled_ids->len + 1) * 0x20);
    }

  /* add 00s "separator" to offset position */
  written &= fpi_byte_writer_change_pos (&writer,
                                         EGIS_ETU905_CMD_CHECK_SEPARATOR_LENGTH);

  for (guint i = 0; i < self->enrolled_ids->len && written; i++)
    {
      written &= fpi_byte_writer_put_data (&writer,
                                           g_ptr_array_index (self->enrolled_ids, i),
                                           EGIS_ETU905_FINGERPRINT_DATA_SIZE);
    }

  /* command suffix */
  written &= fpi_byte_writer_put_data (&writer, cmd_check_suffix,
                                       cmd_check_suffix_len);
  g_assert (written);

  if (length_out)
    *length_out = total_length;

  return fpi_byte_writer_reset_and_get_data (&writer);
}

static void
egis_etu905_enroll_run_state (FpiSsm   *ssm,
                              FpDevice *device)
{
  g_auto(FpiByteWriter) writer = {0};
  EgismocSidData sid_data = {0};
  EnrollPrint *enroll_print = fpi_ssm_get_data (ssm);
  g_autofree guchar *payload = NULL;
  gsize payload_length = 0;
  g_autofree gchar *device_print_id = NULL;
  g_autofree gchar *user_id = NULL;

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case ENROLL_START:
      egis_etu905_exec_cmd (device, cmd_enroll_starting, cmd_enroll_starting_len,
                            NULL, egis_etu905_enroll_begin_cb);
      break;

    case ENROLL_CAPTURE_SENSOR_RESET:
      egis_etu905_exec_cmd (device, cmd_sensor_reset, cmd_sensor_reset_len,
                            NULL, egis_etu905_task_ssm_next_state_cb);
      break;

    case ENROLL_CAPTURE_SENSOR_START_CAPTURE:
      egis_etu905_exec_cmd (device, cmd_sensor_start_capture, cmd_sensor_start_capture_len,
                            NULL,
                            egis_etu905_task_ssm_next_state_cb);
      break;

    case ENROLL_CAPTURE_WAIT_FINGER:
      egis_etu905_wait_finger_on_sensor (ssm, device);
      break;

    case ENROLL_CAPTURE_READ_RESPONSE:
      egis_etu905_exec_cmd (device, cmd_read_capture, cmd_read_capture_len,
                            NULL, egis_etu905_read_capture_cb);
      break;

    case ENROLL_DUPLICATE_CHECK:
      egis_etu905_exec_cmd (device, cmd_duplicate_check, cmd_duplicate_check_len,
                            NULL, egis_etu905_enroll_duplicate_check_cb);
      break;

    case ENROLL_COMMIT_START:
      egis_etu905_exec_cmd (device, cmd_commit_starting, cmd_commit_starting_len,
                            NULL, egis_etu905_commit_start_cb);
      break;

    case ENROLL_COMMIT:
      user_id = fpi_print_generate_user_id (enroll_print->print);
      fp_dbg ("New fingerprint ID: %s", user_id);

      sid_data.reserve_para_1 = GUINT16_TO_LE (EGIS_ETU905_PARA_1_VALUE);
      sid_data.reserve_para_2 = GUINT16_TO_LE (EGIS_ETU905_PARA_2_VALUE);
      sid_data.reserve_para_3 = GUINT16_TO_LE (EGIS_ETU905_PARA_3_VALUE);
      memcpy (sid_data.reserve_para_4, user_id, MIN (EGIS_ETU905_FINGERPRINT_DATA_SIZE, strlen (user_id)));
      egis_etu905_set_print_data (enroll_print->print, (const gchar *) &sid_data.reserve_para_4, user_id);
      fpi_byte_writer_init (&writer);
      if (!fpi_byte_writer_put_data (&writer, cmd_new_print_prefix_type2,
                                     cmd_new_print_prefix_type2_len))
        {
          fpi_ssm_mark_failed (ssm, fpi_device_error_new (FP_DEVICE_ERROR_PROTO));
          break;
        }
      if (!fpi_byte_writer_put_data (&writer, (guint8 *) &sid_data,
                                     sizeof (EgismocSidData)))
        {
          fpi_ssm_mark_failed (ssm, fpi_device_error_new (FP_DEVICE_ERROR_PROTO));
          break;
        }

      payload_length = fpi_byte_writer_get_size (&writer);
      egis_etu905_exec_cmd (device, fpi_byte_writer_reset_and_get_data (&writer),
                            payload_length,
                            g_free, egis_etu905_task_ssm_next_state_cb);
      break;

    case ENROLL_COMMIT_SENSOR_RESET:
      egis_etu905_exec_cmd (device, cmd_sensor_reset, cmd_sensor_reset_len,
                            NULL, egis_etu905_task_ssm_next_state_cb);
      break;

    case ENROLL_COMPLETE:
      egis_etu905_enroll_status_report (device, enroll_print, ENROLL_STATUS_COMPLETE, NULL);
      fpi_ssm_next_state (ssm);
      break;
    }
}

static void
egis_etu905_enroll (FpDevice *device)
{
  FpiDeviceEgisEtu905 *self = FPI_DEVICE_EGIS_ETU905 (device);
  EnrollPrint *enroll_print = g_new0 (EnrollPrint, 1);

  fp_dbg ("Enroll");

  fpi_device_get_enroll_data (device, &enroll_print->print);
  enroll_print->stage = 0;

  g_assert (self->task_ssm == NULL);
  self->task_ssm = fpi_ssm_new (device, egis_etu905_enroll_run_state, ENROLL_STATES);
  fpi_ssm_set_data (self->task_ssm, g_steal_pointer (&enroll_print), g_free);
  fpi_ssm_start (self->task_ssm, egis_etu905_task_ssm_done);
}

static void
egis_etu905_identify_check_cb (FpDevice *device,
                               guchar   *buffer_in,
                               gsize     length_in,
                               GError   *error)
{
  FpiDeviceEgisEtu905 *self = FPI_DEVICE_EGIS_ETU905 (device);
  guint8 device_print_id[EGIS_ETU905_FINGERPRINT_DATA_SIZE];
  FpPrint *print = NULL;
  FpPrint *verify_print = NULL;
  GPtrArray *prints;
  gboolean found = FALSE;
  guint index;

  fp_dbg ("Identify check callback");

  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, error);
      return;
    }

  /* Check that the read payload indicates "match" */
  if (egis_etu905_validate_response_suffix (buffer_in,
                                            length_in,
                                            rsp_identify_match_suffix,
                                            rsp_identify_match_suffix_len))
    {
      gboolean id_known = FALSE;
      FpiByteReader reader;

      /*
         On success, there is a 32 byte array of "something"(?) in chars 14-45
         and then the 32 byte array ID of the matched print comes as chars 46-77
       */
      fpi_byte_reader_init (&reader, buffer_in, length_in);

      if (!fpi_byte_reader_set_pos (&reader,
                                    EGIS_ETU905_IDENTIFY_RESPONSE_PRINT_ID_OFFSET) ||
          !fpi_byte_reader_get_data_static (&reader, device_print_id))
        {
          fpi_ssm_mark_failed (self->task_ssm,
                               fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                         "Identify response too "
                                                         "short for matched print "
                                                         "id."));
          return;
        }

      fp_dbg ("Device-reported matched print id: %s", device_print_id);

      /* While the returned ID should indeed be part of the enrolled list, since
       * we got it, let's double check that this is really the case.
       */
      if (self->enrolled_ids)
        {
          for (guint i = 0; i < self->enrolled_ids->len; i++)
            {
              if (memcmp (g_ptr_array_index (self->enrolled_ids, i),
                          device_print_id,
                          EGIS_ETU905_FINGERPRINT_DATA_SIZE) == 0)
                {
                  id_known = TRUE;
                  break;
                }
            }
        }

      if (!id_known)
        {
          fpi_ssm_mark_failed (self->task_ssm,
                               fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                         "Device reported a match "
                                                         "for an unknown print id."));
          return;
        }

      /* Create a new print from this device_print_id and then see if it matches
       * the one indicated
       */
      print = fp_print_new (device);
      egis_etu905_set_print_data (print, (const char *) device_print_id, NULL);

      fp_info ("Identify successful for: %s", fp_print_get_description (print));

      if (fpi_device_get_current_action (device) == FPI_DEVICE_ACTION_IDENTIFY)
        {
          fpi_device_get_identify_data (device, &prints);
          found = g_ptr_array_find_with_equal_func (prints,
                                                    print,
                                                    (GEqualFunc) fp_print_equal,
                                                    &index);

          if (found)
            fpi_device_identify_report (device, g_ptr_array_index (prints, index), print, NULL);
          else
            fpi_device_identify_report (device, NULL, print, NULL);
        }
      else
        {
          fpi_device_get_verify_data (device, &verify_print);
          fp_info ("Verifying against: %s", fp_print_get_description (verify_print));

          if (fp_print_equal (verify_print, print))
            fpi_device_verify_report (device, FPI_MATCH_SUCCESS, print, NULL);
          else
            fpi_device_verify_report (device, FPI_MATCH_FAIL, print, NULL);
        }
    }
  /* If device was successfully read but it was a "not matched" */
  else if (egis_etu905_validate_response_suffix (buffer_in,
                                                 length_in,
                                                 rsp_identify_notmatch_suffix,
                                                 rsp_identify_notmatch_suffix_len))
    {
      fp_info ("Print was not identified by the device");

      if (fpi_device_get_current_action (device) == FPI_DEVICE_ACTION_VERIFY)
        fpi_device_verify_report (device, FPI_MATCH_FAIL, NULL, NULL);
      else
        fpi_device_identify_report (device, NULL, NULL, NULL);
    }
  else
    {
      fpi_ssm_mark_failed (self->task_ssm,
                           fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                     "Unrecognized response from device."));
      return;
    }

  fpi_ssm_next_state (self->task_ssm);
}

static void
egis_etu905_identify_run_state (FpiSsm   *ssm,
                                FpDevice *device)
{
  FpiDeviceEgisEtu905 *self = FPI_DEVICE_EGIS_ETU905 (device);
  g_autofree guchar *payload = NULL;
  gsize payload_length = 0;

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case IDENTIFY_GET_ENROLLED_IDS:
      /* get enrolled_ids from device for use in check stages below */
      egis_etu905_exec_cmd (device, cmd_list, cmd_list_len,
                            NULL, egis_etu905_list_fill_enrolled_ids_cb);
      break;

    case IDENTIFY_CHECK_ENROLLED_NUM:
      if (self->enrolled_ids->len == 0)
        {
          fpi_ssm_mark_failed (g_steal_pointer (&self->task_ssm),
                               fpi_device_error_new (FP_DEVICE_ERROR_DATA_NOT_FOUND));
          return;
        }
      fpi_ssm_next_state (ssm);
      break;

    case IDENTIFY_SENSOR_RESET:
      egis_etu905_exec_cmd (device, cmd_sensor_reset, cmd_sensor_reset_len,
                            NULL, egis_etu905_task_ssm_next_state_cb);
      break;

    case IDENTIFY_SENSOR_IDENTIFY:
      egis_etu905_exec_cmd (device, cmd_sensor_identify, cmd_sensor_identify_len,
                            NULL, egis_etu905_task_ssm_next_state_cb);
      break;

    case IDENTIFY_WAIT_FINGER:
      egis_etu905_wait_finger_on_sensor (ssm, device);
      break;

    case IDENTIFY_SENSOR_CHECK:
      egis_etu905_exec_cmd (device, cmd_sensor_check, cmd_sensor_check_len,
                            NULL, egis_etu905_task_ssm_next_state_cb);
      break;

    case IDENTIFY_CHECK:
      payload = egis_etu905_get_check_cmd (device, &payload_length);
      egis_etu905_exec_cmd (device, g_steal_pointer (&payload), payload_length,
                            g_free, egis_etu905_identify_check_cb);
      break;

    case IDENTIFY_COMPLETE_SENSOR_RESET:
      egis_etu905_exec_cmd (device, cmd_sensor_reset, cmd_sensor_reset_len,
                            NULL, egis_etu905_task_ssm_next_state_cb);
      break;

    /*
     * In Windows, the driver seems at this point to then immediately take
     * another read from the sensor; this is suspected to be an on-chip
     * "verify". However, because the user's finger is still on the sensor from
     * the identify, then it seems to always return positive. We will consider
     * this extra step unnecessary and just skip it in this driver. This driver
     * will instead handle matching of the FpPrint from the gallery in the
     * "verify" case of the callback egis_etu905_identify_check_cb.
     */
    case IDENTIFY_COMPLETE:
      if (fpi_device_get_current_action (device) == FPI_DEVICE_ACTION_IDENTIFY)
        fpi_device_identify_complete (device, NULL);
      else
        fpi_device_verify_complete (device, NULL);

      fpi_ssm_mark_completed (ssm);
      break;
    }
}

static void
egis_etu905_identify_verify (FpDevice *device)
{
  fp_dbg ("Identify or Verify");
  FpiDeviceEgisEtu905 *self = FPI_DEVICE_EGIS_ETU905 (device);

  g_assert (self->task_ssm == NULL);
  self->task_ssm = fpi_ssm_new (device, egis_etu905_identify_run_state, IDENTIFY_STATES);
  fpi_ssm_start (self->task_ssm, egis_etu905_task_ssm_done);
}

static void
egis_etu905_fw_version_cb (FpDevice *device,
                           guchar   *buffer_in,
                           gsize     length_in,
                           GError   *error)
{
  FpiDeviceEgisEtu905 *self = FPI_DEVICE_EGIS_ETU905 (device);
  FpiByteReader reader;
  const guint8 *fw_version_data = NULL;
  const guint prefix_length = egis_etu905_read_prefix_len + 2 + 3 + 1;
  gsize fw_version_length;
  g_autofree gchar *fw_version = NULL;

  fp_dbg ("Firmware version callback");

  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, error);
      return;
    }

  /* Check that the read payload indicates "success" */
  if (!egis_etu905_validate_response_suffix (buffer_in,
                                             length_in,
                                             rsp_fw_version_suffix,
                                             rsp_fw_version_suffix_len))
    {
      fpi_ssm_mark_failed (self->task_ssm,
                           fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                     "Device firmware response "
                                                     "was not valid."));
      return;
    }

  /*
   * FW Version is 12 bytes: a carriage return (0x0d) plus the version string
   * itself. Always skip [the read prefix] + [2 * check bytes] + [3 * 0x00] that
   * come with every payload Then we will also skip the carriage return and take
   * all but the last 2 bytes as the FW Version
   */
  fpi_byte_reader_init (&reader, buffer_in, length_in);

  if (!fpi_byte_reader_set_pos (&reader, prefix_length))
    {
      fpi_ssm_mark_failed (self->task_ssm,
                           fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                     "Device firmware response "
                                                     "too short for prefix."));
      return;
    }

  fw_version_length = fpi_byte_reader_get_remaining (&reader) - rsp_fw_version_suffix_len;

  if (!fpi_byte_reader_get_data (&reader, fw_version_length, &fw_version_data))
    {
      fpi_ssm_mark_failed (self->task_ssm,
                           fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                     "Device firmware response "
                                                     "too short for version string."));
      return;
    }

  fw_version = g_strndup ((gchar *) fw_version_data, fw_version_length);

  fp_info ("Device firmware version is %s", fw_version);

  fpi_ssm_next_state (self->task_ssm);
}

static void
egis_etu905_cmd_init_cb (FpDevice *device,
                         guchar   *buffer_in,
                         gsize     length_in,
                         GError   *error)
{
  fp_dbg ("cmd init callback");
  FpiDeviceEgisEtu905 *self = FPI_DEVICE_EGIS_ETU905 (device);

  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, error);
      return;
    }

  /* Check that the read payload indicates "success" */
  if (!egis_etu905_validate_response_suffix (buffer_in,
                                             length_in,
                                             rsp_fw_version_suffix,
                                             rsp_fw_version_suffix_len))
    {
      fpi_ssm_mark_failed (self->task_ssm,
                           fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                     "cmd init response "
                                                     "was not valid."));
      return;
    }
  fpi_ssm_next_state (self->task_ssm);
}

static void
egis_etu905_dev_init_done (FpiSsm   *ssm,
                           FpDevice *device,
                           GError   *error)
{
  if (error)
    {
      g_usb_device_release_interface (
        fpi_device_get_usb_device (device), 0, 0, NULL);
      egis_etu905_task_ssm_done (ssm, device, error);
      return;
    }

  egis_etu905_task_ssm_done (ssm, device, NULL);
  fpi_device_open_complete (device, NULL);
}

static void
egis_etu905_dev_init_handler (FpiSsm   *ssm,
                              FpDevice *device)
{
  switch (fpi_ssm_get_cur_state (ssm))
    {
    case DEV_GET_FW_VERSION:
      egis_etu905_exec_cmd (device, cmd_fw_version, cmd_fw_version_len,
                            NULL, egis_etu905_fw_version_cb);
      return;

    case DEV_INIT_CONTROL:
      egis_etu905_exec_cmd (device, cmd_init, cmd_init_len,
                            NULL, egis_etu905_cmd_init_cb);
      return;

    default:
      g_assert_not_reached ();
    }
}

static void
egis_etu905_probe (FpDevice *device)
{
  g_autoptr(GError) error = NULL;
  g_autofree gchar *serial = NULL;
  FpiDeviceEgisEtu905 *self = FPI_DEVICE_EGIS_ETU905 (device);
  GUsbDevice *usb_dev;

  fp_dbg ("%s enter --> ", G_STRFUNC);

  /* Claim usb interface */
  usb_dev = fpi_device_get_usb_device (device);
  if (!g_usb_device_open (usb_dev, &error))
    {
      fp_dbg ("%s g_usb_device_open failed %s", G_STRFUNC, error->message);
      fpi_device_probe_complete (device, NULL, NULL, g_steal_pointer (&error));
      return;
    }

  if (!g_usb_device_reset (usb_dev, &error))
    {
      fp_dbg ("%s g_usb_device_reset failed %s", G_STRFUNC, error->message);
      g_usb_device_close (usb_dev, NULL);
      fpi_device_probe_complete (device, NULL, NULL, g_steal_pointer (&error));
      return;
    }

  if (!g_usb_device_claim_interface (usb_dev, 0, 0, &error))
    {
      fp_dbg ("%s g_usb_device_claim_interface failed %s", G_STRFUNC, error->message);
      g_usb_device_close (usb_dev, NULL);
      fpi_device_probe_complete (device, NULL, NULL, g_steal_pointer (&error));
      return;
    }

  if (fpi_device_emulation_mode_enabled (device))
    serial = g_strdup ("emulated-device");
  else
    serial = g_usb_device_get_string_descriptor (usb_dev,
                                                 g_usb_device_get_serial_number_index (usb_dev),
                                                 &error);

  if (error)
    {
      fp_dbg ("%s g_usb_device_get_string_descriptor failed %s", G_STRFUNC, error->message);
      g_usb_device_release_interface (fpi_device_get_usb_device (FP_DEVICE (device)),
                                      0, 0, NULL);
      g_usb_device_close (usb_dev, NULL);
      fpi_device_probe_complete (device, NULL, NULL, g_steal_pointer (&error));
      return;
    }

  if (fpi_device_get_driver_data (device) & EGIS_ETU905_DRIVER_MAX_ENROLL_STAGES_20)
    self->max_enroll_stages = 20;
  else
    self->max_enroll_stages = EGIS_ETU905_MAX_ENROLL_STAGES_DEFAULT;

  fpi_device_set_nr_enroll_stages (device, self->max_enroll_stages);

  g_usb_device_release_interface (fpi_device_get_usb_device (FP_DEVICE (device)), 0, 0, NULL);
  g_usb_device_close (usb_dev, NULL);

  fpi_device_probe_complete (device, serial, NULL, NULL);
}

static void
egis_etu905_open (FpDevice *device)
{
  fp_dbg ("Opening device");
  FpiDeviceEgisEtu905 *self = FPI_DEVICE_EGIS_ETU905 (device);
  g_autoptr(GError) error = NULL;

  if (!g_usb_device_reset (fpi_device_get_usb_device (device), &error))
    {
      fpi_device_open_complete (device, g_steal_pointer (&error));
      return;
    }

  if (!g_usb_device_claim_interface (fpi_device_get_usb_device (device),
                                     0, 0, &error))
    {
      fpi_device_open_complete (device, g_steal_pointer (&error));
      return;
    }

  g_assert (self->task_ssm == NULL);
  self->task_ssm = fpi_ssm_new (device, egis_etu905_dev_init_handler, DEV_INIT_STATES);
  fpi_ssm_start (self->task_ssm, egis_etu905_dev_init_done);
}

static void
egis_etu905_close (FpDevice *device)
{
  g_autoptr(GError) error = NULL;
  fp_dbg ("Closing device");

  g_usb_device_release_interface (fpi_device_get_usb_device (device),
                                  0, 0, &error);
  fpi_device_close_complete (device, g_steal_pointer (&error));
}

static void
fpi_device_egis_etu905_init (FpiDeviceEgisEtu905 *self)
{
  G_DEBUG_HERE ();
}

static void
fpi_device_egis_etu905_class_init (FpiDeviceEgisEtu905Class *klass)
{
  FpDeviceClass *dev_class = FP_DEVICE_CLASS (klass);

  dev_class->id = FP_COMPONENT;
  dev_class->full_name = EGIS_ETU905_DRIVER_FULLNAME;

  dev_class->type = FP_DEVICE_TYPE_USB;
  dev_class->scan_type = FP_SCAN_TYPE_PRESS;
  dev_class->id_table = egis_etu905_id_table;
  dev_class->nr_enroll_stages = EGIS_ETU905_MAX_ENROLL_STAGES_DEFAULT;
  dev_class->temp_hot_seconds = -1;

  dev_class->probe = egis_etu905_probe;
  dev_class->open = egis_etu905_open;
  dev_class->close = egis_etu905_close;
  dev_class->identify = egis_etu905_identify_verify;
  dev_class->verify = egis_etu905_identify_verify;
  dev_class->enroll = egis_etu905_enroll;
  dev_class->delete = egis_etu905_delete;
  dev_class->clear_storage = egis_etu905_clear_storage;
  dev_class->list = egis_etu905_list;

  fpi_device_class_auto_initialize_features (dev_class);
  dev_class->features |= FP_DEVICE_FEATURE_DUPLICATES_CHECK;
}
