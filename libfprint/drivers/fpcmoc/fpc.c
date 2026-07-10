/*
 * Copyright (c) 2022 Fingerprint Cards AB <tech@fingerprints.com>
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

#include "drivers_api.h"
#include "fpc.h"
#include "fpi-byte-writer.h"

#define FP_COMPONENT "fpcmoc"
#define MAX_ENROLL_SAMPLES (25)
#define CTRL_TIMEOUT (2000)
#define DATA_TIMEOUT (5000)

/* Usb port setting */
#define EP_IN (1 | FPI_USB_ENDPOINT_IN)
#define EP_IN_MAX_BUF_SIZE (2048)

struct _FpiDeviceFpcMoc
{
  FpDevice      parent;
  FpiSsm       *task_ssm;
  FpiSsm       *cmd_ssm;
  gboolean      cmd_suspended;
  gint          enroll_stage;
  gint          immobile_stage;
  gint          max_enroll_stage;
  gint          max_immobile_stage;
  gint          max_stored_prints;
  guint         cmd_data_timeout;
  guint8       *dbid;
  gboolean      do_cleanup;
  GCancellable *interrupt_cancellable;
};

G_DEFINE_TYPE (FpiDeviceFpcMoc, fpi_device_fpcmoc, FP_TYPE_DEVICE);

typedef void (*SynCmdMsgCallback) (FpiDeviceFpcMoc *self,
                                   void            *resp,
                                   GError          *error);

typedef struct
{
  FpcCmdType        cmdtype;
  guint8            request;
  guint16           value;
  guint16           index;
  guint8           *data;
  gsize             data_len;
  gsize             resp_len;
  SynCmdMsgCallback callback;
} CommandData;

static const FpIdEntry id_table[] = {
  { .vid = 0x10A5,  .pid = 0xFFE0,  },
  { .vid = 0x10A5,  .pid = 0xA305,  },
  { .vid = 0x10A5,  .pid = 0xA306,  },
  { .vid = 0x10A5,  .pid = 0xDA04,  },
  { .vid = 0x10A5,  .pid = 0xD805,  },
  { .vid = 0x10A5,  .pid = 0xD205,  },
  { .vid = 0x10A5,  .pid = 0x9524,  },
  { .vid = 0x10A5,  .pid = 0x9544,  },
  { .vid = 0x10A5,  .pid = 0xC844,  },
  { .vid = 0x10A5,  .pid = 0x9B24,  },
  /* terminating entry */
  { .vid = 0,  .pid = 0,  .driver_data = 0 },
};

static void
fpc_suspend_resume_cb (FpiUsbTransfer *transfer,
                       FpDevice       *device,
                       gpointer        user_data,
                       GError         *error)
{
  int ssm_state = fpi_ssm_get_cur_state (transfer->ssm);

  fp_dbg ("%s current ssm state: %d", G_STRFUNC, ssm_state);

  if (ssm_state == FPC_CMD_SUSPENDED)
    {
      if (error)
        fpi_ssm_mark_failed (transfer->ssm, error);

      fpi_device_suspend_complete (device, error);
      /* The resume handler continues to the next state! */
    }
  else if (ssm_state == FPC_CMD_RESUME)
    {
      if (error)
        fpi_ssm_mark_failed (transfer->ssm, error);
      else
        fpi_ssm_jump_to_state (transfer->ssm, FPC_CMD_GET_DATA);

      fpi_device_resume_complete (device, error);
    }
}

static void
fpc_cmd_receive_cb (FpiUsbTransfer *transfer,
                    FpDevice       *device,
                    gpointer        user_data,
                    GError         *error)
{
  FpiDeviceFpcMoc *self = FPI_DEVICE_FPCMOC (device);
  CommandData *data = user_data;
  int ssm_state = 0;

  if (g_error_matches (error, G_IO_ERROR, G_IO_ERROR_CANCELLED) && (self->cmd_suspended))
    {
      g_error_free (error);
      fpi_ssm_jump_to_state (transfer->ssm, FPC_CMD_SUSPENDED);
      return;
    }

  if (error)
    {
      fpi_ssm_mark_failed (transfer->ssm, error);
      return;
    }

  if (data == NULL)
    {
      fpi_ssm_mark_failed (transfer->ssm,
                           fpi_device_error_new (FP_DEVICE_ERROR_GENERAL));
      return;
    }

  ssm_state = fpi_ssm_get_cur_state (transfer->ssm);
  fp_dbg ("%s current ssm request: %d state: %d", G_STRFUNC, data->request, ssm_state);

  /* clean cmd_ssm except capture command for suspend/resume case */
  if (ssm_state != FPC_CMD_SEND || data->request != FPC_CMD_ARM)
    self->cmd_ssm = NULL;

  if (data->cmdtype == FPC_CMDTYPE_TO_DEVICE)
    {
      /* It should not receive any data. */
      if (data->callback)
        data->callback (self, NULL, NULL);

      fpi_ssm_mark_completed (transfer->ssm);
      return;
    }
  else if (data->cmdtype == FPC_CMDTYPE_TO_DEVICE_EVTDATA)
    {
      if (ssm_state == FPC_CMD_SEND)
        {
          fpi_ssm_next_state (transfer->ssm);
          return;
        }

      if (ssm_state == FPC_CMD_GET_DATA)
        {
          fp_dbg ("%s recv evt data length: %ld", G_STRFUNC, transfer->actual_length);
          if (transfer->actual_length == 0)
            {
              fp_err ("%s Expect data but actual_length = 0", G_STRFUNC);
              fpi_ssm_mark_failed (transfer->ssm,
                                   fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID));
              return;
            }

          data->resp_len = transfer->actual_length;

          if (data->callback)
            data->callback (self, transfer->buffer, NULL);

          fpi_ssm_mark_completed (transfer->ssm);
          return;
        }
    }
  else if (data->cmdtype == FPC_CMDTYPE_FROM_DEVICE)
    {
      if (transfer->actual_length == 0)
        {
          fp_err ("%s Expect data but actual_length = 0", G_STRFUNC);
          fpi_ssm_mark_failed (transfer->ssm,
                               fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID));
          return;
        }

      data->resp_len = transfer->actual_length;

      if (data->callback)
        data->callback (self, transfer->buffer, NULL);

      fpi_ssm_mark_completed (transfer->ssm);
      return;
    }
  else
    {
      fp_err ("%s incorrect cmdtype (%x) ", G_STRFUNC, data->cmdtype);
      fpi_ssm_mark_failed (transfer->ssm,
                           fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID));
      return;
    }

  /* should not run here... */
  fpi_ssm_mark_failed (transfer->ssm,
                       fpi_device_error_new (FP_DEVICE_ERROR_GENERAL));
}

static void
fpc_send_ctrl_cmd (FpDevice *dev)
{
  FpiUsbTransfer *transfer = NULL;
  FpiDeviceFpcMoc *self = FPI_DEVICE_FPCMOC (dev);
  CommandData *cmd_data = fpi_ssm_get_data (self->cmd_ssm);
  FpcCmdType cmdtype = FPC_CMDTYPE_UNKNOWN;

  if (!cmd_data)
    {
      fp_err ("%s No cmd_data is set ", G_STRFUNC);
      fpi_ssm_mark_failed (self->cmd_ssm,
                           fpi_device_error_new (FP_DEVICE_ERROR_GENERAL));
    }

  cmdtype = cmd_data->cmdtype;

  if ((cmdtype != FPC_CMDTYPE_FROM_DEVICE) && cmd_data->data_len &&
      (cmd_data->data == NULL))
    {
      fp_err ("%s data buffer is null but len is not! ", G_STRFUNC);
      fpi_ssm_mark_failed (self->cmd_ssm,
                           fpi_device_error_new (FP_DEVICE_ERROR_GENERAL));
    }

  if (cmdtype == FPC_CMDTYPE_UNKNOWN)
    {
      fp_err ("%s unknown cmd type ", G_STRFUNC);
      fpi_ssm_mark_failed (self->cmd_ssm,
                           fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID));
    }

  fp_dbg ("%s CMD: 0x%x, value: 0x%x, index: %x type: %d", G_STRFUNC,
          cmd_data->request, cmd_data->value, cmd_data->index, cmdtype);

  transfer = fpi_usb_transfer_new (dev);
  fpi_usb_transfer_fill_control (transfer,
                                 ((cmdtype == FPC_CMDTYPE_FROM_DEVICE) ?
                                  (G_USB_DEVICE_DIRECTION_DEVICE_TO_HOST) :
                                  (G_USB_DEVICE_DIRECTION_HOST_TO_DEVICE)),
                                 G_USB_DEVICE_REQUEST_TYPE_VENDOR,
                                 G_USB_DEVICE_RECIPIENT_DEVICE,
                                 cmd_data->request,
                                 cmd_data->value,
                                 cmd_data->index,
                                 cmd_data->data_len);

  transfer->ssm = self->cmd_ssm;
  if (cmdtype != FPC_CMDTYPE_FROM_DEVICE &&
      cmd_data->data != NULL &&
      cmd_data->data_len != 0)
    memcpy (transfer->buffer, cmd_data->data, cmd_data->data_len);

  fpi_usb_transfer_submit (transfer, CTRL_TIMEOUT, NULL,
                           fpc_cmd_receive_cb,
                           fpi_ssm_get_data (transfer->ssm));
}

static void
fpc_cmd_ssm_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDeviceFpcMoc *self = FPI_DEVICE_FPCMOC (dev);
  CommandData *data;

  self->cmd_ssm = NULL;
  /* Notify about the SSM failure from here instead. */
  if (error)
    {
      fp_err ("%s error: %s ", G_STRFUNC, error->message);
      data = fpi_ssm_get_data (ssm);
      if (data->callback)
        data->callback (self, NULL, error);
    }
}

static void
fpc_cmd_run_state (FpiSsm   *ssm,
                   FpDevice *dev)
{
  FpiUsbTransfer *transfer = NULL;
  FpiDeviceFpcMoc *self = FPI_DEVICE_FPCMOC (dev);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case FPC_CMD_SEND:
      fpc_send_ctrl_cmd (dev);
      break;

    case FPC_CMD_GET_DATA:
      transfer = fpi_usb_transfer_new (dev);
      transfer->ssm = ssm;
      fpi_usb_transfer_fill_bulk (transfer, EP_IN, EP_IN_MAX_BUF_SIZE);
      fpi_usb_transfer_submit (transfer,
                               self->cmd_data_timeout,
                               self->interrupt_cancellable,
                               fpc_cmd_receive_cb,
                               fpi_ssm_get_data (ssm));
      break;

    case FPC_CMD_SUSPENDED:
      transfer = fpi_usb_transfer_new (dev);
      transfer->ssm = ssm;
      fpi_usb_transfer_fill_control (transfer,
                                     G_USB_DEVICE_DIRECTION_HOST_TO_DEVICE,
                                     G_USB_DEVICE_REQUEST_TYPE_VENDOR,
                                     G_USB_DEVICE_RECIPIENT_DEVICE,
                                     FPC_CMD_INDICATE_S_STATE,
                                     FPC_HOST_MS_SX,
                                     0,
                                     0);

      fpi_usb_transfer_submit (transfer, CTRL_TIMEOUT, NULL,
                               fpc_suspend_resume_cb, NULL);
      break;

    case FPC_CMD_RESUME:
      transfer = fpi_usb_transfer_new (dev);
      transfer->ssm = ssm;
      fpi_usb_transfer_fill_control (transfer,
                                     G_USB_DEVICE_DIRECTION_HOST_TO_DEVICE,
                                     G_USB_DEVICE_REQUEST_TYPE_VENDOR,
                                     G_USB_DEVICE_RECIPIENT_DEVICE,
                                     FPC_CMD_INDICATE_S_STATE,
                                     FPC_HOST_MS_S0,
                                     0,
                                     0);

      fpi_usb_transfer_submit (transfer, CTRL_TIMEOUT, NULL,
                               fpc_suspend_resume_cb, NULL);
      break;
    }

}

static void
fpc_sensor_cmd (FpiDeviceFpcMoc *self,
                gboolean         wait_data_delay,
                CommandData     *cmd_data)
{
  CommandData *data = NULL;

  g_return_if_fail (cmd_data);
  g_return_if_fail (cmd_data->cmdtype != FPC_CMDTYPE_UNKNOWN);

  data = g_memdup2 (cmd_data, sizeof (CommandData));

  g_clear_object (&self->interrupt_cancellable);

  if (wait_data_delay)
    {
      self->cmd_data_timeout = 0;
      self->interrupt_cancellable = g_cancellable_new ();
    }
  else
    {
      self->cmd_data_timeout = DATA_TIMEOUT;
    }

  g_assert (self->cmd_ssm == NULL);
  self->cmd_ssm = fpi_ssm_new (FP_DEVICE (self),
                               fpc_cmd_run_state,
                               FPC_CMD_NUM_STATES);

  fpi_ssm_set_data (self->cmd_ssm, data, g_free);
  fpi_ssm_start (self->cmd_ssm, fpc_cmd_ssm_done);
}

static void
fpc_dev_release_interface (FpiDeviceFpcMoc *self,
                           GError          *error)
{
  g_autoptr(GError) release_error = NULL;

  /* Release usb interface */
  g_usb_device_release_interface (fpi_device_get_usb_device (FP_DEVICE (self)),
                                  0, 0, &release_error);
  /* Retain passed error if set, otherwise propagate error from release. */
  if (error)
    {
      fpi_device_close_complete (FP_DEVICE (self), error);
      return;
    }

  /* Notify close complete */
  fpi_device_close_complete (FP_DEVICE (self), g_steal_pointer (&release_error));
}

static gboolean
check_data (void *data, GError **error)
{
  if (*error != NULL)
    return FALSE;

  if (data == NULL)
    {
      g_propagate_error (error,
                         fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID));
      return FALSE;
    }

  return TRUE;
}

static void
fpc_evt_cb (FpiDeviceFpcMoc *self,
            void            *data,
            GError          *error)
{
  FpiByteReader reader;
  guint32 cmdid;
  guint32 evt_length;
  guint32 evt_status;

  if (!check_data (data, &error))
    {
      fp_err ("%s error: %s", G_STRFUNC, error->message);
      fpi_ssm_mark_failed (self->task_ssm, error);
      return;
    }

  fpi_byte_reader_init (&reader, data, EP_IN_MAX_BUF_SIZE);

  if (!fpi_byte_reader_get_uint32_le (&reader, &cmdid) ||
      !fpi_byte_reader_get_uint32_le (&reader, &evt_length))
    {
      fpi_ssm_mark_failed (self->task_ssm,
                           fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID));
      return;
    }

  if (!fpi_byte_reader_get_uint32_le (&reader, &evt_status))
    {
      fpi_ssm_mark_failed (self->task_ssm,
                           fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID));
      return;
    }

  switch (cmdid)
    {
    case FPC_EVT_FID_DATA:
      {
        gint enum_status;
        guint32 num_ids;

        if (!fpi_byte_reader_get_int32_le (&reader, &enum_status) ||
            !fpi_byte_reader_get_uint32_le (&reader, &num_ids))
          {
            fpi_ssm_mark_failed (self->task_ssm,
                                 fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID));
            return;
          }

        fp_dbg ("%s Enum Fids: status = %d, NumFids = %d", G_STRFUNC,
                enum_status, num_ids);
        if (enum_status || (num_ids > FPC_TEMPLATES_MAX))
          {
            fpi_ssm_mark_failed (self->task_ssm,
                                 fpi_device_error_new_msg (FP_DEVICE_ERROR_DATA_INVALID,
                                                           "Get Fids failed"));
            return;
          }
      }
      break;

    case FPC_EVT_INIT_RESULT:
      {
        guint16 sensor;
        guint16 hw_id;
        guint16 img_w;
        guint16 img_h;
        const guint8 *fw_version;

        if (!fpi_byte_reader_get_uint16_le (&reader, &sensor) ||
            !fpi_byte_reader_get_uint16_le (&reader, &hw_id) ||
            !fpi_byte_reader_get_uint16_le (&reader, &img_w) ||
            !fpi_byte_reader_get_uint16_le (&reader, &img_h) ||
            !fpi_byte_reader_get_data (&reader, MAX_FW_VERSION_STR_LEN, &fw_version))
          {
            fpi_ssm_mark_failed (self->task_ssm,
                                 fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID));
            return;
          }

        fp_dbg ("%s INIT: status=%d, Sensor = %d, HWID = 0x%04X, WxH = %d x %d", G_STRFUNC,
                evt_status, sensor, hw_id, img_w, img_h);

        fp_dbg ("%s INIT: FW version: %s", G_STRFUNC, (const gchar *) fw_version);
      }
      break;

    case FPC_EVT_FINGER_DWN:
      fp_dbg ("%s Got finger down event (%d)", G_STRFUNC, evt_status);
      fpi_device_report_finger_status_changes (FP_DEVICE (self),
                                               FP_FINGER_STATUS_PRESENT,
                                               FP_FINGER_STATUS_NONE);
      if (evt_status != 0)
        {
          /* Redo the current task state if capture failed */
          fpi_ssm_jump_to_state (self->task_ssm, fpi_ssm_get_cur_state (self->task_ssm));
          return;
        }
      break;

    case FPC_EVT_IMG:
      fp_dbg ("%s Got capture event", G_STRFUNC);
      fpi_device_report_finger_status_changes (FP_DEVICE (self),
                                               FP_FINGER_STATUS_NONE,
                                               FP_FINGER_STATUS_PRESENT);
      break;

    default:
      fpi_ssm_mark_failed (self->task_ssm,
                           fpi_device_error_new_msg (FP_DEVICE_ERROR_GENERAL,
                                                     "Unknown Evt (0x%x)!", cmdid));
      return;
    }

  fpi_ssm_next_state (self->task_ssm);
}

static void
fpc_do_abort_cb (FpiDeviceFpcMoc *self,
                 void            *data,
                 GError          *error)
{
  if (error)
    {
      fp_err ("%s error: %s ", G_STRFUNC, error->message);
      fpi_ssm_mark_failed (self->task_ssm, error);
      return;
    }

  fp_dbg ("%s Do abort for reasons", G_STRFUNC);
  fpi_ssm_next_state (self->task_ssm);
}

static void
fpc_do_cleanup_cb (FpiDeviceFpcMoc *self,
                   void            *data,
                   GError          *error)
{
  if (error)
    {
      fp_err ("%s error: %s ", G_STRFUNC, error->message);
      fpi_ssm_mark_failed (self->task_ssm, error);
      return;
    }

  fp_dbg ("%s Do cleanup for reasons", G_STRFUNC);
  self->do_cleanup = FALSE;
  fpi_ssm_next_state (self->task_ssm);
}

static void
fpc_template_delete_cb (FpiDeviceFpcMoc *self,
                        void            *resp,
                        GError          *error)
{
  FpDevice *device = FP_DEVICE (self);

  fpi_device_delete_complete (device, error);
}

static gboolean
parse_print_data (GVariant      *data,
                  guint8        *finger,
                  const guint8 **user_id,
                  gsize         *user_id_len)
{
  g_autoptr(GVariant) user_id_var = NULL;

  g_return_val_if_fail (data, FALSE);
  g_return_val_if_fail (finger, FALSE);
  g_return_val_if_fail (user_id, FALSE);
  g_return_val_if_fail (user_id_len, FALSE);

  *user_id = NULL;
  *user_id_len = 0;
  *finger = FPC_SUBTYPE_NOINFORMATION;

  if (!g_variant_check_format_string (data, "(y@ay)", FALSE))
    return FALSE;

  g_variant_get (data,
                 "(y@ay)",
                 finger,
                 &user_id_var);

  *user_id = g_variant_get_fixed_array (user_id_var, user_id_len, 1);

  if (*user_id_len == 0 || *user_id_len > SECURITY_MAX_SID_SIZE)
    return FALSE;

  if (*user_id_len <= 0 || *user_id[0] == '\0' || *user_id[0] == ' ')
    return FALSE;

  if (*finger != FPC_SUBTYPE_RESERVED)
    return FALSE;

  return TRUE;
}

/******************************************************************************
 *
 *  fpc_template_xxx Function
 *
 *****************************************************************************/
static FpPrint *
fpc_print_from_data (FpiDeviceFpcMoc *self, fpc_fid_data_t *fid_data)
{
  FpPrint *print = NULL;
  GVariant *data;
  GVariant *uid;
  g_autofree gchar *userid = NULL;

  userid = g_strndup ((gchar *) fid_data->identity, fid_data->identity_size);
  print = fp_print_new (FP_DEVICE (self));

  uid = g_variant_new_fixed_array (G_VARIANT_TYPE_BYTE,
                                   fid_data->identity,
                                   fid_data->identity_size,
                                   1);

  data = g_variant_new ("(y@ay)",
                        fid_data->subfactor,
                        uid);

  fpi_print_set_type (print, FPI_PRINT_RAW);
  fpi_print_set_device_stored (print, TRUE);
  g_object_set (print, "fpi-data", data, NULL);
  g_object_set (print, "description", userid, NULL);
  fpi_print_fill_from_user_id (print, userid);

  return print;
}

static void
fpc_template_list_cb (FpiDeviceFpcMoc *self,
                      void            *data,
                      GError          *error)
{
  g_autoptr(GPtrArray) list_result = NULL;
  FpDevice *device = FP_DEVICE (self);
  FpiByteReader reader;
  guint32 cmdid;
  guint32 evt_length;
  guint32 evt_status;

  if (error)
    {
      fpi_device_list_complete (FP_DEVICE (self), NULL, error);
      return;
    }

  if (data == NULL)
    {
      fpi_device_list_complete (FP_DEVICE (self),
                                NULL,
                                fpi_device_error_new_msg (FP_DEVICE_ERROR_DATA_INVALID,
                                                          "Data is null"));
      return;
    }

  fpi_byte_reader_init (&reader, data, EP_IN_MAX_BUF_SIZE);

  if (!fpi_byte_reader_get_uint32_le (&reader, &cmdid) ||
      !fpi_byte_reader_get_uint32_le (&reader, &evt_length) ||
      !fpi_byte_reader_get_uint32_le (&reader, &evt_status))
    {
      fpi_device_list_complete (FP_DEVICE (self),
                                NULL,
                                fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID));
      return;
    }

  if (cmdid != FPC_EVT_FID_DATA)
    {
      fpi_device_list_complete (FP_DEVICE (self),
                                NULL,
                                fpi_device_error_new_msg (FP_DEVICE_ERROR_DATA_INVALID,
                                                          "Recv evt is incorrect: 0x%x",
                                                          cmdid));
      return;
    }

  {
    gint enum_status;
    guint32 num_ids;

    if (!fpi_byte_reader_get_int32_le (&reader, &enum_status) ||
        !fpi_byte_reader_get_uint32_le (&reader, &num_ids))
      {
        fpi_device_list_complete (FP_DEVICE (self),
                                  NULL,
                                  fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID));
        return;
      }

    if (num_ids > FPC_TEMPLATES_MAX)
      {
        fpi_device_list_complete (FP_DEVICE (self),
                                  NULL,
                                  fpi_device_error_new_msg (FP_DEVICE_ERROR_DATA_FULL,
                                                            "Database is full"));
        return;
      }

    list_result = g_ptr_array_new_with_free_func (g_object_unref);

    if (num_ids == 0)
      {
        fp_info ("Database is empty");
        fpi_device_list_complete (device,
                                  g_steal_pointer (&list_result),
                                  NULL);
        return;
      }

    for (guint32 n = 0; n < num_ids; n++)
      {
        FpPrint *print = NULL;
        fpc_fid_data_t fid_data = {0};
        guint8 subfactor;
        guint32 identity_type;
        guint32 identity_size;

        if (!fpi_byte_reader_get_uint8 (&reader, &subfactor) ||
            !fpi_byte_reader_get_uint32_le (&reader, &identity_type) ||
            !fpi_byte_reader_get_uint32_le (&reader, &identity_size))
          {
            fpi_device_list_complete (FP_DEVICE (self),
                                      NULL,
                                      fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID));
            return;
          }

        fid_data.subfactor = subfactor;
        fid_data.identity_type = identity_type;
        fid_data.identity_size = identity_size;

        if ((subfactor != FPC_SUBTYPE_RESERVED) &&
            (identity_type != FPC_IDENTITY_TYPE_RESERVED))
          {
            fp_info ("Incompatible template found (0x%x, 0x%x)",
                     subfactor, identity_type);
            fpi_byte_reader_skip (&reader, SECURITY_MAX_SID_SIZE);
            continue;
          }

        if (!fpi_byte_reader_get_data_static (&reader, fid_data.identity))
          {
            fpi_device_list_complete (FP_DEVICE (self),
                                      NULL,
                                      fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID));
            return;
          }

        print = fpc_print_from_data (self, &fid_data);

        g_ptr_array_add (list_result, g_object_ref_sink (print));
      }
  }

  fp_info ("Query templates complete!");
  fpi_device_list_complete (device,
                            g_steal_pointer (&list_result),
                            NULL);
}

/******************************************************************************
 *
 *  fpc_enroll_xxxx Function
 *
 *****************************************************************************/

static void
fpc_enroll_create_cb (FpiDeviceFpcMoc *self,
                      void            *data,
                      GError          *error)
{
  FpiByteReader reader;
  gint32 status;

  if (!check_data (data, &error))
    {
      fp_err ("%s error: %s ", G_STRFUNC, error->message);
      fpi_ssm_mark_failed (self->task_ssm, error);
      return;
    }

  fpi_byte_reader_init (&reader, data, sizeof (FPC_BEGIN_ENROL));

  if (!fpi_byte_reader_get_int32_le (&reader, &status))
    {
      fpi_ssm_mark_failed (self->task_ssm,
                           fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID));
      return;
    }

  if (status != 0)
    {
      error = fpi_device_error_new_msg (FP_DEVICE_ERROR_GENERAL,
                                        "End Enroll failed: %d",
                                        status);
    }

  if (error)
    {
      fp_err ("%s error: %s ", G_STRFUNC, error->message);
      fpi_ssm_mark_failed (self->task_ssm, error);
      return;
    }
  else
    {
      self->do_cleanup = TRUE;
      fpi_ssm_next_state (self->task_ssm);
    }
}

static void
fpc_enroll_update_cb (FpiDeviceFpcMoc *self,
                      void            *data,
                      GError          *error)
{
  FpiByteReader reader;
  gint32 status;
  guint32 remaining;

  if (!check_data (data, &error))
    {
      fp_err ("%s error: %s ", G_STRFUNC, error->message);
      fpi_ssm_mark_failed (self->task_ssm, error);
      return;
    }

  fpi_byte_reader_init (&reader, data, sizeof (FPC_ENROL));

  if (!fpi_byte_reader_get_int32_le (&reader, &status) ||
      !fpi_byte_reader_get_uint32_le (&reader, &remaining))
    {
      fpi_ssm_mark_failed (self->task_ssm,
                           fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID));
      return;
    }

  fp_dbg ("Enrol Update status: %d, remaining: %d", status, remaining);
  switch (status)
    {
    case FPC_ENROL_STATUS_FAILED_COULD_NOT_COMPLETE:
      error = fpi_device_error_new (FP_DEVICE_ERROR_GENERAL);
      break;

    case FPC_ENROL_STATUS_FAILED_ALREADY_ENROLED:
      error = fpi_device_error_new (FP_DEVICE_ERROR_DATA_DUPLICATE);
      break;

    case FPC_ENROL_STATUS_COMPLETED:
      self->enroll_stage++;
      fpi_device_enroll_progress (FP_DEVICE (self), self->enroll_stage, NULL, NULL);
      fpi_ssm_jump_to_state (self->task_ssm, FPC_ENROLL_COMPLETE);
      return;

    case FPC_ENROL_STATUS_IMAGE_TOO_SIMILAR:
      fp_dbg ("Sample overlapping ratio is too High");
      /* here should tips remove finger and try again */
      if (self->max_immobile_stage)
        {
          self->immobile_stage++;
          if (self->immobile_stage > self->max_immobile_stage)
            {
              fp_dbg ("Skip similar handle due to customer enrollment %d(%d)",
                      self->immobile_stage, self->max_immobile_stage);
              /* Skip too similar handle, treat as normal enroll progress. */
              self->enroll_stage++;
              fpi_device_enroll_progress (FP_DEVICE (self), self->enroll_stage, NULL, NULL);
              /* Used for customer enrollment scheme */
              if (self->enroll_stage >= (self->max_enroll_stage - self->max_immobile_stage))
                {
                  fpi_ssm_jump_to_state (self->task_ssm, FPC_ENROLL_COMPLETE);
                  return;
                }
              break;
            }
        }
      fpi_device_enroll_progress (FP_DEVICE (self),
                                  self->enroll_stage,
                                  NULL,
                                  fpi_device_retry_new (FP_DEVICE_RETRY_REMOVE_FINGER));
      break;

    case FPC_ENROL_STATUS_PROGRESS:
      self->enroll_stage++;
      fpi_device_enroll_progress (FP_DEVICE (self), self->enroll_stage, NULL, NULL);
      /* Used for customer enrollment scheme */
      if (self->enroll_stage >= (self->max_enroll_stage - self->max_immobile_stage))
        {
          fpi_ssm_jump_to_state (self->task_ssm, FPC_ENROLL_COMPLETE);
          return;
        }
      break;

    case FPC_ENROL_STATUS_IMAGE_LOW_COVERAGE:
      fpi_device_enroll_progress (FP_DEVICE (self),
                                  self->enroll_stage,
                                  NULL,
                                  fpi_device_retry_new (FP_DEVICE_RETRY_CENTER_FINGER));
      break;

    case FPC_ENROL_STATUS_IMAGE_LOW_QUALITY:
      fpi_device_enroll_progress (FP_DEVICE (self),
                                  self->enroll_stage,
                                  NULL,
                                  fpi_device_retry_new (FP_DEVICE_RETRY_TOO_SHORT));
      break;

    default:
      fp_err ("%s Unknown result code: %d ", G_STRFUNC, status);
      error = fpi_device_error_new_msg (FP_DEVICE_ERROR_GENERAL,
                                        "Enroll failed: %d",
                                        status);
      break;
    }

  if (error)
    {
      fp_err ("%s error: %s ", G_STRFUNC, error->message);
      fpi_ssm_mark_failed (self->task_ssm, error);
      return;
    }
  else
    {
      fpi_ssm_jump_to_state (self->task_ssm, FPC_ENROLL_CAPTURE);
    }
}

static void
fpc_enroll_complete_cb (FpiDeviceFpcMoc *self,
                        void            *data,
                        GError          *error)
{
  FpiByteReader reader;
  gint32 status;
  guint32 fid;

  self->do_cleanup = FALSE;

  if (check_data (data, &error))
    {
      fpi_byte_reader_init (&reader, data, sizeof (FPC_END_ENROL));

      if (!fpi_byte_reader_get_int32_le (&reader, &status) ||
          !fpi_byte_reader_get_uint32_le (&reader, &fid))
        {
          fpi_ssm_mark_failed (self->task_ssm,
                               fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID));
          return;
        }

      if (status != 0)
        {
          error = fpi_device_error_new_msg (FP_DEVICE_ERROR_GENERAL,
                                            "End Enroll failed: %d",
                                            status);
        }
      else
        {
          fp_dbg ("Enrol End status: %d, fid: 0x%x",
                  status, fid);
        }
    }

  if (error)
    {
      fp_err ("%s error: %s ", G_STRFUNC, error->message);
      fpi_ssm_mark_failed (self->task_ssm, error);
      return;
    }
  else
    {
      fpi_ssm_next_state (self->task_ssm);
    }
}

static void
fpc_enroll_check_duplicate_cb (FpiDeviceFpcMoc *self,
                               void            *data,
                               GError          *error)
{
  FpiByteReader reader;

  if (check_data (data, &error))
    {
      gint32 status;
      guint32 identity_type;
      guint32 identity_size;
      guint32 subfactor;

      fpi_byte_reader_init (&reader, data, sizeof (FPC_IDENTIFY));

      if (!fpi_byte_reader_get_int32_le (&reader, &status) ||
          !fpi_byte_reader_get_uint32_le (&reader, &identity_type) ||
          !fpi_byte_reader_skip (&reader, 4) ||
          !fpi_byte_reader_get_uint32_le (&reader, &identity_size) ||
          !fpi_byte_reader_get_uint32_le (&reader, &subfactor))
        {
          fpi_ssm_mark_failed (self->task_ssm,
                               fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID));
          return;
        }

      if ((status == 0) && (subfactor == FPC_SUBTYPE_RESERVED) &&
          (identity_type == FPC_IDENTITY_TYPE_RESERVED) &&
          (identity_size <= SECURITY_MAX_SID_SIZE))
        {
          fp_info ("%s Got a duplicated template", G_STRFUNC);
          error = fpi_device_error_new (FP_DEVICE_ERROR_DATA_DUPLICATE);
        }
    }

  if (error)
    {
      fp_err ("%s error: %s ", G_STRFUNC, error->message);
      fpi_ssm_mark_failed (self->task_ssm, error);
      return;
    }
  else
    {
      fpi_ssm_next_state (self->task_ssm);
    }

}

static void
fpc_enroll_bindid_cb (FpiDeviceFpcMoc *self,
                      void            *data,
                      GError          *error)
{
  if (error)
    {
      fp_err ("%s error: %s ", G_STRFUNC, error->message);
      fpi_ssm_mark_failed (self->task_ssm, error);
      return;
    }

  fpi_ssm_next_state (self->task_ssm);
}

static void
fpc_enroll_commit_cb (FpiDeviceFpcMoc *self,
                      void            *data,
                      GError          *error)
{
  if (check_data (data, &error))
    {
      FpiByteReader reader;
      gint32 result;

      fpi_byte_reader_init (&reader, data, sizeof (gint32));

      if (!fpi_byte_reader_get_int32_le (&reader, &result))
        {
          fpi_ssm_mark_failed (self->task_ssm,
                               fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID));
          return;
        }

      if (result != 0)
        {
          error = fpi_device_error_new_msg (FP_DEVICE_ERROR_DATA_FULL,
                                            "Save DB failed: %d",
                                            result);
        }
    }

  if (error)
    {
      fp_err ("%s error: %s ", G_STRFUNC, error->message);
      fpi_ssm_mark_failed (self->task_ssm, error);
      return;
    }
  else
    {
      fpi_ssm_mark_completed (self->task_ssm);
    }
}


static void
fpc_enroll_sm_run_state (FpiSsm *ssm, FpDevice *device)
{
  FpiDeviceFpcMoc *self = FPI_DEVICE_FPCMOC (device);
  CommandData cmd_data = {0};
  gsize recv_data_len = 0;

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case FPC_ENROLL_ENUM:
      {
        guint8 buf[sizeof (FPC_FID_DATA)] = {0};
        FpiByteWriter writer;

        fpi_byte_writer_init_with_data (&writer, buf, sizeof (buf), FALSE);
        fpi_byte_writer_put_uint32_le (&writer, FPC_IDENTITY_TYPE_WILDCARD);
        fpi_byte_writer_put_uint32_le (&writer, 16);
        fpi_byte_writer_put_uint32_le (&writer, sizeof (guint32));
        fpi_byte_writer_put_uint32_le (&writer, FPC_SUBTYPE_ANY);
        fpi_byte_writer_put_uint32_le (&writer, FPC_IDENTITY_WILDCARD);

        cmd_data.cmdtype = FPC_CMDTYPE_TO_DEVICE_EVTDATA;
        cmd_data.request = FPC_CMD_ENUM;
        cmd_data.value = 0x0;
        cmd_data.index = 0x0;
        cmd_data.data = buf;
        cmd_data.data_len = sizeof (buf);
        cmd_data.callback = fpc_evt_cb;

        fpc_sensor_cmd (self, FALSE, &cmd_data);
      }
      break;

    case FPC_ENROLL_CREATE:
      {
        recv_data_len = sizeof (FPC_BEGIN_ENROL);
        cmd_data.cmdtype = FPC_CMDTYPE_FROM_DEVICE;
        cmd_data.request = FPC_CMD_BEGIN_ENROL;
        cmd_data.value = 0x0;
        cmd_data.index = 0x0;
        cmd_data.data = NULL;
        cmd_data.data_len = recv_data_len;
        cmd_data.callback = fpc_enroll_create_cb;

        fpc_sensor_cmd (self, FALSE, &cmd_data);
      }
      break;

    case FPC_ENROLL_CAPTURE:
      {
        guint8 buf[4];
        FpiByteWriter writer;

        fpi_byte_writer_init_with_data (&writer, buf, sizeof (buf), FALSE);
        fpi_byte_writer_put_uint32_le (&writer, FPC_CAPTUREID_RESERVED);

        fpi_device_report_finger_status_changes (device,
                                                 FP_FINGER_STATUS_NEEDED,
                                                 FP_FINGER_STATUS_NONE);
        cmd_data.cmdtype = FPC_CMDTYPE_TO_DEVICE_EVTDATA;
        cmd_data.request = FPC_CMD_ARM;
        cmd_data.value = 0x1;
        cmd_data.index = 0x0;
        cmd_data.data = buf;
        cmd_data.data_len = sizeof (buf);
        cmd_data.callback = fpc_evt_cb;

        fpc_sensor_cmd (self, TRUE, &cmd_data);
      }
      break;

    case FPC_ENROLL_GET_IMG:
      {
        cmd_data.cmdtype = FPC_CMDTYPE_TO_DEVICE_EVTDATA;
        cmd_data.request = FPC_CMD_GET_IMG;
        cmd_data.value = 0x0;
        cmd_data.index = 0x0;
        cmd_data.data = NULL;
        cmd_data.data_len = 0;
        cmd_data.callback = fpc_evt_cb;

        fpc_sensor_cmd (self, FALSE, &cmd_data);
      }
      break;

    case FPC_ENROLL_UPDATE:
      {
        recv_data_len = sizeof (FPC_ENROL);
        cmd_data.cmdtype = FPC_CMDTYPE_FROM_DEVICE;
        cmd_data.request = FPC_CMD_ENROL;
        cmd_data.value = 0x0;
        cmd_data.index = 0x0;
        cmd_data.data = NULL;
        cmd_data.data_len = recv_data_len;
        cmd_data.callback = fpc_enroll_update_cb;

        fpc_sensor_cmd (self, FALSE, &cmd_data);
      }
      break;

    case FPC_ENROLL_COMPLETE:
      {
        recv_data_len = sizeof (FPC_END_ENROL);
        cmd_data.cmdtype = FPC_CMDTYPE_FROM_DEVICE;
        cmd_data.request = FPC_CMD_END_ENROL;
        cmd_data.value = 0x0;
        cmd_data.index = 0x0;
        cmd_data.data = NULL;
        cmd_data.data_len = recv_data_len;
        cmd_data.callback = fpc_enroll_complete_cb;

        fpc_sensor_cmd (self, FALSE, &cmd_data);
      }
      break;

    case FPC_ENROLL_CHECK_DUPLICATE:
      {
        recv_data_len = sizeof (FPC_IDENTIFY);
        cmd_data.cmdtype = FPC_CMDTYPE_FROM_DEVICE;
        cmd_data.request = FPC_CMD_IDENTIFY;
        cmd_data.value = 0x0;
        cmd_data.index = 0x0;
        cmd_data.data = NULL;
        cmd_data.data_len = recv_data_len;
        cmd_data.callback = fpc_enroll_check_duplicate_cb;

        fpc_sensor_cmd (self, FALSE, &cmd_data);
      }
      break;

    case FPC_ENROLL_BINDID:
      {
        guint8 buf[sizeof (FPC_FID_DATA)] = {0};
        FpiByteWriter writer;
        FpPrint *print = NULL;
        GVariant *fpi_data = NULL;
        GVariant *uid = NULL;
        guint finger = FPC_SUBTYPE_RESERVED;
        g_autofree gchar *user_id = NULL;
        gssize user_id_len;

        fpi_device_get_enroll_data (device, &print);

        user_id = fpi_print_generate_user_id (print);

        user_id_len = strlen (user_id);
        user_id_len = MIN (SECURITY_MAX_SID_SIZE, user_id_len);

        uid = g_variant_new_fixed_array (G_VARIANT_TYPE_BYTE,
                                         user_id,
                                         user_id_len,
                                         1);
        fpi_data = g_variant_new ("(y@ay)",
                                  finger,
                                  uid);

        fpi_print_set_type (print, FPI_PRINT_RAW);
        fpi_print_set_device_stored (print, TRUE);
        g_object_set (print, "fpi-data", fpi_data, NULL);
        g_object_set (print, "description", user_id, NULL);

        fp_dbg ("user_id: %s, finger: 0x%x", user_id, finger);

        fpi_byte_writer_init_with_data (&writer, buf, sizeof (buf), FALSE);
        fpi_byte_writer_put_uint32_le (&writer, FPC_IDENTITY_TYPE_RESERVED);
        fpi_byte_writer_put_uint32_le (&writer, 16);
        fpi_byte_writer_put_uint32_le (&writer, user_id_len);
        fpi_byte_writer_put_uint32_le (&writer, finger);
        fpi_byte_writer_put_data (&writer, (const guint8 *) user_id, user_id_len);

        cmd_data.cmdtype = FPC_CMDTYPE_TO_DEVICE;
        cmd_data.request = FPC_CMD_BIND_IDENTITY;
        cmd_data.value = 0x0;
        cmd_data.index = 0x0;
        cmd_data.data = buf;
        cmd_data.data_len = sizeof (buf);
        cmd_data.callback = fpc_enroll_bindid_cb;

        fpc_sensor_cmd (self, FALSE, &cmd_data);
      }
      break;

    case FPC_ENROLL_COMMIT:
      {
        recv_data_len = sizeof (gint32);
        cmd_data.cmdtype = FPC_CMDTYPE_FROM_DEVICE;
        cmd_data.request = FPC_CMD_STORE_DB;
        cmd_data.value = 0x0;
        cmd_data.index = 0x0;
        cmd_data.data = NULL;
        cmd_data.data_len = recv_data_len;
        cmd_data.callback = fpc_enroll_commit_cb;

        fpc_sensor_cmd (self, FALSE, &cmd_data);
      }
      break;

    case FPC_ENROLL_DICARD:
      {
        cmd_data.cmdtype = FPC_CMDTYPE_TO_DEVICE;
        cmd_data.request = FPC_CMD_ABORT;
        cmd_data.value = 0x1;
        cmd_data.index = 0x0;
        cmd_data.data = NULL;
        cmd_data.data_len = 0;
        cmd_data.callback = fpc_do_abort_cb;
        fpc_sensor_cmd (self, FALSE, &cmd_data);
      }
      break;

    case FPC_ENROLL_CLEANUP:
      {
        if (self->do_cleanup == TRUE)
          {
            recv_data_len = sizeof (FPC_END_ENROL);
            cmd_data.cmdtype = FPC_CMDTYPE_FROM_DEVICE;
            cmd_data.request = FPC_CMD_END_ENROL;
            cmd_data.value = 0x0;
            cmd_data.index = 0x0;
            cmd_data.data = NULL;
            cmd_data.data_len = recv_data_len;
            cmd_data.callback = fpc_do_cleanup_cb;
            fpc_sensor_cmd (self, FALSE, &cmd_data);
          }
        else
          {
            fpi_ssm_next_state (self->task_ssm);
          }
      }
      break;
    }
}

static void
fpc_enroll_ssm_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDeviceFpcMoc *self = FPI_DEVICE_FPCMOC (dev);
  FpPrint *print = NULL;

  fp_info ("Enrollment complete!");

  if (error)
    {
      fpi_device_enroll_complete (dev, NULL, g_steal_pointer (&error));
      self->task_ssm = NULL;
      return;
    }

  fpi_device_get_enroll_data (FP_DEVICE (self), &print);
  fpi_device_enroll_complete (FP_DEVICE (self), g_object_ref (print), NULL);
  self->task_ssm = NULL;
}

/******************************************************************************
 *
 *  fpc_identify_xxx function
 *
 *****************************************************************************/

static void
fpc_identify_cb (FpiDeviceFpcMoc *self,
                 void            *data,
                 GError          *error)
{
  g_autoptr(GPtrArray) templates = NULL;
  FpDevice *device = FP_DEVICE (self);
  FpiByteReader reader;
  gint32 status;
  guint32 identity_type;
  guint32 identity_size;
  guint32 subfactor;

  if (!check_data (data, &error))
    {
      fp_err ("%s error: %s ", G_STRFUNC, error->message);
      fpi_ssm_mark_failed (self->task_ssm, error);
      return;
    }

  fpi_byte_reader_init (&reader, data, sizeof (FPC_IDENTIFY));

  if (!fpi_byte_reader_get_int32_le (&reader, &status) ||
      !fpi_byte_reader_get_uint32_le (&reader, &identity_type) ||
      !fpi_byte_reader_skip (&reader, 4) ||
      !fpi_byte_reader_get_uint32_le (&reader, &identity_size) ||
      !fpi_byte_reader_get_uint32_le (&reader, &subfactor))
    {
      fpi_ssm_mark_failed (self->task_ssm,
                           fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID));
      return;
    }

  if ((status == 0) && (subfactor == FPC_SUBTYPE_RESERVED) &&
      (identity_type == FPC_IDENTITY_TYPE_RESERVED) &&
      (identity_size <= SECURITY_MAX_SID_SIZE))
    {
      FpPrint *match = NULL;
      fpc_fid_data_t fid_data = {0};

      fid_data.subfactor = subfactor;
      fid_data.identity_type = identity_type;
      fid_data.identity_size = identity_size;

      if (!(fpi_byte_reader_get_data_static) (&reader, identity_size, fid_data.identity))
        {
          fpi_ssm_mark_failed (self->task_ssm,
                               fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID));
          return;
        }

      match = fpc_print_from_data (self, &fid_data);

      fpi_device_get_identify_data (device, &templates);
      g_ptr_array_ref (templates);

      guint matching_index;

      if (g_ptr_array_find_with_equal_func (templates, match,
                                            (GEqualFunc) fp_print_equal, &matching_index))
        {
          FpPrint *print = g_ptr_array_index (templates, matching_index);

          fpi_device_identify_report (device, print, match, error);

          fpi_ssm_mark_completed (self->task_ssm);
          return;
        }
    }

  fpi_device_identify_report (device, NULL, NULL, error);
  fpi_ssm_mark_completed (self->task_ssm);
}

static void
fpc_identify_sm_run_state (FpiSsm *ssm, FpDevice *device)
{
  FpiDeviceFpcMoc *self = FPI_DEVICE_FPCMOC (device);
  CommandData cmd_data = {0};

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case FPC_IDENTIFY_CAPTURE:
      {
        guint8 buf[4];
        FpiByteWriter writer;

        fpi_byte_writer_init_with_data (&writer, buf, sizeof (buf), FALSE);
        fpi_byte_writer_put_uint32_le (&writer, FPC_CAPTUREID_RESERVED);

        fpi_device_report_finger_status_changes (device,
                                                 FP_FINGER_STATUS_NEEDED,
                                                 FP_FINGER_STATUS_NONE);
        cmd_data.cmdtype = FPC_CMDTYPE_TO_DEVICE_EVTDATA;
        cmd_data.request = FPC_CMD_ARM;
        cmd_data.value = 0x1;
        cmd_data.index = 0x0;
        cmd_data.data = buf;
        cmd_data.data_len = sizeof (buf);
        cmd_data.callback = fpc_evt_cb;

        fpc_sensor_cmd (self, TRUE, &cmd_data);
      }
      break;

    case FPC_IDENTIFY_GET_IMG:
      {
        cmd_data.cmdtype = FPC_CMDTYPE_TO_DEVICE_EVTDATA;
        cmd_data.request = FPC_CMD_GET_IMG;
        cmd_data.value = 0x0;
        cmd_data.index = 0x0;
        cmd_data.data = NULL;
        cmd_data.data_len = 0;
        cmd_data.callback = fpc_evt_cb;

        fpc_sensor_cmd (self, FALSE, &cmd_data);
      }
      break;

    case FPC_IDENTIFY_IDENTIFY:
      {
        gsize recv_data_len = sizeof (FPC_IDENTIFY);
        cmd_data.cmdtype = FPC_CMDTYPE_FROM_DEVICE;
        cmd_data.request = FPC_CMD_IDENTIFY;
        cmd_data.value = 0x0;
        cmd_data.index = 0x0;
        cmd_data.data = NULL;
        cmd_data.data_len = recv_data_len;
        cmd_data.callback = fpc_identify_cb;

        fpc_sensor_cmd (self, FALSE, &cmd_data);
      }
      break;

    case FPC_IDENTIFY_CANCEL:
      {
        cmd_data.cmdtype = FPC_CMDTYPE_TO_DEVICE;
        cmd_data.request = FPC_CMD_ABORT;
        cmd_data.value = 0x1;
        cmd_data.index = 0x0;
        cmd_data.data = NULL;
        cmd_data.data_len = 0;
        cmd_data.callback = fpc_do_abort_cb;
        fpc_sensor_cmd (self, FALSE, &cmd_data);

      }
      break;
    }
}

static void
fpc_identify_ssm_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDeviceFpcMoc *self = FPI_DEVICE_FPCMOC (dev);

  fp_info ("Verify_identify complete!");

  if (error && error->domain == FP_DEVICE_RETRY)
    fpi_device_identify_report (dev, NULL, NULL, g_steal_pointer (&error));

  fpi_device_identify_complete (dev, g_steal_pointer (&error));

  self->task_ssm = NULL;
}

/******************************************************************************
 *
 *  fpc_init_xxx function
 *
 *****************************************************************************/
static void
fpc_clear_storage_cb (FpiDeviceFpcMoc *self,
                      void            *resp,
                      GError          *error)
{
  if (error)
    {
      fp_err ("%s error: %s ", G_STRFUNC, error->message);
      fpi_ssm_mark_failed (self->task_ssm, error);
      return;
    }

  fpi_ssm_next_state (self->task_ssm);

}

static void
fpc_clear_sm_run_state (FpiSsm *ssm, FpDevice *device)
{
  FpiDeviceFpcMoc *self = FPI_DEVICE_FPCMOC (device);
  CommandData cmd_data = {0};
  FPC_DB_OP data = {0};
  gsize data_len = 0;

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case FPC_CLEAR_DELETE_DB:
      {
        if (self->dbid)
          {
            data_len = sizeof (FPC_DB_OP);
            data.database_id_size = FPC_DB_ID_LEN;
            data.reserved = 8;
            memcpy (&data.data[0], self->dbid, FPC_DB_ID_LEN);
            cmd_data.cmdtype = FPC_CMDTYPE_TO_DEVICE;
            cmd_data.request = FPC_CMD_DELETE_DB;
            cmd_data.value = 0x0;
            cmd_data.index = 0x0;
            cmd_data.data = (guint8 *) &data;
            cmd_data.data_len = data_len;
            cmd_data.callback = fpc_clear_storage_cb;
            fpc_sensor_cmd (self, FALSE, &cmd_data);
          }
        else
          {
            fpi_ssm_mark_failed (self->task_ssm,
                                 fpi_device_error_new_msg (FP_DEVICE_ERROR_NOT_SUPPORTED,
                                                           "No DBID found"));
          }

      }
      break;

    case FPC_CLEAR_CREATE_DB:
      {
        if (self->dbid)
          {
            data_len = sizeof (FPC_DB_OP);
            data.database_id_size = FPC_DB_ID_LEN;
            data.reserved = 8;
            memcpy (&data.data[0], self->dbid, FPC_DB_ID_LEN);
            cmd_data.cmdtype = FPC_CMDTYPE_TO_DEVICE;
            cmd_data.request = FPC_CMD_LOAD_DB;
            cmd_data.value = 0x1;
            cmd_data.index = 0x0;
            cmd_data.data = (guint8 *) &data;
            cmd_data.data_len = data_len;
            cmd_data.callback = fpc_clear_storage_cb;
            fpc_sensor_cmd (self, FALSE, &cmd_data);
          }
        else
          {
            fpi_ssm_mark_failed (self->task_ssm,
                                 fpi_device_error_new_msg (FP_DEVICE_ERROR_NOT_SUPPORTED,
                                                           "No DBID found"));
          }
      }
      break;
    }
}

static void
fpc_clear_ssm_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDeviceFpcMoc *self = FPI_DEVICE_FPCMOC (dev);

  fp_info ("Clear Storage complete!");

  fpi_device_clear_storage_complete (dev, g_steal_pointer (&error));
  self->task_ssm = NULL;
}

/******************************************************************************
 *
 *  fpc_init_xxx function
 *
 *****************************************************************************/

static void
fpc_init_load_db_cb (FpiDeviceFpcMoc *self,
                     void            *data,
                     GError          *error)
{
  FpiByteReader reader;

  if (error)
    {
      fp_err ("%s error: %s ", G_STRFUNC, error->message);
      fpi_ssm_mark_failed (self->task_ssm, error);
      return;
    }

  if (data == NULL)
    {
      fpi_ssm_mark_failed (self->task_ssm,
                           fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID));
      return;
    }

  fpi_byte_reader_init (&reader, data, sizeof (FPC_LOAD_DB));

  {
    gint32 status;
    guint32 database_id_size;
    const guint8 *db_data;

    if (!fpi_byte_reader_get_int32_le (&reader, &status) ||
        !fpi_byte_reader_skip (&reader, 4) ||
        !fpi_byte_reader_get_uint32_le (&reader, &database_id_size) ||
        !fpi_byte_reader_get_data (&reader, FPC_DB_ID_LEN, &db_data))
      {
        fpi_ssm_mark_failed (self->task_ssm,
                             fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID));
        return;
      }

    if (status)
      {
        fp_err ("%s Load DB failed: %d - Expect to create a new one", G_STRFUNC, status);
        fpi_ssm_next_state (self->task_ssm);
        return;
      }

    g_clear_pointer (&self->dbid, g_free);
    self->dbid = g_memdup2 (db_data, FPC_DB_ID_LEN);
    if (self->dbid == NULL)
      {
        fpi_ssm_mark_failed (self->task_ssm, fpi_device_error_new (FP_DEVICE_ERROR_GENERAL));
        return;
      }

    fp_dbg ("%s got dbid size: %d", G_STRFUNC, database_id_size);
    fp_dbg ("%s dbid: 0x%02x%02x%02x%02x-%02x%02x-%02x%02x-" \
            "%02x%02x-%02x%02x%02x%02x%02x%02x",
            G_STRFUNC,
            db_data[0], db_data[1],
            db_data[2], db_data[3],
            db_data[4], db_data[5],
            db_data[6], db_data[7],
            db_data[8], db_data[9],
            db_data[10], db_data[11],
            db_data[12], db_data[13],
            db_data[14], db_data[15]);

    fpi_ssm_mark_completed (self->task_ssm);
  }
}

static void
fpc_init_sm_run_state (FpiSsm *ssm, FpDevice *device)
{
  FpiDeviceFpcMoc *self = FPI_DEVICE_FPCMOC (device);
  CommandData cmd_data = {0};

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case FPC_INIT:
      {
        guint8 buf[4];
        FpiByteWriter writer;

        fpi_byte_writer_init_with_data (&writer, buf, sizeof (buf), FALSE);
        fpi_byte_writer_put_uint32_le (&writer, FPC_SESSIONID_RESERVED);

        cmd_data.cmdtype = FPC_CMDTYPE_TO_DEVICE_EVTDATA;
        cmd_data.request = FPC_CMD_INIT;
        cmd_data.value = 0x1;
        cmd_data.index = 0x0;
        cmd_data.data = buf;
        cmd_data.data_len = sizeof (buf);
        cmd_data.callback = fpc_evt_cb;

        fpc_sensor_cmd (self, FALSE, &cmd_data);
      }
      break;

    case FPC_INIT_LOAD_DB:
      {
        gsize recv_data_len = sizeof (FPC_LOAD_DB);
        cmd_data.cmdtype = FPC_CMDTYPE_FROM_DEVICE;
        cmd_data.request = FPC_CMD_LOAD_DB;
        cmd_data.value = 0x0;
        cmd_data.index = 0x0;
        cmd_data.data = NULL;
        cmd_data.data_len = recv_data_len;
        cmd_data.callback = fpc_init_load_db_cb;

        fpc_sensor_cmd (self, FALSE, &cmd_data);
      }
      break;
    }
}

static void
fpc_init_ssm_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDeviceFpcMoc *self = FPI_DEVICE_FPCMOC (dev);

  fpi_device_open_complete (dev, g_steal_pointer (&error));
  self->task_ssm = NULL;
}

/******************************************************************************
 *
 *  Interface Function
 *
 *****************************************************************************/

static void
fpc_dev_probe (FpDevice *device)
{
  GUsbDevice *usb_dev;
  GError *error = NULL;
  FpiDeviceFpcMoc *self = FPI_DEVICE_FPCMOC (device);
  g_autofree gchar *product = NULL;
  gint product_id = 0;

  fp_dbg ("%s enter --> ", G_STRFUNC);

  /* Claim usb interface */
  usb_dev = fpi_device_get_usb_device (device);
  if (!g_usb_device_open (usb_dev, &error))
    {
      fp_dbg ("%s g_usb_device_open failed %s", G_STRFUNC, error->message);
      fpi_device_probe_complete (device, NULL, NULL, error);
      return;
    }

  if (!g_usb_device_reset (usb_dev, &error))
    {
      fp_dbg ("%s g_usb_device_reset failed %s", G_STRFUNC, error->message);
      g_usb_device_close (usb_dev, NULL);
      fpi_device_probe_complete (device, NULL, NULL, error);
      return;
    }

  if (!g_usb_device_claim_interface (usb_dev, 0, 0, &error))
    {
      fp_dbg ("%s g_usb_device_claim_interface failed %s", G_STRFUNC, error->message);
      g_usb_device_close (usb_dev, NULL);
      fpi_device_probe_complete (device, NULL, NULL, error);
      return;
    }

  product = g_usb_device_get_string_descriptor (usb_dev,
                                                g_usb_device_get_product_index (usb_dev),
                                                &error);
  if (product)
    fp_dbg ("Device name: %s", product);

  if (error)
    {
      fp_dbg ("%s g_usb_device_get_string_descriptor failed %s", G_STRFUNC, error->message);
      g_usb_device_release_interface (fpi_device_get_usb_device (FP_DEVICE (device)),
                                      0, 0, NULL);
      g_usb_device_close (usb_dev, NULL);
      fpi_device_probe_complete (device, NULL, NULL, error);
      return;
    }

  product_id = g_usb_device_get_pid (usb_dev);
  /* Reserved for customer enroll scheme */
  self->max_immobile_stage = 0;   /* By default is not customer enrollment */
  switch (product_id)
    {
    case 0xFFE0:
    case 0xA305:
    case 0xA306:
    case 0xD805:
    case 0xDA04:
    case 0xD205:
    case 0x9524:
    case 0x9544:
    case 0xC844:
      self->max_enroll_stage = MAX_ENROLL_SAMPLES;
      break;

    default:
      fp_warn ("Device %x is not supported", product_id);
      self->max_enroll_stage = MAX_ENROLL_SAMPLES;
      break;
    }
  fpi_device_set_nr_enroll_stages (device, self->max_enroll_stage);
  g_usb_device_release_interface (fpi_device_get_usb_device (FP_DEVICE (device)),
                                  0, 0, NULL);
  g_usb_device_close (usb_dev, NULL);
  fpi_device_probe_complete (device, NULL, product, error);
}

static void
fpc_dev_open (FpDevice *device)
{
  FpiDeviceFpcMoc *self = FPI_DEVICE_FPCMOC (device);
  GError *error = NULL;

  fp_dbg ("%s enter -->", G_STRFUNC);
  if (!g_usb_device_reset (fpi_device_get_usb_device (device), &error))
    {
      fpi_device_open_complete (FP_DEVICE (self), error);
      return;
    }

  /* Claim usb interface */
  if (!g_usb_device_claim_interface (fpi_device_get_usb_device (device), 0, 0, &error))
    {
      fpi_device_open_complete (FP_DEVICE (self), error);
      return;
    }

  self->task_ssm = fpi_ssm_new (device, fpc_init_sm_run_state,
                                FPC_INIT_NUM_STATES);

  fpi_ssm_start (self->task_ssm, fpc_init_ssm_done);
}

static void
fpc_dev_close (FpDevice *device)
{
  FpiDeviceFpcMoc *self = FPI_DEVICE_FPCMOC (device);

  fp_dbg ("%s enter -->", G_STRFUNC);
  g_clear_pointer (&self->dbid, g_free);
  g_cancellable_cancel (self->interrupt_cancellable);
  g_clear_object (&self->interrupt_cancellable);
  fpc_dev_release_interface (self, NULL);
}

static void
fpc_dev_identify (FpDevice *device)
{
  FpiDeviceFpcMoc *self = FPI_DEVICE_FPCMOC (device);

  fp_dbg ("%s enter -->", G_STRFUNC);
  self->task_ssm = fpi_ssm_new_full (device, fpc_identify_sm_run_state,
                                     FPC_IDENTIFY_NUM_STATES,
                                     FPC_IDENTIFY_CANCEL,
                                     "identify");

  fpi_ssm_start (self->task_ssm, fpc_identify_ssm_done);
}

static void
fpc_dev_enroll (FpDevice *device)
{
  FpiDeviceFpcMoc *self = FPI_DEVICE_FPCMOC (device);

  fp_dbg ("%s enter -->", G_STRFUNC);

  self->enroll_stage = 0;
  self->immobile_stage = 0;
  self->task_ssm = fpi_ssm_new_full (device, fpc_enroll_sm_run_state,
                                     FPC_ENROLL_NUM_STATES,
                                     FPC_ENROLL_DICARD,
                                     "enroll");

  fpi_ssm_start (self->task_ssm, fpc_enroll_ssm_done);
}

static void
fpc_dev_template_list (FpDevice *device)
{
  FpiDeviceFpcMoc *self = FPI_DEVICE_FPCMOC (device);
  CommandData cmd_data = {0};
  guint8 buf[sizeof (FPC_FID_DATA)] = {0};
  FpiByteWriter writer;

  fp_dbg ("%s enter -->", G_STRFUNC);

  fpi_byte_writer_init_with_data (&writer, buf, sizeof (buf), FALSE);
  fpi_byte_writer_put_uint32_le (&writer, FPC_IDENTITY_TYPE_WILDCARD);
  fpi_byte_writer_put_uint32_le (&writer, 16);
  fpi_byte_writer_put_uint32_le (&writer, sizeof (guint32));
  fpi_byte_writer_put_uint32_le (&writer, FPC_SUBTYPE_ANY);
  fpi_byte_writer_put_uint32_le (&writer, FPC_IDENTITY_WILDCARD);

  cmd_data.cmdtype = FPC_CMDTYPE_TO_DEVICE_EVTDATA;
  cmd_data.request = FPC_CMD_ENUM;
  cmd_data.value = 0x0;
  cmd_data.index = 0x0;
  cmd_data.data = buf;
  cmd_data.data_len = sizeof (buf);
  cmd_data.callback = fpc_template_list_cb;

  fpc_sensor_cmd (self, FALSE, &cmd_data);
}

static void
fpc_dev_suspend (FpDevice *device)
{
  FpiDeviceFpcMoc *self = FPI_DEVICE_FPCMOC (device);
  FpiDeviceAction action = fpi_device_get_current_action (device);

  fp_dbg ("%s enter -->", G_STRFUNC);

  if (action != FPI_DEVICE_ACTION_IDENTIFY)
    {
      fpi_device_suspend_complete (device, fpi_device_error_new (FP_DEVICE_ERROR_NOT_SUPPORTED));
      return;
    }

  g_assert (self->cmd_ssm);
  g_assert (fpi_ssm_get_cur_state (self->cmd_ssm) == FPC_CMD_GET_DATA);
  self->cmd_suspended = TRUE;
  g_cancellable_cancel (self->interrupt_cancellable);
}

static void
fpc_dev_resume (FpDevice *device)
{
  FpiDeviceFpcMoc *self = FPI_DEVICE_FPCMOC (device);
  FpiDeviceAction action = fpi_device_get_current_action (device);

  fp_dbg ("%s enter -->", G_STRFUNC);

  if (action != FPI_DEVICE_ACTION_IDENTIFY)
    {
      g_assert_not_reached ();
      fpi_device_resume_complete (device, fpi_device_error_new (FP_DEVICE_ERROR_NOT_SUPPORTED));
      return;
    }

  g_assert (self->cmd_ssm);
  g_assert (self->cmd_suspended);
  g_assert (fpi_ssm_get_cur_state (self->cmd_ssm) == FPC_CMD_SUSPENDED);
  self->cmd_suspended = FALSE;

  g_clear_object (&self->interrupt_cancellable);
  self->interrupt_cancellable = g_cancellable_new ();
  fpi_ssm_jump_to_state (self->cmd_ssm, FPC_CMD_RESUME);
}

static void
fpc_dev_cancel (FpDevice *device)
{
  FpiDeviceFpcMoc *self = FPI_DEVICE_FPCMOC (device);

  fp_dbg ("%s enter -->", G_STRFUNC);
  g_cancellable_cancel (self->interrupt_cancellable);
}

static void
fpc_dev_template_delete (FpDevice *device)
{
  FpiDeviceFpcMoc *self = FPI_DEVICE_FPCMOC (device);
  CommandData cmd_data = {0};
  FpPrint *print = NULL;

  g_autoptr(GVariant) fpi_data = NULL;
  guint8 buf[sizeof (FPC_FID_DATA)] = {0};
  FpiByteWriter writer;
  guint8 finger = FPC_SUBTYPE_NOINFORMATION;
  const guint8 *user_id;
  gsize user_id_len = 0;

  fp_dbg ("%s enter -->", G_STRFUNC);

  fpi_device_get_delete_data (device, &print);

  g_object_get (print, "fpi-data", &fpi_data, NULL);

  if (!parse_print_data (fpi_data, &finger, &user_id, &user_id_len))
    {
      fpi_device_delete_complete (device,
                                  fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID));
      return;
    }

  fpi_byte_writer_init_with_data (&writer, buf, sizeof (buf), FALSE);
  fpi_byte_writer_put_uint32_le (&writer, FPC_IDENTITY_TYPE_RESERVED);
  fpi_byte_writer_put_uint32_le (&writer, 16);
  fpi_byte_writer_put_uint32_le (&writer, user_id_len);
  fpi_byte_writer_put_uint32_le (&writer, (guint32) finger);
  fpi_byte_writer_put_data (&writer, user_id, user_id_len);

  cmd_data.cmdtype = FPC_CMDTYPE_TO_DEVICE;
  cmd_data.request = FPC_CMD_DELETE_TEMPLATE;
  cmd_data.value = 0x0;
  cmd_data.index = 0x0;
  cmd_data.data = buf;
  cmd_data.data_len = sizeof (buf);
  cmd_data.callback = fpc_template_delete_cb;

  fpc_sensor_cmd (self, FALSE, &cmd_data);
  fp_dbg ("%s exit <--", G_STRFUNC);
}

static void
fpc_dev_clear_storage (FpDevice *device)
{
  FpiDeviceFpcMoc *self = FPI_DEVICE_FPCMOC (device);

  fp_dbg ("%s enter -->", G_STRFUNC);
  self->task_ssm = fpi_ssm_new_full (device, fpc_clear_sm_run_state,
                                     FPC_CLEAR_NUM_STATES,
                                     FPC_CLEAR_NUM_STATES,
                                     "Clear storage");

  fpi_ssm_start (self->task_ssm, fpc_clear_ssm_done);
}

static void
fpi_device_fpcmoc_init (FpiDeviceFpcMoc *self)
{
  fp_dbg ("%s enter -->", G_STRFUNC);
  G_DEBUG_HERE ();
}

static void
fpi_device_fpcmoc_class_init (FpiDeviceFpcMocClass *klass)
{
  FpDeviceClass *dev_class = FP_DEVICE_CLASS (klass);

  dev_class->id =               FP_COMPONENT;
  dev_class->full_name =        "FPC MOC Fingerprint Sensor";
  dev_class->type =             FP_DEVICE_TYPE_USB;
  dev_class->scan_type =        FP_SCAN_TYPE_PRESS;
  dev_class->id_table =         id_table;
  dev_class->nr_enroll_stages = MAX_ENROLL_SAMPLES;
  dev_class->temp_hot_seconds = -1;

  dev_class->open   =           fpc_dev_open;
  dev_class->close  =           fpc_dev_close;
  dev_class->probe  =           fpc_dev_probe;
  dev_class->enroll =           fpc_dev_enroll;
  dev_class->delete =           fpc_dev_template_delete;
  dev_class->list   =           fpc_dev_template_list;
  dev_class->identify =         fpc_dev_identify;
  dev_class->suspend =          fpc_dev_suspend;
  dev_class->resume =           fpc_dev_resume;
  dev_class->clear_storage =    fpc_dev_clear_storage;
  dev_class->cancel =           fpc_dev_cancel;

  fpi_device_class_auto_initialize_features (dev_class);
  dev_class->features |= FP_DEVICE_FEATURE_DUPLICATES_CHECK;
}
