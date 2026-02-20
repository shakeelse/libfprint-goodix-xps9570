/*
 * Copyright (C) 2025 Marco Trevisan <marco.trevisan@canonical.com>
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

#include "glib.h"
#include "fpi-context.h"
#include "fpi-device.h"

#define METAINFO_BASE \
  "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n" \
  "<component>\n" \
  "  <id>org.freedesktop.libfprint</id>\n" \
  "  <name>libfprint</name>\n" \
  "  <metadata_license>CC0-1.0</metadata_license>\n" \
  "  <project_license>LGPL-2.1-or-later</project_license>\n" \
  "  <url type=\"homepage\">https://fprint.freedesktop.org</url>\n" \
  "  <url type=\"vcs-browser\">https://gitlab.freedesktop.org/libfprint/libfprint</url>\n" \
  "  <url type=\"help\">https://fprint.freedesktop.org/libfprint-dev</url>\n" \
  "  <url type=\"bugtracker\">https://gitlab.freedesktop.org/libfprint/libfprint/-/work_items</url>\n" \
  "  <summary>Async fingerprint readers library</summary>\n" \
  "  <description>\n" \
  "    <p>\n" \
  "      The fprint project aims to support for consumer fingerprint reader devices.\n" \
  "    </p>\n" \
  "  </description>\n" \
  "  <provides>\n" \
  "%s\n" \
  "  </provides>\n" \
  "</component>\n"

static int
driver_compare (gconstpointer p1, gconstpointer p2)
{
  g_autoptr(FpDeviceClass) cls1 = g_type_class_ref (*(GType *) p1);
  g_autoptr(FpDeviceClass) cls2 = g_type_class_ref (*(GType *) p2);

  return g_strcmp0 (cls1->id, cls2->id);
}

static void
usb_driver_devices_append (GPtrArray           *devices_list,
                           const FpDeviceClass *cls)
{
  const FpIdEntry *entry;

  if (cls->type != FP_DEVICE_TYPE_USB)
    return;

  for (entry = cls->id_table; entry->vid != 0; entry++)
    {
      if (entry->vid == 0 || entry->pid == 0)
        continue;

      g_ptr_array_add (devices_list,
                       g_strdup_printf ("v%04xp%04x", entry->vid, entry->pid));
    }
}

int
main (void)
{
  g_autoptr(GPtrArray) devices_list = g_ptr_array_new_with_free_func (g_free);
  g_autoptr(GArray) drivers = fpi_get_driver_types ();
  g_autoptr(GString) provided_modules = g_string_new (NULL);
  g_autoptr(GError) error = NULL;

  g_log_writer_default_set_use_stderr (TRUE);

  g_array_sort (drivers, driver_compare);

  for (guint i = 0; i < drivers->len; ++i)
    {
      GType driver = g_array_index (drivers, GType, i);
      g_autoptr(FpDeviceClass) cls = g_type_class_ref (driver);

      if (cls->type != FP_DEVICE_TYPE_USB)
        continue;

      usb_driver_devices_append (devices_list, cls);
    }

  for (guint i = 0; i < devices_list->len; ++i)
    {
      const char *device_id = g_ptr_array_index (devices_list, i);

      g_string_append (provided_modules, "    ");
      g_string_append_printf (provided_modules, "<modalias>usb:%s*</modalias>",
                              device_id);

      if (i < devices_list->len - 1)
        g_string_append_c (provided_modules, '\n');
    }

  g_print (METAINFO_BASE, provided_modules->str);
}
