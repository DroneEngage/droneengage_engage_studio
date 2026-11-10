"""
Sender-permission ("p") parse tests - the Python mirror of the C++
fn_getSenderPermission checks added in DE-0136 (commit 5406f97).

The comm server writes "p" as a JSON number, but the JSON type is not
always unsigned: a signed value, a float or a hex string all come from
account stores and server builds. Each form must give the same 32-bit
mask. A missing or non-numeric "p" gives 0 (deny). The old code only
accepted an unsigned int, so the other forms silently became 0 and
every RemoteExecute was denied - the airgap camera-dialog bug.

These tests do not need a bus. They drive _read_sender_permission and
the parse_message permission path directly.

Run: python3 -m unittest test_permission_parse -v   (from de_databus/python/)
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from de_message_parser_base import AndruavMessageParserBase
from messages import (ANDRUAV_PROTOCOL_MESSAGE_PERMISSION,
                      ANDRUAV_PROTOCOL_SENDER,
                      ANDRUAV_PROTOCOL_MESSAGE_CMD,
                      ANDRUAV_PROTOCOL_MESSAGE_TYPE,
                      ANDRUAV_PROTOCOL_SENDER_COMM_SERVER,
                      TYPE_AndruavMessage_RemoteExecute)


class _StubParser(AndruavMessageParserBase):
    """Concrete subclass so the abstract base can be exercised."""
    def __init__(self):
        super().__init__()
        self.remote_execute_calls = []
        self.command_calls = []

    def parse_remote_execute(self, andruav_message):
        self.remote_execute_calls.append(andruav_message)

    def parse_command(self, andruav_message, full_message, full_message_length,
                      message_type, permission):
        self.command_calls.append((message_type, permission))


class TestSenderPermission(unittest.TestCase):
    """Mirror of the DE-0136 block in test_permission_map.cpp."""

    def _parser(self):
        return _StubParser()

    # ── _read_sender_permission: the four accepted forms ──────────────

    def test_unsigned_int_form(self):
        p = self._parser()
        self.assertEqual(p._read_sender_permission(
            {ANDRUAV_PROTOCOL_MESSAGE_PERMISSION: 0xffffffff}), 0xffffffff)

    def test_signed_int_form(self):
        p = self._parser()
        # -1 as a signed int is the same 32-bit mask as 0xffffffff.
        self.assertEqual(p._read_sender_permission(
            {ANDRUAV_PROTOCOL_MESSAGE_PERMISSION: -1}), 0xffffffff)

    def test_signed_min_maps_to_bit_31(self):
        p = self._parser()
        self.assertEqual(p._read_sender_permission(
            {ANDRUAV_PROTOCOL_MESSAGE_PERMISSION: -2147483648}), 0x80000000)

    def test_float_form(self):
        p = self._parser()
        self.assertEqual(p._read_sender_permission(
            {ANDRUAV_PROTOCOL_MESSAGE_PERMISSION: 4294967295.0}), 0xffffffff)

    def test_hex_string_form(self):
        p = self._parser()
        self.assertEqual(p._read_sender_permission(
            {ANDRUAV_PROTOCOL_MESSAGE_PERMISSION: "0xffffffff"}), 0xffffffff)

    def test_decimal_string_form(self):
        p = self._parser()
        self.assertEqual(p._read_sender_permission(
            {ANDRUAV_PROTOCOL_MESSAGE_PERMISSION: "4294967295"}), 0xffffffff)

    # ── _read_sender_permission: deny forms ───────────────────────────

    def test_missing_p_is_zero(self):
        p = self._parser()
        self.assertEqual(p._read_sender_permission({}), 0)

    def test_junk_string_is_zero(self):
        p = self._parser()
        self.assertEqual(p._read_sender_permission(
            {ANDRUAV_PROTOCOL_MESSAGE_PERMISSION: "junk"}), 0)

    def test_bool_is_zero_not_one(self):
        # bool is a subclass of int in Python; a JSON true must not
        # become mask 1.
        p = self._parser()
        self.assertEqual(p._read_sender_permission(
            {ANDRUAV_PROTOCOL_MESSAGE_PERMISSION: True}), 0)
        self.assertEqual(p._read_sender_permission(
            {ANDRUAV_PROTOCOL_MESSAGE_PERMISSION: False}), 0)

    def test_none_value_is_zero(self):
        p = self._parser()
        self.assertEqual(p._read_sender_permission(
            {ANDRUAV_PROTOCOL_MESSAGE_PERMISSION: None}), 0)

    def test_negative_float_is_zero(self):
        p = self._parser()
        self.assertEqual(p._read_sender_permission(
            {ANDRUAV_PROTOCOL_MESSAGE_PERMISSION: -1.0}), 0)

    def test_nan_float_is_zero(self):
        p = self._parser()
        self.assertEqual(p._read_sender_permission(
            {ANDRUAV_PROTOCOL_MESSAGE_PERMISSION: float('nan')}), 0)

    def test_non_dict_envelope_is_zero(self):
        p = self._parser()
        self.assertEqual(p._read_sender_permission("not a dict"), 0)

    # ── parse_message threads the parsed mask to parse_command ───────
    # This is the regression guard: before the fix a float/hex "p" made
    # parse_command receive 0 (deny). Now it receives the real mask.

    def _envelope(self, permission_value):
        env = {ANDRUAV_PROTOCOL_SENDER: "gcs_1",
               ANDRUAV_PROTOCOL_MESSAGE_TYPE: TYPE_AndruavMessage_RemoteExecute,
               ANDRUAV_PROTOCOL_MESSAGE_CMD: {"C": 7}}
        if permission_value is not None:
            env[ANDRUAV_PROTOCOL_MESSAGE_PERMISSION] = permission_value
        return env

    def _permission_seen_by_parse_command(self, permission_value):
        p = self._parser()
        env = self._envelope(permission_value)
        # RemoteExecute is intercepted by parse_remote_execute before
        # parse_command runs, so use a non-RemoteExecute type to observe
        # the mask that parse_command would gate on.
        env[ANDRUAV_PROTOCOL_MESSAGE_TYPE] = 1020  # DroneReport: not RemoteExecute
        p.parse_message(env, "{}", 2)
        self.assertEqual(len(p.command_calls), 1)
        return p.command_calls[0][1]

    def test_parse_message_unsigned_threads_through(self):
        self.assertEqual(
            self._permission_seen_by_parse_command(0xffffffff), 0xffffffff)

    def test_parse_message_signed_threads_through(self):
        # Before the fix this was 0 (the bug). Now it is the full mask.
        self.assertEqual(
            self._permission_seen_by_parse_command(-1), 0xffffffff)

    def test_parse_message_float_threads_through(self):
        # Before the fix this was 0 (the bug). Now it is the full mask.
        self.assertEqual(
            self._permission_seen_by_parse_command(4294967295.0), 0xffffffff)

    def test_parse_message_hex_string_threads_through(self):
        # Before the fix this was 0 (the bug). Now it is the full mask.
        self.assertEqual(
            self._permission_seen_by_parse_command("0xffffffff"), 0xffffffff)

    def test_parse_message_missing_p_threads_zero(self):
        self.assertEqual(self._permission_seen_by_parse_command(None), 0)

    def test_parse_message_junk_string_threads_zero(self):
        self.assertEqual(
            self._permission_seen_by_parse_command("junk"), 0)

    # ── comm-server sender flag is independent of "p" ────────────────
    # Regression guard: the _is_system flag must not depend on the "p"
    # form. (The C++ comm-server bypass keys on sd, not p.)

    def test_system_flag_set_for_comm_server_sender(self):
        p = self._parser()
        env = {ANDRUAV_PROTOCOL_SENDER: ANDRUAV_PROTOCOL_SENDER_COMM_SERVER,
               ANDRUAV_PROTOCOL_MESSAGE_TYPE: 1020,
               ANDRUAV_PROTOCOL_MESSAGE_PERMISSION: 0}
        p.parse_message(env, "{}", 2)
        self.assertTrue(p.is_system)

    def test_system_flag_clear_for_unit_sender(self):
        p = self._parser()
        env = {ANDRUAV_PROTOCOL_SENDER: "unit_1",
               ANDRUAV_PROTOCOL_MESSAGE_TYPE: 1020,
               ANDRUAV_PROTOCOL_MESSAGE_PERMISSION: 0xffffffff}
        p.parse_message(env, "{}", 2)
        self.assertFalse(p.is_system)


if __name__ == "__main__":
    unittest.main()
