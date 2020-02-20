# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import unittest

from src.objects.dsm_allread import dsmAllRead


class TestDsmAllRead(unittest.TestCase):
    """
    testing the dsm type member access methods

    """

    def setUp(self):
        self.ardsm = dsmAllRead("x", int, 5, 1, 1, 1.0)

    def test_pid(self):
        self.assertEqual(self.ardsm.pid, 1, "testing pid access")

    def test_updated(self):
        self.assertEqual(self.ardsm.last_update(1), 1.0, "testing last timestamp ar update")

    def test_name(self):
        self.assertEqual(self.ardsm.name, "x", "testing allread name access")

    def test_dtype(self):
        self.assertEqual(self.ardsm.data_type, int, "testing allread datatype access")

    def test_value(self):
        self.assertEqual(self.ardsm.get_val(1), 1, "testing allread value access")

    def test_last_update(self):
        self.assertEqual(self.ardsm.last_update(1), 1.0, "testing allread owner access")
        self.ardsm.set_update(2.0, 0)
        self.assertEqual(self.ardsm.last_update(1), 1.0, "testing alread owner access")

    def test_val(self):
        self.assertEqual(self.ardsm.get_val(1), 1, "testing allread owner access")
        self.ardsm.set_val(2, 1)
        self.assertEqual(self.ardsm.get_val(1), 2, "testing allread owner access")


if __name__ == '__main__':
    unittest.main()
