# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import unittest

from src.datatypes.var_types import ShareType
from src.objects.dsm import DSM


class TestDsm(unittest.TestCase):
    """
    testing the dsm type member access methods

    """

    def setUp(self):
        self.awdsm = DSM("x", int, 2, 1)
        self.ardsm = DSM("y", int, 2, 0, 1, ShareType.ALL_READ)

    def test_pid(self):
        self.assertEqual(self.awdsm.pid, 1, "testing pid access")
        self.assertEqual(self.ardsm.pid, 0, "testing pid access")

    def test_updated(self):
        self.assertEqual(self.awdsm.last_update(), 0.0, "testing last timestamp of aw update")
        self.assertEqual(self.ardsm.last_update(0), 0.0, "testing last timestamp ar update")

    def test_name(self):
        self.assertEqual(self.awdsm.name, "x", "testing allwrite name access")
        self.assertEqual(self.ardsm.name, "y", "testing allread name access")

    def test_dtype(self):
        self.assertEqual(self.awdsm.data_type, int, "testing allwrite name access")
        self.assertEqual(self.ardsm.data_type, int, "testing allread name access")

    def test_value(self):
        self.assertEqual(self.awdsm.get_val(), None, "testing allwrite value access")
        self.assertEqual(self.ardsm.get_val(0), 1, "testing allread value access")

    def test_owner(self):
        self.assertEqual(self.awdsm.owner, ShareType.ALL_WRITE, "testing allwrite owner access")
        self.assertEqual(self.ardsm.owner, ShareType.ALL_READ, "testing allread owner access")

    def test_last_update(self):
        self.assertEqual(self.awdsm.last_update(), 0.0, "testing allwrite owner access")
        self.awdsm.set_update(1.0)
        self.assertEqual(self.awdsm.last_update(), 1.0, "testing allwrite owner access")
        self.assertEqual(self.ardsm.last_update(0), 0.0, "testing allread owner access")
        self.ardsm.set_update(1.0, 0)
        self.assertEqual(self.ardsm.last_update(0), 1.0, "testing alread owner access")

    def test_val(self):
        self.assertEqual(self.awdsm.get_val(), None, "testing allwrite owner access")
        self.awdsm.set_val(1)
        self.assertEqual(self.awdsm.get_val(), 1, "testing allwrite owner access")
        self.assertEqual(self.ardsm.get_val(0), 1, "testing allread owner access")
        self.ardsm.set_val(2, 0)
        self.assertEqual(self.ardsm.get_val(0), 2, "testing allread owner access")


if __name__ == '__main__':
    unittest.main()
