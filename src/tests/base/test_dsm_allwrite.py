# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import unittest

from src.objects.dsm_allwrite import dsmAllWrite


class TestDsmAllwrite(unittest.TestCase):
    """
    testing the dsm type member access methods

    """

    def setUp(self):
        self.awdsm = dsmAllWrite("x", int, 2, 0.0)

    def test_updated(self):
        self.assertEqual(self.awdsm.last_update(), 0.0, "testing last timestamp of aw update")

    def test_name(self):
        self.assertEqual(self.awdsm.name, "x", "testing allwrite name access")

    def test_dtype(self):
        self.assertEqual(self.awdsm.data_type, int, "testing allwrite name access")

    def test_value(self):
        self.assertEqual(self.awdsm.get_val(), 2, "testing allwrite value access")

    def test_last_update(self):
        self.assertEqual(self.awdsm.last_update(), 0.0, "testing allwrite update access")
        self.awdsm.set_update(1.0)
        self.assertEqual(self.awdsm.last_update(), 1.0, "testing allwrite update access")

    def test_val(self):
        self.assertEqual(self.awdsm.get_val(), 2, "testing allwrite value access")
        self.awdsm.set_val(1)
        self.assertEqual(self.awdsm.get_val(), 1, "testing allwrite value access")


if __name__ == '__main__':
    unittest.main()
