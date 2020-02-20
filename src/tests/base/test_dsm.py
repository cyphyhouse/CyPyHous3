# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import unittest

from src.objects.abstract.dsm import dsm



class DummyDsm(dsm):

    def __init__(self, name:str, dtype:type):
        super(dsm, self).__init__()
        self.name = name
        self.data_type = dtype

    def last_update(self):
        pass

    def set_update(self, **kwargs) -> None:
        pass

    def get_val(self):
        pass

    def set_val(self, **kwargs) -> None:
        pass


class TestDsm(unittest.TestCase):
    """
    testing the dsm type member access methods

    """

    def setUp(self):
        self.dsm = DummyDsm("x", int)


    def test_name(self):
        self.assertEqual(self.dsm.name, "x", "testing name access")

    def test_dtype(self):
        self.assertEqual(self.dsm.data_type, int, "testing type access")



if __name__ == '__main__':
    unittest.main()