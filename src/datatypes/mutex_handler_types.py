#  Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import src.functionality.basicMutexHandler as bmh

# dictionary to store type of mutex associated with each handler

mutex_types = dict()
mutex_types[bmh.BasicMutexHandler] = bmh.BasicMutex
