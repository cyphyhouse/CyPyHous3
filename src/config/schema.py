#!/usr/bin/env python3
import sys

import yamale

# TODO: refine validation schema. using yamale.

"""
commandline validation of config. 
"""

if len(sys.argv) < 3:
    print("usage: ./schema.py <schema_file_path> <validation_file_path>")
else:
    schemafile = sys.argv[1]
    yamlfile = sys.argv[2]
    schema = yamale.make_schema(schemafile)
    data = yamale.make_data(yamlfile)
    yamale.validate(schema, data)
