"""Worlds and models must conform to the official sdformat schema. Needs the schema: set SDFORMAT_SDF_DIR to a checkout of
https://github.com/gazebosim/sdformat/tree/main/sdf/1.9 (CI does this); skipped otherwise."""
import glob
import os
import sys
import xml.etree.ElementTree as ET

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, 'mobile_robot_gazebo', 'tools'))
import sdf_lint  # noqa: E402

SCHEMA_DIR = os.environ.get('SDFORMAT_SDF_DIR', '')
pytestmark = pytest.mark.skipif(not os.path.isfile(os.path.join(SCHEMA_DIR, 'root.sdf')), reason='SDFORMAT_SDF_DIR not set')
FILES = sorted(glob.glob(os.path.join(REPO, 'mobile_robot_gazebo', 'worlds', '*.sdf')) +
               glob.glob(os.path.join(REPO, 'mobile_robot_gazebo', 'models', '*', 'model.sdf')))


@pytest.fixture(scope='module')
def schema():
    return sdf_lint.load_schema(SCHEMA_DIR)


@pytest.mark.parametrize('path', FILES, ids=lambda p: os.path.relpath(p, REPO))
def test_file_conforms_to_schema(path, schema):
    assert sdf_lint.lint(path, schema) == []


def test_linter_catches_planted_errors(schema, tmp_path):
    bad = tmp_path / 'bad.sdf'
    bad.write_text('<sdf version="1.9"><world name="w"><physics name="p" type="dart"><bogus_tag>1</bogus_tag></physics>'
                   '<model name="m"><link name="l"><collision name="c"><geometry><box><size>1 1 1</size></box></geometry>'
                   '<surface><friction><ode><mu3>2</mu3></ode></friction></surface></collision></link></model></world></sdf>')
    found = ' '.join(sdf_lint.lint(str(bad), schema))
    assert 'bogus_tag' in found and 'mu3' in found
