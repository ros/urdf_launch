import subprocess
import xml.dom.minidom

from launch import LaunchContext
from launch.frontend import Parser
from launch.substitutions import SubstitutionFailure
from launch.utilities import perform_substitutions

import pytest

from urdf_launch import XacroFile

ROBOT = """<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot">
  <xacro:arg name="prefix" default=""/>
  <xacro:arg name="rpy" default="0 0 0"/>
  <xacro:include filename="link.xacro"/>
  <xacro:link name="$(arg prefix)base_link"/>
  <joint name="$(arg prefix)joint" type="fixed">
    <origin rpy="$(arg rpy)"/>
  </joint>
</robot>
"""

LINK = """<?xml version="1.0"?>
<root xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="link" params="name"><link name="${name}"/></xacro:macro>
</root>
"""


@pytest.fixture
def robot(tmp_path):
    (tmp_path / 'link.xacro').write_text(LINK)
    path = tmp_path / 'robot.urdf.xacro'
    path.write_text(ROBOT)
    return path


def substitute(text):
    Parser.load_launch_extensions()  # as the frontends: by the entry point
    return perform_substitutions(LaunchContext(), Parser().parse_substitution(text))


def links_and_rpy(text):
    doc = xml.dom.minidom.parseString(text)
    links = [e.getAttribute('name') for e in doc.getElementsByTagName('link')]
    return links, doc.getElementsByTagName('origin')[0].getAttribute('rpy')


def test_defaults(robot):
    assert links_and_rpy(substitute(f'$(xacro-file {robot})')) == (['base_link'], '0 0 0')


def test_arguments(robot):
    text = substitute(f"$(xacro-file {robot} prefix:=a__ 'rpy:=0 0 1')")
    assert links_and_rpy(text) == (['a__base_link'], '0 0 1')


def test_python(robot):
    text = perform_substitutions(LaunchContext(), [XacroFile(str(robot), ['prefix:=b__'])])
    assert links_and_rpy(text)[0] == ['b__base_link']


def test_same_as_command(robot):
    command = subprocess.run(['xacro', str(robot), 'prefix:=c__'], capture_output=True, text=True, check=True)
    assert substitute(f'$(xacro-file {robot} prefix:=c__)') == command.stdout


def test_errors(tmp_path, robot):
    with pytest.raises(SubstitutionFailure, match='missing.xacro'):
        substitute(f'$(xacro-file {tmp_path / "missing.xacro"})')
    with pytest.raises(SubstitutionFailure, match='name:=value'):
        substitute(f'$(xacro-file {robot} prefix)')
    broken = tmp_path / 'broken.xacro'
    broken.write_text('<robot>')
    with pytest.raises(SubstitutionFailure, match='XML parsing'):
        substitute(f'$(xacro-file {broken})')
    undefined = tmp_path / 'undefined.xacro'
    undefined.write_text('<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><link name="${undefined_name}"/></robot>')
    with pytest.raises(SubstitutionFailure, match='undefined_name'):
        substitute(f'$(xacro-file {undefined})')
