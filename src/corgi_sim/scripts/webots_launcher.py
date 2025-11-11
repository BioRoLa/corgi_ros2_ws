#!/usr/bin/env python3

# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
This launcher simply starts Webots.
ROS2 version - updated for ament/colcon packages.
"""

import optparse
import os
import sys
import subprocess
from ament_index_python.packages import get_package_share_directory


# Set up WEBOTS_PROJECT_PATH and workspace prefix
try:
    corgi_sim_share = get_package_share_directory('corgi_sim')
    # install prefix is two levels up from share dir: <prefix>/share/corgi_sim
    install_prefix = os.path.dirname(os.path.dirname(corgi_sim_share))
    # Ensure WEBOTS_PROJECT_PATH contains our package share
    if 'WEBOTS_PROJECT_PATH' not in os.environ:
        os.environ['WEBOTS_PROJECT_PATH'] = corgi_sim_share
    else:
        os.environ['WEBOTS_PROJECT_PATH'] += f":{corgi_sim_share}"
except Exception as e:
    print(f"Warning: Could not set WEBOTS_PROJECT_PATH or detect install prefix: {e}")
    install_prefix = None

optParser = optparse.OptionParser()
optParser.add_option("--world", dest="world", default="", help="Path to the world to load.")
optParser.add_option("--mode", dest="mode", default="realtime", help="Startup mode.")
optParser.add_option("--no-gui", dest="noGui", default="true", help="Start Webots with minimal GUI.")
optParser.add_option("--stream", dest="stream", default="false", help="Start Webots streaming server.")
options, args = optParser.parse_args()

if 'WEBOTS_HOME' not in os.environ:
    sys.exit('WEBOTS_HOME environment variable not defined.')

webots_bin = os.path.join(os.environ['WEBOTS_HOME'], 'webots')
webots_cmd = f'"{webots_bin}" --mode={options.mode} "{options.world}"'

if options.stream.lower() != 'false':
    if options.stream.lower() == 'true':
        webots_cmd += ' --stream="port=1234;mode=x3d;monitorActivity"'
    else:
        webots_cmd += ' --stream="' + options.stream + '"'

if options.noGui.lower() == 'true':
    webots_cmd += ' --stdout --stderr --batch --minimize'

# Launch webots ensuring ROS 2 environment is sourced so the controller can import messages/services
if install_prefix and os.path.exists(os.path.join(install_prefix, 'setup.bash')):
    bash_cmd = f'source "{install_prefix}/setup.bash" && export WEBOTS_PROJECT_PATH="{os.environ.get("WEBOTS_PROJECT_PATH", "")}" && exec {webots_cmd}'
    subprocess.call(['/bin/bash', '-lc', bash_cmd])
else:
    # Fallback: run directly without sourcing (may fail to import custom msgs)
    subprocess.call(webots_cmd, shell=True)
