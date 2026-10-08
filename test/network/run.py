#!/usr/bin/env python3
"""Compile actual firmware modules against host doubles; no hardware/network writes."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
root = Path(__file__).resolve().parents[2]
test = Path(__file__).resolve().parent
flags = [os.environ.get('CXX', 'clang++'), '-I'+str(test/'stubs'), '-I'+str(root.parent/'MeshProtocol/src'), '-DMESH_TEST_TOKENS', '-std=c++17', '-Wall', '-Wextra',
         '-I'+str(test/'stubs'), '-I'+str(root/'include'),
         '-I'+str(root/'.pio/libdeps/receiver/ArduinoJson/src')]
if sys.platform == 'darwin':
    sdk = subprocess.check_output(['xcrun', '--show-sdk-path'], text=True).strip()
    flags += ['-isystem', sdk+'/usr/include/c++/v1']
with tempfile.TemporaryDirectory(prefix='lora-network-test-') as out:
    for name, defines in [('network_test', ['TEST_NETWORK_RECOVERY', 'SENDER']),
                          ('network_test', ['TEST_NETWORK_RECOVERY', 'RECEIVER']),
                          ('mqtt_test', ['TEST_PRIMARY_MQTT', 'RECEIVER'])]:
        binary = str(Path(out)/name)
        subprocess.run(flags + ['-D'+d for d in defines] + [str(test/(name+'.cpp')), str(root.parent/'MeshProtocol/src/registry.cpp'), '-o', binary], check=True)
        subprocess.run([binary], check=True)
