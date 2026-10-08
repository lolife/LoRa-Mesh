#!/usr/bin/env python3
"""Host checks for the firmware wire contracts and actual TX worker."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
root = Path(__file__).resolve().parents[2]
test = Path(__file__).resolve().parent
flags = [os.environ.get('CXX', 'clang++'), '-I'+str(test/'stubs'), '-I'+str(root.parent/'MeshProtocol/src'), '-DMESH_TEST_TOKENS', '-std=c++17', '-Wall', '-Wextra',
         '-I'+str(test/'stubs'), '-I'+str(root/'include')]
if sys.platform == 'darwin':
    sdk = subprocess.check_output(['xcrun', '--show-sdk-path'], text=True).strip()
    flags += ['-isystem', sdk+'/usr/include/c++/v1']
with tempfile.TemporaryDirectory(prefix='lora-mesh-test-') as out:
    for name in ['protocol_test', 'tx_test', 'mesh_test', 'display_motion_test']:
        binary = str(Path(out)/name)
        subprocess.run(flags + ['-D'+('TEST_MESH' if name == 'mesh_test' else 'TEST_TX_WORKER'), str(test/(name+'.cpp')), str(root.parent/'MeshProtocol/src/registry.cpp'), '-o', binary], check=True)
        subprocess.run([binary], check=True)
