import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))


def pytest_configure(config):
    config.addinivalue_line("markers", "slow: runs for half a minute or more (-m 'not slow' leaves them out)")
