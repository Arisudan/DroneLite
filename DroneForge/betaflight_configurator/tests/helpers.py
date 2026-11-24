import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from server.cli import Cli  # noqa: E402
from server.controller import Controller  # noqa: E402
from server.emulator import Emulator  # noqa: E402
from server.link import Link  # noqa: E402
from server.safety import Safety  # noqa: E402


class Rig:
    """Emulator + Link (MSP open). Only the emulator pty is ever opened, never a real port."""

    def __init__(self, armed_fn=None, reboot_delay=0.4):
        self.tmp = tempfile.mkdtemp(prefix="bftest")
        self.emu = Emulator(path=os.path.join(self.tmp, "ttyBFEMU"), reboot_delay=reboot_delay)
        self.link = Link()
        self.link.open(self.emu.path)
        self.cli = Cli(self.link)
        self.safety = Safety(os.path.join(self.tmp, "backups"), armed_fn=armed_fn)

    def close(self):
        self.link.close()
        self.emu.stop()
