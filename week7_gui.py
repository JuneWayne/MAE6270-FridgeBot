"""
loads gui 
reset control slider values to zero when pressing stop button
"""

from pathlib import Path
import sys

from PyQt5 import uic
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QApplication, QMainWindow


class WheelControlWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi(str(Path(__file__).with_name("week7_gui.ui")), self)
        self.StopButton.clicked.connect(self.stop_controls)
        self.statusbar.showMessage("Motor interface: not connected")

    @pyqtSlot()
    # define function that sets all speed values to zero
    def stop_controls(self):
        self.LeftValue.setValue(0)
        self.RightValue.setValue(0)
        self.statusbar.showMessage(
            "wheel speed set to zero"
        )


def main():
    app = QApplication(sys.argv)
    window = WheelControlWindow()
    window.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
