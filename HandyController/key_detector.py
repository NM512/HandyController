from multiprocessing import Process, Array, parent_process
import time
import sys
import keyboard
from pynput import keyboard

class KeyDetector:
    def __init__(self, target_keys=('g', 'h', 'j', 'b', 'n', 'm')):
        self.target_keycode_dict = {keyboard.KeyCode.from_char(key):i for i, key in enumerate(target_keys)}
        self.flags = Array('b', len(target_keys))
        self.p = Process(target=self._start_process, args=(self.flags,))
        self.p.start()
        # use this because daemon option doesn't work with ESC in robogym env
        self.p_check = Process(target=self._kill_daemons)
        self.p_check.start()

    def _start_process(self, flags):
        with keyboard.Listener(on_press=lambda event: self._on_press(event, flags),
                               on_release=lambda event: self._on_release(event, flags)) as listener:
            listener.join()

    def _on_press(self, keycode, flags):
        if keycode in self.target_keycode_dict:
            flags[self.target_keycode_dict[keycode]] = True

    def _on_release(self, keycode, flags):
        try:
            flags[self.target_keycode_dict[keycode]] = False
        except KeyError:
            pass

    def read_keys(self):
        return list(self.flags)

    def _kill_daemons(self):
        while True:
            if not parent_process().is_alive():
                self.p.terminate()
                sys.exit(0)

if __name__ == "__main__":
    key_det = KeyDetector()
    while True:
        time.sleep(0.1)
        print(key_det.read_keys())
