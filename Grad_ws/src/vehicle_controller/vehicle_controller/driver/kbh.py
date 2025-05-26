from pynput import keyboard

class KeyboardHandler:
    """
    Handles keyboard input and stores key states using bitwise operations.
    
    ### Usage  
    - `keyboard_handler.get_binary_state()`  # Get the pressed keys in binary form (up down left right)
    - Functions for key events can be passed during initialization.
    """

    KEY_MAP = {
        keyboard.Key.up   : 1 << 3, # 1000 (8)
        keyboard.Key.down : 1 << 2, # 0100 (4)
        keyboard.Key.left : 1 << 1, # 0010 (2)
        keyboard.Key.right: 1 << 0  # 0001 (1)
    }

    def __init__(self, up_func=lambda: None, down_func=lambda: None, left_func=lambda: None, right_func=lambda: None, 
                 neutral_ud_func=lambda: None, neutral_lr_func=lambda: None):

        self.pressed_keys = 0  # Stores key states as a 4-bit integer
        self.func_map = {
            keyboard.Key.up: up_func,
            keyboard.Key.down: down_func,
            keyboard.Key.left: left_func,
            keyboard.Key.right: right_func
        }
        self.neutral_ud_func = neutral_ud_func  # No UP or DOWN pressed
        self.neutral_lr_func = neutral_lr_func  # No LEFT or RIGHT pressed

        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()  # Non-blocking

    def on_press(self, key):
        if key in self.KEY_MAP:
            key_bit = self.KEY_MAP[key]
            if not (self.pressed_keys & key_bit):  # If key is NOT already pressed
                self.pressed_keys |= key_bit  # Mark key as pressed
                self.process_action(key,1)

    def on_release(self, key):
        if key in self.KEY_MAP:
            self.pressed_keys &= ~self.KEY_MAP[key]  # Clear the corresponding bit
            self.process_action(key,0)

    def process_action(self, key, state):
        """
        state: press(1) or release(0)
        """

        if key in (keyboard.Key.up, keyboard.Key.down):
            is_both_ud = int(bool(self.pressed_keys & self.KEY_MAP[keyboard.Key.up]) == bool(self.pressed_keys & self.KEY_MAP[keyboard.Key.down]))
            if is_both_ud:
                self.neutral_ud_func()          # Reset
            elif state:
                self.func_map[key]()            # call corresponding function
            else:
                other_key = keyboard.Key.down if key == keyboard.Key.up else keyboard.Key.up   # swap keys
                self.func_map[other_key]()      # call other function
        else:
            is_both_lr = int(bool(self.pressed_keys & self.KEY_MAP[keyboard.Key.left]) == bool(self.pressed_keys & self.KEY_MAP[keyboard.Key.right]))
            if is_both_lr:
                self.neutral_lr_func()
            elif state:
                self.func_map[key]()
            else:
                other_key = keyboard.Key.right if key == keyboard.Key.left else keyboard.Key.left
                self.func_map[other_key]()

    def get_binary_state(self):
        return format(self.pressed_keys, '04b')         # Convert integer to binary string
