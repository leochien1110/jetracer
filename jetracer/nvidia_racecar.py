from .racecar import Racecar
import traitlets
from adafruit_servokit import ServoKit


class NvidiaRacecar(Racecar):
    
    i2c_address = traitlets.Integer(default_value=0x40)
    steering_gain = traitlets.Float(default_value=-0.65)
    steering_offset = traitlets.Float(default_value=0.1)
    steering_channel = traitlets.Integer(default_value=0)
    throttle_gain = traitlets.Float(default_value=0.8)
    throttle_offset = traitlets.Float(default_value=-0.1)
    throttle_channel = traitlets.Integer(default_value=1)
    
    def __init__(self, *args, **kwargs):
        super(NvidiaRacecar, self).__init__(*args, **kwargs)
        self.kit = ServoKit(channels=16, address=self.i2c_address)
        self.steering_motor = self.kit.continuous_servo[self.steering_channel]
        self.throttle_motor = self.kit.continuous_servo[self.throttle_channel]
        
        # ESC state tracking for double-click reverse
        self._last_throttle = 0.0
        self._esc_state = 'neutral'  # 'forward', 'neutral', 'brake_ready', 'reverse'
        self._brake_initiated = False
    
    @traitlets.observe('steering')
    def _on_steering(self, change):
        self.steering_motor.throttle = change['new'] * self.steering_gain + self.steering_offset
    
    @traitlets.observe('throttle')
    def _on_throttle(self, change):
        new_throttle = change['new']
        
        # Handle ESC double-click reverse logic
        output_throttle = self._handle_esc_logic(new_throttle)
        
        # Apply the calculated throttle to the motor
        self.throttle_motor.throttle = output_throttle
        
        # Update last throttle for next iteration
        self._last_throttle = new_throttle
    
    def _handle_esc_logic(self, throttle_value):
        """
        Handle ESC double-click reverse logic according to Waveshare documentation:
        1. First negative throttle -> Brake only
        2. Return to neutral (0) 
        3. Second negative throttle -> Actually reverse
        """
        
        # Threshold for considering throttle as neutral
        NEUTRAL_THRESHOLD = 0.05
        
        # Determine if we're in neutral, forward, or reverse zones
        is_neutral = abs(throttle_value) < NEUTRAL_THRESHOLD
        is_forward_command = throttle_value > NEUTRAL_THRESHOLD
        is_reverse_command = throttle_value < -NEUTRAL_THRESHOLD
        
        was_neutral = abs(self._last_throttle) < NEUTRAL_THRESHOLD
        was_forward = self._last_throttle > NEUTRAL_THRESHOLD
        was_reverse = self._last_throttle < -NEUTRAL_THRESHOLD
        
        if is_forward_command:
            # Forward movement - reset ESC state
            self._esc_state = 'forward'
            self._brake_initiated = False
            output = throttle_value * self.throttle_gain + self.throttle_offset
            print(f"🚗 FORWARD: throttle={throttle_value:.3f} → output={output:.3f}")
            
        elif is_neutral:
            # Neutral - maintain current state but allow for reverse preparation
            if self._esc_state == 'forward' and self._brake_initiated:
                self._esc_state = 'brake_ready'
                print(f"⚠️  ESC READY FOR REVERSE (returned to neutral after brake)")
            elif self._esc_state not in ['brake_ready']:
                self._esc_state = 'neutral'
            self._brake_initiated = False
            output = self.throttle_offset  # Neutral position
            print(f"⏸️  NEUTRAL: output={output:.3f}")
            
        elif is_reverse_command:
            if self._esc_state in ['neutral', 'forward'] and not self._brake_initiated:
                # First reverse command - brake only
                self._esc_state = 'forward'  # Stay in forward state
                self._brake_initiated = True
                output = abs(throttle_value) * (-self.throttle_gain) + self.throttle_offset
                print(f"🛑 BRAKE (1st reverse): throttle={throttle_value:.3f} → output={output:.3f}")
                
            elif self._esc_state == 'brake_ready':
                # Second reverse command after returning to neutral - actual reverse
                self._esc_state = 'reverse'
                self._brake_initiated = False
                output = abs(throttle_value) * (-self.throttle_gain) + self.throttle_offset
                print(f"⬅️  REVERSE (2nd reverse): throttle={throttle_value:.3f} → output={output:.3f}")
                
            elif self._esc_state == 'reverse':
                # Already in reverse, continue reverse
                output = abs(throttle_value) * (-self.throttle_gain) + self.throttle_offset
                print(f"⬅️  REVERSE (continue): throttle={throttle_value:.3f} → output={output:.3f}")
                
            else:
                # Fallback - treat as brake
                output = abs(throttle_value) * (-self.throttle_gain) + self.throttle_offset
                print(f"🛑 BRAKE (fallback): throttle={throttle_value:.3f} → output={output:.3f}")
        
        else:
            # Fallback for edge cases
            output = self.throttle_offset
            print(f"❓ UNKNOWN STATE: throttle={throttle_value:.3f} → output={output:.3f}")
        
        return output
    
    def auto_calibrate_throttle_direction(self):
        """
        Utility method to help calibrate throttle direction if needed.
        This method can be called if the throttle direction seems inverted.
        """
        print("🔧 Auto-calibrating throttle direction...")
        print("Current throttle_gain:", self.throttle_gain)
        self.throttle_gain = -self.throttle_gain
        print("New throttle_gain:", self.throttle_gain)
        print("✅ Calibration complete. Test forward/reverse again.")
        