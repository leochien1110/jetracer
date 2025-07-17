from .racecar import Racecar
import traitlets
from adafruit_servokit import ServoKit
import threading
import time
import logging


class NvidiaRacecar(Racecar):
    
    i2c_address = traitlets.Integer(default_value=0x40)
    steering_gain = traitlets.Float(default_value=-0.65)
    steering_offset = traitlets.Float(default_value=0.1)
    steering_channel = traitlets.Integer(default_value=0)
    throttle_gain = traitlets.Float(default_value=0.8)
    throttle_offset = traitlets.Float(default_value=-0.1)
    throttle_channel = traitlets.Integer(default_value=1)
    debug_enabled = traitlets.Bool(default_value=False)  # Control debug output
    
    def __init__(self, *args, **kwargs):
        super(NvidiaRacecar, self).__init__(*args, **kwargs)
        self.kit = ServoKit(channels=16, address=self.i2c_address)
        self.steering_motor = self.kit.continuous_servo[self.steering_channel]
        self.throttle_motor = self.kit.continuous_servo[self.throttle_channel]
        
        # Setup logging
        self.logger = logging.getLogger(f"{self.__class__.__name__}")
        if not self.logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter('%(name)s - %(levelname)s - %(message)s')
            handler.setFormatter(formatter)
            self.logger.addHandler(handler)
        
        # Set initial logging level based on debug_enabled
        self._update_logging_level()
        
        # ESC automatic direction change handling
        self._current_direction = 'neutral'  # 'forward', 'reverse', 'neutral'
        self._desired_throttle = 0.0
        self._actual_motor_throttle = 0.0
        self._direction_change_timer = None
        self._direction_change_lock = threading.Lock()
        self._is_changing_direction = False
        
        # Thresholds
        self.NEUTRAL_THRESHOLD = 0.05
        self.DIRECTION_CHANGE_DELAY = 0.2  # 0.5 seconds as requested
    
    @traitlets.observe('debug_enabled')
    def _on_debug_enabled_changed(self, change):
        """Update logging level when debug_enabled changes"""
        self._update_logging_level()
    
    def _update_logging_level(self):
        """Update the logging level based on debug_enabled setting"""
        if self.debug_enabled:
            self.logger.setLevel(logging.DEBUG)
        else:
            self.logger.setLevel(logging.WARNING)
    
    def enable_debug(self):
        """Enable debug logging output"""
        self.debug_enabled = True
        
    def disable_debug(self):
        """Disable debug logging output"""
        self.debug_enabled = False
    
    @traitlets.observe('steering')
    def _on_steering(self, change):
        self.steering_motor.throttle = change['new'] * self.steering_gain + self.steering_offset
    
    @traitlets.observe('throttle')
    def _on_throttle(self, change):
        new_throttle = change['new']
        self._desired_throttle = new_throttle
        
        # Handle automatic direction changes
        self._handle_automatic_direction_change(new_throttle)
    
    def _handle_automatic_direction_change(self, desired_throttle):
        """
        Automatically handle ESC direction changes by detecting when user wants
        to change direction and performing the brake -> neutral -> reverse sequence.
        """
        with self._direction_change_lock:
            # Determine desired direction
            if abs(desired_throttle) < self.NEUTRAL_THRESHOLD:
                desired_direction = 'neutral'
            elif desired_throttle > 0:
                desired_direction = 'forward'
            else:
                desired_direction = 'reverse'
            
            # Check if we need to change direction
            if (self._current_direction != desired_direction and 
                self._current_direction != 'neutral' and 
                desired_direction != 'neutral' and
                not self._is_changing_direction):
                
                self.logger.debug(f"🔄 Direction change detected: {self._current_direction} → {desired_direction}")
                self._start_direction_change_sequence(desired_throttle, desired_direction)
            else:
                # No direction change needed, apply throttle directly
                self._apply_direct_throttle(desired_throttle, desired_direction)
    
    def _start_direction_change_sequence(self, target_throttle, target_direction):
        """Start the automatic brake -> neutral -> reverse sequence"""
        self._is_changing_direction = True
        
        # Step 1: Apply brake (first reverse/forward command)
        brake_throttle = -0.15 if self._current_direction == 'forward' else 0.15
        brake_output = self._calculate_motor_output(brake_throttle)
        self._apply_motor_throttle(brake_output)
        self.logger.debug(f"🛑 AUTO-BRAKE: Applying brake throttle={brake_throttle:.3f} → output={brake_output:.3f}")
        
        # Step 2: Schedule neutral phase
        def neutral_phase():
            with self._direction_change_lock:
                if self._is_changing_direction:  # Check if still needed
                    neutral_output = self.throttle_offset
                    self._apply_motor_throttle(neutral_output)
                    self.logger.debug(f"⚠️  AUTO-NEUTRAL: ESC ready for direction change → output={neutral_output:.3f}")
                    
                    # Step 3: Schedule final direction change
                    final_timer = threading.Timer(self.DIRECTION_CHANGE_DELAY, self._complete_direction_change, 
                                                 args=[target_throttle, target_direction])
                    final_timer.start()
        
        # Start the sequence with neutral phase after delay
        self._direction_change_timer = threading.Timer(self.DIRECTION_CHANGE_DELAY, neutral_phase)
        self._direction_change_timer.start()
    
    def _complete_direction_change(self, target_throttle, target_direction):
        """Complete the direction change sequence"""
        with self._direction_change_lock:
            if self._is_changing_direction:  # Check if still needed
                self._current_direction = target_direction
                self._is_changing_direction = False
                
                # Apply the final target throttle
                output = self._calculate_motor_output(target_throttle)
                self._apply_motor_throttle(output)
                
                direction_emoji = "🚗" if target_direction == 'forward' else "⬅️"
                self.logger.debug(f"{direction_emoji} AUTO-{target_direction.upper()}: Direction change complete! "
                      f"throttle={target_throttle:.3f} → output={output:.3f}")
    
    def _apply_direct_throttle(self, throttle_value, direction):
        """Apply throttle directly without direction change sequence"""
        # Cancel any pending direction change if going to neutral
        if direction == 'neutral' and self._is_changing_direction:
            self._cancel_direction_change()
        
        # Update current direction if not neutral
        if direction != 'neutral':
            self._current_direction = direction
        
        output = self._calculate_motor_output(throttle_value)
        self._apply_motor_throttle(output)
        
        # Log the action
        if abs(throttle_value) < self.NEUTRAL_THRESHOLD:
            self.logger.debug(f"⏸️  NEUTRAL: output={output:.3f}")
        elif throttle_value > 0:
            self.logger.debug(f"🚗 FORWARD: throttle={throttle_value:.3f} → output={output:.3f}")
        else:
            self.logger.debug(f"⬅️  REVERSE: throttle={throttle_value:.3f} → output={output:.3f}")
    
    def _calculate_motor_output(self, throttle_value):
        """Calculate the actual motor output value"""
        if abs(throttle_value) < self.NEUTRAL_THRESHOLD:
            return self.throttle_offset
        else:
            return throttle_value * self.throttle_gain + self.throttle_offset
    
    def _apply_motor_throttle(self, motor_value):
        """Apply the calculated value directly to the motor"""
        self._actual_motor_throttle = motor_value
        self.throttle_motor.throttle = motor_value
    
    def _cancel_direction_change(self):
        """Cancel any pending direction change sequence"""
        if self._direction_change_timer:
            self._direction_change_timer.cancel()
        self._is_changing_direction = False
        self.logger.debug("🚫 Direction change cancelled")
    
    def auto_calibrate_throttle_direction(self):
        """
        Utility method to help calibrate throttle direction if needed.
        This method can be called if the throttle direction seems inverted.
        """
        self.logger.info("🔧 Auto-calibrating throttle direction...")
        self.logger.info(f"Current throttle_gain: {self.throttle_gain}")
        self.throttle_gain = -self.throttle_gain
        self.logger.info(f"New throttle_gain: {self.throttle_gain}")
        self.logger.info("✅ Calibration complete. Test forward/reverse again.")
    
    def get_esc_status(self):
        """Get current ESC status for debugging"""
        status = {
            'current_direction': self._current_direction,
            'desired_throttle': self._desired_throttle,
            'actual_motor_throttle': self._actual_motor_throttle,
            'is_changing_direction': self._is_changing_direction,
            'debug_enabled': self.debug_enabled
        }
        self.logger.debug(f"📊 ESC Status: {status}")
        return status
        