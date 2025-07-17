git #!/usr/bin/env python3
"""
JetRacer Calibration Script

This script provides a command-line interface for calibrating JetRacer
steering and throttle parameters.
"""

import argparse
import sys
import time

def main():
    """Main calibration function"""
    parser = argparse.ArgumentParser(description='JetRacer Calibration Tool')
    parser.add_argument('--steering', action='store_true', 
                       help='Calibrate steering parameters')
    parser.add_argument('--throttle', action='store_true',
                       help='Calibrate throttle parameters')
    parser.add_argument('--debug', action='store_true',
                       help='Enable debug output')
    parser.add_argument('--dry-run', action='store_true',
                       help='Show what would be done without actually moving motors')
    
    args = parser.parse_args()
    
    if not (args.steering or args.throttle):
        print("Please specify --steering or --throttle (or both)")
        return 1
    
    try:
        from jetracer import NvidiaRacecar
        
        if args.dry_run:
            print("DRY RUN MODE - No motors will be moved")
            car = None
        else:
            print("Initializing JetRacer...")
            car = NvidiaRacecar()
            if args.debug:
                car.enable_debug()
        
        if args.steering:
            calibrate_steering(car, args.dry_run)
        
        if args.throttle:
            calibrate_throttle(car, args.dry_run)
            
        print("✅ Calibration complete!")
        return 0
        
    except ImportError as e:
        print(f"❌ Error importing jetracer: {e}")
        print("Make sure jetracer is properly installed")
        return 1
    except Exception as e:
        print(f"❌ Calibration error: {e}")
        return 1

def calibrate_steering(car, dry_run=False):
    """Calibrate steering parameters"""
    print("\n🔧 Steering Calibration")
    print("=" * 30)
    
    if dry_run:
        print("Would test steering range: -1.0 to +1.0")
        print("Would check for proper left/right movement")
        return
    
    if car is None:
        return
        
    print("Testing steering range...")
    print("The wheels should turn left, then right, then center")
    
    # Test sequence
    car.steering = -1.0  # Full left
    print("🔄 Full left")
    time.sleep(2)
    
    car.steering = 1.0   # Full right  
    print("🔄 Full right")
    time.sleep(2)
    
    car.steering = 0.0   # Center
    print("🔄 Center")
    time.sleep(1)
    
    print("Current steering parameters:")
    print(f"  steering_gain: {car.steering_gain}")
    print(f"  steering_offset: {car.steering_offset}")

def calibrate_throttle(car, dry_run=False):
    """Calibrate throttle parameters"""
    print("\n🔧 Throttle Calibration")
    print("=" * 30)
    
    if dry_run:
        print("Would test forward/reverse movement")
        print("Would test automatic ESC direction changes")
        return
        
    if car is None:
        return
    
    print("⚠️  Make sure the car has plenty of space!")
    print("Testing throttle - forward, then reverse")
    
    # Test forward
    print("🚗 Testing forward...")
    car.throttle = 0.3
    time.sleep(2)
    car.throttle = 0.0
    time.sleep(1)
    
    # Test reverse (automatic ESC handling)
    print("⬅️  Testing reverse (automatic ESC sequence)...")
    car.throttle = -0.3
    time.sleep(3)  # Give time for automatic sequence
    car.throttle = 0.0
    
    print("Current throttle parameters:")
    print(f"  throttle_gain: {car.throttle_gain}")
    print(f"  throttle_offset: {car.throttle_offset}")
    print(f"  debug_enabled: {car.debug_enabled}")

if __name__ == '__main__':
    sys.exit(main()) 