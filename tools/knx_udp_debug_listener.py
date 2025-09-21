#!/usr/bin/env python3
"""
KNX UDP Debug Listener

This script listens for UDP debug messages from the WLED KNX usermod
on port 5140 and displays them in real-time. This is useful for debugging
KNX functionality when you can't access the ESP32 serial monitor directly.

Usage:
    python knx_udp_debug_listener.py [--port 5140] [--bind-ip 0.0.0.0]

The ESP32 will broadcast debug messages to 255.255.255.255:5140 by default.
"""

import socket
import sys
import argparse
from datetime import datetime

def listen_for_debug_messages(bind_ip="0.0.0.0", port=5140):
    """Listen for UDP debug messages from WLED KNX usermod."""
    
    print(f"KNX UDP Debug Listener")
    print(f"Listening on {bind_ip}:{port}")
    print(f"Waiting for debug messages from WLED ESP32...")
    print("-" * 60)
    
    try:
        # Create UDP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        # Bind to address and port
        sock.bind((bind_ip, port))
        
        while True:
            try:
                # Receive data from UDP socket
                data, addr = sock.recvfrom(1024)
                timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                message = data.decode('utf-8', errors='replace').strip()
                
                print(f"[{timestamp}] {addr[0]:15s} | {message}")
                
            except UnicodeDecodeError:
                # Handle non-UTF8 data gracefully
                timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                print(f"[{timestamp}] {addr[0]:15s} | <binary data: {len(data)} bytes>")
                
            except KeyboardInterrupt:
                print("\nShutting down listener...")
                break
                
    except Exception as e:
        print(f"Error: {e}")
        return 1
        
    finally:
        try:
            sock.close()
        except:
            pass
            
    return 0

def main():
    parser = argparse.ArgumentParser(description="Listen for KNX UDP debug messages")
    parser.add_argument('--port', type=int, default=5140,
                       help='UDP port to listen on (default: 5140)')
    parser.add_argument('--bind-ip', default='0.0.0.0',
                       help='IP address to bind to (default: 0.0.0.0 for all interfaces)')
    
    args = parser.parse_args()
    
    return listen_for_debug_messages(args.bind_ip, args.port)

if __name__ == '__main__':
    sys.exit(main())