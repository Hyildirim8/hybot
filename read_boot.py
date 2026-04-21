import serial, time, re
with serial.Serial('/dev/ttyACM1', 115200, timeout=0.5) as s:
    # No DTR reset - just read what's coming
    end = time.time() + 30
    while time.time() < end:
        line = s.readline()
        if line:
            text = line.decode('utf-8', errors='replace').strip()
            if 'vel_sub' not in text and text:
                clean = re.sub(r'\x1b\[[0-9;]*m', '', text)
                if clean.strip():
                    print(clean)
