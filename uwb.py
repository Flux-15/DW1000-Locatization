import time
import turtle
import cmath
import serial
import serial.tools.list_ports
import json
import sys

# Default Serial Port for Anchor 2 (e.g. "COM6" or "COM8")
SERIAL_PORT = "COM6"
BAUD_RATE = 115200

# Parse command line argument -p / --port if provided
for i, arg in enumerate(sys.argv):
    if arg in ("-p", "--port") and i + 1 < len(sys.argv):
        SERIAL_PORT = sys.argv[i + 1]

distance_a1_a2 = 1.25
meter2pixel = 100
range_offset = 0.0


def screen_init(width=1200, height=800, t=turtle):
    t.setup(width, height)
    t.tracer(False)
    t.hideturtle()
    t.speed(0)


def turtle_init(t=turtle):
    t.hideturtle()
    t.speed(0)


def draw_line(x0, y0, x1, y1, color="black", t=turtle):
    t.pencolor(color)
    t.up()
    t.goto(x0, y0)
    t.down()
    t.goto(x1, y1)
    t.up()


def draw_cycle(x, y, r, color="black", t=turtle):
    t.pencolor(color)
    t.up()
    t.goto(x, y - r)
    t.setheading(0)
    t.down()
    t.circle(r)
    t.up()


def fill_cycle(x, y, r, color="black", t=turtle):
    t.up()
    t.goto(x, y)
    t.down()
    t.dot(r, color)
    t.up()


def write_txt(x, y, txt, color="black", t=turtle, f=('Arial', 12, 'normal')):
    t.pencolor(color)
    t.up()
    t.goto(x, y)
    t.down()
    t.write(txt, move=False, align='left', font=f)
    t.up()


def draw_rect(x, y, w, h, color="black", t=turtle):
    t.pencolor(color)
    t.up()
    t.goto(x, y)
    t.down()
    t.goto(x + w, y)
    t.goto(x + w, y + h)
    t.goto(x, y + h)
    t.goto(x, y)
    t.up()


def fill_rect(x, y, w, h, color=("black", "black"), t=turtle):
    t.begin_fill()
    draw_rect(x, y, w, h, color, t)
    t.end_fill()


def clean(t=turtle):
    t.clear()


def draw_ui(t):
    write_txt(-300, 250, "UWB Real-Time Positioning", "black", t, f=('Arial', 32, 'bold'))
    fill_rect(-400, 200, 800, 40, "black", t)
    write_txt(-50, 205, "WALL", "yellow", t, f=('Arial', 24, 'bold'))
    write_txt(-380, -280, f"Connected to Anchor 2 on {SERIAL_PORT} @ {BAUD_RATE} bps", "gray", t, f=('Arial', 12, 'normal'))


def draw_uwb_anchor(x, y, txt, range_val, t):
    r = 20
    fill_cycle(x, y, r, "green", t)
    write_txt(x + r, y, f"{txt}: {range_val:.2f}M", "black", t, f=('Arial', 16, 'normal'))


def draw_uwb_tag(x, y, txt, t):
    pos_x = -250 + int(x * meter2pixel)
    pos_y = 150 - int(y * meter2pixel)
    r = 20
    fill_cycle(pos_x, pos_y, r, "blue", t)
    write_txt(pos_x, pos_y, f"{txt}: ({x:.2f}, {y:.2f})", "black", t, f=('Arial', 16, 'bold'))


def connect_serial(port, baud):
    print(f"*** Attempting connection to Anchor 2 on {port} @ {baud} baud... ***")
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        print(f"*** Successfully connected to {port} ***\n")
        return ser
    except Exception as e:
        print(f"[ERROR] Could not open serial port {port}: {e}")
        ports = list(serial.tools.list_ports.comports())
        if ports:
            print("Available COM ports detected on your system:")
            for p in ports:
                print(f"  -> {p.device}: {p.description}")
            print(f"\nTip: You can run: python uwb.py -p {ports[0].device}")
        else:
            print("No COM ports found. Ensure Anchor 2 is plugged in via USB.")
        sys.exit(1)


def read_serial_data(ser):
    try:
        if ser.in_waiting > 0:
            line = ser.readline().decode('UTF-8', errors='ignore').strip()
            if not line:
                return None
            if line.startswith("{") and line.endswith("}"):
                try:
                    data = json.loads(line)
                    return data
                except json.JSONDecodeError:
                    pass
            else:
                # Print non-JSON lines (e.g. debug prints from Arduino)
                print(f"[Anchor 2]: {line}")
    except Exception as e:
        print(f"[Serial Read Error]: {e}")
    return None


def uwb_range_offset(uwb_range):
    return uwb_range + range_offset


def main():
    ser = connect_serial(SERIAL_PORT, BAUD_RATE)

    screen_init()
    t_ui = turtle.Turtle()
    t_a1 = turtle.Turtle()
    t_a2 = turtle.Turtle()
    t_a3 = turtle.Turtle()
    turtle_init(t_ui)
    turtle_init(t_a1)
    turtle_init(t_a2)
    turtle_init(t_a3)

    a1_range = 0.0
    a2_range = 0.0

    draw_ui(t_ui)
    turtle.update()

    print("Listening for Tag position stream from Anchor 2...")

    while True:
        data = read_serial_data(ser)
        if data is not None:
            updated = False
            # Check if links/ranges are present to draw anchors
            if "links" in data:
                for one in data["links"]:
                    if one.get("A") == "1782":
                        clean(t_a1)
                        a1_range = uwb_range_offset(float(one.get("R", 0)))
                        draw_uwb_anchor(-250, 150, "A1782(0,0)", a1_range, t_a1)
                        updated = True

                    if one.get("A") == "1783":
                        clean(t_a2)
                        a2_range = uwb_range_offset(float(one.get("R", 0)))
                        draw_uwb_anchor(-250 + meter2pixel * distance_a1_a2,
                                        150, f"A1783({distance_a1_a2})", a2_range, t_a2)
                        updated = True

            # Check if coordinates (x, y) calculated by Tag are present
            if "x" in data and "y" in data:
                x = float(data["x"])
                y = float(data["y"])
                print(f"Tag Position -> X: {x:.2f} m, Y: {y:.2f} m | R1: {a1_range:.2f} m | R2: {a2_range:.2f} m")
                clean(t_a3)
                draw_uwb_tag(x, y, "TAG", t_a3)
                updated = True

            if updated:
                turtle.update()
        else:
            time.sleep(0.01)
            turtle.update()

    turtle.mainloop()


if __name__ == '__main__':
    main()