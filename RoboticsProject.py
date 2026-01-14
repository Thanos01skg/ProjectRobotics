import tkinter as tk
from tkinter import simpledialog, messagebox
import math
import time

# --- Window Settings ---
WIDTH = 800
HEIGHT = 600
CX = WIDTH / 2
CY = HEIGHT / 2 # The center (0,0) of the screen

# --- Helper Functions ---

def to_screen(x, y):
    """Converts robot coordinates to screen pixels."""
    return CX + x, CY - y

def inverse_kinematics(x, y, l1, l2, left_mode):
    """
    Calculates the angle and position of the elbow.
    Uses the Law of Sines (simpler and more stable).
    """
    dist = math.sqrt(x**2 + y**2)
    
    # Check: If the point is out of range
    if dist > (l1 + l2) or dist < abs(l1 - l2) or dist == 0:
        return None

    # Law of Sines for the angle of the 2nd arm
    # cos(a) = (b^2 + c^2 - a^2) / 2bc
    cos_angle = (l1**2 + dist**2 - l2**2) / (2 * l1 * dist)
    
    # Minor bug fixes (π.χ. 1.0000001)
    cos_angle = max(-1, min(1, cos_angle))
    
    alpha = math.acos(cos_angle)
    theta = math.atan2(y, x) # The angle of the target

    # Solution selection (Left-hand or Right-hand)
    if left_mode:
        q1 = theta + alpha
    else:
        q1 = theta - alpha

    # Elbow position calculation
    elbow_x = l1 * math.cos(q1)
    elbow_y = l1 * math.sin(q1)
    return elbow_x, elbow_y

def check_path(x1, y1, x2, y2, l1, l2):
    """
    Checks if the straight line passes through the 'forbidden zone'
    (the inner circle).
    """
    min_r = abs(l1 - l2)
    steps = 40 # We break the line into 40 points
    for i in range(steps + 1):
        t = i / steps
        # We get an intermediate point
        px = x1 + (x2 - x1) * t
        py = y1 + (y2 - y1) * t
        # If its distance is less than the minimum radius -> Error
        if math.sqrt(px**2 + py**2) < min_r - 1:
            return False
    return True

# --- Main Program---

def main():
    # Create a window
    root = tk.Tk()
    root.title("Robotic Arm Simple")
    
    # Canvas (This is where we paint)
    canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg="black")
    canvas.pack()

    # 1. DATA ENTRY
    root.withdraw() # We hide the window for a while
    try:
        L1 = float(simpledialog.askstring("Input", "Length L1:") or 200)
        L2 = float(simpledialog.askstring("Input", "Length L2:") or 150)
        is_left = messagebox.askyesno("Choice", "Left-handed solution;")
        start_x = float(simpledialog.askstring("Input", "Starting X:") or 150)
        start_y = float(simpledialog.askstring("Input", "Starting Y:") or 150)
    except (ValueError, TypeError):
        return # If he presses Cancel or gives an incorrect value, it closes.
    
    root.deiconify() # We display the window

    # 2. ENVIRONMENTAL DESIGN
    # Axes
    canvas.create_line(CX, 0, CX, HEIGHT, fill="green")
    canvas.create_line(0, CY, WIDTH, CY, fill="green")
    
    # Boundaries (Circles)
    max_r = L1 + L2
    min_r = abs(L1 - L2)
    
    # External (Cyan)
    x1, y1 = to_screen(-max_r, max_r)
    x2, y2 = to_screen(max_r, -max_r)
    canvas.create_oval(x1, y1, x2, y2, outline="cyan")
    
    # Internal (Red - Forbidden)
    x1, y1 = to_screen(-min_r, min_r)
    x2, y2 = to_screen(min_r, -min_r)
    canvas.create_oval(x1, y1, x2, y2, outline="red")

    # 3. STARTING POSITION
    res = inverse_kinematics(start_x, start_y, L1, L2, is_left)
    if not res:
        messagebox.showerror("Error", "The starting point is impossible!")
        return

    elbow_x, elbow_y = res
    curr_x, curr_y = start_x, start_y

    # Creating the robot lines (we make them once and then move them)
    bx, by = to_screen(0, 0)
    ex, ey = to_screen(elbow_x, elbow_y)
    tx, ty = to_screen(curr_x, curr_y)

    link1 = canvas.create_line(bx, by, ex, ey, fill="yellow", width=3)
    link2 = canvas.create_line(ex, ey, tx, ty, fill="yellow", width=3)
    joint = canvas.create_oval(ex-4, ey-4, ex+4, ey+4, fill="red") # Elbow

    root.update()

    # 4. MOVEMENT LOOP
    while True:
        target_str = simpledialog.askstring("Movement", "Give new X,Y (e.g. 100,100):")
        if not target_str: break # Exit if Cancel is pressed

        try:
            parts = target_str.split(',')
            nx, ny = float(parts[0]), float(parts[1])

            # Check 1: Is the point within range?
            if inverse_kinematics(nx, ny, L1, L2, is_left) is None:
                messagebox.showerror("Error", "Point out of range!")
                continue

            # Check 2: Does the line pass through the prohibited zone?
            if not check_path(curr_x, curr_y, nx, ny, L1, L2):
                messagebox.showerror("Caution", "The movement crosses the prohibited zone!")
                continue
            
            # ANIMATION
            steps = 40
            dx = (nx - curr_x) / steps
            dy = (ny - curr_y) / steps
            
            # We draw the red route line
            sx, sy = to_screen(curr_x, curr_y)
            fx, fy = to_screen(nx, ny)
            canvas.create_line(sx, sy, fx, fy, fill="red", width=1)

            for _ in range(steps):
                curr_x += dx
                curr_y += dy
                
                res = inverse_kinematics(curr_x, curr_y, L1, L2, is_left)
                if res:
                    ex_new, ey_new = res
                    
                    # Conversion to pixels
                    scr_ex, scr_ey = to_screen(ex_new, ey_new)
                    scr_tx, scr_ty = to_screen(curr_x, curr_y)

                    # Location update (without deleting/recreating)
                    canvas.coords(link1, bx, by, scr_ex, scr_ey)
                    canvas.coords(link2, scr_ex, scr_ey, scr_tx, scr_ty)
                    canvas.coords(joint, scr_ex-4, scr_ey-4, scr_ex+4, scr_ey+4)
                    
                    root.update()
                    time.sleep(0.04)
            
            curr_x, curr_y = nx, ny

        except ValueError:
            messagebox.showerror("Error", "Incorrect format! Write e.g.: 150,150")

    root.destroy()

if __name__ == "__main__":
    main()