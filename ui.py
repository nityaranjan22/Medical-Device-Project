import tkinter as tk

# Define function to handle button clicks
def button_click(button_number):
    print(f"Button {button_number} clicked")

# Define function to handle reset button click
def reset_click():
    print("Reset button clicked")

# Define function to handle stop button click
def stop_click():
    print("Stop button clicked")

# Create main window
root = tk.Tk()
root.title("PWM Devices")

# Define button properties
button_width = 15
button_height = 3
button_bg_color = "#2c3e50" # dark blue
button_fg_color = "#ecf0f1" # light gray
button_font = ("Arial", 14, "bold")

# Create top row of buttons for PWM devices 1-6
top_frame = tk.Frame(root)
top_frame.pack(side=tk.TOP, pady=10)

for i in range(6):
    button = tk.Button(top_frame, text=f"PWM Device {i+1}", command=lambda x=i+1: button_click(x), bg=button_bg_color, fg=button_fg_color, font=button_font, height=button_height, width=button_width)
    button.pack(side=tk.LEFT, padx=10)

# Create bottom row of buttons for PWM devices 7-12
bottom_frame = tk.Frame(root)
bottom_frame.pack(side=tk.TOP, pady=10)

for i in range(6, 12):
    button = tk.Button(bottom_frame, text=f"PWM Device {i+1}", command=lambda x=i+1: button_click(x), bg=button_bg_color, fg=button_fg_color, font=button_font, height=button_height, width=button_width)
    button.pack(side=tk.LEFT, padx=10)

# Create row for Reset and Stop buttons
reset_frame = tk.Frame(root, bg="#e74c3c") # red background
reset_frame.pack(side=tk.TOP, pady=10)

reset_button = tk.Button(reset_frame, text="Reset", command=reset_click, bg=button_bg_color, fg=button_fg_color, font=button_font, height=button_height, width=button_width)
reset_button.pack(side=tk.LEFT, padx=10)

stop_button = tk.Button(reset_frame, text="Stop", command=stop_click, bg=button_bg_color, fg=button_fg_color, font=button_font, height=button_height, width=button_width)
stop_button.pack(side=tk.LEFT, padx=10)

# Configure root window properties
root.configure(bg="#3498db") # light blue

# Start GUI event loop
root.mainloop()
