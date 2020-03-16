import Tkinter as tk
from Tkinter import *
import tkMessageBox
import serial
#ser = serial.Serial('/dev/ttyACM0', 9600)

root = tk.Tk()

heading = tk.Label(root, text = "Please enter coordinates in Decimal Degrees format (111.1111,111.1111)")
GPS1 = tk.Label(root, text = "Coordinate 1: " )
GPS1Entry = tk.Entry(root)
GPS2 = tk.Label(root, text = "Coordinate 2: " )
GPS2Entry = tk.Entry(root)
GPS3 = tk.Label(root, text = "Coordinate 3: " )
GPS3Entry = tk.Entry(root)

def hi():
    global value1
    global value2
    global value3
    value1 = GPS1Entry.get()
    value2 = GPS2Entry.get()
    value3 = GPS3Entry.get()
    print(value1)
    print(value2)
    print(value3)
    ser.write(value1)
    ser.write('|')
    ser.write(value2)
    ser.write('|')
    ser.write(value3)
    ser.write('|')
    
def RunGPS():
    heading.pack()
    GPS1.pack()
    GPS1Entry.pack()
    GPS2.pack()
    GPS2Entry.pack()
    GPS3.pack()
    GPS3Entry.pack()                        
    plotButton = tk.Button(root, text = "Store and Run", command = hi)
    plotButton.pack()
    
def nothing():
    y = 1
    y = y +1

menubar = Menu(root)
menubar.add_command(label="Remote Control", command = nothing)
menubar.add_command(label="GPS Autonomous", command = RunGPS)
root.config(menu = menubar)

root.mainloop()
