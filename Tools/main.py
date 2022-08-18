import sys
import tkinter as tk
import os
import re
from datetime import datetime

argslen = len(sys.argv)
#show help if provided no arguments
if argslen == 1:
	print("\"dist/main/main.exe\" [path] [columns] [delimiter] [separator]")
	print("path: absolute path to file to read")
	print("columns: number of columns to divide bytes over; default is \"3\"")
	print("delimiter: regex character set to use when reading from file; default is whitespace (\"\\s\")")
	print("separator: separator character to use when writing to file; default is comma (\",\")")
	mainwindow = tk.Tk()
	mainwindow.withdraw()
	mainwindow.mainloop()

#default values for optional arguments
if argslen < 3:
	#columns
	sys.argv.append("3")
if argslen < 4:
	#delimiter
	sys.argv.append("\\s")
if argslen < 5:
	#separator
	sys.argv.append(",")

#show error and exit if specified file does not exist
if not os.path.exists(sys.argv[1]):
	print("[path] does not exist")
	sys.exit()

#read file data to variable
dataread = ""
with open(sys.argv[1], "rt") as datafile:
	dataread = datafile.read()

#convert data to csv format with specified amount columns
datafound = re.findall("([0-9A-Fa-f]{2})" + sys.argv[3], dataread)
dataoutput = ""
datacounter = 0
datacolumns = int(sys.argv[2])
for databyte in datafound:
	#count columns for current row
	datacounter += 1
	dataoutput += databyte
	#check if amount of columns for current row is same as specified columns
	if datacounter == datacolumns:
		#add a new row
		dataoutput += "\n"
		datacounter = 0
	else:
		#add a new column with specified delimiter
		dataoutput += sys.argv[4]

#write converted data to file with name as current timestamp and extension csv in "output" folder of app's folder
datafile = os.path.dirname(__file__)
if getattr(sys, "frozen", False):
	datafile = os.path.dirname(os.path.dirname(datafile))
dataroot = os.path.normpath(datafile + "/output")
if not os.path.exists(dataroot):
	os.makedirs(dataroot)
dataname = datetime.now().strftime("%Y%m%d%H%M%S%f")
datapath = os.path.normpath(dataroot + "/" + dataname) + ".csv"
with open(datapath, "wt") as datawrite:
	datawrite.write(dataoutput)
