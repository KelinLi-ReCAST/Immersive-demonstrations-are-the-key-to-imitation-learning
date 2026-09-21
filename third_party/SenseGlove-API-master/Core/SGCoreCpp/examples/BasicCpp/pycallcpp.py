import ctypes
dll = ctypes.cdll.LoadLibrary
lib = dll('./libpycallcpp.so') 
lib.main()