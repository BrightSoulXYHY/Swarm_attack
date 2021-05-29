# import all libraries
import win32gui, win32ui, win32con, win32api
from ctypes import windll
import numpy

# get all RflySim3D window handles with class name UnrealWindow
def window_enumeration_handler(hwnd, window_hwnds):
    if win32gui.GetClassName(hwnd) == "UnrealWindow":
        window_hwnds.append(hwnd)

# 利用回调函数window_enumeration_handler把获得的窗口句柄
# 弄到window_hwnds里面，作为函数返回值，一般为降序
def getWndHandls():
    window_hwnds = []
    win32gui.EnumWindows(window_enumeration_handler, window_hwnds)
    return window_hwnds

# define a class to store information of window handles
class WinInfo:
    def __init__(self, hWnd, width, height, saveDC, saveBitMap, mfcDC, hWndDC,title):
        self.hWnd = hWnd
        self.width = width
        self.height = height
        self.saveDC = saveDC
        self.saveBitMap = saveBitMap
        self.mfcDC = mfcDC
        self.hWndDC = hWndDC
        self.title = title

# get the window infomation of a handle
def getHwndInfo(hWnd):
    left, top, right, bot = win32gui.GetClientRect(hWnd)
    width = right - left
    height = bot - top
    print((width,height))
    if hWnd and width == 0 and height == 0:
        print("The RflySim3D window cannot be in minimized mode")
        sys.exit(1)

    # retrieve the device context (DC) for the entire window, including title bar, menus, and scroll bars.
    hWndDC = win32gui.GetWindowDC(hWnd)
    mfcDC = win32ui.CreateDCFromHandle(hWndDC)
    
    # creates a memory device context (DC) compatible with the specified device.
    saveDC = mfcDC.CreateCompatibleDC()
    
    # create a bitmap object
    saveBitMap = win32ui.CreateBitmap()
    saveBitMap.CreateCompatibleBitmap(mfcDC, width, height)

    title = win32ui.GetWindowText(hWnd)

    # return image info.
    info = WinInfo(hWnd, width, height, saveDC, saveBitMap, mfcDC, hWndDC, title)
    return info


# get the image from RflySim3D window's client area
def getCVImg(wInfo):
    wInfo.saveDC.SelectObject(wInfo.saveBitMap)

    #  copies a visual window into the specified device context (DC)
    # The last int value：0-save the whole window，1-only client area
    result = windll.user32.PrintWindow(wInfo.hWnd, wInfo.saveDC.GetSafeHdc(), 1)
    signedIntsArray = wInfo.saveBitMap.GetBitmapBits(True)

    # get the image from bitmap array
    im_opencv = numpy.frombuffer(signedIntsArray, dtype='uint8')
    im_opencv.shape = (wInfo.height, wInfo.width, 4)

    # return the image
    return im_opencv

# clear all objects
def clearHWND(wInfo):
    win32gui.DeleteObject(wInfo.saveBitMap.GetHandle())
    wInfo.saveDC.DeleteDC()
    wInfo.mfcDC.DeleteDC()
    win32gui.ReleaseDC(wInfo.hWnd, wInfo.hWndDC)
    