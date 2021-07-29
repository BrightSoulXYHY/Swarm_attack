# import all libraries
import win32gui, win32con,win32ui
from ctypes import windll
import numpy
import d3dshot
import sys
import cv2

# Enable new Screen capture API for UE4 4.23 and above
isNewUE=False

if isNewUE:
    # Use d3dshot to capture screen from UE 4.23+
    windll.shcore.SetProcessDpiAwareness(2)
    d = d3dshot.create(capture_output="numpy",frame_buffer_size=1)
    #d1 = d3dshot.create(capture_output="numpy",frame_buffer_size=1)

# get all RflySim3D window handles with class name UnrealWindow
def window_enumeration_handler(hwnd, window_hwnds):
    if win32gui.GetClassName(hwnd) == "UnrealWindow":
        window_hwnds.append(hwnd)

# Get handles of all RflySim3D windows
def getWndHandls():
    window_hwnds = []
    win32gui.EnumWindows(window_enumeration_handler, window_hwnds)
    window_index=[]
    for hw in window_hwnds:
        name=win32gui.GetWindowText(hw)
        nameList = name.split('-',1)
        windIndex=nameList[1]
        if windIndex.isdigit():
            inx=int(windIndex)
            window_index.append(inx)

    # Windows are recognized by RflySim3D-*, where * is the number
    for j in range(len(window_index)):
        for i in range(len(window_index)-1-j):
            if window_index[i]>window_index[i+1]:
                window_index[i], window_index[i+1] = window_index[i+1], window_index[i]
                window_hwnds[i], window_hwnds[i+1] = window_hwnds[i+1], window_hwnds[i]
        
    print("window_hwnds:",window_hwnds)
    print("window_index:",window_index)
    return window_hwnds

# define a class to store information of window handles
class WinInfo:
    def __init__(self, hWnd, width, height, saveDC, saveBitMap, mfcDC, hWndDC):
        self.hWnd = hWnd
        self.width = width
        self.height = height
        self.saveDC = saveDC
        self.saveBitMap = saveBitMap
        self.mfcDC = mfcDC
        self.hWndDC = hWndDC

# get the window infomation of a handle
def getHwndInfo(hWnd):
    left, top, right, bot = win32gui.GetClientRect(hWnd)
    width = right - left
    height = bot - top
    print((width,height))
    if hWnd and width == 0 and height == 0:
        print("The RflySim3D window cannot be in minimized mode")
        sys.exit(1)

    
    if not isNewUE: # Use Windows API to capture screen for UE4.22 windows
        # retrieve the device context (DC) for the entire window, 
        # including title bar, menus, and scroll bars.
        hWndDC = win32gui.GetWindowDC(hWnd)
        mfcDC = win32ui.CreateDCFromHandle(hWndDC)
        
        # creates a memory device context (DC) compatible with the specified device.
        saveDC = mfcDC.CreateCompatibleDC()
        
        # create a bitmap object
        saveBitMap = win32ui.CreateBitmap()
        saveBitMap.CreateCompatibleBitmap(mfcDC, width, height)

        # return image info.
        info = WinInfo(hWnd, width, height, saveDC, saveBitMap, mfcDC, hWndDC)
        return info
    else:
        # Use Windows Desktop Duplication API to capture screen for all UE4 windows
        info = WinInfo(hWnd, width, height, 0, 0, 0, 0)
        return info        


# get the image from RflySim3D window's client area
def getCVImg(wInfo):
    if not isNewUE:
        wInfo.saveDC.SelectObject(wInfo.saveBitMap)

        #  copies a visual window into the specified device context (DC)
        # The last int value：0-save the whole window，1-only client area
        result = windll.user32.PrintWindow(wInfo.hWnd, wInfo.saveDC.GetSafeHdc(), 1)
        signedIntsArray = wInfo.saveBitMap.GetBitmapBits(True)

        # get the image from bitmap array
        im_opencv = numpy.frombuffer(signedIntsArray, dtype='uint8')
        #print(im_opencv.shape)
        im_opencv.shape = (wInfo.height, wInfo.width, 4)
        im_opencv = cv2.cvtColor(im_opencv, cv2.COLOR_BGRA2BGR)
        # return the image
        return im_opencv
    
    else:
        if not win32gui.IsWindow(wInfo.hWnd):
            sys.exit() # exit if RflySim3D is closed
            
        # Get the top left position of the window
        (x, y)=win32gui.ClientToScreen(wInfo.hWnd,(0,0))
        # Get a screnshot
        #frame_stack=d.screenshot(region=(x, y, wInfo.width+x, wInfo.height+y))
        
        frame_stack=d.screenshot()
        frame_stack=frame_stack[y:wInfo.height+y,x:wInfo.width+x]   

        # covert color for opencv
        frame_stack=cv2.cvtColor(frame_stack, cv2.COLOR_RGB2BGR)
        # get scale_factor
        sc = d.display.scale_factor
        w1=int(wInfo.width/sc+0.5)
        h1=int(wInfo.height/sc+0.5)
        # resize the windows
        frame_stack = cv2.resize(frame_stack, (w1, h1))
        return frame_stack

def getCVImgList(wInfoList):
    outList=[]
    if not isNewUE:
        for wInfo in wInfoList:
            wInfo.saveDC.SelectObject(wInfo.saveBitMap)
            #  copies a visual window into the specified device context (DC)
            # The last int value：0-save the whole window，1-only client area
            result = windll.user32.PrintWindow(wInfo.hWnd, wInfo.saveDC.GetSafeHdc(), 1)
            signedIntsArray = wInfo.saveBitMap.GetBitmapBits(True)

            # get the image from bitmap array
            im_opencv = numpy.frombuffer(signedIntsArray, dtype='uint8')
            #print(im_opencv.shape)
            im_opencv.shape = (wInfo.height, wInfo.width, 4)
            im_opencv = cv2.cvtColor(im_opencv, cv2.COLOR_BGRA2BGR)
            outList.append(im_opencv)
    else:
        screenC = d.screenshot()
        for wInfo in wInfoList:
            if not win32gui.IsWindow(wInfo.hWnd):
                sys.exit() # exit if RflySim3D is closed
                
            # Get the top left position of the window
            (x, y)=win32gui.ClientToScreen(wInfo.hWnd,(0,0))
            frame_stack=screenC[y:wInfo.height+y,x:wInfo.width+x]   
            # covert color for opencv
            frame_stack=cv2.cvtColor(frame_stack, cv2.COLOR_RGB2BGR)
            # get scale_factor
            sc = d.display.scale_factor
            w1=int(wInfo.width/sc+0.5)
            h1=int(wInfo.height/sc+0.5)
            # resize the windows
            frame_stack = cv2.resize(frame_stack, (w1, h1))
            outList.append(frame_stack)
    return outList

# move window to desired position and set if always topmost
def moveWd(hwd,x=0,y=0,topMost=False):
    left, top, right, bot = win32gui.GetWindowRect(hwd)
    width = right - left
    height = bot - top
    DispTop = win32con.HWND_TOP
    if topMost and isNewUE:
        DispTop=win32con.HWND_TOPMOST
    win32gui.SetWindowPos(hwd, DispTop, x,y,width,height, win32con.SWP_SHOWWINDOW|win32con.SWP_NOSIZE)

    return (x+width,y+height)
    
    
# clear all objects
def clearHWND(wInfo):
    if not isNewUE:
        win32gui.DeleteObject(wInfo.saveBitMap.GetHandle())
        wInfo.saveDC.DeleteDC()
        wInfo.mfcDC.DeleteDC()
        win32gui.ReleaseDC(wInfo.hWnd, wInfo.hWndDC)
    