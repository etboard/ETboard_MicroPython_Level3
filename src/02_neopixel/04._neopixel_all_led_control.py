# ******************************************************************************************
# FileName     : 04.neopixel_all_led_control.py
# Description  : 네오픽셀의 모든 LED를 순차적으로 켜 보기
# Author       : 박은정
# Created Date : 2023.9.12
# Reference    :
# Modified     : 
# ******************************************************************************************


# import
import time
from machine import ADC, Pin
from ETboard.lib.pin_define import *
import neopixel


# variable
np = neopixel.NeoPixel(Pin(D2, Pin.OUT), 12) # 네오픽셀의 핀을 D2로 지정하고 12개의 LED를 초기화


#setup
def setup():                                 # pass를 사용하여 건너뜀
    pass


#loop
def loop():
    np[0] = (255, 0, 0)                      # 네오픽셀[0]의 색상을 빨강으로 지정
    np.write()                               # 네오픽셀 출력
    time.sleep(1)                            # 1초간 대기

    np[1] = (255, 0, 0)                      # 네오픽셀[1]의 색상을 빨강으로 지정
    np.write()                               # 네오픽셀 출력
    time.sleep(1)                            # 1초간 대기

    np[2] = (255, 0, 0)                      # 네오픽셀[2]의 색상을 빨강으로 지정
    np.write()                               # 네오픽셀 출력
    time.sleep(1)                            # 1초간 대기

    np[3] = (255, 0, 0)                      # 네오픽셀[3]의 색상을 빨강으로 지정
    np.write()                               # 네오픽셀 출력
    time.sleep(1)                            # 1초간 대기

    np[4] = (255, 0, 0)                      # 네오픽셀[4]의 색상을 빨강으로 지정
    np.write()                               # 네오픽셀 출력
    time.sleep(1)                            # 1초간 대기

    np[5] = (255, 0, 0)                      # 네오픽셀[5]의 색상을 빨강으로 지정
    np.write()                               # 네오픽셀 출력
    time.sleep(1)                            # 1초간 대기

    np[6] = (255, 0, 0)                      # 네오픽셀[6]의 색상을 빨강으로 지정
    np.write()                               # 네오픽셀 출력
    time.sleep(1)                            # 1초간 대기

    np[7] = (255, 0, 0)                      # 네오픽셀[7]의 색상을 빨강으로 지정
    np.write()                               # 네오픽셀 출력
    time.sleep(1)                            # 1초간 대기

    np[8] = (255, 0, 0)                      # 네오픽셀[8]의 색상을 빨강으로 지정
    np.write()                               # 네오픽셀 출력
    time.sleep(1)                            # 1초간 대기

    np[9] = (255, 0, 0)                      # 네오픽셀[9]의 색상을 빨강으로 지정
    np.write()                               # 네오픽셀 출력
    time.sleep(1)                            # 1초간 대기

    np[10] = (255, 0, 0)                     # 네오픽셀[10]의 색상을 빨강으로 지정
    np.write()                               # 네오픽셀 출력
    time.sleep(1)                            # 1초간 대기

    np[11] = (255, 0, 0)                     # 네오픽셀[11]의 색상을 빨강으로 지정
    np.write()                               # 네오픽셀 출력
    time.sleep(1)                            # 1초간 대기


if __name__ == '__main__':
    setup()
    while True:
        loop()


# ==========================================================================================
#
#  (주)한국공학기술연구원 http://et.ketri.re.kr
#
# ==========================================================================================