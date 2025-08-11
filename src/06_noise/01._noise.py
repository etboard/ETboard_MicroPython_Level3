# ******************************************************************************************
# FileName     : 02._noise_oled.py
# Description  : 소음 센서 값을 db로 변환하여 OLED에 출력하기
# Author       : 박은정
# Created Date : 25.08.07 : 박은정
# Modified     : 
# ******************************************************************************************


import time
import utime
import math
from machine import ADC, Pin, Timer
from ETboard.lib.pin_define import *


#===========================================================================================
# 사운드 센서 사용하기
#===========================================================================================
sound_sensor = ADC(Pin(A5))                              # A5번 핀에 센서 연결
sound_sensor.atten(ADC.ATTN_11DB)                        # 입력 전압 범위 확장 (3.3V까지)
sound_sensor.width(ADC.WIDTH_12BIT)                      # 12비트 ADC 해상도 설정 (0~4095)


#===========================================================================================
# 전역 변수
#===========================================================================================
# 사운드 센서 관련 변수
SAMPLE_BUFFER_SIZE = 1024
SAMPLING_FREQ = 8000
sample_buffer = []

# 캘리브레이션 변수
dc_baseline = 2048
calibration_samples = 1000

# 측정 결과 변수
rms = 0
db = 0

# 히스토리 및 상수
HISTORY_SIZE = 128
rms_history = [0] * HISTORY_SIZE
ADC_TO_VOLTAGE = 3.3 / 4095
RMS_DB_DATA = [
    (1.7, 39.9), (2.1, 46.3), (3.5, 66.8), (7.4, 88.2),
    (13.9, 96.5), (20.4, 98.4), (29.3, 103.6), (31.8, 103.9)
]

sound_timer = None

previous_time = 0


#===========================================================================================
def setup():                                             # 응용 프로그램 설정하기
#===========================================================================================
    global sound_timer

    # 사운드 센서 초기화
    calibrate_dc_offset()
    sound_timer = Timer(0)
    sound_timer.init(freq=SAMPLING_FREQ, mode=Timer.PERIODIC, callback=sample_sound_sensor)


#===========================================================================================
def loop():                                              # 응용 프로그램 루프
#===========================================================================================
    do_sensing_proces()                                  # 센싱 처리
    et_short_periodic_process()


#===========================================================================================
def do_sensing_proces():                                 # 센싱 처리
#===========================================================================================
    sound_sensor_update()


#===========================================================================================
def calibrate_dc_offset():                               # DC 바이어스 제거용 캘리브레이션
#===========================================================================================
    global dc_baseline

    calibration_buffer = []
    for i in range(calibration_samples):
        calibration_buffer.append(sound_sensor.read())
        time.sleep_ms(1)

    dc_baseline = sum(calibration_buffer) / len(calibration_buffer)


#===========================================================================================
def calculate_db(buffer):                                # RMS 계산
#===========================================================================================
    if not buffer or len(buffer) < 2: return 0, 0

    dc_component = sum(buffer) / len(buffer)
    rms_raw = math.sqrt(sum((sample - dc_component) ** 2 for sample in buffer) / len(buffer))

    return rms_to_db(rms_raw)


#===========================================================================================
def rms_to_db(rms):                                      # db 변환
#===========================================================================================
    if rms <= RMS_DB_DATA[0][0]: return RMS_DB_DATA[0][1]

    if rms >= RMS_DB_DATA[-1][0]: return RMS_DB_DATA[-1][1]

    for i in range(len(RMS_DB_DATA) - 1):
        x1, y1 = RMS_DB_DATA[i]
        x2, y2 = RMS_DB_DATA[i + 1]
        if x1 <= rms <= x2:
            return y1 + (rms - x1) * (y2 - y1) / (x2 - x1)

    return 0


#===========================================================================================
def sample_sound_sensor(timer):                          # 샘플 수집
#===========================================================================================
    global sample_buffer

    if len(sample_buffer) < SAMPLE_BUFFER_SIZE:
        sample_buffer.append(sound_sensor.read())


#===========================================================================================
def sound_sensor_update():                               # RMS 계산
#===========================================================================================
    global sample_buffer, rms, db

    if len(sample_buffer) >= SAMPLE_BUFFER_SIZE:
        data = sample_buffer[:]
        sample_buffer.clear()
        db = calculate_db(data)


#===========================================================================================
def et_short_periodic_process():                         # 짧은 주기 처리                    
#===========================================================================================
    global previous_time

    interval = 1                                         # 1초마다 정보 표시
    now = int(round(time.time()))

    if now - previous_time < interval:                   # 1초가 지나지 않았다면
        return

    previous_time = now
    display_information()


#===========================================================================================
def display_information():                               # 정보 표시               
#===========================================================================================
    global db
    string_db = "{:.1f}".format(db)

    print('dB: ' + string_db + 'dB')


#===========================================================================================
# 시작 지점                     
#===========================================================================================
if __name__ == "__main__":
    setup()    
    while True:
        loop()


#===========================================================================================
#                                                    
# (주)한국공학기술연구원 http://et.ketri.re.kr       
#
#===========================================================================================
