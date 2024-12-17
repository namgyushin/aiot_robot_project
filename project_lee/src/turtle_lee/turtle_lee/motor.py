from periphery import GPIO
import time

# GPIO 핀 번호 설정
IR_PIN = 661  # 적외선 모듈 신호 핀 (OUT 핀에 연결)
MOTOR_PIN1 = 573  # 모터 제어 핀 1
MOTOR_PIN2 = 572  # 모터 제어 핀 2

def initialize_pin(pin, direction):
    """GPIO 핀 초기화 함수"""
    return GPIO(pin, direction)  # 방향 설정 ("in" 또는 "out")

def activate_motor(motor_pin1, motor_pin2, duration):
    """모터를 일정 시간 동안 가동하는 함수"""
    print("모터 가동 준비 중... 1초 대기")
    time.sleep(1)  # 모터 가동 전에 1초 대기
    print("모터 가동 시작!")
    motor_pin1.write(True)  # 모터 핀 1 활성화
    motor_pin2.write(False) # 모터 핀 2 비활성화
    time.sleep(duration)    # 지정된 시간 동안 대기
    motor_pin1.write(False) # 모터 핀 1 비활성화
    motor_pin2.write(False) # 모터 핀 2 비활성화
    print("모터 가동 종료!")

def detect_ir_signal(ir_pin, motor_pin1, motor_pin2):
    """적외선 신호 감지 함수"""
    print("적외선 신호 감지 대기 중...")
    try:
        while True:
            signal = ir_pin.read()  # GPIO 핀에서 신호 읽기

            # 디지털 노이즈 필터링
            if signal == 0:  # LOW 신호 감지
                time.sleep(0.05)  # 50ms 대기 후 다시 확인
                if ir_pin.read() == 0:  # 여전히 LOW면 신호로 처리
                    print("적외선 신호 감지!")
                    activate_motor(motor_pin1, motor_pin2, 1)  # 3초간 모터 가동
            else:
                print("적외선 신호 없음")
            time.sleep(0.1)  # 100ms 간격으로 확인
    except KeyboardInterrupt:
        print("감지 종료")
    finally:
        ir_pin.close()
        motor_pin1.close()
        motor_pin2.close()

def main():
    ir_pin = initialize_pin(IR_PIN, "in")
    motor_pin1 = initialize_pin(MOTOR_PIN1, "out")
    motor_pin2 = initialize_pin(MOTOR_PIN2, "out")

    print(f"GPIO {IR_PIN}, {MOTOR_PIN1}, {MOTOR_PIN2} 초기화 완료!")
    detect_ir_signal(ir_pin, motor_pin1, motor_pin2)

if __name__ == "__main__":
    main()
