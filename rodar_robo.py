import RPi.GPIO as GPIO
from pins import *
import time
from Encoder import Encoder

GPIO.setmode(GPIO.BCM)
data_list = []
right_encoder = Encoder(RENCA, RENCB)
right_encoder.start()
left_encoder = Encoder(LENCA, LENCB)
left_encoder.start()

try:
    GPIO.setup(RIN1, GPIO.OUT)
    GPIO.setup(RIN2, GPIO.OUT)
    GPIO.setup(RENA, GPIO.OUT)
    GPIO.output(RIN1, GPIO.HIGH)
    GPIO.output(RIN2, GPIO.LOW)
    GPIO.output(RENA, GPIO.HIGH)

    GPIO.setup(LIN1, GPIO.OUT)
    GPIO.setup(LIN2, GPIO.OUT)
    GPIO.setup(LENA, GPIO.OUT)
    GPIO.output(LIN1, GPIO.HIGH)
    GPIO.output(LIN2, GPIO.LOW)

    pwm_r = GPIO.PWM(RENA, 1000)
    pwm_l = GPIO.PWM(LENA, 1000)

    for i in range(0,101,10):  # left
        for j in range(0,101,10):  # right
            v_left = []
            v_right = []
            start_time = time.time()
            
            while time.time() - start_time < 3:
                pwm_l.start(i)
                pwm_r.start(j)
                v_left.append(-left_encoder.get_w())
                v_right.append(right_encoder.get_w())
                time.sleep(0.1)
                
            data = {
                'v_left': i * 6.28/100,
                'v_right': j * 6.28/100,
                'v_left_real': sorted(v_left)[len(v_left) // 2],  # Calcula a mediana usando sorted
                'v_right_real': sorted(v_right)[len(v_right) // 2]  # Calcula a mediana usando sorted
            }
            data_list.append(data)


        print("Data:", data)
        print(i,"%")
        # Escrever para arquivo CSV
        with open('dataset.csv', 'a') as f:
            f.write("v_left,v_right,v_left_real,v_right_real\n")
            for item in data_list:
                f.write(f"{item['v_left']},{item['v_right']},{item['v_left_real']},{item['v_right_real']}\n")
        
        pwm_l.start(0)
        pwm_r.start(0)
        input("Pressione Enter para continuar...")

    

    while True:
        time.sleep(0.1)

except KeyboardInterrupt:
    GPIO.cleanup()
