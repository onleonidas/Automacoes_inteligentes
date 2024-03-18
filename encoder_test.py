import RPi.GPIO as GPIO
from time import sleep
from Encoder import Encoder
from pins import *

def main():
    
    try:
        right_encoder = Encoder(RENCA, RENCB)
        right_encoder.start()
        left_encoder = Encoder(LENCA, LENCB)
        left_encoder.start()
        while True :
            
            if(right_encoder.new_data_flag() and right_encoder.get_w() != 0):
                print("right angular velocity is: ")
                print(right_encoder.get_w()) #ur
                right_encoder.data_collected()

            if(left_encoder.new_data_flag() and left_encoder.get_w() != 0):
                print("left angular velocity is: ")
                print(-left_encoder.get_w()) #ul
                left_encoder.data_collected()       



    except KeyboardInterrupt:
        GPIO.cleanup()

if __name__ == '__main__':
    main()