# Programska koda zaključne naloge razvoja sistema za uravnoteženje inverznega nihala
# by: Pavel Hočevar, 2022

import smbus
import time
import RPi.GPIO as GPIO
import numpy as np
import matplotlib.pyplot as plt
from gpiozero import Button, LED


class MPU6050():
    def __init__(self, gx_off=0, gy_off=0, gz_off=0, ax_off=0, ay_off=0, az_off=0):
        '''
        initializacija: določitev pin, lokacija i2c,...
        '''
        self.PWR_MGMT_1 = 0x6B  # i2c config
        self.SMPLRT_DIV = 0x19  # sample rate divider
        self.CONFIG = 0x1A  # ???
        self.GYRO_CONFIG = 0x1B  # občutljivost gyro
        self.INT_ENABLE = 0x38  # enable pin
        self.ACCEL_XOUT_H = 0x3B  # data
        self.ACCEL_YOUT_H = 0x3D
        self.ACCEL_ZOUT_H = 0x3F
        self.GYRO_XOUT_H = 0x43
        self.GYRO_YOUT_H = 0x45
        self.GYRO_ZOUT_H = 0x47

        self.bus = smbus.SMBus(1)
        self.Device_Address = 0x68  # lokacija i2c za senzor

        self.dt = 0
        self.t_tot = 0
        self.i = 0

        self.a_x = 0
        self.a_y = 0
        self.a_z = 0

        self.g_x = 0
        self.g_y = 0
        self.g_z = 0

        self.vg_x = 0
        self.vg_y = 0
        self.vg_z = 0

        self.ax_off = ax_off
        self.ay_off = ay_off
        self.az_off = az_off

        self.gyro_x = 0
        self.gyro_y = 0
        self.gyro_z = 0
        self.gx_off = gx_off
        self.gy_off = gy_off
        self.gz_off = gz_off

    def MPU_init(self):
        '''
        initial setup (range za acc, gyro,...)
        '''
        self.bus.write_byte_data(self.Device_Address, self.PWR_MGMT_1, 1)  # 1 = reset, 0 = sleep
        self.bus.write_byte_data(self.Device_Address, self.SMPLRT_DIV, 3)  # frekvenca 1kHz
        self.bus.write_byte_data(self.Device_Address, self.CONFIG, 1)  # low pass filter
        # self.bus.write_byte_data(self.Device_Address, self.GYRO_CONFIG, 24)  # občutljivost gyroskopa na 2000
        self.bus.write_byte_data(self.Device_Address, self.INT_ENABLE, 1)
        self.bus.write_byte_data(self.Device_Address, self.GYRO_CONFIG, 1)  # občutljivost gyroskopa na 250

    def read_raw_data(self, addr):
        '''
        poda negativne vrednosti ko grem čez 0,
        prebere podatke iz busa. vrednosti vrnjene v raw_data obliki (-32768, 32768)
        '''
        high = self.bus.read_byte_data(self.Device_Address, addr)
        low = self.bus.read_byte_data(self.Device_Address, addr + 1)

        value = ((high << 8) | low)

        if (value > 32768):
            value = value - 65536

        return value

    def get_data(self, dt_int, print_state=False):
        '''
        prebere podatke MPU6050 senzorja. acc, gyro, temp(nism se sprogramirou, nerabm)
        print_state: True/False, if True poda že delno obdelan data, le print statment
        '''
        self.dt = dt_int
        # start = time.time()
        acc_x = self.read_raw_data(self.ACCEL_XOUT_H)  # vrednost v g
        acc_y = self.read_raw_data(self.ACCEL_YOUT_H)  # raw data [-32750, 32750]
        acc_z = self.read_raw_data(self.ACCEL_ZOUT_H)

        self.gyro_x = self.read_raw_data(self.GYRO_XOUT_H)  # vrednost v stopinjah na sekundo
        self.gyro_y = self.read_raw_data(self.GYRO_YOUT_H)  # raw data [-32750, 32750]
        self.gyro_z = self.read_raw_data(self.GYRO_ZOUT_H)

        self.a_x = (acc_x / 16384.0) - self.ax_off  # občutljivost acc = 2g -> g
        self.a_y = (acc_y / 16384.0) - self.ay_off  # definirano na g vrednosti v originalu
        self.a_z = (acc_z / 16384.0) - self.az_off

        self.vg_x = (self.gyro_x - self.gx_off) / 131.0  # občutljivost acc za 200deg_s = 131.0
        self.vg_y = (self.gyro_y - self.gy_off) / 131.0  # za 2000 je 16.375
        self.vg_z = (self.gyro_z - self.gz_off) / 131.0

        self.g_x += self.vg_x * self.dt  # sprememba pomika v zasuk
        self.g_y += self.vg_y * self.dt
        self.g_z += self.vg_z * self.dt

        if print_state == True:
            print(f'i = {self.i},\ndt = {self.dt:12.4f}, t_tot = {self.t_tot:7.5f},\ng_x = {self.g_x:7.5f}, '
                  f'g_y = {self.g_y:7.5f}, g_z = {self.g_z:7.5f},\n'
                  f'a_x = {self.a_x:7.5f}, a_y = {self.a_y:7.5f}, a_z = {self.a_z:7.5f}\n'
                  f'g_x raw = {self.gyro_x}, g_y raw = {self.gyro_y}, g_z raw = {self.gyro_z}\n'
                  f'a_x raw = {acc_x}, a_y raw = {acc_y}, a_z raw = {acc_z}\n')
        # self.dt = time.time() - start
        self.t_tot += self.dt
        self.i += 1

    def calibrate(self, sample_size):
        '''
        odstrani napako ('drift') iz MPU6050 senzorja
        sample_size: int, pove iz koliko podatkov bomo povprečili napako
        off_rate: float, čas pavze med posamičnimi meritvami

        trenutno implementiran le premik ničle
        kalibracija deluje pravilno le če uporabljamo acc samo v eni osi
        '''
        seznam_gx_off = []  # seznami v katere sproti shranjujem merjene vrednosti
        seznam_gy_off = []
        seznam_gz_off = []
        seznam_ax_off = []
        seznam_ay_off = []
        seznam_az_off = []

        for i in range(sample_size):
            print(f'ne premikaj senzorja!   Kalibriram: {sample_size - i}')
            self.get_data(self.dt)  # poženemo senzor in preberemo podatke
            seznam_gx_off.append(self.gyro_x)  # shranim podatke v seznam
            seznam_gy_off.append(self.gyro_y)
            seznam_gz_off.append(self.gyro_z)
            seznam_ax_off.append(self.a_x)
            seznam_ay_off.append(self.a_y)
            seznam_az_off.append(self.a_z)
            # time.sleep(off_time)

        self.gx_off = sum(seznam_gx_off) / len(seznam_gx_off)  # izračunamo in shranimo povprečno vrednost napake
        self.gy_off = sum(seznam_gy_off) / len(seznam_gy_off)
        self.gz_off = sum(seznam_gz_off) / len(seznam_gz_off)
        self.ax_off = sum(seznam_ax_off) / len(seznam_ax_off)
        self.ay_off = sum(seznam_ay_off) / len(seznam_ay_off)
        self.az_off = sum(seznam_az_off) / len(seznam_az_off) - 1  # ta os meri gravitacijo, kalibriram na 1g

        self.g_x = 0  # odstranimo napako, ki je nastala med kalibriranjem
        self.g_y = 0
        self.g_z = 0

        self.dt = 0  # ponastavitev na začetne vrednosti (za lažjo obdelavo podatkov)
        self.t_tot = 0
        self.i = 0

        print(f'{self.gx_off}, {self.gy_off}, {self.gz_off}, {self.ax_off}, {self.ay_off}-1, {self.az_off}')
        print(f'\ngx_off = {self.gx_off}, gy_off = {self.gy_off}, gz_off = {self.gz_off}\n'
              f'ax_off = {self.ax_off}, ay_off = {self.ay_off}, az_off = {self.az_off}\n\n'
              f'senzor je kalibriran!!!')

        time.sleep(0.2)  # čas da se senzor 'umiri'

        return self.gx_off, self.gy_off, self.gz_off, self.ax_off, self.ay_off - 1, self.az_off

    def reset_position(self):
        self.g_x = 0  # odstranimo napako, ki je nastala med kalibriranjem
        self.g_y = 0
        self.g_z = 0

        self.dt = 0  # ponastavitev na začetne vrednosti (za lažjo obdelavo podatkov)
        self.t_tot = 0
        self.i = 0


class Motor():
    def __init__(self, pwm_pin, dir_pin, brake_pin):
        '''
        Razred, ki kontrolira gibanje DC motorja
        z uporabo H-mostiča, ki ima možnost
        uporabe zavore
        '''
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        self.pwm_pin = pwm_pin
        self.dir_pin = dir_pin
        self.brake_pin = brake_pin

        self.dc = 0.0
        self.direction = 1  # 1 = naprej, 0 = nazaj

        self.j = 1  # stanje, na začetku je aktiven
        self.t1 = 0  # začetni čas
        self.t2 = 0  # končni čas

        self.rps = 0
        self.rps_avr = 0
        self.rpm = self.rps * 60

        GPIO.setup(self.pwm_pin, GPIO.OUT)
        GPIO.setup(self.dir_pin, GPIO.OUT)
        GPIO.setup(self.brake_pin, GPIO.OUT)

        GPIO.output(self.brake_pin, GPIO.LOW)

        self.pwm = GPIO.PWM(self.pwm_pin, 85)  # določi frekvenco (nikje med 50 in 100 Hz)
        self.pwm.start(0)

    def run(self, i_mot):
        '''
        iz duty cicla dobim smer in hitrost
        '''
        if i_mot > 0:
            motor.move(abs(i_mot), 1)
        else:
            motor.move(abs(i_mot), -1)

    def move(self, dc, direction, t_off=0):  # mogoče boljše če je duty_cycle class wide
        '''
        dir: 1 = Naprej, -1 = Nazaj
        '''
        if direction == 1:
            GPIO.output(self.dir_pin, GPIO.HIGH)
            self.pwm.ChangeDutyCycle(dc)
            time.sleep(t_off)
        elif direction == -1:
            GPIO.output(self.dir_pin, GPIO.LOW)
            self.pwm.ChangeDutyCycle(dc)
            time.sleep(t_off)

        self.dc = dc
        self.direction = direction

        GPIO.output(self.brake_pin, GPIO.HIGH)
        GPIO.output(self.brake_pin, GPIO.LOW)

    def stop(self, end_brake_duration):
        '''
        ustavi motor in PWM cikel, mogoče moram dodati cleanup, garbage col.???
        '''
        self.pwm.ChangeDutyCycle(0)
        time.sleep(0.5)
        GPIO.output(self.brake_pin, GPIO.HIGH)
        time.sleep(end_brake_duration)
        GPIO.output(self.brake_pin, GPIO.LOW)
        # self.pwm.stop(0)

    def meri_RPM(self):
        '''
        funkcija ki meri RPM motorja z uporabo tcrt5000 senzorja
        ko prvič zazna črno polovico vztrajnika začne meriti čas, ko gre mimo naslednič
        spet prebere in dobi dt/periodo, iz katere izračuna rpm!!!
        '''
        if (self.j % 2) == 0:
            self.t1 = time.time_ns()
            self.j += 1
        elif (self.j % 2) == 1:
            self.t2 = time.time_ns()
            self.j += 1
            perioda = self.t2 - self.t1  # dt!
            self.rps = 1e09 / perioda  # revolutions per second/frekvenca
            self.rpm = self.rps * 60  # revolutions per minute

            if len(rps_seznam) <= 3:
                rps_seznam.append(self.rps)
            else:
                rps_seznam.pop(0)
                rps_seznam.append(self.rps)

            self.rps_avr = sum(rps_seznam) / len(rps_seznam)


#  initializacija gumbov
gumb_run = Button(24)
gumb_calibration = Button(25)

# initializacija motorja
motor = Motor(4, 23, 27)

#  za risanje grafov
x = []  # čas
y1 = []  # u, PID signal ki ga pošiljam motorju
y2 = []  # dejanski naklon (odziv sistema)

#  initializacija senzorja naklona
# mpu = MPU6050()
mpu = MPU6050(-353.8232, -344.5672, -41.6718, -0.1857013671875, 0.009913330078125, 0.88972685546875)
mpu.MPU_init()

#  initializacija merilnika hitrosti
sensor = Button(17)

led_run = LED(6)  # 5
led_ok = LED(5)  # 13
led_cal = LED(13)  # 6

try:
    while True:
        led_cal.off()
        led_ok.off()
        led_run.off()

        if gumb_run.is_pressed:
            print('running...')
            led_run.on()

            mpu.reset_position()
            dt = 0
            i = 0
            cas = 0  # za risanje grafov

            # merjenje RPM/RPS
            rps_seznam = []  # seznam za average (trenutno 5)
            sensor.when_pressed = motor.meri_RPM

            #  PID konstante + par varnostnih zadev
            shut_down_limit = 7.5
            sum_g_y = 0

            Kp = 22.5  # koef. za PID (za ° 0.85, 0.6, 1.0), (za rad 30, 35, 36, 35, 27, 21)
            Ki = 0.0175  # = 1/Ti (Ti = integralini čas (isti kot dt?)) (za ° 0.1, 0.3, 0.1) (za rad 0.05, 0.05, 0.075, 0.02, 0.0175)
            Kd = 0.5  # časovna stabilnost (za ° 0.2, 0.05, 0.07) (za rad 12, 8, 2, 1.5, 0.75)

            run = True

            try:
                while run:
                    # glavni loop, berem g_y, senzor se najbolje odziva v tej osi
                    start = time.time()  # funkcija get_data potrebuje celotni cas operacije (ne le dt med merjenjem = 0.0056 približno)
                    mpu.get_data(dt, print_state=False)

                    # vmesna kalibracija
                    if motor.rps_avr > 40:
                        led_ok.on()
                        if motor.direction == 1:
                            mpu.g_y += 0.012
                        else:
                            mpu.g_y -= 0.012
                    else:
                        led_ok.off()

                    sum_g_y += np.deg2rad(mpu.g_y)
                    u = (Kp * (np.deg2rad(mpu.g_y))) + (Ki * mpu.vg_y) + (Kd * sum_g_y)  # po PID (noče ostati na 0!!!)
                    i = np.interp(u, [-1, 1], [-100, 100])

                    motor.run(i)

                    # pogoji za izhod iz zanke (naklon, gumbi)
                    if mpu.g_y >= shut_down_limit or mpu.g_y <= -shut_down_limit or gumb_run.is_pressed and gumb_calibration.is_pressed:
                        run = False

                    cas += dt
                    x.append(cas)
                    y1.append(u)
                    y2.append(mpu.g_y)

                    dt = time.time() - start

            except KeyboardInterrupt:
                run = False
                print('keyboard interrupt')

            except:
                run = False
                print('nepričakovan error')

            # ugasne PWM, ustavi motor ko me vrže iz loopa
            print('ugašam...')

            led_run.off()
            led_ok.off()
            motor.stop(3)


        elif gumb_calibration.is_pressed:
            led_cal.on()
            mpu.calibrate(2000)
            led_cal.off()
            led_ok.on()
        else:
            led_ok.on()
            print('čakam input')
            time.sleep(0.1)

except:
    led_cal.off()
    led_ok.off()
    led_run.off()

plotting = False  # Če true nam izriše graf poteka odika od ravnovesne lege

if plotting == True:
    plt.figure(figsize=(12, 6))
    plt.title('Kocka debug')
    plt.plot(x, y1, label='u [0-1, kao]')
    plt.plot(x, y2, label='nagib [°]')
    plt.xlabel('čas [s]')
    plt.legend()
    plt.grid()
    plt.show()
