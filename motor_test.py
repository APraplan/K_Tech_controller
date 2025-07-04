from position_controller import PositionController
from K_Tech_controller import K_Tech_RS485
import time
import keyboard

motor = K_Tech_RS485('/dev/ttyUSB0')  # or COMx on Windows
motor_id = 1

motor.motor_on(motor_id)

angle = motor.read_multi_loop_angle(motor_id)
time.sleep(0.5)

jump = False
move_up = True
done = True

motor.multi_loop_angle2(motor_id, angle, 360)

while True:
    if keyboard.is_pressed("Q"):
        motor.motor_stop(motor_id)
        motor.motor_off(motor_id)
            
        motor.close()
        break

    if keyboard.is_pressed("W"):
        print("Jumping")
        if done == True:
            pc = PositionController(angle, angle - 180)
            jump = True

    if jump:
        motor.multi_loop_angle2(motor_id, pc.get_pos(), 360)
        if move_up and pc.target_reached():

            move_up = False
            pc = PositionController(angle - 180, angle)

        elif not move_up and pc.target_reached():
            move_up = True
            jump = False
            done = True

    else:
        motor.read_state3(motor_id)
    

    # if move_up:
    #     motor.multi_loop_angle2(motor_id, pc.get_pos(), 360)

    #     if pc.target_reached():
    #         move_up = False
    #         pc = PositionController(angle + 90, angle)

    # else:
    #     motor.multi_loop_angle2(motor_id, pc.get_pos(), 360)

    #     if pc.target_reached():
    #         move_up = True
    #         pc = PositionController(angle, angle + 90)


