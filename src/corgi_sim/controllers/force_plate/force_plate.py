from controller import Robot
import csv
import os

robot = Robot()
timestep = int(robot.getBasicTimeStep())

force_plates = [robot.getDevice(f'force_plate_{i}') for i in range(1, 5)]
for plate in force_plates:
    plate.enable(timestep)

columns = [f'{axis}_{i}' for i in range(1, 5) for axis in ['Fx', 'Fy', 'Fz']]

with open(os.path.join(os.getenv('HOME'), 'corgi_ws/corgi_ros_ws/output_data/sim_force_plate.csv'), 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(columns)
    
    while robot.step(timestep) != -1:
        row = [value for plate in force_plates for value in plate.getValues()[:3]]
        row[2] -= 9.81
        row[5] -= 9.81
        row[8] -= 9.81
        row[11] -= 9.81
        
        writer.writerow(row)
