import numpy as np
import json

full_data = []

with open("IMUDATASTANDINGFINAL.txt", 'r') as f:
    lines = f.readlines()
    lines = lines [::-1]

    total_lines = len(lines)

    prev = [[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0]]
    frame = [[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0]]

    frame_queue = []
    counter = 0

    for i in lines:
        if len(frame_queue) == 30:
            frame_queue_copy = []
            for j in range(0,len(frame_queue),1):
                frame_queue_copy.append([])
                frame_queue_copy[j].append(frame_queue[j][0][:])
                frame_queue_copy[j].append(frame_queue[j][1][:])
                frame_queue_copy[j].append(frame_queue[j][2][:])
                frame_queue_copy[j].append(frame_queue[j][3][:])
                frame_queue_copy[j].append(frame_queue[j][4][:])
                frame_queue_copy[j].append(frame_queue[j][5][:])
            full_data.append(frame_queue_copy)

            frame_queue.pop(0)
            frame_queue.pop(0)
            frame_queue.pop(0)
            frame_queue.pop(0)
            frame_queue.pop(0)
            frame_queue.pop(0)
            frame_queue.pop(0)
            frame_queue.pop(0)
            frame_queue.pop(0)
            frame_queue.pop(0)

        if len(i) > 5:
            if i.startswith("MPU"):
                frame_queue = []
                prev = [[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0]]
                frame = [[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0]]

            elif i.startswith("S") or i.startswith("W") or i.startswith("U") or i.startswith("D"):
                break

            else:
                imu_num = int(i[0])
                print(counter)

                ax = int(i[i.find("AX")+3:i.find("AY")-1])
                frame[imu_num][0] = (abs(prev[imu_num][0] - ax)) / 32768
                prev[imu_num][0] = ax

                ay = int(i[i.find("AY")+3:i.find("AZ")-1])
                frame[imu_num][1] = (abs(prev[imu_num][1] - ay)) / 32768
                prev[imu_num][1] = ay

                az = int(i[i.find("AZ")+3:i.find("GX")-1])
                frame[imu_num][2] = (abs(prev[imu_num][2] - az)) / 32768
                prev[imu_num][2] = az

                gx = int(i[i.find("GX")+3:i.find("GY")-1])
                frame[imu_num][3] = (abs(prev[imu_num][3] - gx)) / 32768
                prev[imu_num][3] = gx

                gy = int(i[i.find("GY")+3:i.find("GZ")-1])
                frame[imu_num][4] = (abs(prev[imu_num][4] - gy)) / 32768
                prev[imu_num][4] = gy

                gz = int(i[i.find("GZ")+3:len(i)])
                frame[imu_num][5] = (abs(prev[imu_num][5] - gz)) / 32768
                prev[imu_num][5] = gz

                counter = counter + 1
                

                if imu_num == 5:
                    new_frame = []
                    new_frame.append(frame[0][:])
                    new_frame.append(frame[1][:])
                    new_frame.append(frame[2][:])
                    new_frame.append(frame[3][:])
                    new_frame.append(frame[4][:])
                    new_frame.append(frame[5][:])
                    frame_queue.append(new_frame)

with open("standing_data.json", "w") as file:
    json.dump(full_data, file)