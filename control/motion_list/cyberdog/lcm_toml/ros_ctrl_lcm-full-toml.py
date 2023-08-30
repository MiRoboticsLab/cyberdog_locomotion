import lcm
import sys
import os
import time
import toml

sys.path.append('../../common/lcm_type/lcm')
import robot_control_cmd_lcmt

def findAllFile(base):
    for root, ds, fs in os.walk(base):
        for f in fs:
            yield f

def main():
    base='./'
    num=0
    filelist=[]
    for i in findAllFile(base):
        filelist.append(i)
        print(str(num)+","+str(filelist[num]))
        num=num+1
    print('Input a toml ctrl file number:')
    numInput=int(input())

    lc=lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
    msg=robot_control_cmd_lcmt.robot_control_cmd_lcmt()
    file = os.path.join(base,filelist[numInput])
    print("Load file=%s\n" % file)
    steps = toml.load(file)
    for step in steps['step']:
        msg.mode = step['mode']
        msg.value = step['value']
        msg.contact = step['contact']
        msg.gait_id = step['gait_id']
        msg.duration = step['duration']
        msg.life_count += 1

        for i in range(3):
            msg.vel_des[i] = step['vel_des'][i]
            msg.rpy_des[i] = step['rpy_des'][i]
            msg.pos_des[i] = step['pos_des'][i]
            msg.acc_des[i] = step['acc_des'][i]
            msg.acc_des[i+3] = step['acc_des'][i+3]
            msg.foot_pose[i] = step['foot_pose'][i]
            msg.ctrl_point[i] = step['ctrl_point'][i]
        for i in range(2):
            msg.step_height[i] = step['step_height'][i]

        lc.publish("robot_control_cmd",msg.encode())
        print('robot_control_cmd lcm publish mode :',msg.mode , "gait_id :",msg.gait_id , "msg.life_count=" , msg.life_count)
        time.sleep( 0.2 )
    for i in range(300): #60s
        lc.publish("robot_control_cmd",msg.encode())
        time.sleep( 0.2 )


if __name__ == '__main__':
    main()
