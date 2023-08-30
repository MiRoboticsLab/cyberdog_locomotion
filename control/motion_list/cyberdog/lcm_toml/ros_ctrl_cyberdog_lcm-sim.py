import lcm
import sys
import os
import numpy as np
import time

sys.path.append('../../common/lcm_type/lcm')
import motion_control_request_lcmt


def findAllFile(base):
    for root, ds, fs in os.walk(base):
        for f in fs:
            yield f

def main():
    lc=lcm.LCM("udpm://239.255.76.67:7671?ttl=255")

    msg=motion_control_request_lcmt.motion_control_request_lcmt()

    base='../motion_list/'
    num=0
    filelist=[]
    for i in findAllFile(base):
        filelist.append(i)
        print(str(num)+","+str(filelist[num]))
        num=num+1
    print('Input a lcm ctrl file number:')
    numInput=int(input())
    motionFile=open(os.path.join(base,filelist[numInput]),"r")
    motionLines=motionFile.readlines()
    motionFile.close()
    num=0
    count=0
    while num<len(motionLines):
        line=motionLines[num].strip('\n')
        if len(line)>0:
            if line[0]>"#" or line[0]<"#":
                if line=="cmd":
                    msg.pattern=np.int8(motionLines[num+1].strip('\n'))
                    for i in range(3):
                        msg.linear[i]=float(motionLines[num+2].strip('\n').split(",",2)[i])
                        msg.angular[i]=float(motionLines[num+3].strip('\n').split(",",2)[i])
                        msg.point[i]=float(motionLines[num+4].strip('\n').split(",",2)[i])
                    for i in range(4):
                        msg.quaternion[i]=float(motionLines[num+5].strip('\n').split(",",3)[i])
                    msg.body_height=float(motionLines[num+6].strip('\n'))
                    msg.gait_height=float(motionLines[num+7].strip('\n'))
                    msg.order=np.int8(motionLines[num+8].strip('\n'))
                    lc.publish("exec_request",msg.encode())
                    count=count+1
                    if count%5 == 0:
                        print('cmd exec_request lcm publish pattern =',msg.pattern)
                    time.sleep( 0.2 )
        num=num+1
        if num>=len(motionLines):
            num=0
            motionFile=open(os.path.join(base,filelist[numInput]),"r")
            motionLines=motionFile.readlines()
            motionFile.close()


if __name__ == '__main__':
    main()
