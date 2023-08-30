import lcm
import sys
import os
sys.path.append('../../common/lcm_type/lcm')
import trajectory_command_lcmt


def findAllFile(base):
    for root, ds, fs in os.walk(base):
        for f in fs:
            yield f

def main():
    lc=lcm.LCM("udpm://239.255.76.67:7671?ttl=255")

    msg=trajectory_command_lcmt.trajectory_command_lcmt()


    

    base='../motion_list/'
    num=0
    filelist=[]
    for i in findAllFile(base):
        filelist.append(i)
        print(str(num)+","+str(filelist[num]))
        num=num+1
    print('Input a motion file number:')
    numInput=int(input())
    motionFile=open(os.path.join(base,filelist[numInput]),"r")
    motionLines=motionFile.readlines()
    motionFile.close()
    num=0
    msg.duration=0
    while num<len(motionLines):
        
        line=motionLines[num].strip('\n')
        if len(line)>0:
            if line[0]>"#" or line[0]<"#":
                if line=="pose":
                    msg.motionType=str(line)
                    for i in range(4):
                        msg.pose_foot_support[i]=float(motionLines[num+1].strip('\n').split(",",3)[i])
                    for i in range(6):
                        msg.pose_body_cmd[i]=float(motionLines[num+2].strip('\n').split(",",5)[i])
                    for i in range(3):
                        msg.pose_foot_cmd[i]=float(motionLines[num+3].strip('\n').split(",",2)[i])
                        msg.pose_ctrl_point[i]=float(motionLines[num+4].strip('\n').split(",",2)[i])
                    msg.duration=int(motionLines[num+5].strip('\n'))
                    lc.publish("motion-list",msg.encode())

                
                else:
                    if line=="locomotion":
                        msg.motionType=str(line)
                        for i in range(3):
                            msg.locomotion_vel[i]=float(motionLines[num+1].strip('\n').split(",",2)[i])
                        msg.locomotion_omni=int(motionLines[num+2].strip('\n'))
                        msg.locomotion_gait=int(motionLines[num+3].strip('\n'))
                        msg.duration=int(motionLines[num+4].strip('\n'))
                        lc.publish("motion-list",msg.encode())
                    else:
                        if line=="jump":
                            msg.motionType=str(line)
                            for i in range(4):
                                msg.jump_contact[i]=float(motionLines[num+1].strip('\n').split(",",3)[i])
                            for i in range(3):
                                msg.jump_x_acc[i]=float(motionLines[num+2].strip('\n').split(",",2)[i])
                                msg.jump_w_acc[i]=float(motionLines[num+3].strip('\n').split(",",2)[i])
                                msg.duration=int(motionLines[num+4].strip('\n'))
                            lc.publish("motion-list",msg.encode())
                        else:
                            if line=="transition":
                                msg.motionType=str(line)
                                msg.trans_height=float(motionLines[num+1].strip('\n'))
                                msg.duration=int(motionLines[num+2].strip('\n'))
                                lc.publish("motion-list",msg.encode())
                            else:
                                if line=="swingleg":
                                    msg.motionType=str(line)
                                    for i in range(4):
                                        msg.pose_foot_support[i]=float(motionLines[num+1].strip('\n').split(",",3)[i])
                                    for i in range(6):
                                        msg.pose_body_cmd[i]=float(motionLines[num+2].strip('\n').split(",",5)[i])
                                    for i in range(3):
                                        msg.pose_foot_cmd[i]=float(motionLines[num+3].strip('\n').split(",",2)[i])
                                        msg.pose_ctrl_point[i]=float(motionLines[num+4].strip('\n').split(",",2)[i])
                                    msg.duration=int(motionLines[num+5].strip('\n'))
                                    lc.publish("motion-list",msg.encode())
                                else:
                                     if line=="torctrlposture":
                                        msg.motionType=str(line)
                                        for i in range(4):
                                            msg.pose_foot_support[i]=float(motionLines[num+1].strip('\n').split(",",3)[i])
                                        # for i in range(4):
                                        #     msg.stateEstate_foot_support[i]=float(motionLines[num+2].strip('\n').split(",",3)[i])
                                        for i in range(6):
                                            msg.pose_body_cmd[i]=float(motionLines[num+2].strip('\n').split(",",5)[i])
                                        for i in range(3):
                                            msg.pose_foot_cmd[i]=float(motionLines[num+3].strip('\n').split(",",2)[i])
                                        msg.duration=int(motionLines[num+4].strip('\n'))
                                        lc.publish("motion-list",msg.encode())

        num=num+1
if __name__ == '__main__':
    main()
