#!/usr/bin/env python
#-*-coding:utf-8-*_

## talker demo that published std_msgs/ColorRGBA messages
## to the 'color' topic. To see these messages, type:
## rostopic echo color
## this demo shows some of the more advanced APIs in rospy
import os, sys,time
from scipy.integrate import quad
from scipy.interpolate import InterpolatedUnivariateSpline
from visualization_msgs.msg import Marker
import numpy as np
import math
import roslib
#roslib.load_manifest('insgps_y')
import rospy
import matplotlib.pyplot as plt
# from macaron.msg import Floats
from macaron.msg import base_frame
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Int32,ColorRGBA, Header  
from sensor_msgs.msg import NavSatFix
PI=math.acos(-1)
present_num=0




class GPSTxT_leader():
        def __init__(self):
                global offset_X,offset_Y,bm,now_po,now_msg
                self.base_frame=rospy.Publisher('base_frame',base_frame,queue_size=3)
                self.big_node_num=rospy.Publisher('index',Int32,queue_size=1)
                self.plot = rospy.Publisher('/track', Marker, queue_size=100)
                self.now_plot = rospy.Publisher('/driving', Marker, queue_size=3)
                
                ## Eightlines position 
                self.offset_X=550922
                self.offset_Y=199952
                ## K-city position
                # self.offset_X=515540
                # self.offset_Y=179847
                self.map_initializer()
                ## realtime plot



                self.GPSTxT_leader()
                
        def map_initializer(self):
            global p,poly,dt,Nsample,b,p_length,leng,direction

        #    a=np.loadtxt("catkin_ws/src/macaron/scripts/eightlight/8line.txt",delimiter=',',dtype='double')
        #    a=np.loadtxt("catkin_ws/src/macaron/scripts/eightlight/8line_8.12_proper.txt",delimiter=',',dtype='double')
        #    a=np.loadtxt("catkin_ws/src/macaron/scripts/eightlight/8line_8.12_rev.txt",delimiter=',',dtype='double')
        #    a=np.loadtxt("catkin_ws/src/macaron/scripts/eightlight/C_c8.txt",delimiter=',',dtype='double')
        #    a=np.loadtxt("catkin_ws/src/macaron/scripts/eightlight/C_cw8.txt",delimiter=',',dtype='double')
            a=np.loadtxt("catkin_ws/src/macaron/scripts/eightlight/C_won4.txt",delimiter=',',dtype='double')
        #    a=np.loadtxt("catkin_ws/src/macaron/scripts/eightlight/hyehwa.txt",delimiter=',',dtype='double')
        #    a=np.loadtxt("catkin_ws/src/macaron/scripts/eightlight/hywhwa_comback.txt",delimiter=',',dtype='double')
        #    a=np.loadtxt("catkin_ws/src/macaron/scripts/eightlight/won4.txt",delimiter=',',dtype='double')
                        
        #    a=np.loadtxt("catkin_ws/src/macaron/scripts/eightlight/C_won4.txt",delimiter=',',dtype='double')
        #    a=np.loadtxt("catkin_ws/src/macaron/scripts/eightlight/8line_8.12_proper.txt",delimiter=',',dtype='double')
        
            line_name = "catkin_ws/src/macaron/scripts/eightlight/8line_npy"+".npy"     
            np.save(line_name,a)
            b=np.load(line_name)
            self.track_initiate()
        #     print(b)
        #     plt.plot(b[:,0],b[:,1])
        #     plt.grid(True)
        
        #     for i in range(b.shape[0]):
            P=b.T
            leng=int(P.size/2)
            print(leng)
        
            alpha = 0.5 # 접선벡터의 가중치. 이 값에 따라 곡선의 장력이 결정된다.
            dp = np.zeros((2, leng))
        
            for xy in range(0,2):
                for a in range(1,leng-1):
                    dp[xy, a] = (1 - alpha)*(P[xy, a+1] - P[xy, a-1])
        
        #     print(dp)
            dt = 0.1
            Nsample = int(1/ dt)
            p = np.zeros((2, Nsample * (leng-1) + 1))
            poly=np.zeros((4,leng,2))
            print(p.shape)
            p_length=int(p.size/2)

            for big_node in range(0,leng -1):
                for xy in range(0,2):
                    t = 0
                    poly[0][big_node][xy]= +2*P[xy,big_node] -2*P[xy,big_node+1]+1*dp[xy,big_node]+1*dp[xy,big_node + 1]
                    poly[1][big_node][xy]= -3*P[xy,big_node] +3*P[xy,big_node+1]-2*dp[xy,big_node]-1*dp[xy,big_node + 1]
                    poly[2][big_node][xy] = dp[xy,big_node]
                    poly[3][big_node][xy] = P[xy,big_node]
                    for small_node in range(0,Nsample):
                        p[xy][small_node + Nsample * big_node] = poly[0][big_node][xy]*t*t*t + poly[1][big_node][xy]*t*t + poly[2][big_node][xy]*t + poly[3][big_node][xy]
                        t=t+dt
        
            #print(poly)
        
            p[0, Nsample * (leng-1)] = P[0, leng-1]
            p[1, Nsample * (leng-1)] = P[1, leng-1]
        
        #     plt.plot(p[0][:],p[1][:], 'r', label='f(x)')
        #     plt.grid(True)
            #plt.show()

        def listener(self):
            my_State=NavSatFix()
            my_State=rospy.wait_for_message("/fix", NavSatFix)
            lon =my_State.longitude
            lat =my_State.latitude
        
            st_lat = 38.0
            st_lon = 127.0
            k_0 =1.0 
            a = 6378137.0 
            b = 6356752.31 
            f =(a-b)/a 
            d_y = 200000.0 
            d_x = 600000.0 
        
            e = (a*a - b*b) / (a*a) 
            e_2 = (a*a - b*b) / (b*b) 
            M = a * ((1 - e/4.0 - 3.0*e*e/64.0 - 5.0*pow(e,6.0)/256.0)* math.radians(lat) - (3.0*e/8.0 + 3.0*e*e/32.0 + 45.0*e*e*e/1024.0)* math.sin(2* math.radians(lat)) + (15.0*e*e/256.0 + 45.0*e*e*e/1024.0)* math.sin(4.0* math.radians(lat)) - 35.0*e*e*e/3072.0* math.sin(6.0* math.radians(lat))) 
            C = (e / (1-e))* math.cos( math.radians(lat)) 
            T = pow( math.tan( math.radians(lat)),2) 
            A = ( math.radians(lon - st_lon))* math.cos( math.radians(lat)) 
            N = a/ math.sqrt(1-(e)*( math.sin( math.radians(lat))* math.sin( math.radians(lat)))) 
            M_0 = a * ((1 - e/4.0 - 3.0*e*e/64.0 - 5.0*e*e*e/256.00)* math.radians(st_lat) - (3.0*e/8.0 + 3.0*e*e/32.0 + 45.0*e*e*e/1024.0)* math.sin(2.0* math.radians(st_lat)) + (15.0*e*e/256.0 + 45.0*e*e*e/1024.0)* math.sin(4.0* math.radians(st_lat)) - 35.0*e*e*e/3072.0* math.sin(6.0* math.radians(st_lat))) 
            y_tm = (d_y+k_0*N*(A + (A*A*A/6.0)*(1-T+C) + (A*A*A*A*A/120.0) * (5.0 - 18.0*T + T*T + 72.0*C - 58.0*e_2))) 
            x_tm = (d_x + k_0*(M - M_0 + N* math.tan(math.radians(lat))*(A*A/2.0 + (A*A*A*A/24.0)*(5.0-T+9.0*C+4.0*C*C)+(A*A*A*A*A*A/720.0)*(61.0-58.0*T+T*T+600.0*C-330.0*e_2)))) 
            #print(x_tm,y_tm)
            return x_tm,y_tm

        def track_initiate(self):
                global b, now_po
                count = 0
                bm=np.zeros((b.shape[0],2))
                for i in range(0,b.shape[0]-1):
                    bm[i][0]=b[i][0]-self.offset_X
                    bm[i][1]=b[i][1]-self.offset_Y
                po=Point()
                ## plotting code about text, if you want to see realtime see ins_y node
                for i in range(b.shape[0]):
                    if count<b.shape[0]:
                        rviz_msg = Marker(
                            type=Marker.POINTS,
                            # points=Point(bm[i][0],bm[i][1],0),
                            lifetime=rospy.Duration(0),
                            scale=Vector3(0.5,0.5,0.1),
                            header=Header(frame_id='map'),
                            color=ColorRGBA(0.0, 0.0, 1.0, 0.8)
                            )
                        po.x=bm[count][0]
                        po.y=bm[count][1]
                        po.z=0
                        ##for smooth plotting
                        rospy.sleep(0.02)
                        rviz_msg.points=[po]
                        rviz_msg.id=count
                        # rviz_msg.points.x=p.x
                        self.plot.publish(rviz_msg)
                        count+=1
            

        def GPSTxT_leader(self):
                global distance_difference,present_num,search,b, now_po
                base=base_frame()
                node_index=Int32()
                while not rospy.is_shutdown():
                        my_x,my_y=self.listener()

                        path_leng=0
                        search=0
                        a=0
                        a_grad=0
                        distance_difference=100

                        while(abs(distance_difference)>0.01):
                                distance_difference=(my_x-p[0][present_num+1])**2 + (my_y-p[1][present_num+1])**2-(my_x-p[0][present_num])**2-(my_y-p[1][present_num])**2
                                search=search+1
                        
                                a=-distance_difference/(1+search/100)
                        
                                if(a>-1 and a<0):
                                        a=-1
                                elif(a<1 and a>0):
                                        a=1
                                
                                
                                present_num=present_num+int(a)
                                if present_num>p_length-2:
                                        present_num=p_length-2
                                        break                        
                                elif present_num<0:
                                        present_num=0
                                        break
                                elif (search==10):
                                        break
                    
                        plt.plot(my_x,my_y, 'ro', label='position')

                        Fsum=0
                        s=[0]
                    
                        while(Fsum<10):
                                big_node=int((present_num+path_leng)/Nsample)
                                if(big_node>leng-1):
                                        break

                                ft = lambda t: ((3* poly[0][big_node][1]*t*t + 2*poly[1][big_node][1]*t + poly[2][big_node][1])**2+(3* poly[0][big_node][0]*t*t + 2*poly[1][big_node][0]*t + poly[2][big_node][0])**2)**0.5
                                for small_node in range(0,Nsample):
                                        F,err=quad(ft, float(small_node)/Nsample,float(small_node+1)/Nsample)
                                        Fsum=Fsum+F
                                        path_leng=path_leng+1
                                        if Fsum>10:
                                                break
                                        elif present_num+path_leng>p_length-1:
                                                break
                                        s.append(Fsum)

                        #print(len(s),path_leng)                    
                                #s=[Fsum]
                        if(len(s)>2):
                                distance=((my_x-p[0][present_num])**2 + (my_y-p[1][present_num])**2)**0.5      
                                fpx=np.poly1d(np.polyfit(s,p[0][present_num:present_num+path_leng],3))
                                fpy=np.poly1d(np.polyfit(s,p[1][present_num:present_num+path_leng],3))
                                ix=fpx([0,1,2,3,4,5,6,7,8,9,10])
                                iy=fpy([0,1,2,3,4,5,6,7,8,9,10])
                                iangle=[]
                                idistance=[]
                                for scurve in range(0,Nsample):
                                        iangle.append(math.atan2(iy[scurve+1]-iy[scurve],ix[scurve+1]-ix[scurve])*180/PI)
                                        idistance.append(math.sqrt((iy[scurve+1]-iy[scurve])**2+(ix[scurve+1]-ix[scurve])**2))
                                
                                a=ix[10]-ix[0]
                                b=iy[10]-iy[0]
                                c=my_x-ix[0]
                                d=my_y-iy[0]

                                if (a*d-b*c)>0:
                                        direction=1
                                else:
                                        direction=-1



                                now_po=Point()
                                now_msg = Marker(
                                        type=Marker.POINTS,
                                        # points=Point(bm[i][0],bm[i][1],0),
                                        lifetime=rospy.Duration(0),
                                        scale=Vector3(0.5,0.5,0.1),
                                        header=Header(frame_id='map'),
                                        color=ColorRGBA(0.0, 1.0, 0.0, 0.8)
                                        )

                                now_po.x=p[0][present_num]-self.offset_X
                                now_po.y=p[1][present_num]-self.offset_Y
                                now_po.z=0

                                now_msg.points=[now_po]
                                now_msg.id=6
                                # rviz_msg.points.x=p.x
                                self.now_plot.publish(now_msg)
                                
                                print("------------------------------")
                                print(direction,distance)
                                print(present_num,search,s[len(s)-1])
                                print(my_x,my_y)
                                print("------------------------------")

                                #                        print(ix)                        print(iy)                        print(iangle)                        print(idistance)

                                # now_po.x=my_x-self.offset_X
                                # now_po.y=my_y-self.offset_Y
                                # now_po.z=0
                                # now_po.points=[po]
                                # now_po.id=0

                                # rviz_msg.points.x=p.x
                                # self.now_plot.publish(now_po)
                                node_index.data=int(present_num/Nsample)
                                base.distance=direction*distance
                                base.s_x=ix
                                base.s_y=iy
                                base.s_a=iangle
                                base.s_d=idistance
                                base.tm_x=my_x
                                base.tm_y=my_y
                                self.big_node_num.publish(node_index)
                                self.base_frame.publish(base)

def main():
        rospy.init_node('GPS_reader')
        try:
                GPSTxT_leader()

        except rospy.ROSInterruptException:
                pass


## start code
if __name__ == '__main__':
        main()
