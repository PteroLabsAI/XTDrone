import rclpy
import sys
import numpy
from scipy.optimize import linear_sum_assignment
from rclpy.node import Node
from std_msgs.msg import String,Float32MultiArray,Int32MultiArray
from geometry_msgs.msg import Twist
from formation_dict import formations_for

formation_dict = formations_for(int(sys.argv[2]))
# The pattern goes out at the followers' and the communication node's 30 Hz; the ROS 2 port's 200 Hz burned a core.
LEADER_HZ = 30.0

class Leader(Node):
    def __init__(self,uav_type,leader_id,uav_num):
        self.id=leader_id
        self.uav_num=uav_num
        self.formation_config='waiting'
        self.new_formation=formation_dict["origin"]
        self.communication_topology=None
        self.timer_period=1.0/LEADER_HZ

        super().__init__('leader')
        self.cmd_vel_sub = self.create_subscription(Twist,"/xtdrone/leader/cmd_vel_flu",self.cmd_vel_callback,10)
        self.leader_cmd_sub = self.create_subscription(String,"/xtdrone/leader/cmd",self.cmd_callback,10)

        self.formation_pattern_pub =self.create_publisher(Float32MultiArray,'/xtdrone/formation_pattern',10)
        self.communication_topology_pub =self.create_publisher(Int32MultiArray,'/xtdrone/communication_topology',10)
        # The leader is steered in its own body frame, as the ROS 1 leader was.
        self.vel_flu_pub =self.create_publisher(Twist,'/xtdrone/'+uav_type+'_'+str(self.id)+'/cmd_vel_flu',10)
        self.cmd_pub =self.create_publisher(String,'/xtdrone/'+uav_type+'_'+str(self.id)+'/cmd',10)
        
        self.timer=self.create_timer(self.timer_period,self.timer_callback)

    def cmd_vel_callback(self, msg):
        # Passed straight through, so the communication node sees the sender stop when it stops.
        self.vel_flu_pub.publish(msg)

    def cmd_callback(self, msg):
        if msg.data in formation_dict.keys():
            self.formation_config = msg.data
            print("Formation pattern: ", self.formation_config)
            target = formation_dict[self.formation_config]
            # cost[i, j]: how far follower i flies from the slot it holds now to slot j of the new pattern.
            cost = numpy.linalg.norm(self.new_formation[:, :, None] - target[:, None, :], axis=0)
            _, slot = linear_sum_assignment(cost)
            self.new_formation = target[:, slot]
            self.communication_topology = self.get_communication_topology(self.new_formation)
        else:
            # Commands are events: forwarded once, so a refused one can simply be sent again.
            self.cmd_pub.publish(msg)

    def get_communication_topology(self, rel_posi):

        c_num = int((self.uav_num) / 2)
        min_num_index_list = [0] * c_num

        comm = [[] for i in range(self.uav_num)]
        communication = numpy.ones((self.uav_num, self.uav_num)) * 0
        nodes_next = []
        node_flag = [self.uav_num - 1]
        node_mid_flag = []

        rel_d = [0] * (self.uav_num - 1)

        for i in range(0, self.uav_num - 1):
            rel_d[i] = pow(rel_posi[0][i], 2) + pow(rel_posi[1][i], 2) + pow(rel_posi[2][i], 2)

        c = numpy.copy(rel_d)
        c.sort()
        count = 0

        for j in range(0,c_num):
            for i in range(0,self.uav_num-1):
                if rel_d[i] == c[j]:
                    if not i in node_mid_flag:
                        min_num_index_list[count] = i
                        node_mid_flag.append(i)
                        count = count + 1
                        if count == c_num:
                            break
            if count == c_num:
                break

        for j in range(0, c_num):
            nodes_next.append(min_num_index_list[j])

            comm[self.uav_num - 1].append(min_num_index_list[j])

        size_ = len(node_flag)

        while (nodes_next != []) and (size_ < (self.uav_num - 1)):

            next_node = nodes_next[0]
            nodes_next = nodes_next[1:]
            min_num_index_list = [0] * c_num
            node_mid_flag = []
            rel_d = [0] * (self.uav_num - 1)
            for i in range(0, self.uav_num - 1):

                if i == next_node or i in node_flag:

                    rel_d[i] = 2000
                else:

                    rel_d[i] = pow((rel_posi[0][i] - rel_posi[0][next_node]), 2) + pow(
                        (rel_posi[1][i] - rel_posi[1][next_node]), 2) + pow((rel_posi[2][i] - rel_posi[2][next_node]),
                                                                            2)
            c = numpy.copy(rel_d)
            c.sort()
            count = 0

            for j in range(0, c_num):
                for i in range(0, self.uav_num - 1):
                    if rel_d[i] == c[j]:
                        if not i in node_mid_flag:
                            min_num_index_list[count] = i
                            node_mid_flag.append(i)
                            count = count + 1
                            if count == c_num:
                                break
                if count == c_num:
                    break
            node_flag.append(next_node)

            size_ = len(node_flag)

            for j in range(0, c_num):

                if min_num_index_list[j] in node_flag:

                    nodes_next = nodes_next

                else:
                    if min_num_index_list[j] in nodes_next:
                        nodes_next = nodes_next
                    else:
                        nodes_next.append(min_num_index_list[j])

                    comm[next_node].append(min_num_index_list[j])

        for i in range(0, self.uav_num):
            for j in range(0, self.uav_num - 1):
                if i == 0:
                    if j in comm[self.uav_num - 1]:
                        communication[j + 1][i] = 1
                    else:
                        communication[j + 1][i] = 0
                else:
                    if j in comm[i - 1] and i < (j+1):
                        communication[j + 1][i] = 1
                    else:
                        communication[j + 1][i] = 0
            
        for i in range(1, self.uav_num):  # 防止某个无人机掉队
            if sum(communication[i]) == 0:
                communication[i][0] = 1
        return communication


    def timer_callback(self):
        formation_pattern=Float32MultiArray()
        data1=self.new_formation.flatten().tolist()
        data2=[]
        for i in data1:
            j=float(i)
            data2.append(j)
        formation_pattern.data=data2
        self.formation_pattern_pub.publish(formation_pattern)
        if(not self.communication_topology is None):
            communication_topology = Int32MultiArray()
            topology1=self.communication_topology.flatten().tolist()
            topology2=[]
            for i in topology1:
                j=int(i)
                topology2.append(j)
            communication_topology.data =topology2
            self.communication_topology_pub.publish(communication_topology)


def main():
    rclpy.init()
    leader=Leader(sys.argv[1],0,int(sys.argv[2]))
    rclpy.spin(leader)
    rclpy.shutdown()

if __name__ == '__main__':
    main()  

