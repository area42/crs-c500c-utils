.include "rosserial.r3"

const TOPICID_TEST = 256
const TOPICID_JOINTSTATE = 257
const TOPICID_JOINTTRAJECTORY = 258
const TOPICID_COMMAND = 259

main
    int msg_len
    int offset
    int topic_id
    int[2] prevtime
    int[2] nowtime
    int[2] starttime
    ploc robot_pos
    float[8] joint_angles
    float deg_to_rad 
    deg_to_rad = 3.1415926535 / 180.0

    printf("ROS Server starting\n")

    mtime(&prevtime)

    open(g_rosserial_fd, "/dev/sio1", O_RDWR | O_NONBLOCK, 0)
    sio_ioctl_conf serial_conf
    ioctl(g_rosserial_fd, IOCTL_GETC, &serial_conf)
    serial_conf.baud = 115200
    serial_conf.OutxCtsFlow = 1
    serial_conf.OutxDsrFlow = 1
    serial_conf.DtrControl = 1
    serial_conf.DsrSensitivity = 1
    serial_conf.OutX = 0
    serial_conf.InX = 0
    serial_conf.RtsControl = 1
    serial_conf.fifotrig = 2
    ioctl(g_rosserial_fd, IOCTL_PUTC, &serial_conf)
    
    while 1==1
        rosserial_readwrite()
        
        msg_len = rosserial_start_read_packet(topic_id)
        
        if msg_len != 0 and msg_len != -1 then
            printf("Got message for topic {}, len {}\n", topic_id, msg_len)
        end if
        
        if msg_len == 0 and topic_id == 0 then
            ;; Request for topic list
            ;;printf("Sending topic list\n")
            rosserial_publish(TOPICID_JOINTSTATE, "/joint_states", "sensor_msgs/JointState", "3066dcd76a6cfaef579bd0f34173e9fd")
            rosserial_subscribe(TOPICID_JOINTTRAJECTORY, "/joint_trajectory_point", "trajectory_msgs/JointTrajectoryPoint", "f3cd1e1c4d320c79d6985c904ae5dcd3")
            rosserial_subscribe(TOPICID_COMMAND, "/f3_command", "std_msgs/String", "992ce8a1687cec8c8bd883ec73ca41d1")
        elseif msg_len > 0 then
            printf("Got message for topic {}, len {}\n", topic_id, msg_len)
        else
            delay(10)
        end if
        
        rosserial_end_read_packet(msg_len)
        
        mtime(&nowtime)
    
        if nowtime[0] > prevtime[0] + 50 or prevtime[1] < nowtime[1] then
            ;; printf("Sending test message\n")
            ;;offset = rosserial_start_packet(TOPICID_TEST)
            ;;offset = rosmsg_write_string(g_rosserial_txfifo, offset, "Testing!")
            ;;rosserial_end_packet(offset)

            loc_class_set(robot_pos, loc_precision)
            here(robot_pos)
            motor_to_joint(robot_pos, joint_angles)
            

            offset = rosserial_start_packet(TOPICID_JOINTSTATE)
            

            offset = rosmsg_write_header(g_rosserial_txfifo, offset, 0,nowtime, "")
            
            ;; array of joint names
            offset = rosmsg_write_int32(g_rosserial_txfifo, offset, 6)
            offset = rosmsg_write_string(g_rosserial_txfifo, offset, "joint_a1")
            offset = rosmsg_write_string(g_rosserial_txfifo, offset, "joint_a2")
            offset = rosmsg_write_string(g_rosserial_txfifo, offset, "joint_a3")
            offset = rosmsg_write_string(g_rosserial_txfifo, offset, "joint_a4")
            offset = rosmsg_write_string(g_rosserial_txfifo, offset, "joint_a5")
            offset = rosmsg_write_string(g_rosserial_txfifo, offset, "joint_a6")
            
            ;; array of joint positions
            offset = rosmsg_write_int32(g_rosserial_txfifo, offset, 6)
            offset = rosmsg_write_float64(g_rosserial_txfifo, offset, joint_angles[0] * deg_to_rad)
            offset = rosmsg_write_float64(g_rosserial_txfifo, offset, joint_angles[1] * deg_to_rad)
            offset = rosmsg_write_float64(g_rosserial_txfifo, offset, joint_angles[2] * deg_to_rad)
            offset = rosmsg_write_float64(g_rosserial_txfifo, offset, joint_angles[3] * deg_to_rad)
            offset = rosmsg_write_float64(g_rosserial_txfifo, offset, joint_angles[4] * deg_to_rad)
            offset = rosmsg_write_float64(g_rosserial_txfifo, offset, joint_angles[5] * deg_to_rad)
            
            ;; array of joint velocities
            offset = rosmsg_write_int32(g_rosserial_txfifo, offset, 0)
            
            ;; array of joint efforts
            offset = rosmsg_write_int32(g_rosserial_txfifo, offset, 0)
            
            rosserial_end_packet(offset)
            
            mtime(&prevtime)
      end if
    end while
end main
