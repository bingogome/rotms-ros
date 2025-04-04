"""
MIT License

Copyright (c) 2022 Yihao Liu, Johns Hopkins University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import rospy, rospkg, math
from std_msgs.msg import String

import trimesh, numpy, yaml

def app():
    """
    Node content
    """
    rospy.init_node('NodeICP')
    rospy.Subscriber("/ICP/DoICP", String, callback)
    rospy.spin()

def callback(data):
    """
    ICP request callback
    """
    rospy.loginfo("[ROTMS INFO] " + "Starting ICP ...")
    initmat, dig = initICP()
    rospy.loginfo("[ROTMS INFO] " + "ICP initialized.")
    rospy.loginfo("[ROTMS INFO] " + "ICP Started.")
    matrix, transformed, cost = icp(dig, data.data, initmat)
    matrix = numpy.linalg.inv(matrix) 
    rospy.loginfo("[ROTMS INFO]" + "Result quaternion: ")
    resrot = mat2quat(matrix[0:3,0:3])
    print(resrot)
    rospy.loginfo("[ROTMS INFO]" + "Result translation: ")
    respos = matrix[0:3,3] / 1000.0
    print(respos)
    rospy.loginfo("[ROTMS INFO]" + "ICP completed. Cost: %s", str(cost))

    # save results
    rospack = rospkg.RosPack()
    datapath = rospack.get_path('rotms_ros_operations')
    regpath = datapath + "/share/config/reg.yaml"
    f = open(regpath, "w")
    filecontent = \
        "TRANSLATION: # translation: x,y,z\n" + \
        "\n" + \
        "  {\n" + \
        "    x: " + str(respos[0]) + ",\n" + \
        "    y: " + str(respos[1]) + ",\n" + \
        "    z: " + str(respos[2]) + "\n" + \
        "  }\n" + \
        "\n" + \
        "ROTATION: # quat: x,y,z,w\n" + \
        "\n" + \
        "  {\n" + \
        "    x: " + str(resrot[0]) + ",\n" + \
        "    y: " + str(resrot[1]) + ",\n" + \
        "    z: " + str(resrot[2]) + ",\n" + \
        "    w: " + str(resrot[3]) + "\n" + \
        "  }\n" + \
        "\n" + \
        "FLAG_ICP: 1 # Flag for whether generated by ICP"
    f.write(filecontent)
    f.close()

def initICP():
    """
    Initialization of the ICP
    """
    # Load the initial registration result from pair-point reg
    rospack = rospkg.RosPack()
    datapath = rospack.get_path('rotms_ros_operations')
    regpath = datapath + "/share/config/reg.yaml"
    with open(regpath.strip(), "r") as stream:
        try:
            reg = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
            return
    rot = [reg["ROTATION"]["x"],reg["ROTATION"]["y"],reg["ROTATION"]["z"],reg["ROTATION"]["w"]]
    rot = quat2mat(rot)
    # convert ROS m unit to mm
    p = [reg["TRANSLATION"]["x"] * 1000.0,reg["TRANSLATION"]["y"] * 1000.0,reg["TRANSLATION"]["z"] * 1000.0]
    rot_ = numpy.array(rot).transpose()
    p_ = -numpy.matmul(rot_, numpy.array(p).reshape((3,1)))
    initmat = numpy.concatenate((rot_,p_), axis=1)
    initmat = numpy.concatenate((initmat, numpy.array([[0.0,0.0,0.0,1.0]])))
    # Load the digitized point cloud
    digpath = datapath + "/share/config/icpdig.yaml"
    with open(digpath.strip(), "r") as stream:
        try:
            dig_dict = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
            return

    dig = []
    for k in dig_dict.keys():
        # convert ROS m unit to mm
        dig.extend([float(i) * 1000.0 for i in dig_dict[k].strip().split(',')[:-1]])
        
    dig = numpy.array(dig).reshape((-1,3))

    return initmat, dig

def icp(dig, meshpath, initmat):
    """
    Call ICP from trimesh package
    """
    mesh = trimesh.load_mesh(meshpath)
    matrix, transformed, cost = trimesh.registration.icp(dig, mesh, threshold=0.5, scale=False, reflection=False, initial=initmat, max_iterations=300)
    return matrix, transformed, cost

def quat2mat(q):
    qx, qy, qz, qw = q[0], q[1], q[2], q[3]
    mat = [ \
        [1-2*qy*qy-2*qz*qz, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw], \
        [2*qx*qy+2*qz*qw, 1-2*qx*qx-2*qz*qz, 2*qy*qz-2*qx*qw], \
        [2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx*qx-2*qy*qy]]
    return mat

def mat2quat(R):
    if R[0][0]+R[1][1]+R[2][2] > 0:
        qw = math.sqrt(1.0 + R[0][0] + R[1][1] + R[2][2]) / 2.0 * 4.0
        x = (R[2][1] - R[1][2]) / qw
        y = (R[0][2] - R[2][0]) / qw
        z = (R[1][0] - R[0][1]) / qw
        # print(x,y,z,qw)
        return [x,y,z,qw/4]
    elif (R[0][0] > R[1][1]) and (R[0][0] > R[2][2]):
        s = math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2.0
        qw = (R[2][1] - R[1][2]) / s
        qx = 0.25 * s
        qy = (R[0][1] + R[1][0]) / s
        qz = (R[0][2] + R[2][0]) / s
        return [qx, qy, qz, qw]
    elif R[1][1] > R[2][2]:
        s = math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2.0
        qw = (R[0][2] - R[2][0]) / s
        qx = (R[0][1] + R[1][0]) / s
        qy = 0.25 * s
        qz = (R[1][2] + R[2][1]) / s
        return [qx, qy, qz, qw]
    else:
        s = math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2.0
        qw = (R[1][0] - R[0][1]) / s
        qx = (R[0][2] + R[2][0]) / s
        qy = (R[1][2] + R[2][1]) / s
        qz = 0.25 * s
        return [qx, qy, qz, qw]



"""
Start the node
"""
if __name__ == '__main__':
    try:
        app()
    except rospy.ROSInterruptException:
        pass
