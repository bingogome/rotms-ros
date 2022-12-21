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

import rospy, rospkg
from std_msgs.msg import String

import trimesh, numpy, yaml

def app():
    """
    Node content
    """
    rospy.init_node('NodeICP')
    rospy.Subscriber("/ICP/MeshPath", String, callback)
    rospy.spin()

def callback(data):
    """
    ICP request callback
    """
    rospy.loginfo("[ROTMS INFO] " + "Starting ICP ...")
    initmat, dig = initICP()
    rospy.loginfo("[ROTMS INFO] " + "ICP initialized.")
    # matrix, transformed, cost = icp(dig, data.data, initmat)
    # matrix = numpy.linalg.inv(matrix) 
    # rospy.loginfo("[ROTMS INFO]" + "ICP completed. Cost: %s", str(cost))
    
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
    rot = [reg["ROTATION"]["x"],reg["ROTATION"]["y"],reg["ROTATION"]["z"],reg["ROTATION"]["w"]]
    rot = quat2mat(rot)
    p = [reg["TRANSLATION"]["x"],reg["TRANSLATION"]["y"],reg["TRANSLATION"]["z"]]
    rot_ = numpy.array(rot).transpose()
    p_ = -numpy.matmul(rot_, numpy.array(p).reshape((3,1)))
    initmat = numpy.concatenate((rot_,p_), axis=1)
    initmat = numpy.concatenate((initmat, numpy.array([[0.0,0.0,0.0,1.0]])))
    # Load the digitized point cloud
    digpath = datapath + "/share/config/icpdig.yaml"
    with open(digpath.strip(), "r") as stream:
        try:
            dig = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    dig = numpy.array([float(i) for i in dig["points"].strip().split(',')]).reshape((-1,3))

    return initmat, dig

def icp(dig, meshpath, initmat):
    """
    Call ICP from trimesh package
    """
    mesh = trimesh.load_mesh(meshpath)
    matrix, transformed, cost = trimesh.registration.icp(dig, mesh, initial=initmat, max_iterations=200)
    return matrix, transformed

def quat2mat(q):
    qx, qy, qz, qw = q[0], q[1], q[2], q[3]
    mat = [ \
        [1-2*qy*qy-2*qz*qz, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw], \
        [2*qx*qy+2*qz*qw, 1-2*qx*qx-2*qz*qz, 2*qy*qz-2*qx*qw], \
        [2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx*qx-2*qy*qy]]
    return mat

"""
Start the node
"""
if __name__ == '__main__':
    try:
        app()
    except rospy.ROSInterruptException:
        pass