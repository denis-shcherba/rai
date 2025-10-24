import robotic as ry
import matplotlib.pyplot as plt
import time 

def test_noisy_cam():
    C = ry.Config()
    C.addFile("$RAI_PATH/scenarios/pandaSingle.g")

    CameraView = ry.CameraView(C)
    CameraView.setCamera(C.getFrame("cameraWrist"))
    
    # C.addFrame("box").setPosition([0., .5, 0.8]).setShape(ry.ST.ssBox, [0.2, 0.1, 0.1, .001]).setColor([1,0,0])
    C.addFrame("mesh").setMeshFile("$RAI_PATH/g1/meshes/giraffehd.h5").setPosition([.1,.5,.8])

    C.view(True)

    S = ry.Simulation(C, ry.SimulationEngine.physx)
    S.selectSensor("cameraWrist")
    S.setSimulateDepthNoise(True)


    fx, fy, cx, cy = CameraView.getFxycxy()
    print([fx, fy, cx, cy])
    
    pcl = C.addFrame("pc", "cameraWrist")


    for t in range(1_000):
        img, depth = S.getImageAndDepth()
        #img, depth = CameraView.computeImageAndDepth(C, True)
        
        points = S.depthData2pointCloud(depth, [fx, fy, cx, cy])
        pcl.setPointCloud(points).setColor([1, 1, 0])
        time.sleep(.01)
        C.view(False)

    plt.imshow(depth)
    plt.show()


if __name__ == "__main__":
    test_noisy_cam()
    