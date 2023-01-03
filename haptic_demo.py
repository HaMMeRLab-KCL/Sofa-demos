import Sofa
import Sofa.SofaGL as SG
import Sofa.Core as SC
import Sofa.Simulation as SS
import SofaRuntime
import os
import time
sofa_directory = os.environ["SOFA_ROOT"]
import pygame
from OpenGL.GL import *
from OpenGL.GLU import *

display_size = (1920, 1080)
white = [255, 255, 255]
red = [255, 0, 0]

def init_display(node: SC.Node):
    """
    Define the initial window for the pygame rendering

    Args:
        node (SC.Node): Root node for a Sofa simulation scene
    """
    pygame.display.init()
    pygame.display.set_mode(display_size, pygame.DOUBLEBUF | pygame.OPENGL)
    pygame.display.set_caption("Haptic demo")
    
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glEnable(GL_LIGHTING)
    glEnable(GL_DEPTH_TEST)
    SG.glewInit()
    SS.initVisual(node)
    SS.initTextures(node)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, (display_size[0] / display_size[1]), 0.1, 50.0)

    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    pygame.display.get_surface().fill(red)
    pygame.display.flip()

def simple_render(rootNode: SC.Node):
    """
    Get the OpenGL context to render an image of the simulation state

    Args:
        rootNode (SC.Node): Sofa root node 
    """
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glEnable(GL_LIGHTING)
    glEnable(GL_DEPTH_TEST)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, (display_size[0] / display_size[1]), 0.1, 50.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

    cameraMVM = rootNode.camera.getOpenGLModelViewMatrix()
    glMultMatrixd(cameraMVM)
    SG.draw(rootNode)

    pygame.display.get_surface().fill(red)
    pygame.display.flip()

def createScene(root: SC.Node):
    """
    This function is necessary to run Sofa from both the runSofa executable and the python interpreter (IDE)

    Args:
        root (SC.Node): Root node for a Sofa simulation scene
    """
    # Register all the common component in the factory.
    SofaRuntime.PluginRepository.addFirstPath(os.path.join(sofa_directory, 'bin'))
    root.addObject("RequiredPlugin", name="Sofa.Component.IO.Mesh")
    root.addObject("RequiredPlugin", name="Sofa.Component.Engine.Transform")
    root.addObject("RequiredPlugin", name="Sofa.Component.LinearSolver.Direct")
    root.addObject("RequiredPlugin", name="Sofa.Component.Mass")
    root.addObject("RequiredPlugin", name="Sofa.Component.ODESolver.Backward")
    root.addObject("RequiredPlugin", name="Sofa.Component.SolidMechanics.Spring")
    root.addObject("RequiredPlugin", name="Sofa.Component.StateContainer")
    root.addObject("RequiredPlugin", name="Sofa.Component.Topology.Container.Constant")
    root.addObject("RequiredPlugin", name="Sofa.Component.Visual")
    root.addObject("RequiredPlugin", name="Sofa.GL.Component.Rendering3D")
    root.addObject("RequiredPlugin", name="SofaConstraint")
    root.addObject("RequiredPlugin", name="SofaHaptics")
    root.addObject("RequiredPlugin", name="SofaMeshCollision")
    root.addObject("RequiredPlugin", name="SofaUserInteraction")
    root.addObject("RequiredPlugin", name="Sofa.Component.SceneUtility")
    root.addObject("RequiredPlugin", name="SofaPython3")
    root.addObject("RequiredPlugin", name="Geomagic")

    root.addObject("VisualStyle", displayFlags="showVisualModels showBehaviorModels hideCollisionModels")

    root.addObject("DefaultPipeline", name="pipeline", depth=6, verbose=0)
    root.addObject("BruteForceBroadPhase")
    root.addObject("BVHNarrowPhase")
    root.addObject("DefaultContactManager", name="response", response="FrictionContactConstraint")
    root.addObject("LocalMinDistance", name="proximity", alarmDistance=0.15, contactDistance=0.05, angleCone=0.0)
    root.addObject("FreeMotionAnimationLoop")
    root.addObject("LCPConstraintSolver", tolerance=0.001, maxIt=1000)
    root.addObject("GeomagicDriver", name="GeomagicDevice", deviceName="Default Device", scale=2.0, drawDeviceFrame=False, positionBase=[0, 0, 0], orientationBase=[0.707, 0, 0, -0.707])
    # place light and a camera
    root.addObject("LightManager")
    root.addObject("DirectionalLight", direction=[0,1,0])
    root.addObject("InteractiveCamera", name="camera", position=[0,20,0],
                            lookAt=[0,0,0], distance=37,
                            fieldOfView=45, zNear=0.63, zFar=55.69)

    #########################################
    ############ SOFT SPHERE ################
    #########################################

    soft_sphere = root.addChild("SoftSphere")
    soft_sphere.addObject("EulerImplicitSolver", rayleighMass=0.01, rayleighStiffness=0.1)
    soft_sphere.addObject("SparseLDLSolver", template="CompressedRowSparseMatrixMat3x3d")
    soft_sphere.addObject("MeshOBJLoader", name="loaderS", filename="mesh/sphere_05.obj", scale3d=[0.05, 0.05, 0.05], translation=[-5,0,0])
    soft_sphere.addObject("SparseGridRamificationTopology", name="grid", n=[6,6,6], src="@loaderS", nbVirtualFinerLevels=3, finestConnectivity=0)
    soft_sphere.addObject("MechanicalObject", name="gridDof", position="@grid.position")
    soft_sphere.addObject("TetrahedronFEMForceField", name="FEM", youngModulus=8e4, poissonRatio=0.45, method="large")
    soft_sphere.addObject("DiagonalMass", totalMass=10.0, topology="@grid", geometryState="@gridDof")
    soft_sphere.addObject("BoxROI", name="boxRoi1", box=[-5.6,-0.6,-0.6,-4.4,0.6,0.6], drawBoxes=1)
    soft_sphere.addObject("FixedConstraint", indices="@boxRoi1.indices")
    soft_sphere.addObject("LinearSolverConstraintCorrection")

    col_soft_sphere = soft_sphere.addChild("MappingSphere")
    col_soft_sphere.addObject("MechanicalObject", name="dofs", position="@../loaderS.position")
    col_soft_sphere.addObject("TriangleSetTopologyContainer", name="Container1", src="@../loaderS")
    col_soft_sphere.addObject("TriangleSetTopologyModifier")
    col_soft_sphere.addObject("TriangleSetGeometryAlgorithms", template="Vec3d")
    col_soft_sphere.addObject("TriangleCollisionModel", bothSide=False, group=1, contactStiffness=1e5)
    col_soft_sphere.addObject("LineCollisionModel", group=1, contactStiffness=1e5)
    col_soft_sphere.addObject("PointCollisionModel", group=1, contactStiffness=1e5)
    col_soft_sphere.addObject("BarycentricMapping", input="@../gridDof", output="@dofs")

    visu_soft_sphere = soft_sphere.addChild("VisuSurface")
    visu_soft_sphere.addObject("OglModel", name="Visual", color="blue", src="@../loaderS")
    visu_soft_sphere.addObject("BarycentricMapping", input="@..", output="@Visual")

    #########################################
    ############ RIGID SPHERE ###############
    #########################################
    
    rigid_sphere = root.addChild("RigidSphere")
    rigid_sphere.addObject("MeshOBJLoader", name="loaderR", filename="mesh/sphere_05.obj", scale3d=[0.05,0.05,0.05], translation=[5,0,0])
    rigid_sphere.addObject("MechanicalObject", position="@loaderR.position")
    rigid_sphere.addObject("MeshTopology", name="grid", src="@loaderR")
    rigid_sphere.addObject("TriangleCollisionModel", name="SphereTC", simulated=False, moving=False, bothSide=False, contactStiffness=1e5)
    rigid_sphere.addObject("LineCollisionModel", name="SphereLC", simulated=False, moving=False, contactStiffness=1e5)
    rigid_sphere.addObject("PointCollisionModel", name="SpherePC", simulated=False, moving=False, contactStiffness=1e5)

    visu_rigid_sphere = rigid_sphere.addChild("SphereVisu")
    visu_rigid_sphere.addObject("OglModel", name="SphereVisualModel", src='@../loaderR', color='gray')
    visu_rigid_sphere.addObject("BarycentricMapping", input='@..', output='@SphereVisualModel')

    #########################################
    ############## TOUCH X ##################
    #########################################
    
    omni = root.addChild("Omni")
    omni.addObject("MechanicalObject", name="DOFs", template="Rigid3d", position="@GeomagicDevice.positionDevice", ry=180, rx=90)

    #########################################
    ############## INSTRUMENT ###############
    #########################################

    instrument = root.addChild("Instrument")
    instrument.addObject("EulerImplicitSolver", name="ODE solver", rayleighStiffness=0.05, rayleighMass=1.0)
    instrument.addObject("SparseLDLSolver", template="CompressedRowSparseMatrixMat3x3d")
    instrument.addObject("MechanicalObject", name="instrumentState", template="Rigid3d")
    instrument.addObject("UniformMass", name="mass", totalMass=0.01)
    instrument.addObject("RestShapeSpringsForceField", stiffness=1e6, angularStiffness=1e6, external_rest_shape='@../Omni/DOFs', points=0, external_points=0)
    instrument.addObject("LCPForceFeedback", activate=True, forceCoef=5e-5)
    instrument.addObject("LinearSolverConstraintCorrection")

    visu_instrument = instrument.addChild("VisualModel")
    visu_instrument.addObject("MeshOBJLoader", name="meshLoader_1", filename="Demos/Dentistry/data/mesh/dental_instrument.obj", handleSeams=1)
    visu_instrument.addObject("OglModel", name="InstrumentVisualModel", src="@meshLoader_1", color="1.0 0.2 0.2 1.0", ry=-180, rz=-90, dz=3.5, dx=-0.3)
    visu_instrument.addObject("RigidMapping", name="MM->VM mapping", input="@instrumentState", output="@InstrumentVisualModel")

    col_instrument = instrument.addChild("CollisionModel")
    col_instrument.addObject("MeshOBJLoader", filename="Demos/Dentistry/data/mesh/dental_instrument_centerline.obj", name="loader")
    col_instrument.addObject("MeshTopology", src="@loader", name="InstrumentCollisionModel")
    col_instrument.addObject("MechanicalObject", src="@loader", name="instrumentCollisionState", ry=-180, rz=-90, dz=3.5, dx=-0.3)
    col_instrument.addObject("LineCollisionModel", contactStiffness=10)
    col_instrument.addObject("PointCollisionModel", contactStiffness=10)
    col_instrument.addObject("RigidMapping", name="MM->CM mapping", input="@instrumentState", output="@instrumentCollisionState")


def main():
    SofaRuntime.importPlugin("SofaComponentAll")
    root = SC.Node("root")
    createScene(root)
    SS.init(root)
    init_display(root)

    try:
        while True:
            SS.animate(root, root.getDt())
            SS.updateVisual(root)
            simple_render(root)
            time.sleep(root.getDt())
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()