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
im_dir = "imgs/cair-cas-logo-alpha.png"

class ImageLoader:
    
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.width = 0
        self.height = 0
        self.img_data = 0
    
    def load(self, im_dir):
        image = pygame.image.load(im_dir).convert_alpha()
        img_data = pygame.image.tostring(image, 'RGBA')
        self.width = image.get_width()
        self.height = image.get_height()

        self.texID = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, self.texID)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, self.width, self.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, img_data)
        glGenerateMipmap(GL_TEXTURE_2D)
        glBindTexture(GL_TEXTURE_2D, 0)     

    def draw(self):

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glTranslate(self.x, self.y, 0)

        glEnable(GL_TEXTURE_2D)
        glBindTexture(GL_TEXTURE_2D, self.texID)
        glBegin(GL_QUADS)
        glTexCoord2f(0, 0)
        glVertex2f(0, 0)
        glTexCoord2f(1, 0)
        glVertex2f(self.width, 0)
        glTexCoord2f(1, 1)
        glVertex2f(self.width, self.height)
        glTexCoord2f(0, 1)
        glVertex2f(0, self.height)
        glEnd()
        glDisable(GL_TEXTURE_2D)


def init_display(node: SC.Node, im_loader: ImageLoader):
    """
    Define the initial window for the pygame rendering

    Args:
        node (SC.Node): Root node for a Sofa simulation scene
    """
    pygame.display.init()
    pygame.display.set_mode(display_size, pygame.DOUBLEBUF | pygame.OPENGL)
    pygame.display.set_caption("Haptic demo")
    glClearColor(1, 1, 1, 1)
    
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluOrtho2D(0, display_size[0], display_size[1], 1)
    glMatrixMode(GL_MODELVIEW)

    glLoadIdentity()
    glDisable(GL_DEPTH_TEST)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    im_loader.load(im_dir)
    im_loader.draw()

    glEnable(GL_LIGHTING)
    glEnable(GL_DEPTH_TEST)
    SG.glewInit()
    SS.initVisual(node)
    SS.initTextures(node)

    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, (display_size[0] / display_size[1]), 0.1, 50.0)
    
    # Set the background to white
    # glClearColor(1, 1, 1, 1)
    # glClear(GL_COLOR_BUFFER_BIT)

    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

    pygame.display.flip()

def simple_render(rootNode: SC.Node, im_loader: ImageLoader):
    """
    Get the OpenGL context to render an image of the simulation state

    Args:
        rootNode (SC.Node): Sofa root node 
    """
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluOrtho2D(0, display_size[0], display_size[1], 1)
    glMatrixMode(GL_MODELVIEW)

    glLoadIdentity()
    glDisable(GL_DEPTH_TEST)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    im_loader.draw()

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

    root.addObject("VisualStyle", displayFlags="showVisualModels hideBehaviorModels hideCollisionModels")

    root.addObject("DefaultPipeline", name="pipeline", depth=6, verbose=0)
    root.addObject("BruteForceBroadPhase")
    root.addObject("BVHNarrowPhase")
    root.addObject("DefaultContactManager", name="response", response="FrictionContactConstraint")
    root.addObject("LocalMinDistance", name="proximity", alarmDistance=0.3, contactDistance=0.15, angleCone=0.0)
    root.addObject("FreeMotionAnimationLoop")
    root.addObject("LCPConstraintSolver", tolerance=1e-3, maxIt=1e3)
    root.addObject("GeomagicDriver", name="GeomagicDevice", deviceName="Default Device", scale=2.0, drawDeviceFrame=0, positionBase=[0, 0, 0], orientationBase=[0.707, 0, 0, -0.707])
    # place light and a camera
    root.addObject("LightManager")
    root.addObject("DirectionalLight", direction=[0,1,0], color=[1,0.85,0.85])
    root.addObject("InteractiveCamera", name="camera", position=[0,17.5,0],
                            lookAt=[0,0,0], distance=30,
                            fieldOfView=45, zNear=0.63, zFar=55.69)

    #########################################
    ############# PINK BUNNY ################
    #########################################

    bunny = root.addChild("bunny")
    bunny.addObject("EulerImplicitSolver", rayleighMass=0.1, rayleighStiffness=0.1)
    bunny.addObject("SparseLDLSolver", template="CompressedRowSparseMatrixMat3x3d")
    bunny.addObject("MeshVTKLoader", name="loader", filename='mesh/Hollow_Stanford_Bunny.vtu')
    bunny.addObject("TetrahedronSetTopologyContainer", src="@loader", name="container")
    bunny.addObject("TetrahedronSetTopologyModifier")
    bunny.addObject("MechanicalObject", name="tetras", template="Vec3d", showIndices=False, rx=-90)
    bunny.addObject("UniformMass", totalMass=0.5)
    bunny.addObject("TetrahedronFEMForceField", template="Vec3d", name="FEM", method="large", poissonRatio=0.35, youngModulus=2e5)

    bunny.addObject("BoxROI", name='boxROI', box=[-5, 5, 4.5, 5, -5, 5], drawBoxes=False, position='@tetras.rest_position', tetrahedra='@container.tetrahedra')
    bunny.addObject('FixedConstraint', indices='@boxROI.indices')
    bunny.addObject('LinearSolverConstraintCorrection')

    col_bunny = bunny.addChild('col')
    col_bunny.addObject('TriangleSetTopologyContainer', name='container')
    col_bunny.addObject('TriangleSetTopologyModifier')
    col_bunny.addObject('Tetra2TriangleTopologicalMapping', name='mapping', input="@../container", output="@container")
    col_bunny.addObject('TriangleCollisionModel', contactStiffness=1e3)
    col_bunny.addObject('PointCollisionModel', contactStiffness=1e3)

    visu_bunny = col_bunny.addChild('visu')
    visu_bunny.addObject('OglModel', name='Visual', color=[1, 0.87, 0.87, 1])
    visu_bunny.addObject('IdentityMapping', input='@../../tetras', output='@Visual')

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
    instrument.addObject("LCPForceFeedback", activate=True, forceCoef=3e-4)
    instrument.addObject("LinearSolverConstraintCorrection")

    visu_instrument = instrument.addChild("VisualModel")
    visu_instrument.addObject("MeshOBJLoader", name="meshLoader_1", filename="Demos/Dentistry/data/mesh/dental_instrument.obj", handleSeams=1)
    visu_instrument.addObject("OglModel", name="InstrumentVisualModel", src="@meshLoader_1", color="0.2 0.2 1.0 1.0", ry=-180, rz=-90, dy=5, dz=3.5, dx=-0.3, scale=1.2)
    visu_instrument.addObject("RigidMapping", name="MM->VM mapping", input="@instrumentState", output="@InstrumentVisualModel")

    col_instrument = instrument.addChild("CollisionModel")
    col_instrument.addObject("MeshOBJLoader", filename="Demos/Dentistry/data/mesh/dental_instrument_centerline.obj", name="loader")
    col_instrument.addObject("MeshTopology", src="@loader", name="InstrumentCollisionModel")
    col_instrument.addObject("MechanicalObject", src="@loader", name="instrumentCollisionState", ry=-180, rz=-90, dy=5, dz=3.5, dx=-0.3, scale=1.2)
    col_instrument.addObject("LineCollisionModel", contactStiffness=1e4)
    col_instrument.addObject("PointCollisionModel", contactStiffness=1e4)
    col_instrument.addObject("RigidMapping", name="MM->CM mapping", input="@instrumentState", output="@instrumentCollisionState")


def main():
    SofaRuntime.importPlugin("SofaComponentAll")
    im_loader = ImageLoader(50, 50)
    root = SC.Node("root")
    createScene(root)
    SS.init(root)
    init_display(root, im_loader)

    try:
        while True:
            SS.animate(root, root.getDt())
            SS.updateVisual(root)
            simple_render(root, im_loader)
            time.sleep(root.getDt())
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()