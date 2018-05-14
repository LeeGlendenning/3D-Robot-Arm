// ==========================================================================
package robot;

import Jama.Matrix;
import Jama.SingularValueDecomposition;
import com.jme3.app.SimpleApplication;
import com.jme3.input.KeyInput;
import com.jme3.input.controls.ActionListener;
import com.jme3.input.controls.KeyTrigger;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.scene.Geometry;
import com.jme3.scene.shape.Sphere;
import com.jme3.system.AppSettings;
import com.jme3.scene.shape.Line;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer.Type;
import com.jme3.util.BufferUtils;
import com.jme3.input.controls.AnalogListener;
import com.jme3.material.RenderState.BlendMode;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Node;
import com.jme3.scene.shape.Box;
import com.jme3.scene.shape.Cylinder;

public class Main extends SimpleApplication {

    private Geometry line;
    private Geometry target;
    private Geometry sphereLowerJoint;
    private Geometry cylinderLowerArm;
    private Geometry sphereUpperJoint;
    private Geometry cylinderUpperArm;
    private Geometry boxClaw1;
    private Geometry boxClaw2;
    private Node sObject;
    private Node nodeLowerJoint;
    private Node nodeUpperJoint;
    private int activeJoint = 0;
    private int rotationDirection = 0;
    // joint angles in radians
    private float j1yAngle = 0f;
    private float j1zAngle = 0f;
    private float j2zAngle = 0f;
            
    public static void main(String[] args) {
        Main app = new Main();
        // app.setShowSettings(false);
        AppSettings settings = new AppSettings(true);
        /*
        settings.put("Width", 640);
        settings.put("Height", 480);
        */
        settings.put("Title", "Robot Arm");
        // VSynch
        // settings.put("VSync", true)
        // Anti-Aliasing
        // settings.put("Samples", 4);
        // Initialize sphere control        
        app.setSettings(settings);
        app.start();
    }
    
    private Vector3f startP; 
    private Vector3f endP;
    Vector3f [] vertices;

    /**
     * Simple analoglistener which takes the camera-based directions and adds it
     * to the local translation of the target. Implicitly assumes that the target is
     * directly attached to scene root.
     */
    final private AnalogListener analogListener;

    
    private final ActionListener actionListener;

    
    public Main() {
        this.actionListener = new ActionListener() {    
            @Override
            public void onAction(String name, boolean keyPressed, float tpf) {
                if (name.equals("Next") && !keyPressed) {
                    // Ray from target position at last time step to current.
                    // Reset our line drawing
                    startP.set(endP);
                    // and redraw
                    updateLine();
                } else if (name.equals("Joint0") && !keyPressed) {
                    activeJoint = 0;
                    System.out.println("active joint: 0");
              } else if (name.equals("Joint1") && !keyPressed) {
                    activeJoint = 1;
                    System.out.println("active joint: 1");
              } else if (name.equals("Joint2") && !keyPressed) {
                    activeJoint = 2;
                    System.out.println("active joint: 2");
              } else if (name.equals("Rotate+") && !keyPressed) {
                    rotationDirection = 1;
              } else if (name.equals("Rotate-") && !keyPressed) {
                    rotationDirection = -1;
              }
            }
        };
        
        this.analogListener = new AnalogListener() {
            @Override
            public void onAnalog(String name, float value, float tpf) {
                value *= 10.0;
                // find forward/backward direction and scale it down
                Vector3f camDir = cam.getDirection().clone().multLocal(value);
                // find right/left direction
                Vector3f camLeft = cam.getLeft().clone().multLocal(value);
                // find up/down direction
                Vector3f camUp = cam.getUp().clone().multLocal(value);
                boolean pChange = false;
                if (name.equals("Left")) {
                    Vector3f v = target.getLocalTranslation();
                    target.setLocalTranslation(v.add(camLeft));
                    pChange = true;
                }
                if (name.equals("Right")) {
                    Vector3f v = target.getLocalTranslation();
                    target.setLocalTranslation(v.add(camLeft.negateLocal()));
                    pChange = true;
                }
                if (name.equals("Forward")) {
                    Vector3f v = target.getLocalTranslation();
                    target.setLocalTranslation(v.add(camDir));
                    pChange = true;
                }
                if (name.equals("Back")) {
                    Vector3f v = target.getLocalTranslation();
                    target.setLocalTranslation(v.add(camDir.negateLocal()));
                    pChange = true;
                }
                if (name.equals("Up")) {
                    Vector3f v = target.getLocalTranslation();
                    target.setLocalTranslation(v.add(camUp));
                    pChange = true;
                }
                if (name.equals("Down")) {
                    Vector3f v = target.getLocalTranslation();
                    target.setLocalTranslation(v.add(camUp.negateLocal()));
                    pChange = true;
                }
                if (pChange) updateLine();
            }
        };
    }

    @Override
    public void simpleInitApp() {
        // Left mouse button press to rotate camera
        flyCam.setDragToRotate(true);
        flyCam.setMoveSpeed(20);
        // Do not display stats or fps
        setDisplayFps(false);
        setDisplayStatView(false);
        // move the camera back (10 is the default)
        // cam.setLocation(new Vector3f(0f, 0f, 5.0f));
        // also: cam.setRotation(Quaternion)
        
        float armWidth = 0.3f;
        float armHeight = 2.0f;
        float sphereRadius = 0.4f;

        
        // Generate the robot - starting with a box for the base
        sObject = new Node();
        Geometry base = new Geometry("Base", new Box(0.5f, 0.5f, 0.5f));
        base.setLocalTranslation(-3.5f, -2.5f, 0.0f);
        Material matBase = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        // Use transparency - just to make sure we can always see the target
        matBase.setColor("Color", new ColorRGBA( 0.7f, 0.7f, 0.7f, 0.5f)); // silver'ish
        matBase.getAdditionalRenderState().setBlendMode(BlendMode.Alpha);
        base.setMaterial(matBase);
        base.setQueueBucket(RenderQueue.Bucket.Transparent);
        
        sObject.attachChild(base);
        rootNode.attachChild(sObject);
        
        
        // LOWER JOINT
        nodeLowerJoint = new Node();
        sphereLowerJoint = new Geometry("Sphere", new Sphere(6, 12, sphereRadius));
        Material jointSphere = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        jointSphere.setColor("Color", ColorRGBA.Blue);
        sphereLowerJoint.setMaterial(jointSphere);
        nodeLowerJoint.attachChild(sphereLowerJoint);
        sObject.attachChild(nodeLowerJoint);
        Quaternion quatRotateNeg40j1 = new Quaternion();
        quatRotateNeg40j1.fromAngles(new float[]{0, (float) Math.toRadians(j1yAngle), (float) Math.toRadians(j1zAngle)});
        nodeLowerJoint.setLocalRotation(quatRotateNeg40j1);
        nodeLowerJoint.setLocalTranslation(base.getLocalTranslation().clone().addLocal(0f, sphereRadius+0.2f, 0f));
        
        // LOWER ARM
        cylinderLowerArm = new Geometry("Cylinder", new Cylinder(6, 12, armWidth, armHeight));
        cylinderLowerArm.setMaterial(matBase);
        cylinderLowerArm.setQueueBucket(RenderQueue.Bucket.Transparent);
        nodeLowerJoint.attachChild(cylinderLowerArm);
        Matrix3f rotate90x = new Matrix3f();
        rotate90x.fromAngleAxis((float) Math.toRadians(90f), new Vector3f(1f,0f,0f));
        cylinderLowerArm.setLocalRotation(rotate90x);
        cylinderLowerArm.setLocalTranslation(sphereLowerJoint.getLocalTranslation().clone().addLocal(0f, 2*(sphereRadius+0.1f), 0f));
        
        // UPPER JOINT
        nodeUpperJoint = new Node();
        sphereUpperJoint = new Geometry("Sphere", new Sphere(6, 12, sphereRadius));
        sphereUpperJoint.setMaterial(jointSphere);
        nodeUpperJoint.attachChild(sphereUpperJoint);
        nodeLowerJoint.attachChild(nodeUpperJoint);
        Quaternion quatRotateNeg40j2 = new Quaternion();
        quatRotateNeg40j2.fromAngles(new float[]{0, 0, (float) Math.toRadians(j2zAngle)});
        nodeUpperJoint.setLocalRotation(quatRotateNeg40j2);
        nodeUpperJoint.setLocalTranslation(cylinderLowerArm.getLocalTranslation().clone().addLocal(0f, 2*(sphereRadius+0.1f), 0f));
        
        // UPPER ARM
        cylinderUpperArm = new Geometry("Cylinder", new Cylinder(6, 12, armWidth, armHeight));
        cylinderUpperArm.setMaterial(matBase);
        cylinderUpperArm.setQueueBucket(RenderQueue.Bucket.Transparent);
        nodeUpperJoint.attachChild(cylinderUpperArm);
        cylinderUpperArm.setLocalRotation(rotate90x);
        cylinderUpperArm.setLocalTranslation(sphereUpperJoint.getLocalTranslation().clone().addLocal(0f, 2*(sphereRadius+0.1f), 0f));
        
        // CLAW1
        boxClaw1 = new Geometry("Claw1", new Box(0.3f, 0.1f, 0.1f));
        boxClaw1.setMaterial(matBase);
        boxClaw1.setQueueBucket(RenderQueue.Bucket.Transparent);
        nodeUpperJoint.attachChild(boxClaw1);
        Matrix3f rotate60z= new Matrix3f();
        rotate60z.fromAngleAxis((float) Math.toRadians(60f), new Vector3f(0f,0f,1f));
        boxClaw1.setLocalRotation(rotate60z);
        boxClaw1.setLocalTranslation(cylinderUpperArm.getLocalTranslation().clone().addLocal(0.15f, 2*(sphereRadius+0.1f) + 0.1f, 0f));
        
        // CLAW2
        boxClaw2 = new Geometry("Claw2", new Box(0.3f, 0.1f, 0.1f));
        boxClaw2.setMaterial(matBase);
        boxClaw2.setQueueBucket(RenderQueue.Bucket.Transparent);
        nodeUpperJoint.attachChild(boxClaw2);
        Matrix3f rotateNeg60z= new Matrix3f();
        rotateNeg60z.fromAngleAxis((float) Math.toRadians(-60f), new Vector3f(0f,0f,1f));
        boxClaw2.setLocalRotation(rotateNeg60z);
        boxClaw2.setLocalTranslation(cylinderUpperArm.getLocalTranslation().clone().addLocal(-0.15f, 2*(sphereRadius+0.1f) + 0.1f, 0f));
        
        // Generate a sphere as a symbol for the target point
        target = new Geometry("Sphere", new Sphere(6, 12, 0.1f));
        Material matSphere = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        matSphere.setColor("Color", ColorRGBA.Red);
        target.setMaterial(matSphere);

        rootNode.attachChild(target);
        
        
        
        // Set up line drawing from last position to current position of target
        // We artifically create a time step based on the number of interactions
        startP = new Vector3f(target.getLocalTranslation());
        endP = new Vector3f(target.getLocalTranslation()); 
        vertices = new Vector3f[]{startP,endP};
        Line ln = new Line(startP,endP);
        ln.setLineWidth(2);
        line = new Geometry("line", ln);
        Material mat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        mat.setColor("Color", ColorRGBA.Green);
        line.setMaterial(mat);
        
        rootNode.attachChild(line);

        /** Set up interaction keys */
        setUpKeys();

        // Can be used to change mapping of mouse/keys to camera behaviour
        /*
         if (inputManager != null) {
         inputManager.deleteMapping("FLYCAM_RotateDrag");
         flyCam.setDragToRotate(true);
         inputManager.addMapping("FLYCAM_RotateDrag", new MouseButtonTrigger(MouseInput.BUTTON_RIGHT));
         inputManager.addListener(flyCam, "FLYCAM_RotateDrag");
         }
         */
        
    }
    
    
    private float EPS = 0.1f;
    private float h = 0.1f;
    
    /**
     * Calculate inverse kinematic jacobian and apply rotation to joints
     * 
     * @param jointOrientations float array with joint angles in radians
     */
    private void jacobianIK(float[] jointOrientations) {
        float[] dO;
        while ( boxClaw1.getWorldTranslation().distance(target.getWorldTranslation()) > EPS ) {
            dO = getDeltaOrientation();

            jointOrientations[0] -= dO[0]*h; // j1y
            jointOrientations[1] -= dO[1]*h; // j1z
            jointOrientations[2] -= dO[2]*h; // j2z

            j1yAngle = jointOrientations[0];
            j1zAngle = jointOrientations[1];
            j2zAngle = jointOrientations[2];

            // Rotate arms based on calculated dO
            nodeLowerJoint.setLocalRotation(new Quaternion().fromAngles(new float[]{0, j1yAngle, j1zAngle}));
            nodeUpperJoint.setLocalRotation(new Quaternion().fromAngles(new float[]{0, 0, j2zAngle}));

            // If target it out of range of arm, don't get stuck in infinite loop
            if(boxClaw1.getWorldTranslation().distance(target.getWorldTranslation()) > 4.1f){
                break;
            }
        }
    }
    
    /**
     * Find delta changes to joint angles using jacobian SVD
     * 
     * @return delta changes to joint angles
     */
    private float[] getDeltaOrientation() {
        Matrix jT = getJacobianSVD();
        
        Vector3f tempV = endP.subtract(boxClaw1.getWorldTranslation()); // target - end effector position
        // 3x1 matrix
        Matrix v = new Matrix(new double[][]{{tempV.x}, {tempV.y}, {tempV.z}});
        
        Matrix dO = jT.times(v); // Matrix-Vector Mult. should result in 3x1 matrix
        
        return new float[]{(float)dO.get(0,0), (float)dO.get(1,0), (float)dO.get(2,0)};
    }
    
    /**
     * Construct jacobian and take SVD of it
     * 
     * @return jacobian SVD matrix
     */
    private Matrix getJacobianSVD() {
        
        Vector3f j1y = Vector3f.UNIT_Y.cross(nodeLowerJoint.getWorldTranslation().subtract(boxClaw1.getWorldTranslation())); // joint1y
        Vector3f j1z = Vector3f.UNIT_Z.cross(nodeLowerJoint.getWorldTranslation().subtract(boxClaw1.getWorldTranslation())); // joint1z
        Vector3f j2z = Vector3f.UNIT_Z.cross(nodeUpperJoint.getWorldTranslation().subtract(boxClaw1.getWorldTranslation())); // joint2z
        
        Matrix j = new Matrix(new double[][]{{j1y.x, j1z.x, j2z.x}, {j1y.y, j1z.y, j2z.y}, {j1y.z, j1z.z, j2z.z}});
        
        return getSvd(j);
    }
    
    /**
     * Calculate singular value decomposition of given matrix m
     * 
     * @param m - matrix to get SVD from
     * @return SVD of input matrix m
     */
    private Matrix getSvd(Matrix m) {
        SingularValueDecomposition svd = new Jama.SingularValueDecomposition(m);
        double[] sDiag = svd.getSingularValues();
        // take recipricol of sDiag and put in 2D double array to become matrix S
        double[][] tempS = new double[sDiag.length][sDiag.length];
        for (int i = 0; i < sDiag.length; i ++) {
            if (sDiag[i] != 0) {
                sDiag[i] = 1f / sDiag[i];
            }
            // Create new array, S, with recipricol sDiag vals
            for (int j = 0; j < sDiag.length; j ++) {
                if (j == i) {
                    tempS[i][j] = sDiag[i];
                } else {
                    tempS[i][j] = 0;
                }
            }
        }
        
        Matrix u = svd.getU();
        Matrix sPlus = new Matrix(tempS);
        Matrix v = svd.getV();
        
        return v.times(sPlus.times(u.transpose()));
    }
    
    /**
     * Utility method prints all values in a given matrix to stdout
     * 
     * @param m - matrix to print values of
     */
    private void printMatrix(Matrix m) {
        for (int i = 0; i < m.getRowDimension(); i ++) {
            for (int j = 0; j < m.getColumnDimension(); j ++) {
                System.out.print(m.get(i, j) + " ");
            }
            System.out.println();
        }
    }

    private void setUpKeys() {
        inputManager.addMapping("Left", new KeyTrigger(KeyInput.KEY_LEFT));
        inputManager.addMapping("Right", new KeyTrigger(KeyInput.KEY_RIGHT));
        inputManager.addMapping("Up", new KeyTrigger(KeyInput.KEY_UP));
        inputManager.addMapping("Down", new KeyTrigger(KeyInput.KEY_DOWN));
        inputManager.addMapping("Forward", new KeyTrigger(KeyInput.KEY_PGUP));
        inputManager.addMapping("Back", new KeyTrigger(KeyInput.KEY_PGDN));
        inputManager.addListener(analogListener,
                "Left", "Right", "Up", "Down", "Forward", "Back");
        inputManager.addMapping("Next", new KeyTrigger(KeyInput.KEY_SPACE));
        inputManager.addListener(actionListener, "Next");
        
        inputManager.addMapping("Joint0", new KeyTrigger(KeyInput.KEY_0));
        inputManager.addMapping("Joint1", new KeyTrigger(KeyInput.KEY_1));
        inputManager.addMapping("Joint2", new KeyTrigger(KeyInput.KEY_2));
        inputManager.addMapping("Rotate+", new KeyTrigger(KeyInput.KEY_EQUALS));
        inputManager.addMapping("Rotate-", new KeyTrigger(KeyInput.KEY_MINUS));
        inputManager.addListener(actionListener, "Joint0");
        inputManager.addListener(actionListener, "Joint1");
        inputManager.addListener(actionListener, "Joint2");
        inputManager.addListener(actionListener, "Rotate+");
        inputManager.addListener(actionListener, "Rotate-");
    }
       
    // Helper routine to update line
    void updateLine() {
        Mesh m = line.getMesh();
        endP.set(target.getLocalTranslation());
        m.setBuffer(Type.Position, 3, BufferUtils.createFloatBuffer(vertices));
        m.updateBound();
        
        // Tutorial section on how to use the Jama package
        java.util.Random rand = new java.util.Random();
        double[][] dm = new double[3][3];
        for ( int r=0; r<3; r++) {
            if (r!=1) { // make it singular
                for ( int c=0; c<3; c++) {
                    dm[r][c] = rand.nextFloat();
                }
            }
        }
        Jama.Matrix jm3 = new Jama.Matrix(dm);
        Jama.SingularValueDecomposition svd = new Jama.SingularValueDecomposition(jm3);
        
        double[] s = svd.getSingularValues();
        for ( double e:s) {
            //System.out.print(e + " ");
        }
        //System.out.println();
        
        if (boxClaw1.getWorldTranslation().distance(target.getWorldTranslation()) < 4.1f) {
            jacobianIK(new float[]{j1yAngle, j1zAngle, j2zAngle}); //pass current joint angles in degrees {j1y, j1z, j2z}
        } else {
            // Target is out of range so set arm to default location
            nodeUpperJoint.setLocalRotation(new Quaternion().fromAngles(new float[]{0, 0, 0}));
            nodeLowerJoint.setLocalRotation(new Quaternion().fromAngles(new float[]{0, 0, 0}));
        }
    }
    
    // Update the position every sec
    @Override
    public void simpleUpdate(float tpf) {
        if (rotationDirection != 0) {
            Quaternion quatRotate10z = new Quaternion();
            quatRotate10z.fromAngles(new float[]{0f, 0f, (float) Math.toRadians((float) rotationDirection * 10f)});
            switch (activeJoint) {
                case 0:
                    //System.out.println("Rotate " + rotationDirection +  " joint 0");
                    Quaternion quatRotate10y = new Quaternion();
                    quatRotate10y.fromAngles(new float[]{0f, (float) Math.toRadians((float) rotationDirection * 10f), 0f});
                    nodeLowerJoint.rotate(quatRotate10y);
                    break;
                case 1:
                    System.out.println("Rotate " + rotationDirection +  " joint 1");
                    
                    nodeLowerJoint.rotate(quatRotate10z);
                    break;
                case 2:
                    System.out.println("Rotate " + rotationDirection +  " joint 2");
                    nodeUpperJoint.rotate(quatRotate10z);
                    break;
                default:
                    break;
            }
            rotationDirection = 0;
        }
    }
}
