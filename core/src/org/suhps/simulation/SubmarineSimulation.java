package org.suhps.simulation;

import java.io.*;
import java.text.*;
import java.util.*;

import com.badlogic.gdx.*;
import com.badlogic.gdx.graphics.*;
import com.badlogic.gdx.graphics.glutils.*;
import com.badlogic.gdx.math.*;
import com.badlogic.gdx.physics.box2d.*;

public class SubmarineSimulation extends ApplicationAdapter implements InputProcessor {

    private static final String TAG = "SUHPS";

    // Properties of the submarine
    private static final float SUB_WIDTH = 2.2f;
    private static final float SUB_HEIGHT = 0.6f;
    private static final float SUB_MASS = 140f;
    private static final float SUB_CROSS_SECTIONAL_AREA = MathUtils.PI * 0.3f * 0.3f;
    private static final float SUB_FLUID_DENSITY = 1000f;
    private static final float SUB_DRAG_COEFFICIENT = 0.04f;
    private static final float SUB_LIFT_COEFFICIENT_SLOPE = MathUtils.PI / 2f;
    private static final float SUB_SPINNING_DRAG_COEFFICIENT = 2f;
    private static final float SUB_INITIAL_SPEED = 0f;
    private static final float SUB_INITIAL_ANGLE = MathUtils.PI;

    // Properties of the simulation
    private static float SIM_THETA = 0f;
    private static final float SIM_THRUST = 150f;
    private static final String SIM_CSV_DIRECTORY = "~/Desktop";

    // Propeties of the course
    private static final float COURSE_WIDTH = 90f;
    private static final float COURSE_HEIGHT = 52f;

    private World mWorld;
    private Body mSubmarine;

    private OrthographicCamera mCamera;
    private ShapeRenderer mShapeRenderer;
    private Box2DDebugRenderer mRenderer;

    private FileWriter mCsvWriter;
    private List<Body> mObstacles;

    private boolean mPaused = true;
    private int mFrameNumber = 0;

    @Override
    public void create() {
        Box2D.init();

        try {
            SimpleDateFormat dt = new SimpleDateFormat("yyyyMMdd hhmmss");
            String path = SIM_CSV_DIRECTORY.replaceFirst("~", System.getProperty("user.home"));
            path += "/Sub " + dt.format(new Date()) + ".csv";
            Gdx.app.log(TAG, "Writing CSV file to: " + path);
            mCsvWriter = new FileWriter(path);
            mCsvWriter.append("Time,X,Y,Angle\n");
        } catch (IOException e) {
            e.printStackTrace();
        }

        Gdx.app.setLogLevel(Application.LOG_DEBUG);

        mObstacles = new ArrayList<Body>();

        mCamera = new OrthographicCamera(Gdx.graphics.getWidth(),
                                         Gdx.graphics.getHeight());

        mWorld = new World(new Vector2(0, 0), false);

        mRenderer = new Box2DDebugRenderer();
        mShapeRenderer = new ShapeRenderer();

        mSubmarine = createSubmarine();
        mSubmarine.setLinearVelocity(SUB_INITIAL_SPEED, 0f);

        createCourse();

        Gdx.input.setInputProcessor(this);
    }

    @Override
    public void dispose() {
        if (mCsvWriter != null) {
            try {
                mCsvWriter.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    private Body createSubmarine() {
        BodyDef bodyDef = new BodyDef();
        bodyDef.type = BodyDef.BodyType.DynamicBody;
        bodyDef.position.set(COURSE_WIDTH / 2f - 15, COURSE_HEIGHT / 4f);
        bodyDef.angle = SUB_INITIAL_ANGLE;

        Body body = mWorld.createBody(bodyDef);

        PolygonShape shape = new PolygonShape();
        shape.setAsBox(SUB_WIDTH / 2f, SUB_HEIGHT / 2f);

        float area = SUB_WIDTH * SUB_HEIGHT;
        float density = SUB_MASS / area;

        FixtureDef fixtureDef = new FixtureDef();
        fixtureDef.shape = shape;
        fixtureDef.density = density;
        fixtureDef.friction = 0f;
        fixtureDef.restitution = 0f;

        Fixture fixture = body.createFixture(fixtureDef);

        shape.dispose();

        return body;
    }

    private void createObstacle(float x, float y) {
        CircleShape shape = new CircleShape();
        shape.setRadius(0.1f);

        BodyDef bodyDef = new BodyDef();
        bodyDef.position.set(new Vector2(x, y));

        Body body = mWorld.createBody(bodyDef);
        body.createFixture(shape, 0.0f);
        shape.dispose();

        mObstacles.add(body);
    }

    private void createWalls() {
        BodyDef bodyDef = new BodyDef();
        bodyDef.position.set(new Vector2(0, 0));

        Body body = mWorld.createBody(bodyDef);

        ChainShape shape = new ChainShape();
        shape.createLoop(new float[] {
            -COURSE_WIDTH / 2f, -COURSE_HEIGHT / 2f,
            +COURSE_WIDTH / 2f, -COURSE_HEIGHT / 2f,
            +COURSE_WIDTH / 2f, +COURSE_HEIGHT / 2f,
            -COURSE_WIDTH / 2f, +COURSE_HEIGHT / 2f
        });
        body.createFixture(shape, 0.0f);
        shape.dispose();
    }

    private void createBarrier() {
        CircleShape circleShape = new CircleShape();
        circleShape.setRadius(4f);

        BodyDef bodyDef = new BodyDef();
        bodyDef.position.set(new Vector2(0, 0));

        Body body = mWorld.createBody(bodyDef);
        body.createFixture(circleShape, 0.0f);
        circleShape.dispose();

        PolygonShape polygonShape = new PolygonShape();
        polygonShape.setAsBox(COURSE_WIDTH / 4f, 1.5f);

        bodyDef = new BodyDef();
        bodyDef.position.set(new Vector2(COURSE_WIDTH / 4f, 0));

        body = mWorld.createBody(bodyDef);
        body.createFixture(polygonShape, 0.0f);
        polygonShape.dispose();
    }

    private void createCourse() {
        createWalls();
        createBarrier();

        createObstacle(COURSE_WIDTH / 4f, COURSE_HEIGHT / 4f - 6.5f);
        createObstacle(COURSE_WIDTH / 4f, COURSE_HEIGHT / 4f + 6.5f);

        createObstacle(-COURSE_WIDTH / 4f, COURSE_HEIGHT / 4f - 6.5f);
        createObstacle(-COURSE_WIDTH / 4f, COURSE_HEIGHT / 4f + 6.5f);
        createObstacle(-COURSE_WIDTH / 6f, COURSE_HEIGHT / 4f - 6.5f);
        createObstacle(-COURSE_WIDTH / 6f, COURSE_HEIGHT / 4f + 6.5f);

        float gap = -COURSE_WIDTH / 8f;
        createObstacle(-gap, -COURSE_HEIGHT / 4f);
        createObstacle(0, -COURSE_HEIGHT / 4f);
        createObstacle(+gap, -COURSE_HEIGHT / 4f);
        createObstacle(+gap * 2f, -COURSE_HEIGHT / 4f);

        createObstacle(COURSE_WIDTH / 4f, -COURSE_HEIGHT / 4f - 6.5f);
        createObstacle(COURSE_WIDTH / 4f, -COURSE_HEIGHT / 4f + 6.5f);
    }

    private void drawForce(Vector2 position, Vector2 value) {
        Vector2 start = position;
        Vector2 end = Vector2.X.set(value).scl(0.01f).add(position);
        mShapeRenderer.x(position, 0.08f);
        mShapeRenderer.line(position, end);
    }

    private void applyThrust() {
        Vector2 thrust = new Vector2(SIM_THRUST, 0);
        thrust.rotate(SIM_THETA);
        thrust.rotateRad(mSubmarine.getAngle());

        Vector2 position = mSubmarine.getWorldPoint(new Vector2(-SUB_WIDTH / 2f, 0f));

        //Gdx.app.debug(TAG, "Thrust = " + thrust);

        if (!mPaused) {
            mSubmarine.applyForce(thrust, position, true);
        }

        mShapeRenderer.setColor(1f, 0f, 0f, 1f);
        drawForce(position, thrust);
    }

    private void applyDrag() {
        Vector2 velocity = mSubmarine.getLinearVelocity();

        float v2 = velocity.len() * velocity.len();
        float value = 0.5f * SUB_FLUID_DENSITY * SUB_CROSS_SECTIONAL_AREA * SUB_DRAG_COEFFICIENT * v2;
        Vector2 drag = velocity.cpy().nor().scl(-value);

        //Gdx.app.debug(TAG, "Velocity = " + velocity + ", Drag = " + drag);

        if (!mPaused) {
            mSubmarine.applyForceToCenter(drag, true);
        }

        Vector2 position = mSubmarine.getWorldCenter().cpy();

        mShapeRenderer.setColor(0f, 1f, 0f, 1f);
        drawForce(position, drag);
    }

    private float wrapAngle(float angle) {
        while (angle > MathUtils.PI) {
            angle -= MathUtils.PI2;
        }

        while (angle < -MathUtils.PI) {
            angle += MathUtils.PI2;
        }

        return angle;
    }

    private void applyLift() {
        Vector2 velocity = mSubmarine.getLinearVelocity();
        float angle = wrapAngle(mSubmarine.getAngle());

        float alpha = wrapAngle(angle - wrapAngle(velocity.angleRad()));

        //Gdx.app.log(TAG, "" + angle + "\t" + velocity.angleRad() + "\t" + alpha);

        if (Math.abs(alpha) < MathUtils.degreesToRadians * 15) {
            float liftCoefficient = alpha * SUB_LIFT_COEFFICIENT_SLOPE;

            float v2 = velocity.len() * velocity.len();
            //Gdx.app.log(TAG, "" + v2);

            float value = 0.5f * SUB_FLUID_DENSITY * SUB_CROSS_SECTIONAL_AREA * liftCoefficient * v2;

            Vector2 lift = velocity.cpy().nor().rotate90(1).scl(value);

            //Gdx.app.debug(TAG, "Velocity = " + velocity + ", Lift = " + lift);

            Vector2 position = mSubmarine.getWorldPoint(new Vector2(SUB_WIDTH / 4f, 0f));

            if (!mPaused) {
                mSubmarine.applyForce(lift, position, true);
            }

            mShapeRenderer.setColor(0f, 0f, 1f, 1f);
            drawForce(position, lift);
        }
    }

    private void applySpinningDrag() {
        float angularVelocity = mSubmarine.getAngularVelocity();

        float v2 = angularVelocity * angularVelocity;
        float value = (0.5f * SUB_FLUID_DENSITY * SUB_CROSS_SECTIONAL_AREA * SUB_SPINNING_DRAG_COEFFICIENT * v2) / SUB_WIDTH;

        float drag = -value;
        if (angularVelocity < 0) {
            drag = value;
        }

        if (!mPaused) {
            mSubmarine.applyTorque(drag, true);
        }

        //Gdx.app.debug(TAG, "Angular Velocity = " + angularVelocity + ", Spinning Drag = " + drag);
    }

    @Override
    public void render() {
        Gdx.gl.glClearColor(0, 0, 0, 1);
        Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT);

        mShapeRenderer.begin(ShapeRenderer.ShapeType.Filled);
        mShapeRenderer.identity();
        mShapeRenderer.setColor(0.95f, 1f, 1f, 1);
        mShapeRenderer.rect(-COURSE_WIDTH / 2f, -COURSE_HEIGHT / 2f, COURSE_WIDTH, COURSE_HEIGHT);
        mShapeRenderer.end();

        Vector2 position = mSubmarine.getWorldCenter();
        mCamera.update();

        mShapeRenderer.setProjectionMatrix(mCamera.combined);

        if (!mPaused) {
            mWorld.step(1 / 100f, 6, 2);
        }

        mShapeRenderer.begin(ShapeRenderer.ShapeType.Filled);

        // sub
        mShapeRenderer.identity();
        mShapeRenderer.translate(position.x, position.y, 0);
        mShapeRenderer.rotate(0, 0, 1, mSubmarine.getAngle() * MathUtils.radiansToDegrees);
        mShapeRenderer.setColor(0, 0, 0, 1);
        mShapeRenderer.ellipse(-SUB_WIDTH, -SUB_HEIGHT, SUB_WIDTH * 2f, SUB_HEIGHT * 2f);

        // obstacles
        mShapeRenderer.setColor(0.5f, 0.5f, 0.5f, 1);

        for (Body body : mObstacles) {
            mShapeRenderer.identity();
            mShapeRenderer.translate(body.getWorldCenter().x, body.getWorldCenter().y, 0);
            mShapeRenderer.circle(0, 0, 0.4f, 8);
        }

        // barrier
        mShapeRenderer.identity();
        mShapeRenderer.circle(0, 0, 4f, 32);
        mShapeRenderer.rect(0, -1.5f, COURSE_WIDTH / 2f, 3f);

        mShapeRenderer.end();

        mShapeRenderer.begin(ShapeRenderer.ShapeType.Line);

        mShapeRenderer.identity();

        applyThrust();
        applyDrag();
        applyLift();
        applySpinningDrag();

        mShapeRenderer.end();

        mRenderer.render(mWorld, mCamera.combined);

        if (Gdx.input.isKeyPressed(Input.Keys.LEFT)) {
            SIM_THETA += 1f;
        }

        if (Gdx.input.isKeyPressed(Input.Keys.RIGHT)) {
            SIM_THETA -= 1f;
        }

        if (!mPaused) {
            if (mCsvWriter != null) {
                try {
                    mCsvWriter.append(mFrameNumber + "," + position.x + "," + position.y + "," + mSubmarine.getAngle() * MathUtils.radiansToDegrees + "\n");
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }

            mFrameNumber += 1;
        }
    }

    @Override
    public void resize(int w, int h) {
        Gdx.app.debug(TAG, "Resizing to " + w + "x" + h + ".");
        mCamera.setToOrtho(false);
        mCamera.position.set(0, 0, 0);

        float zoom1 = COURSE_WIDTH / w;
        float zoom2 = COURSE_HEIGHT / h;

        mCamera.zoom = Math.max(zoom1, zoom2);

        mCamera.update();
    }

    @Override
    public boolean keyDown(int keycode) {
        return false;
    }

    @Override
    public boolean keyUp(int keycode) {
        if (keycode == Input.Keys.P) {
            mPaused = !mPaused;
            return true;
        } else {
            return false;
        }
    }

    @Override
    public boolean keyTyped(char character) {
        return false;
    }

    @Override
    public boolean touchDown(int x, int y, int pointer, int button) {
        return false;
    }

    @Override
    public boolean touchUp(int x, int y, int pointer, int button) {
        return false;
    }

    @Override
    public boolean touchDragged(int x, int y, int pointer) {
        return false;
    }

    @Override
    public boolean mouseMoved(int x, int y) {
        return false;
    }

    @Override
    public boolean scrolled(int amount) {
        return false;
    }

}
